/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "bt_mgmt_scan_for_broadcast_internal.h"

#include <zephyr/zbus/zbus.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/slist.h>
#include <zephyr/drivers/display.h>
#include <lvgl.h>

#include "bt_mgmt.h"
#include "macros_common.h"
#include "nrf5340_audio_common.h"
#include "led.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_mgmt_scan);

/* Any value above 0xFFFFFF is invalid, so one can use 0xFFFFFFFF to denote
 * an invalid broadcast ID.
 */
#define INVALID_BROADCAST_ID 0xFFFFFFFF
#define PA_SYNC_SKIP	     2
/* Similar to retries for connections */
#define SYNC_RETRY_COUNT     6

ZBUS_CHAN_DECLARE(bt_mgmt_chan);

struct bt_le_scan_cb scan_callback;
static bool scan_cb_registered;
static bool sync_cb_registered;
static char const *srch_name;
static struct bt_le_per_adv_sync *pa_sync;
static uint32_t broadcaster_broadcast_id;

static uint8_t num_broadcasters;

static const struct gpio_dt_spec center_led_g = GPIO_DT_SPEC_GET(DT_NODELABEL(rgb1_green), gpios);

struct broadcast_source {
	char name[BLE_SEARCH_NAME_MAX_LEN];
	uint32_t broadcast_id;
};

#define NAME_SIZE_MAX	 250
#define BROADCASTERS_MAX 3
#define TIMEOUT_S	 5
#define V_OFFSET_PIXELS	 30
static lv_obj_t *header_label_1;
static lv_obj_t *header_label_2;
static lv_obj_t *label_name[BROADCASTERS_MAX];
static lv_obj_t *label_time[BROADCASTERS_MAX];
static lv_obj_t *btn[BROADCASTERS_MAX];

struct name_timeout_pair {
	sys_snode_t node;
	char name[NAME_SIZE_MAX];
	uint64_t last_seen;
	bool update;
	uint32_t broadcast_id;
};

static struct name_timeout_pair n_t_pair[BROADCASTERS_MAX];
static sys_slist_t avail_list;
static sys_slist_t filled_list;
static K_MUTEX_DEFINE(module_list_lock);

static uint8_t last_seen_string_gen(char *buf, uint32_t last_seen_s)
{
	if (last_seen_s <= TIMEOUT_S) {
		return sprintf(buf, "#00ff00 ~%d s#", last_seen_s);
	} else if (last_seen_s <= 60) {
		return sprintf(buf, "#ff0000 ~%d s#", last_seen_s);
	} else if (last_seen_s <= 3600) {
		return sprintf(buf, "#ff0000 ~%d m#", last_seen_s / 60);
	} else {
		return sprintf(buf, "#ff0000 ~%d h#", last_seen_s / 3600);
	}
}

static void timer_worker(struct k_work *work)
{
	static struct name_timeout_pair *n_t_pair;
	sys_snode_t *node;
	char line_buf[300] = {'\0'};

	num_broadcasters = 0;
	sprintf(line_buf, "Scanning %d", (uint32_t)(k_uptime_get() / 1000));
	lv_label_set_text(header_label_1, line_buf);

	SYS_SLIST_FOR_EACH_NODE(&filled_list, node) {
		n_t_pair = CONTAINER_OF(node, struct name_timeout_pair, node);

		uint32_t last_seen_ago_s =
			(uint32_t)((k_uptime_get() - n_t_pair->last_seen) / 1000);
		// LOG_INF("Name: %s. id: %d Last seen: %d s ago", n_t_pair->name,
		// n_t_pair->broadcast_id, last_seen_ago_s);
		char last_seen_buf[30];
		(void)last_seen_string_gen(last_seen_buf, last_seen_ago_s);
		sprintf(line_buf, "#0000ff %s# (0x%x)", n_t_pair->name, n_t_pair->broadcast_id);

		if (n_t_pair->update) {
			lv_label_set_text(label_name[num_broadcasters], line_buf);
			n_t_pair->update = false;
		}

		lv_label_set_text(label_time[num_broadcasters], last_seen_buf);

		num_broadcasters++;
	}

	// LOG_WRN("Num active broadcasters %d", num_broadcasters);
	if (num_broadcasters) {
		(void)gpio_pin_configure_dt(&center_led_g, GPIO_OUTPUT_ACTIVE);
	} else {
		(void)gpio_pin_configure_dt(&center_led_g, GPIO_OUTPUT_INACTIVE);
	}

	lv_task_handler();
}

K_WORK_DEFINE(timer_work, timer_worker);

static void broadcast_scan_timer_handler(struct k_timer *dummy)
{
	k_work_submit(&timer_work);
};

/* Shall be called for each found broadcaster */
int name_add(char *name, uint8_t name_size, uint32_t broadcast_id)
{
	uint64_t time_now = k_uptime_get();
	static struct name_timeout_pair *n_t_pair;
	sys_snode_t *node;

	SYS_SLIST_FOR_EACH_NODE(&filled_list, node) {
		n_t_pair = CONTAINER_OF(node, struct name_timeout_pair, node);
		if (strcmp(n_t_pair->name, name) == 0) {
			n_t_pair->last_seen = time_now;
			return 0;
		}
	}
	/* Add a new node */
	node = sys_slist_get(&avail_list);
	if (!node) {
		LOG_WRN("List is full, removing oldest item");
		uint64_t oldest_timestamp = UINT64_MAX;
		sys_snode_t *oldest_node = NULL;

		SYS_SLIST_FOR_EACH_NODE(&filled_list, node) {
			n_t_pair = CONTAINER_OF(node, struct name_timeout_pair, node);
			if (n_t_pair->last_seen < oldest_timestamp) {
				oldest_timestamp = n_t_pair->last_seen;
				oldest_node = node;
			}
		}

		bool removed = sys_slist_find_and_remove(&filled_list, oldest_node);

		__ASSERT(removed, "A node was not removed from the full list");
		node = sys_slist_get(&avail_list);
		if (!node) {
			__ASSERT(false, "There should be an available node");
		}
	}
	n_t_pair = CONTAINER_OF(node, struct name_timeout_pair, node);
	memcpy(n_t_pair->name, name, name_size);
	n_t_pair->last_seen = time_now;
	n_t_pair->update = true;
	n_t_pair->broadcast_id = broadcast_id;
	sys_slist_append(&filled_list, &n_t_pair->node);
	LOG_INF("Added new node with id %d", broadcast_id);

	return 0;
}

K_TIMER_DEFINE(broadcast_scan_timer, broadcast_scan_timer_handler, NULL);

static void scan_restart_worker(struct k_work *work)
{
	int ret;

	/* Delete pending PA sync before restarting scan */
	ret = bt_mgmt_pa_sync_delete(pa_sync);
	if (ret) {
		LOG_WRN("Failed to delete pending PA sync: %d", ret);
	}

	ret = bt_mgmt_scan_start(0, 0, BT_MGMT_SCAN_TYPE_BROADCAST, NULL);
	if (ret) {
		LOG_WRN("Failed to restart scanning for broadcast: %d", ret);
	}
}

K_WORK_DEFINE(scan_restart_work, scan_restart_worker);

static void pa_sync_timeout(struct k_timer *timer)
{
	LOG_WRN("PA sync create timed out, restarting scanning");

	k_work_submit(&scan_restart_work);
}

K_TIMER_DEFINE(pa_sync_timer, pa_sync_timeout, NULL);

static uint16_t interval_to_sync_timeout(uint16_t interval)
{
	uint16_t timeout;

	/* Ensure that the following calculation does not overflow silently */
	__ASSERT(SYNC_RETRY_COUNT < 10, "SYNC_RETRY_COUNT shall be less than 10");

	/* Add retries and convert to unit in 10s of ms */
	timeout = ((uint32_t)interval * SYNC_RETRY_COUNT) / 10;

	/* Enforce restraints */
	timeout = CLAMP(timeout, BT_GAP_PER_ADV_MIN_TIMEOUT, BT_GAP_PER_ADV_MAX_TIMEOUT);

	return timeout;
}

static void periodic_adv_sync(const struct bt_le_scan_recv_info *info, uint32_t broadcast_id)
{
	int ret;
	struct bt_le_per_adv_sync_param param;

	bt_le_scan_cb_unregister(&scan_callback);
	scan_cb_registered = false;

	ret = bt_le_scan_stop();
	if (ret) {
		LOG_WRN("Stop scan failed: %d", ret);
	}

	bt_addr_le_copy(&param.addr, info->addr);
	param.options = 0;
	param.sid = info->sid;
	param.skip = PA_SYNC_SKIP;
	param.timeout = interval_to_sync_timeout(info->interval);

	broadcaster_broadcast_id = broadcast_id;

	/* Set timeout to same value as PA sync timeout in ms */
	k_timer_start(&pa_sync_timer, K_MSEC(param.timeout * 10), K_NO_WAIT);

	ret = bt_le_per_adv_sync_create(&param, &pa_sync);
	if (ret) {
		LOG_ERR("Could not sync to PA: %d", ret);
		return;
	}
}

/**
 * @brief	Check and parse advertising data for broadcast name and ID.
 *
 * @param[in]	data		Advertising data to check and parse.
 * @param[out]	user_data	Will contain pointer to broadcast_source struct to be populated.
 *
 * @retval	true	Continue to parse LTVs.
 * @retval	false	Stop parsing LTVs.
 */
static bool scan_check_broadcast_source(struct bt_data *data, void *user_data)
{
	struct broadcast_source *source = (struct broadcast_source *)user_data;
	struct bt_uuid_16 adv_uuid;

	if (data->type == BT_DATA_BROADCAST_NAME && data->data_len) {
		/* Ensure that broadcast name is at least one character shorter than the value of
		 * BLE_SEARCH_NAME_MAX_LEN
		 */

		if (data->data_len < BLE_SEARCH_NAME_MAX_LEN) {
			memcpy(source->name, data->data, data->data_len);
			source->name[data->data_len] = '\0';
		}

		return true;
	}

	if (data->type != BT_DATA_SVC_DATA16) {
		return true;
	}

	if (data->data_len < BT_UUID_SIZE_16 + BT_AUDIO_BROADCAST_ID_SIZE) {
		return true;
	}

	if (!bt_uuid_create(&adv_uuid.uuid, data->data, BT_UUID_SIZE_16)) {
		return false;
	}

	if (bt_uuid_cmp(&adv_uuid.uuid, BT_UUID_BROADCAST_AUDIO)) {
		return true;
	}

	source->broadcast_id = sys_get_le24(data->data + BT_UUID_SIZE_16);

	return true;
}

/**
 * @brief	Callback handler for scan receive when scanning for broadcasters.
 *
 * @param[in]	info	Advertiser packet and scan response information.
 * @param[in]	ad	Received advertising data.
 */
static void scan_recv_cb(const struct bt_le_scan_recv_info *info, struct net_buf_simple *ad)
{
	struct broadcast_source source = {.broadcast_id = INVALID_BROADCAST_ID};

	/* We are only interested in non-connectable periodic advertisers */
	if ((info->adv_props & BT_GAP_ADV_PROP_CONNECTABLE) || info->interval == 0) {
		return;
	}

	bt_data_parse(ad, scan_check_broadcast_source, (void *)&source);
	name_add(source.name, strlen(source.name), source.broadcast_id);
}

static void pa_synced_cb(struct bt_le_per_adv_sync *sync,
			 struct bt_le_per_adv_sync_synced_info *info)
{
	int ret;
	struct bt_mgmt_msg msg;

	if (sync != pa_sync) {
		LOG_WRN("Synced to unknown source");
		return;
	}

	LOG_DBG("PA synced");

	k_timer_stop(&pa_sync_timer);

	msg.event = BT_MGMT_PA_SYNCED;
	msg.pa_sync = sync;
	msg.broadcast_id = broadcaster_broadcast_id;

	ret = zbus_chan_pub(&bt_mgmt_chan, &msg, K_NO_WAIT);
	ERR_CHK(ret);
}

static void pa_sync_terminated_cb(struct bt_le_per_adv_sync *sync,
				  const struct bt_le_per_adv_sync_term_info *info)
{
	int ret;
	struct bt_mgmt_msg msg;

	LOG_DBG("Periodic advertising sync lost");

	msg.event = BT_MGMT_PA_SYNC_LOST;
	msg.pa_sync = sync;

	ret = zbus_chan_pub(&bt_mgmt_chan, &msg, K_NO_WAIT);
	ERR_CHK(ret);
}

static struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced = pa_synced_cb,
	.term = pa_sync_terminated_cb,
};

static void btn_event_cb(lv_event_t *event)
{
	uint8_t device = event->user_data;
	LOG_WRN("Clicked! %d", device);
}

static int display_init(void)
{
	const struct device *display_dev;
	static lv_style_t style_common;

	lv_style_init(&style_common);
	lv_style_set_text_font(&style_common, &lv_font_montserrat_24);
	lv_obj_t *scr = lv_scr_act();
	lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(display_dev)) {
		LOG_ERR("Device not ready, aborting.");
		return 0;
	}
	lv_disp_t *disp = lv_disp_get_default();
	int width = lv_disp_get_hor_res(disp);

	header_label_1 = lv_label_create(lv_scr_act());
	lv_obj_add_style(header_label_1, &style_common, 0);
	lv_obj_align(header_label_1, LV_ALIGN_TOP_LEFT, 0, 0);
	header_label_2 = lv_label_create(lv_scr_act());
	lv_obj_add_style(header_label_2, &style_common, 0);
	lv_obj_align(header_label_2, LV_ALIGN_TOP_LEFT, 0, V_OFFSET_PIXELS);

	static lv_style_t style_transp;
	lv_style_init(&style_transp);
	lv_style_set_bg_opa(&style_transp, LV_OPA_TRANSP);

	for (int i = 0; i < BROADCASTERS_MAX; i++) {
		label_name[i] = lv_label_create(lv_scr_act());
		lv_obj_add_event_cb(label_name[i], btn_event_cb, LV_EVENT_PRESSED,
				    NULL); /*Assign a callback to the button*/

		lv_obj_align(label_name[i], LV_ALIGN_TOP_LEFT, 0,
			     i * V_OFFSET_PIXELS + V_OFFSET_PIXELS * 2);
		lv_label_set_recolor(label_name[i], true);
		lv_obj_add_style(label_name[i], &style_common, 0);
		lv_obj_set_width(label_name[i], 260);
		lv_label_set_long_mode(label_name[i], LV_LABEL_LONG_SCROLL_CIRCULAR);
		lv_label_set_text(label_name[i], "-");

		label_time[i] = lv_label_create(lv_scr_act());
		lv_label_set_recolor(label_time[i], true);
		lv_obj_align(label_time[i], LV_ALIGN_TOP_RIGHT, 0,
			     i * V_OFFSET_PIXELS + V_OFFSET_PIXELS * 2);
		lv_obj_add_style(label_time[i], &style_common, 0);
		lv_label_set_text(label_time[i], "-");

		btn[i] = lv_btn_create(lv_scr_act());
		lv_obj_set_pos(btn[i], 0, i * V_OFFSET_PIXELS + V_OFFSET_PIXELS * 2);
		lv_obj_set_size(btn[i], width, V_OFFSET_PIXELS);
		lv_obj_add_style(btn[i], &style_transp, LV_STATE_DEFAULT);
		lv_obj_add_event_cb(btn[i], btn_event_cb, LV_EVENT_PRESSED, i);
		// lv_obj_add_style(btn[i], LV_PART_MAIN, LV_STATE_PRESSED, &style_transp);
	}

	lv_task_handler();
	display_blanking_off(display_dev);

	lv_label_set_text(header_label_1, "Scanning");
	lv_label_set_text(header_label_2, LV_SYMBOL_BLUETOOTH "Broadcaster            Last seen");

	lv_task_handler();
	return 0;
}

int bt_mgmt_scan_for_broadcast_start(struct bt_le_scan_param *scan_param, char const *const name)
{
	int ret;

	ret = display_init();
	if (ret) {
		return ret;
	}

	sys_slist_init(&avail_list);
	sys_slist_init(&filled_list);

	for (int i = 0; i < BROADCASTERS_MAX; ++i) {
		sys_slist_append(&avail_list, &n_t_pair[i].node);
	}

	if (!sync_cb_registered) {
		bt_le_per_adv_sync_cb_register(&sync_callbacks);
		sync_cb_registered = true;
	}

	if (!scan_cb_registered) {
		scan_callback.recv = scan_recv_cb;
		bt_le_scan_cb_register(&scan_callback);
		scan_cb_registered = true;
	} else {
		if (name == srch_name) {
			return -EALREADY;
		}
		/* Already scanning, stop current scan to update param in case it has changed */
		ret = bt_le_scan_stop();
		if (ret) {
			LOG_ERR("Failed to stop scan: %d", ret);
			return ret;
		}
	}

	srch_name = name;

	ret = bt_le_scan_start(scan_param, NULL);
	if (ret) {
		return ret;
	}

	k_timer_start(&broadcast_scan_timer, K_MSEC(25), K_MSEC(25));

	return 0;
}
