#include "aura_display.h"
#include <zephyr/zephyr.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(aura_display, 4);

#define NAME_SIZE_MAX	 250
#define BROADCASTERS_MAX 3
#define TIMEOUT_S	 5
#define V_OFFSET_PIXELS	 30

static uint8_t num_broadcasters;

static lv_obj_t *header_label_1;
static lv_obj_t *label_name[BROADCASTERS_MAX];
static lv_obj_t *label_time[BROADCASTERS_MAX];
static lv_obj_t *btn[BROADCASTERS_MAX];

struct name_timeout_pair {
	sys_snode_t node;
	char name[NAME_SIZE_MAX];
	uint64_t last_seen;
	bool update;
	uint32_t broadcast_id;
	struct bt_le_scan_recv_info info;
};

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

static void btn_event_cb(lv_event_t *event)
{
	uint8_t device = event->user_data;
	LOG_WRN("Clicked! %d", device);
	// periodic_adv_sync(&n_t_pair[device].info, n_t_pair[device].broadcast_id);
}

static struct name_timeout_pair n_t_pair[BROADCASTERS_MAX];
static sys_slist_t avail_list;
static sys_slist_t filled_list;
// static K_MUTEX_DEFINE(module_list_lock);

static void timer_worker(struct k_work *work)
{
	static struct name_timeout_pair *n_t_pair;
	sys_snode_t *node;
	char line_buf[300] = {'\0'};

	num_broadcasters = 0;
	sprintf(line_buf, LV_SYMBOL_BLUETOOTH "Scan.. %d", (uint32_t)(k_uptime_get() / 1000));
	lv_label_set_text(header_label_1, line_buf);

	SYS_SLIST_FOR_EACH_NODE(&filled_list, node) {
		n_t_pair = CONTAINER_OF(node, struct name_timeout_pair, node);

		uint32_t last_seen_ago_s =
			(uint32_t)((k_uptime_get() - n_t_pair->last_seen) / 1000);
		// LOG_INF("Name: %s. id: %d Last seen: %d s ago", n_t_pair->name,
		// n_t_pair->broadcast_id, last_seen_ago_s);
		char last_seen_buf[30];
		(void)last_seen_string_gen(last_seen_buf, last_seen_ago_s);
		sprintf(line_buf, "#0000ff %s# (0x%x) rssi: %d", n_t_pair->name,
			n_t_pair->broadcast_id, n_t_pair->info.rssi);

		if (n_t_pair->update) {
			lv_label_set_text(label_name[num_broadcasters], line_buf);
			n_t_pair->update = false;
		}

		lv_label_set_text(label_time[num_broadcasters], last_seen_buf);

		num_broadcasters++;
	}

	lv_task_handler();
}

K_WORK_DEFINE(timer_work, timer_worker);

static void broadcast_scan_timer_handler(struct k_timer *dummy)
{
	k_work_submit(&timer_work);
};

/* Shall be called for each found broadcaster */
int aura_display_submit(const struct bt_le_scan_recv_info *info, char *name, uint8_t name_size,
			uint32_t broadcast_id)
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
	memcpy(&n_t_pair->info, info, sizeof(struct bt_le_scan_recv_info));
	sys_slist_append(&filled_list, &n_t_pair->node);
	LOG_INF("Added new node with id %d", broadcast_id);

	return 0;
}

K_TIMER_DEFINE(broadcast_scan_timer, broadcast_scan_timer_handler, NULL);

int aura_display_init(void)
{
	const struct device *display_dev;
	static lv_style_t style_common;

	sys_slist_init(&avail_list);
	sys_slist_init(&filled_list);

	for (int i = 0; i < BROADCASTERS_MAX; ++i) {
		sys_slist_append(&avail_list, &n_t_pair[i].node);
	}

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

	lv_task_handler();

	k_timer_start(&broadcast_scan_timer, K_MSEC(25), K_MSEC(25));
	return 0;
}