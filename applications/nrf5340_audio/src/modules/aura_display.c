#include "aura_display.h"
#include <zephyr/zbus/zbus.h>
#include <zephyr/kernel.h>

#include "macros_common.h"
#include "broadcast_sink.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(aura_display, 4);

ZBUS_CHAN_DEFINE(display_action_chan, struct brcast_src_info, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0));

ZBUS_CHAN_DECLARE(display_action_chan);

#define BROADCASTERS_MAX 6
#define TIMEOUT_S	 5
#define V_OFFSET_PIXELS	 30
#define VIEW_FOCUS_LINES 6

static uint8_t num_broadcasters;

static lv_obj_t *header_label_1;
static lv_obj_t *label_name[BROADCASTERS_MAX];
static lv_obj_t *label_time[BROADCASTERS_MAX];
static lv_obj_t *label_focus[VIEW_FOCUS_LINES];
static lv_obj_t *btn[BROADCASTERS_MAX];
static lv_obj_t *btn_ret_to_overview;

static lv_obj_t *screen_overview;
static lv_obj_t *screen_focus;

static struct brcast_src_info bc_src_info[BROADCASTERS_MAX];
static sys_slist_t avail_list;
static sys_slist_t filled_list;

static lv_style_t style_btn_trans;
static lv_style_t style_btn_default;

static uint8_t brcaster_in_focus;

static struct audio_codec_info *codec_info_disp;
int num_codecs;

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

static void btn_event_to_overview(lv_event_t *event)
{
	lv_scr_load(screen_overview);
	num_codecs = 0;
}

static void btn_event_to_focus(lv_event_t *event)
{
	int ret;
	uint32_t device = (uint32_t)event->user_data;

	brcaster_in_focus = device;
	lv_scr_load(screen_focus);

	LOG_WRN("Syncing");
	struct brcast_src_info brcast_to_send = bc_src_info[brcaster_in_focus];

	ret = zbus_chan_pub(&display_action_chan, &brcast_to_send, K_NO_WAIT);
	ERR_CHK(ret);
}

static void scan_buf_get(char *buf, uint32_t uptime_s)
{
	uint32_t remainder = uptime_s % 4;

	if (remainder == 0) {
		sprintf(buf, "Scan   ");
	} else if (remainder == 1) {
		sprintf(buf, "Scan.  ");
	} else if (remainder == 2) {
		sprintf(buf, "Scan.. ");
	} else {
		sprintf(buf, "Scan...");
	}
}

static void page_overview_draw(void)
{
	struct brcast_src_info *bc_src_info_loc;
	sys_snode_t *node;
	uint32_t uptime_s = (uint32_t)(k_uptime_get() / 1000);
	char line_buf[300] = {'\0'};
	char scan_buf[8];
	scan_buf_get(scan_buf, uptime_s);

	num_broadcasters = 0;
	sprintf(line_buf, LV_SYMBOL_BLUETOOTH "%s %d", scan_buf, uptime_s);
	lv_label_set_text(header_label_1, line_buf);

	SYS_SLIST_FOR_EACH_NODE(&filled_list, node) {
		bc_src_info_loc = CONTAINER_OF(node, struct brcast_src_info, node);

		uint32_t last_seen_ago_s =
			(uint32_t)((k_uptime_get() - bc_src_info_loc->last_seen) / 1000);
		char last_seen_buf[30];
		(void)last_seen_string_gen(last_seen_buf, last_seen_ago_s);
		sprintf(line_buf, "#0000ff %s# (0x%x)", bc_src_info_loc->name,
			bc_src_info_loc->broadcast_id);

		if (bc_src_info_loc->update) {
			lv_label_set_text(label_name[num_broadcasters], line_buf);
			bc_src_info_loc->update = false;
		}

		lv_label_set_text(label_time[num_broadcasters], last_seen_buf);

		num_broadcasters++;
	}
}

static void page_focus_draw()
{
	char line_buf[300] = {'\0'};
	uint32_t last_seen_ago_s =
		(uint32_t)((k_uptime_get() - bc_src_info[brcaster_in_focus].last_seen) / 1000);
	char last_seen_buf[30];
	(void)last_seen_string_gen(last_seen_buf, last_seen_ago_s);
	sprintf(line_buf, "#0000ff %s# (0x%x)", bc_src_info[brcaster_in_focus].name,
		bc_src_info[brcaster_in_focus].broadcast_id);
	lv_label_set_text(label_focus[0], line_buf);

	sprintf(line_buf, "rssi: %d ", bc_src_info[brcaster_in_focus].info.rssi);
	lv_label_set_text(label_focus[1], line_buf);

	if (num_codecs == 0) {
		sprintf(line_buf, "trying to sync...");
		lv_label_set_text(label_focus[2], line_buf);
		lv_label_set_text(label_focus[3], "-");
		lv_label_set_text(label_focus[4], "-");
		lv_label_set_text(label_focus[5], "-");
	} else if (num_codecs < 0) {
		sprintf(line_buf, "Sync failed");
		lv_label_set_text(label_focus[2], line_buf);
		lv_label_set_text(label_focus[3], "-");
		lv_label_set_text(label_focus[4], "-");
		lv_label_set_text(label_focus[5], "-");

	} else {
		for (int i = 0; i < num_codecs; i++) {
			sprintf(line_buf, "subgr: %d bis: %d freq: %d, bps: %d",
				codec_info_disp[i].subgroup, codec_info_disp[i].bis,
				codec_info_disp[i].frequency, codec_info_disp[i].bitrate);
			lv_label_set_text(label_focus[i + 2], line_buf);
		}
	}
}

static void timer_worker(struct k_work *work)
{
	if (lv_scr_act() == screen_overview) {
		page_overview_draw();
	} else if (lv_scr_act() == screen_focus) {
		page_focus_draw();
	} else {
		LOG_ERR("Unknown screen active.");
	}

	lv_task_handler();
}

K_WORK_DEFINE(timer_work, timer_worker);

static void broadcast_scan_timer_handler(struct k_timer *dummy)
{
	k_work_submit(&timer_work);
};

int aura_display_submit_codec_info(struct audio_codec_info codec_info[], int num)
{
	if (num == 0) {
		LOG_WRN("no codecs submitted");
		return 0;
	}

	if (lv_scr_act() != screen_focus) {

		return 0;
	}
	codec_info_disp = codec_info;
	num_codecs = num;

	return 0;
}

/* Shall be called for each found broadcaster */
int aura_display_submit_scan(const struct bt_le_scan_recv_info *info, char *name, uint8_t name_size,
			     uint32_t broadcast_id)
{
	uint64_t time_now = k_uptime_get();
	struct brcast_src_info *bc_src_info;
	sys_snode_t *node;

	SYS_SLIST_FOR_EACH_NODE(&filled_list, node) {
		bc_src_info = CONTAINER_OF(node, struct brcast_src_info, node);
		if (strcmp(bc_src_info->name, name) == 0) {
			/* This device already exists */
			bc_src_info->last_seen = time_now;
			memcpy(&bc_src_info->info, info, sizeof(struct bt_le_scan_recv_info));
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
			bc_src_info = CONTAINER_OF(node, struct brcast_src_info, node);
			if (bc_src_info->last_seen < oldest_timestamp) {
				oldest_timestamp = bc_src_info->last_seen;
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
	bc_src_info = CONTAINER_OF(node, struct brcast_src_info, node);
	memcpy(bc_src_info->name, name, name_size);
	bc_src_info->last_seen = time_now;
	bc_src_info->update = true;
	bc_src_info->broadcast_id = broadcast_id;
	memcpy(&bc_src_info->info, info, sizeof(struct bt_le_scan_recv_info));
	sys_slist_append(&filled_list, &bc_src_info->node);
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

	screen_overview = lv_obj_create(NULL);
	screen_focus = lv_obj_create(NULL);
	lv_scr_load(screen_overview);

	for (int i = 0; i < BROADCASTERS_MAX; ++i) {
		sys_slist_append(&avail_list, &bc_src_info[i].node);
	}

	lv_style_init(&style_common);
	lv_style_set_text_font(&style_common, &lv_font_montserrat_24);
	lv_obj_clear_flag(screen_overview, LV_OBJ_FLAG_SCROLLABLE);

	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(display_dev)) {
		LOG_ERR("Device not ready, aborting.");
		return 0;
	}
	lv_disp_t *disp = lv_disp_get_default();
	int width = lv_disp_get_hor_res(disp);
	int height = lv_disp_get_ver_res(disp);

	/* Overview page */
	header_label_1 = lv_label_create(screen_overview);
	lv_obj_add_style(header_label_1, &style_common, 0);
	lv_obj_align(header_label_1, LV_ALIGN_TOP_LEFT, 0, 0);

	lv_style_init(&style_btn_trans);
	lv_style_init(&style_btn_default);
	lv_style_set_bg_opa(&style_btn_trans, LV_OPA_TRANSP);
	lv_style_set_bg_color(&style_btn_default, lv_color_hex(0x123456));
	lv_style_set_bg_opa(&style_btn_default, LV_OPA_50);

	for (int i = 0; i < BROADCASTERS_MAX; i++) {
		label_name[i] = lv_label_create(screen_overview);
		lv_obj_add_event_cb(label_name[i], btn_event_to_focus, LV_EVENT_PRESSED,
				    NULL); /*Assign a callback to the button*/

		lv_obj_align(label_name[i], LV_ALIGN_TOP_LEFT, 0,
			     i * V_OFFSET_PIXELS + V_OFFSET_PIXELS);
		lv_label_set_recolor(label_name[i], true);
		lv_obj_add_style(label_name[i], &style_common, 0);
		lv_obj_set_width(label_name[i], 260);
		lv_label_set_long_mode(label_name[i], LV_LABEL_LONG_SCROLL_CIRCULAR);
		lv_label_set_text(label_name[i], "-");

		label_time[i] = lv_label_create(screen_overview);
		lv_label_set_recolor(label_time[i], true);
		lv_obj_align(label_time[i], LV_ALIGN_TOP_RIGHT, 0,
			     i * V_OFFSET_PIXELS + V_OFFSET_PIXELS);
		lv_obj_add_style(label_time[i], &style_common, 0);
		lv_label_set_text(label_time[i], "-");

		btn[i] = lv_btn_create(screen_overview);
		lv_obj_set_pos(btn[i], 0, i * V_OFFSET_PIXELS + V_OFFSET_PIXELS);
		lv_obj_set_size(btn[i], width, V_OFFSET_PIXELS);
		lv_obj_add_style(btn[i], &style_btn_trans, LV_STATE_DEFAULT);
		lv_obj_add_event_cb(btn[i], btn_event_to_focus, LV_EVENT_PRESSED, (void *)i);
		// lv_obj_add_style(btn[i], LV_PART_MAIN, LV_STATE_PRESSED, &style_btn_trans);
	}

	/* Overview page */
	for (int i = 0; i < VIEW_FOCUS_LINES; i++) {
		label_focus[i] = lv_label_create(screen_focus);
		lv_obj_align(label_focus[i], LV_ALIGN_TOP_LEFT, 0, i * V_OFFSET_PIXELS);
		lv_label_set_recolor(label_focus[i], true);
		lv_obj_add_style(label_focus[i], &style_common, 0);
		lv_obj_set_width(label_focus[i], 260);
		lv_label_set_long_mode(label_focus[i], LV_LABEL_LONG_SCROLL_CIRCULAR);
		lv_label_set_text(label_focus[i], "-");
	}

	btn_ret_to_overview = lv_btn_create(screen_focus);
	lv_obj_set_pos(btn_ret_to_overview, 0, height - 50);
	lv_obj_set_size(btn_ret_to_overview, 50, 50);
	lv_obj_add_style(btn_ret_to_overview, &style_btn_default, LV_STATE_DEFAULT);
	lv_obj_add_event_cb(btn_ret_to_overview, btn_event_to_overview, LV_EVENT_PRESSED, NULL);
	lv_obj_t *label_btn_ret_to_overview = lv_label_create(btn_ret_to_overview);
	lv_label_set_text(label_btn_ret_to_overview, LV_SYMBOL_LEFT);
	lv_obj_set_style_text_align(label_btn_ret_to_overview, LV_TEXT_ALIGN_CENTER, 0);

	lv_task_handler();
	display_blanking_off(display_dev);

	lv_task_handler();

	k_timer_start(&broadcast_scan_timer, K_MSEC(25), K_MSEC(25));
	return 0;
}