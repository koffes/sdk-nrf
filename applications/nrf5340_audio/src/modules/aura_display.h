#ifndef _AURA_DISPLAY_H_
#define _AURA_DISPLAY_H_

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/sys/slist.h>
#include <zephyr/drivers/display.h>
#include <lvgl.h>

#include "broadcast_sink.h"

#define NAME_SIZE_MAX 250

struct brcast_src_info {
	sys_snode_t node;
	char name[NAME_SIZE_MAX];
	uint64_t last_seen;
	bool update;
	uint32_t broadcast_id;
	struct bt_le_scan_recv_info info;
};

int aura_display_submit_codec_info(struct audio_codec_info codec_info[], int num);

int aura_display_submit_scan(const struct bt_le_scan_recv_info *info, char *name, uint8_t name_size,
			uint32_t broadcast_id);

int aura_display_init(void);

#endif /* _AURA_DISPLAY_H_ */
