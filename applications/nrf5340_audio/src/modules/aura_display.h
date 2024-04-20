#ifndef _AURA_DISPLAY_H_
#define _AURA_DISPLAY_H_

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/sys/slist.h>
#include <zephyr/drivers/display.h>
#include <lvgl.h>

int aura_display_submit(const struct bt_le_scan_recv_info *info, char *name, uint8_t name_size,
		     uint32_t broadcast_id);

int aura_display_init(void);


#endif /* _AURA_DISPLAY_H_ */
