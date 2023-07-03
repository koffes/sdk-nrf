/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "bt_cont_ctrl.h"

#include <zephyr/zbus/zbus.h>

#include "ble_audio_services.h"
#include "nrf5340_audio_common.h"
#include "macros_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_cont_ctrl, CONFIG_AUDIO_SERVICES_LOG_LEVEL);

ZBUS_CHAN_DEFINE(cont_media_chan, struct media_control_play_pause_msg, NULL, NULL,
		 ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(0));

static void media_control_cb(bool play)
{
	int ret;
	struct media_control_play_pause_msg msg;

	if (play) {
		msg.event = MEDIA_PLAY;
	} else {
		msg.event = MEDIA_PAUSE;
	}

	ret = zbus_chan_pub(&cont_media_chan, &msg, K_NO_WAIT);
	ERR_CHK_MSG(ret, "zbus publication failed");
}

int bt_cont_ctrl_play_pause(struct bt_conn *conn)
{
	int ret;

	if (IS_ENABLED(CONFIG_BT_MCC)) {
		ret = ble_mcs_play_pause(NULL);
		if (ret) {
			LOG_WRN("Failed to change streaming state");
			return ret;
		}
	}
	return 0;
}

int bt_cont_ctrl_conn_disconn(struct bt_conn *conn)
{
	int ret;

	if (IS_ENABLED(CONFIG_BT_MCC)) {
		ret = ble_mcp_conn_disconnected(conn);
		if (ret) {
			LOG_ERR("ble_msc_conn_disconnected failed with %d", ret);
		}
	}

	if (IS_ENABLED(CONFIG_BT_MCS)) {
		ret = ble_mcp_conn_disconnected(NULL);
		if (ret) {
			LOG_ERR("ble_msc_conn_disconnected failed with %d", ret);
		}
	}

	return 0;
}

int bt_cont_ctrl_discover(struct bt_conn *conn)
{
	int ret;

	if (IS_ENABLED(CONFIG_BT_MCC)) {
		ret = ble_mcs_discover(conn);
		if (ret) {
			return ret;
		}
	}

	return 0;
}

int bt_cont_ctrl_init(void)
{
	int ret;

	if (IS_ENABLED(CONFIG_BT_MCS)) {
		ret = ble_mcs_server_init(media_control_cb);
		if (ret) {
			LOG_ERR("MCS server init failed");
			return ret;
		}
	}

	if (IS_ENABLED(CONFIG_BT_MCC)) {
		ret = ble_mcs_client_init();
		if (ret) {
			LOG_ERR("MCS client init failed");
			return ret;
		}
	}

	return 0;
}
