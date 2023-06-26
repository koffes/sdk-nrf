/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "bt_mgmt.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>

#include "ble_hci_vsc.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_mgmt);

#define CONNECTION_PARAMETERS                                                                      \
	BT_LE_CONN_PARAM(CONFIG_BLE_ACL_CONN_INTERVAL, CONFIG_BLE_ACL_CONN_INTERVAL,               \
			 CONFIG_BLE_ACL_SLAVE_LATENCY, CONFIG_BLE_ACL_SUP_TIMEOUT)

static uint8_t bonded_num;

static void bond_check(const struct bt_bond_info *info, void *user_data)
{
	char addr_buf[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(&info->addr, addr_buf, BT_ADDR_LE_STR_LEN);

	LOG_DBG("Stored bonding found: %s", addr_buf);
	bonded_num++;
}

static void bond_connect(const struct bt_bond_info *info, void *user_data)
{
	int ret;
	const bt_addr_le_t *adv_addr = user_data;
	struct bt_conn *conn;

	if (!bt_addr_le_cmp(&info->addr, adv_addr)) {
		LOG_DBG("Found bonded device");

		ret = bt_le_scan_stop();
		if (ret) {
			LOG_WRN("Stop scan failed: %d", ret);
		}

		ret = bt_conn_le_create(adv_addr, BT_CONN_LE_CREATE_CONN, CONNECTION_PARAMETERS,
					&conn);
		if (ret) {
			LOG_WRN("Create ACL connection failed: %d", ret);
			bt_mgmt_scan_start(0, 0);
		}
	}
}

static int device_found(uint8_t type, const uint8_t *data, uint8_t data_len,
			const bt_addr_le_t *addr)
{
	int ret;
	struct bt_conn *conn;

	if ((data_len == DEVICE_NAME_PEER_LEN) &&
	    (strncmp(DEVICE_NAME_PEER, data, DEVICE_NAME_PEER_LEN) == 0)) {
		LOG_DBG("Device found");

		ret = bt_le_scan_stop();
		if (ret) {
			LOG_WRN("Stop scan failed: %d", ret);
		}

		ret = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, CONNECTION_PARAMETERS, &conn);
		if (ret) {
			LOG_ERR("Could not init connection");
			bt_mgmt_scan_start(0, 0);
			return ret;
		}

		return 0;
	}

	return -ENOENT;
}

/**
 * @brief  Parse BLE advertisement package
 */
static void ad_parse(struct net_buf_simple *p_ad, const bt_addr_le_t *addr)
{
	while (p_ad->len > 1) {
		uint8_t len = net_buf_simple_pull_u8(p_ad);
		uint8_t type;

		/* Check for early termination */
		if (len == 0) {
			return;
		}

		if (len > p_ad->len) {
			LOG_WRN("AD malformed");
			return;
		}

		type = net_buf_simple_pull_u8(p_ad);

		if (device_found(type, p_ad->data, len - 1, addr) == 0) {
			return;
		}

		(void)net_buf_simple_pull(p_ad, len - 1);
	}
}

static void on_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			    struct net_buf_simple *p_ad)
{
	switch (type) {
	case BT_GAP_ADV_TYPE_ADV_DIRECT_IND:
		/* Direct advertising has no payload, so no need to parse */
		if (bonded_num) {
			bt_foreach_bond(BT_ID_DEFAULT, bond_connect, (void *)addr);
		}
		break;
	case BT_GAP_ADV_TYPE_ADV_IND:
		/* Fall through */
	case BT_GAP_ADV_TYPE_EXT_ADV:
		/* Fall through */
	case BT_GAP_ADV_TYPE_SCAN_RSP:
		/* Note: May lead to connection creation */
		if (bonded_num < CONFIG_BT_MAX_PAIRED) {
			ad_parse(p_ad, addr);
		}
		break;
	default:
		break;
	}
}

void bt_mgmt_scan_start(int scan_intvl, int scan_win)
{
	int ret;

	static int scan_interval = CONFIG_BT_BACKGROUND_SCAN_INTERVAL;
	static int scan_window = CONFIG_BT_BACKGROUND_SCAN_WINDOW;

	if (scan_intvl != 0) {
		scan_interval = scan_intvl;
	}

	if (scan_win != 0) {
		scan_window = scan_win;
	}

	struct bt_le_scan_param *scan_param =
		BT_LE_SCAN_PARAM(NRF5340_AUDIO_GATEWAY_SCAN_TYPE, BT_LE_SCAN_OPT_FILTER_DUPLICATE,
				 scan_interval, scan_window);

	/* Reset number of bonds found */
	bonded_num = 0;

	bt_foreach_bond(BT_ID_DEFAULT, bond_check, NULL);

	if (bonded_num >= CONFIG_BT_MAX_PAIRED) {
		LOG_INF("All bonded slots filled, will not accept new devices");
	}

	ret = bt_le_scan_start(scan_param, on_device_found);
	if (ret && ret != -EALREADY) {
		LOG_ERR("Scanning failed to start: %d", ret);
		return;
	}

	LOG_INF("Scanning successfully started");
}
