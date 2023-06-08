/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _BT_MGMT_H_
#define _BT_MGMT_H_

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>

#define LE_AUDIO_EXTENDED_ADV_NAME                                                                 \
	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_USE_NAME,                            \
			CONFIG_BLE_ACL_EXT_ADV_INT_MIN, CONFIG_BLE_ACL_EXT_ADV_INT_MAX, NULL)

#define LE_AUDIO_EXTENDED_ADV_CONN_NAME                                                            \
	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_CONNECTABLE |                        \
				BT_LE_ADV_OPT_USE_NAME,                                            \
			CONFIG_BLE_ACL_EXT_ADV_INT_MIN, CONFIG_BLE_ACL_EXT_ADV_INT_MAX, NULL)

#define LE_AUDIO_PERIODIC_ADV                                                                      \
	BT_LE_PER_ADV_PARAM(CONFIG_BLE_ACL_PER_ADV_INT_MIN, CONFIG_BLE_ACL_PER_ADV_INT_MAX,        \
			    BT_LE_PER_ADV_OPT_NONE)

#define BT_LE_ADV_FAST_CONN                                                                        \
	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, BT_GAP_ADV_FAST_INT_MIN_1,                      \
			BT_GAP_ADV_FAST_INT_MAX_1, NULL)

#if (CONFIG_SCAN_MODE_ACTIVE)
#define NRF5340_AUDIO_GATEWAY_SCAN_TYPE	  BT_LE_SCAN_TYPE_ACTIVE
#define NRF5340_AUDIO_GATEWAY_SCAN_PARAMS BT_LE_SCAN_ACTIVE
#elif (CONFIG_SCAN_MODE_PASSIVE)
#define NRF5340_AUDIO_GATEWAY_SCAN_TYPE	  BT_LE_SCAN_TYPE_PASSIVE
#define NRF5340_AUDIO_GATEWAY_SCAN_PARAMS BT_LE_SCAN_PASSIVE
#else
#error "Please select either CONFIG_SCAN_MODE_ACTIVE or CONFIG_SCAN_MODE_PASSIVE"
#endif

#define DEVICE_NAME_PEER     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_PEER_LEN (sizeof(DEVICE_NAME_PEER) - 1)

/**
 * @brief	Disconnect from a remote device or cancel pending connection
 *
 * @param	conn	Connection to disconnect
 * @param	reason	Reason code for the disconnection
 */
void bt_mgmt_conn_disconnect(struct bt_conn *conn, uint8_t reason);

/**
 * @brief	Start scanning for advertisements
 *
 * @param	scan_interval	Scan interval in units of 0.625ms
 * @param	scan_window	Scan window in units of 0.625ms
 *
 * @note	If 0 is given as the input the previous/default value should be used
 */
void bt_mgmt_scan_start(int scan_interval, int scan_window);

/**
 * @brief	Restart advertising
 *
 * @note	Will use the same advertisin parameters as
 *		when bt_mgmt_adv_start was called
 *
 * @return	0 if success, error otherwise
 */
int bt_mgmt_adv_restart(void);

/**
 * @brief	Create and start advertising for ACL connection
 *
 * @param[in]	ext_adv		The data to be put in the extended advertisement
 * @param[in]	ext_adv_size	Size of ext_adv
 * @param[in]	per_adv		The data for the periodic advertisement, can be NULL
 * @param[in]	per_adv_size	Size of per_adv
 * @param[in]	connectable	Specify if advertisement should be connectable or not
 *
 * @return	0 if success, error otherwise
 */
int bt_mgmt_adv_start(const struct bt_data *ext_adv, size_t ext_adv_size,
		      const struct bt_data *per_adv, size_t per_adv_size, bool connectable);

/**
 * @brief	Initialize the advertising part of the Bluetooth management module
 *
 * @note	Will be called from bt_mgmt_init()
 */
void bt_mgmt_adv_init(void);

/**
 * @brief	Initialize the Bluetooth management module
 *
 * @return	0 if success, error otherwise
 */
int bt_mgmt_init(void);

#endif /* _BT_MGMT_H_ */
