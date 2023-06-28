/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/* be gone bad file */

#include "ble_core.h"

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/gpio.h>
#include <errno.h>

#include "macros_common.h"
#include "ble_hci_vsc.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ble, CONFIG_BLE_LOG_LEVEL);

#define NET_CORE_RESPONSE_TIMEOUT_MS 500

/* Note that HCI_CMD_TIMEOUT is currently set to 10 seconds in Zephyr */
#define NET_CORE_WATCHDOG_TIME_MS 1000

static ble_core_ready_t m_ready_callback;

static void net_core_timeout_handler(struct k_timer *timer_id);
static void net_core_watchdog_handler(struct k_timer *timer_id);

static struct k_work net_core_ctrl_version_get_work;

K_TIMER_DEFINE(net_core_timeout_alarm_timer, net_core_timeout_handler, NULL);
K_TIMER_DEFINE(net_core_watchdog_timer, net_core_watchdog_handler, NULL);

/* If NET core out of response for a time defined in NET_CORE_RESPONSE_TIMEOUT
 * show error message for indicating user.
 */
static void net_core_timeout_handler(struct k_timer *timer_id)
{
	ERR_CHK_MSG(-EIO, "No response from NET core, check if NET core is programmed");
}

/* Callback called by the Bluetooth stack in Zephyr when Bluetooth is ready */
static void on_bt_ready(int err)
{

	m_ready_callback();
}

static void net_core_watchdog_handler(struct k_timer *timer_id)
{
	k_work_submit(&net_core_ctrl_version_get_work);
}

int ble_core_le_pwr_ctrl_disable(void)
{
	return ble_hci_vsc_op_flag_set(BLE_HCI_VSC_OP_DIS_POWER_MONITOR, 1);
}

int ble_core_init(
	ble_core_ready_t ready_callback) /* THis is to be moved to bt_ctrl_cfg (most of it) */
{
	int ret;

	return 0;
}
