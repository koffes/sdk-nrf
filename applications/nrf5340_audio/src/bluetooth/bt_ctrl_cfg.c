/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "bt_ctrl_cfg.h"

#include <stdbool.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_ctrl_cfg, 4);

int bt_ctrl_version_get(uint16_t *ctrl_version)
{
	int ret;
	struct net_buf *rsp;

	ret = bt_hci_cmd_send_sync(BT_HCI_OP_READ_LOCAL_VERSION_INFO, NULL, &rsp);
	if (ret) {
		return ret;
	}

	struct bt_hci_rp_read_local_version_info *rp = (void *)rsp->data;

	*ctrl_version = sys_le16_to_cpu(rp->hci_revision);

	net_buf_unref(rsp);

	return 0;
}

int bt_ctrl_cfg(bool watchdog_enable)
{

#if CONFIG_BT_LL_SOFTDEVICE

#elif CONFIG_BT_LL_ACS_NRF53

#else
#error Unsupported controller
#endif
	return 0;
}
