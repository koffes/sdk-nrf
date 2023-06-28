/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _BT_CTRL_CFG_H_
#define _BT_CTRL_CFG_H_

#include <stdbool.h>

int bt_ctrl_cfg_version_get(void);

int bt_ctrl_cfg(bool watchdog_enable);

#endif
