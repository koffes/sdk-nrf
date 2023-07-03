/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _BT_CONT_CTRL_H_
#define _BT_CONT_CTRL_H_

#include <zephyr/bluetooth/conn.h>

int bt_cont_ctrl_play_pause(struct bt_conn *conn);

int bt_cont_ctrl_conn_disconn(struct bt_conn *conn);

int bt_cont_ctrl_discover(struct bt_conn *conn);

int bt_cont_ctrl_init(void);

#endif
