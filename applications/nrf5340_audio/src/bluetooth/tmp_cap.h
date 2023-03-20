/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _TMP_CAP_H_
#define _TMP_CAP_H_

#include <zephyr/bluetooth/audio/audio.h>
//#include <zephyr/bluetooth/bluetooth.h>
//#include <zephyr/bluetooth/hci.h>
//#include <zephyr/bluetooth/audio/bap.h>
//#include <zephyr/bluetooth/audio/bap_lc3_preset.h>


struct tmp_cap_storage {
	struct bt_conn *conn;
	/* Must be the same size as sink_codec_cap and source_codec_cap */
	struct bt_codec codec[5];
};


int tmp_cap_push(struct tmp_cap_storage temp_cap[], struct bt_conn* conn, uint8_t num_caps, struct bt_codec const *const codec);

int tmp_cap_peek(struct tmp_cap_storage temp_cap[], struct bt_conn const* conn, uint8_t num_caps, struct bt_codec *codec);

int tmp_cap_erase(struct tmp_cap_storage temp_cap[], struct bt_conn const * conn);

#endif