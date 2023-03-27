/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _ISO_BROADCAST_SINK_H_
#define _ISO_BROADCAST_SINK_H_

#include <stdint.h>

/** @brief Initialize the ISO broadcast sink.
 *
 * @note This code is intended for CI testing and is based on a Zephyr sample.
 * Please see Zephyr ISO samples for a more implementation friendly starting point.
 *
 * @retval 0 The initialization was successful.
 */
int iso_broadcast_sink_init(void);

#endif
