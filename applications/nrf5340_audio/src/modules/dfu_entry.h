/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _DFU_ENTRY_H_
#define _DFU_ENTRY_H_

/**
 * @brief	Pointer to function that initializes the BLE subsystem
 */
typedef int (*ble_init_func)(void *ready_callback);

/**
 * @brief Check button pressed status to advertise SMP_SVR service only
 *
 */
void dfu_entry_check(void);

#endif /* _DFU_ENTRY_H_ */
