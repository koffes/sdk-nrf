/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _AUDIO_I2S_FILE_BOTTOM_H_
#define _AUDIO_I2S_FILE_BOTTOM_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Open a host file used by the file-backed I2S TX/RX simulation.
 *
 * Runs on the host side of the native simulator, where the real open() flags and file mode
 * are available. RX files are opened read-only. TX files are created/truncated for writing.
 *
 * @param[in] path	Host file path.
 * @param[in] for_write	True to open for writing, false for read-only.
 *
 * @retval	Host file descriptor (>= 0) on success.
 * @retval	Negated native simulator errno (mid) value on failure.
 */
int audio_i2s_file_open_bottom(const char *path, bool for_write);

#ifdef __cplusplus
}
#endif

#endif /* _AUDIO_I2S_FILE_BOTTOM_H_ */
