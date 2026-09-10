/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/* Compiled into the native_simulator host executable: real host headers are safe here. */

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

#include <nsi_errno.h>
#include <nsi_tracing.h>

#include "audio_i2s_file_bottom.h"

int audio_i2s_file_open_bottom(const char *path, bool for_write)
{
	int fd;

	if (for_write) {
		fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0600);
	} else {
		fd = open(path, O_RDONLY);
	}

	if (fd < 0) {
		nsi_print_warning("audio_i2s_file: %s could not be opened (%s)\n", path,
				  strerror(errno));
		return -nsi_errno_to_mid(errno);
	}

	return fd;
}
