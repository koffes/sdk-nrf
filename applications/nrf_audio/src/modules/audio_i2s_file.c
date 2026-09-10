/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/* File-based stand-in for the real I2S/TDM peripheral driver, used on BabbleSim and other
 * POSIX arch targets where no such hardware is modeled. RX audio is read from a host file
 * (silence if missing/exhausted) and TX audio is appended to a host file, at the same block
 * cadence and double-buffering contract as the real audio_i2s.c.
 */

#include "audio_i2s.h"

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <nsi_host_trampolines.h>
#include <nsi_hw_scheduler.h>

#include "audio_i2s_file_bottom.h"

LOG_MODULE_REGISTER(audio_i2s_file, CONFIG_MODULE_AUDIO_I2S_FILE_LOG_LEVEL);

enum audio_i2s_state {
	AUDIO_I2S_STATE_UNINIT,
	AUDIO_I2S_STATE_IDLE,
	AUDIO_I2S_STATE_STARTED,
};

static enum audio_i2s_state state = AUDIO_I2S_STATE_UNINIT;

static int rx_fd = -1;
static int tx_fd = -1;

static const uint8_t *active_tx_buf;
static uint32_t *active_rx_buf;
static const uint8_t *pending_tx_buf;
static uint32_t *pending_rx_buf;
static bool pending_valid;

static i2s_blk_comp_callback_t i2s_blk_comp_callback;

static struct k_timer blk_timer;
static struct k_work blk_work;

static void rx_block_fill(uint32_t *rx_buf)
{
	uint8_t *dst = (uint8_t *)rx_buf;
	size_t filled = 0;

	if (rx_fd >= 0) {
		while (filled < BLOCK_SIZE_BYTES) {
			long n = nsi_host_read(rx_fd, &dst[filled], BLOCK_SIZE_BYTES - filled);

			if (n <= 0) {
				/* EOF or error: pad the remainder with silence */
				break;
			}

			filled += (size_t)n;
		}
	}

	if (filled < BLOCK_SIZE_BYTES) {
		memset(&dst[filled], 0, BLOCK_SIZE_BYTES - filled);
	}
}

static void tx_block_write(const uint8_t *tx_buf)
{
	if (tx_fd < 0) {
		return;
	}

	if (nsi_host_write(tx_fd, tx_buf, BLOCK_SIZE_BYTES) < 0) {
		LOG_WRN_ONCE("Failed to write simulated I2S TX data to file");
	}
}

static void blk_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	const uint8_t *done_tx_buf = active_tx_buf;
	uint32_t *done_rx_buf = active_rx_buf;

	if (pending_valid) {
		active_tx_buf = pending_tx_buf;
		active_rx_buf = pending_rx_buf;
		pending_valid = false;
	}

	if (done_tx_buf != NULL) {
		tx_block_write(done_tx_buf);
	}

	if (done_rx_buf != NULL) {
		rx_block_fill(done_rx_buf);
	}

	if (i2s_blk_comp_callback != NULL) {
		i2s_blk_comp_callback((uint32_t)nsi_hws_get_time(), done_rx_buf,
				      (uint32_t const *)done_tx_buf);
	}
}

static void blk_timer_expiry(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	k_work_submit(&blk_work);
}

void __noinline audio_i2s_set_next_buf(const uint8_t *tx_buf, uint32_t *rx_buf)
{
	__ASSERT_NO_MSG(state == AUDIO_I2S_STATE_STARTED);
	if (IS_ENABLED(CONFIG_STREAM_BIDIRECTIONAL) || (CONFIG_AUDIO_DEV == GATEWAY)) {
		__ASSERT_NO_MSG(rx_buf != NULL);
	}

	if (IS_ENABLED(CONFIG_STREAM_BIDIRECTIONAL) || (CONFIG_AUDIO_DEV == HEADSET)) {
		__ASSERT_NO_MSG(tx_buf != NULL);
	}

	pending_tx_buf = tx_buf;
	pending_rx_buf = rx_buf;
	pending_valid = true;
}

void __noinline audio_i2s_start(const uint8_t *tx_buf, uint32_t *rx_buf)
{
	__ASSERT_NO_MSG(state == AUDIO_I2S_STATE_IDLE);
	if (IS_ENABLED(CONFIG_STREAM_BIDIRECTIONAL) || (CONFIG_AUDIO_DEV == GATEWAY)) {
		__ASSERT_NO_MSG(rx_buf != NULL);
	}

	if (IS_ENABLED(CONFIG_STREAM_BIDIRECTIONAL) || (CONFIG_AUDIO_DEV == HEADSET)) {
		__ASSERT_NO_MSG(tx_buf != NULL);
	}

	active_tx_buf = tx_buf;
	active_rx_buf = rx_buf;
	pending_valid = false;

	k_timer_start(&blk_timer, K_USEC(AUDIO_I2S_BLK_PERIOD_US), K_USEC(AUDIO_I2S_BLK_PERIOD_US));

	state = AUDIO_I2S_STATE_STARTED;
}

void audio_i2s_stop(void)
{
	__ASSERT_NO_MSG(state == AUDIO_I2S_STATE_STARTED);

	k_timer_stop(&blk_timer);

	state = AUDIO_I2S_STATE_IDLE;
}

void audio_i2s_blk_comp_cb_register(i2s_blk_comp_callback_t blk_comp_callback)
{
	i2s_blk_comp_callback = blk_comp_callback;
}

void audio_i2s_init(void)
{
	__ASSERT_NO_MSG(state == AUDIO_I2S_STATE_UNINIT);

	rx_fd = audio_i2s_file_open_bottom(CONFIG_AUDIO_I2S_FILE_RX_PATH, false);
	if (rx_fd < 0) {
		LOG_WRN("No simulated I2S RX file, feeding silence");
		rx_fd = -1;
	}

	tx_fd = audio_i2s_file_open_bottom(CONFIG_AUDIO_I2S_FILE_TX_PATH, true);
	if (tx_fd < 0) {
		LOG_WRN("Could not open simulated I2S TX file, discarding TX data");
		tx_fd = -1;
	}

	k_work_init(&blk_work, blk_work_handler);
	k_timer_init(&blk_timer, blk_timer_expiry, NULL);

	state = AUDIO_I2S_STATE_IDLE;
}
