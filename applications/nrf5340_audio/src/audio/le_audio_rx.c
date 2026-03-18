/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "le_audio_rx.h"

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <nrfx_clock.h>

#include "streamctrl.h"
#include "audio_datapath.h"
#include "macros_common.h"
#include "audio_system.h"
#include "audio_sync_timer.h"
#include "audio_defines.h"
#include "le_audio.h"
#include "le_audio_rx_buffer.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(le_audio_rx, CONFIG_LE_AUDIO_RX_LOG_LEVEL);

struct rx_stats {
	uint32_t recv_cnt;
	uint32_t bad_frame_cnt;
};

static bool initialized;
static struct k_thread audio_datapath_thread_data;
static k_tid_t audio_datapath_thread_id;
K_THREAD_STACK_DEFINE(audio_datapath_thread_stack, CONFIG_AUDIO_DATAPATH_STACK_SIZE);

K_MSGQ_DEFINE(ble_q_rx, sizeof(struct net_buf *), CONFIG_BUF_BLE_RX_PACKET_NUM, sizeof(void *));

static int rx_frame_enqueue(struct net_buf *audio_frame)
{
	int ret;
	static uint32_t num_overruns;
	struct net_buf *stale_buf = NULL;

	if (k_msgq_num_free_get(&ble_q_rx) == 0) {
		ret = k_msgq_get(&ble_q_rx, (void *)&stale_buf, K_NO_WAIT);
		if ((ret == 0) && (stale_buf != NULL)) {
			num_overruns++;
			if ((num_overruns % 100) == 1) {
				LOG_WRN("BLE ISO RX overrun: Num: %d", num_overruns);
			}
			net_buf_unref(stale_buf);
		}
	}

	ret = k_msgq_put(&ble_q_rx, (void *)&audio_frame, K_NO_WAIT);
	if (ret) {
		return ret;
	}

	return 0;
}

/* Callback for handling ISO RX */
void le_audio_rx_data_handler(struct net_buf *audio_frame_rx, struct audio_metadata *meta,
			      uint8_t location_index)
{
	int ret;
	static struct rx_stats rx_stats[CONFIG_BT_AUDIO_CONCURRENT_RX_STREAMS_MAX];
	static uint32_t num_thrown;

	if (!initialized) {
		ERR_CHK_MSG(-EPERM, "Data received but le_audio_rx is not initialized");
	}

	rx_stats[location_index].recv_cnt++;

	if (meta->bad_data) {
		rx_stats[location_index].bad_frame_cnt++;
	}

	if ((rx_stats[location_index].recv_cnt % 100) == 0 && rx_stats[location_index].recv_cnt) {
		/* NOTE: The string below is used by the Nordic CI system */
		LOG_DBG("ISO RX SDUs: Loc: %d Total: %d Bad: %d", location_index,
			rx_stats[location_index].recv_cnt, rx_stats[location_index].bad_frame_cnt);
	}

	if (stream_state_get() != STATE_STREAMING) {
		/* Throw away data and flush any partially assembled frames */
		num_thrown++;
		le_audio_rx_buffer_reset();
		if ((num_thrown % 100) == 1) {
			LOG_WRN("Not in streaming state (%d), thrown %d packet(s)",
				stream_state_get(), num_thrown);
		}
		return;
	}

	if (location_index != 0 && (CONFIG_AUDIO_DEV == GATEWAY)) {
		/* Only the first device will be used as mic input on gateway */
		return;
	}

	uint8_t num_active_streams = 0;
	enum bt_audio_location stream_locations = 0;

	ret = le_audio_concurrent_sync_num_get(&num_active_streams, &stream_locations);
	if (ret) {
		LOG_ERR("Failed to get number of active streams: %d", ret);
		return;
	}

	/* Capture timestamp of when audio frame is received */
	meta->data_rx_ts_us = audio_sync_timer_capture();

	/* Hand the packet to the pairing buffer.  Channels sharing a close SDU
	 * reference timestamp are merged; others are held until their pair arrives.
	 */
	ret = le_audio_rx_buffer_packet_add(audio_frame_rx, meta, num_active_streams,
					    stream_locations);
	if (ret == -EALREADY) {
		LOG_DBG("Discarding duplicate RX packet for timestamp %u", meta->ref_ts_us);
		return;
	} else if (ret) {
		LOG_ERR("Failed to buffer RX frame: %d", ret);
		return;
	}

	/* One incoming packet completes at most one pair, so a single
	 * ready-check per callback invocation is sufficient.
	 */
	struct net_buf *audio_frame = NULL;

	ret = le_audio_rx_buffer_ready_get(&audio_frame);
	if (ret) {
		/* Pair not yet complete — nothing to forward. */
		return;
	}

	ret = rx_frame_enqueue(audio_frame);
	if (ret) {
		LOG_ERR("Failed to enqueue paired RX frame: %d", ret);
		net_buf_unref(audio_frame);
	}
}

/**
 * @brief	Receive data from BLE through a k_fifo and send to USB or audio datapath.
 */
static void audio_datapath_thread(void *dummy1, void *dummy2, void *dummy3)
{
	int ret;
	struct net_buf *audio_frame = NULL;

	while (1) {
		ret = k_msgq_get(&ble_q_rx, (void *)&audio_frame, K_FOREVER);
		ERR_CHK(ret);

		if (IS_ENABLED(CONFIG_AUDIO_SOURCE_USB) && (CONFIG_AUDIO_DEV == GATEWAY)) {
			ret = audio_system_decode(audio_frame);
			ERR_CHK(ret);
		} else {
			audio_datapath_stream_out(audio_frame);
		}

		net_buf_unref(audio_frame);

		STACK_USAGE_PRINT("audio_datapath_thread", &audio_datapath_thread_data);
	}
}

static int audio_datapath_thread_create(void)
{
	int ret;

	audio_datapath_thread_id = k_thread_create(
		&audio_datapath_thread_data, audio_datapath_thread_stack,
		CONFIG_AUDIO_DATAPATH_STACK_SIZE, (k_thread_entry_t)audio_datapath_thread, NULL,
		NULL, NULL, K_PRIO_PREEMPT(CONFIG_AUDIO_DATAPATH_THREAD_PRIO), 0, K_NO_WAIT);
	ret = k_thread_name_set(audio_datapath_thread_id, "Audio_datapath");
	if (ret) {
		LOG_ERR("Failed to create audio_datapath thread");
		return ret;
	}

	return 0;
}

int le_audio_rx_init(void)
{
	int ret;

	if (initialized) {
		return -EALREADY;
	}

	ret = audio_datapath_thread_create();
	if (ret) {
		return ret;
	}

	initialized = true;

	return 0;
}
