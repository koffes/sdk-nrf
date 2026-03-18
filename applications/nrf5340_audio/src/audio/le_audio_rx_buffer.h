/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 * @defgroup audio_app_le_audio_rx_buffer LE Audio RX pairing buffer
 * @{
 * @brief Channel-pairing buffer for LE Audio RX.
 *
 * Incoming ISO packets from different channels may arrive out of order.
 * This module buffers them so that channels belonging to the same audio
 * frame (identified by close SDU reference timestamps) are merged and
 * forwarded together in chronological order.
 *
 * Supports up to @kconfig{CONFIG_LE_AUDIO_RX_BUFFER_CHANNELS_MAX} channels
 * simultaneously. The per-channel queue depth (number of frames that may be
 * buffered while waiting for matching channels) is controlled by
 * @kconfig{CONFIG_LE_AUDIO_RX_BUFFERED_PACKETS_PER_CHANNEL}.
 */

#ifndef _LE_AUDIO_RX_BUFFER_H_
#define _LE_AUDIO_RX_BUFFER_H_

#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/net_buf.h>

#include <audio_defines.h>

/**
 * @brief Add an incoming ISO packet to the pairing buffer.
 *
 * The packet is either merged into an existing in-flight slot whose
 * SDU reference timestamp is within @c SDU_REF_CH_DELTA_MAX_US of
 * @p meta->ref_ts_us, or placed in a new slot.
 *
 * @param audio_frame_rx    Pointer to the received audio buffer.
 * @param meta              Metadata for the received packet.
 * @param num_active_streams Number of channels expected per frame.
 * @param stream_locations  Bitmask of all active channel locations.
 *
 * @retval 0        Packet accepted.
 * @retval -EALREADY Duplicate packet (same location + close timestamp);
 *                   caller should silently discard.
 * @retval -ENOMEM  Buffer pool exhausted.
 * @retval -EINVAL  Invalid arguments or internal offset error.
 */
int le_audio_rx_buffer_packet_add(struct net_buf const *audio_frame_rx,
				  struct audio_metadata const *meta, uint8_t num_active_streams,
				  enum bt_audio_location stream_locations);

/**
 * @brief Retrieve the oldest fully paired frame, if one is ready.
 *
 * The caller takes ownership of the returned @c net_buf and must call
 * @c net_buf_unref() when finished.
 *
 * @param[out] audio_frame  Set to the oldest complete paired frame.
 *
 * @retval 0       Frame available; @p audio_frame is valid.
 * @retval -ENOENT No complete frame is currently available.
 * @retval -EINVAL @p audio_frame is NULL.
 */
int le_audio_rx_buffer_ready_get(struct net_buf **audio_frame);

/**
 * @brief Discard all buffered (incomplete) frames.
 *
 * Should be called when the stream stops or the topology changes.
 */
void le_audio_rx_buffer_reset(void);

/** @} */

#endif /* _LE_AUDIO_RX_BUFFER_H_ */
