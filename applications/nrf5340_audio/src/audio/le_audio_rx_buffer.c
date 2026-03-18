/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "le_audio_rx_buffer.h"

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "audio_datapath.h"

LOG_MODULE_DECLARE(le_audio_rx, CONFIG_LE_AUDIO_RX_LOG_LEVEL);

/*
 * Maximum number of in-flight pairing slots.
 *
 * Each channel may have up to (BUFFERED_PACKETS_PER_CHANNEL + 1) frames
 * in flight at once.  Those same net_buf objects are later handed off to
 * ble_q_rx, so the pool must also cover BUF_BLE_RX_PACKET_NUM entries
 * that are queued for the datapath thread.
 */
#define PAIR_SLOTS_MAX                                                                             \
	(CONFIG_LE_AUDIO_RX_BUFFER_CHANNELS_MAX *                                                  \
	 (CONFIG_LE_AUDIO_RX_BUFFERED_PACKETS_PER_CHANNEL + 1))

#define RX_BUF_POOL_SIZE (PAIR_SLOTS_MAX + CONFIG_BUF_BLE_RX_PACKET_NUM)

NET_BUF_POOL_FIXED_DEFINE(ble_rx_pool, RX_BUF_POOL_SIZE,
			  (CONFIG_BT_ISO_RX_MTU * CONFIG_BT_AUDIO_CONCURRENT_RX_STREAMS_MAX),
			  sizeof(struct audio_metadata), NULL);

/**
 * @brief One in-flight pairing slot.
 *
 * A slot is occupied when @c buf is non-NULL.  Channels are merged into
 * the slot until @c audio_metadata_num_loc_get(buf) == @c expected_streams,
 * at which point the slot is "complete" and ready for forwarding.
 */
struct rx_pair_slot {
	struct net_buf *buf;
	uint8_t expected_streams;
	enum bt_audio_location stream_locations;
};

static struct rx_pair_slot slots[PAIR_SLOTS_MAX];

/* ---------------------------------------------------------------------------
 * Internal helpers
 * ---------------------------------------------------------------------------
 */

static uint32_t ts_diff(uint32_t a, uint32_t b)
{
	return (a > b) ? (a - b) : (b - a);
}

/**
 * Map a single-bit location mask to its 0-based index within @p total_loc.
 * E.g. if total_loc = FRONT_LEFT|FRONT_RIGHT and srch_loc = FRONT_RIGHT,
 * *index = 1.
 */
static int location_to_index(enum bt_audio_location total_loc, enum bt_audio_location srch_loc,
			     uint8_t *index)
{
	uint8_t idx = 0;

	while (total_loc) {
		if (srch_loc & 0x01) {
			*index = idx;
			return 0;
		}
		if (total_loc & 0x01) {
			idx++;
		}
		total_loc >>= 1;
		srch_loc >>= 1;
	}

	return -EINVAL;
}

/**
 * Copy the PCM/LC3 payload of @p src into the correct channel offset inside
 * the combined @p dst buffer.
 */
static int frame_data_copy(struct net_buf *dst, struct net_buf const *src,
			   struct audio_metadata const *meta,
			   enum bt_audio_location stream_locations)
{
	uint32_t offset = 0;

	if (dst == NULL || src == NULL || meta == NULL) {
		LOG_ERR("frame_data_copy: invalid arguments");
		return -EINVAL;
	}

	if (meta->locations != stream_locations) {
		uint8_t idx = 0;
		int ret = location_to_index(stream_locations, meta->locations, &idx);

		if (ret) {
			LOG_ERR("frame_data_copy: invalid location index, ret: %d", ret);
			return ret;
		}

		offset = idx * meta->bytes_per_location;

		if (offset > (dst->size - src->len)) {
			LOG_ERR("frame_data_copy: offset %u out of range (buf size %u, "
				"frame len %u)",
				offset, dst->size, src->len);
			return -EINVAL;
		}
	}

	if (src->len && !meta->bad_data) {
		memcpy(&dst->data[offset], src->data, src->len);
	}

	return 0;
}

static void slot_free(struct rx_pair_slot *s)
{
	if (s->buf != NULL) {
		net_buf_unref(s->buf);
		s->buf = NULL;
	}
}

static bool slot_is_complete(struct rx_pair_slot const *s)
{
	struct audio_metadata const *m;

	if (s->buf == NULL) {
		return false;
	}

	m = net_buf_user_data(s->buf);

	return audio_metadata_num_loc_get(m) >= s->expected_streams;
}

/** Return the index of the occupied slot with the smallest ref_ts_us. */
static int slot_oldest_find(void)
{
	int best = -1;

	for (int i = 0; i < ARRAY_SIZE(slots); i++) {
		struct audio_metadata const *current_match;
		struct audio_metadata const *best_match;

		if (slots[i].buf == NULL) {
			continue;
		}

		if (best < 0) {
			best = i;
			continue;
		}

		current_match = net_buf_user_data(slots[i].buf);
		best_match = net_buf_user_data(slots[best].buf);

		if ((int32_t)(current_match->ref_ts_us - best_match->ref_ts_us) < 0) {
			best = i;
		}
	}

	return best;
}

/** Return the index of the oldest occupied slot that contains @p loc. */
static int slot_oldest_for_loc_find(enum bt_audio_location loc)
{
	int best = -1;

	for (int i = 0; i < ARRAY_SIZE(slots); i++) {
		struct audio_metadata const *current_match;
		struct audio_metadata const *best_match;

		if (slots[i].buf == NULL) {
			continue;
		}

		current_match = net_buf_user_data(slots[i].buf);

		if (!(current_match->locations & loc)) {
			continue;
		}

		if (best < 0) {
			best = i;
			continue;
		}

		best_match = net_buf_user_data(slots[best].buf);

		if ((int32_t)(current_match->ref_ts_us - best_match->ref_ts_us) < 0) {
			best = i;
		}
	}

	return best;
}

/** Return the index of the first free (buf == NULL) slot, or -ENOMEM. */
static int slot_free_find(void)
{
	for (int i = 0; i < ARRAY_SIZE(slots); i++) {
		if (slots[i].buf == NULL) {
			return i;
		}
	}

	return -ENOMEM;
}

/**
 * Find a slot that already holds a close-enough timestamp but does NOT yet
 * contain @p loc.  Returns the slot index, or -1 if none found.
 */
static int slot_match_find(enum bt_audio_location loc, uint32_t ref_ts_us)
{
	int best = -1;
	uint32_t best_diff = UINT32_MAX;

	for (int i = 0; i < ARRAY_SIZE(slots); i++) {
		struct audio_metadata const *m;
		uint32_t diff;

		if (slots[i].buf == NULL) {
			continue;
		}

		m = net_buf_user_data(slots[i].buf);

		/* Skip if this location is already present in the slot */
		if (m->locations & loc) {
			continue;
		}

		diff = ts_diff(ref_ts_us, m->ref_ts_us);
		if (diff >= SDU_REF_CH_DELTA_MAX_US) {
			continue;
		}

		if (diff < best_diff) {
			best = i;
			best_diff = diff;
		}
	}

	return best;
}

/**
 * Return true if a slot already exists that has @p loc with a close timestamp.
 * Used to detect and discard duplicate packets.
 */
static bool slot_is_duplicate(enum bt_audio_location loc, uint32_t ref_ts_us)
{
	for (int i = 0; i < ARRAY_SIZE(slots); i++) {
		struct audio_metadata const *meta;

		if (slots[i].buf == NULL) {
			continue;
		}

		meta = net_buf_user_data(slots[i].buf);

		if ((meta->locations & loc) &&
		    ts_diff(ref_ts_us, meta->ref_ts_us) < SDU_REF_CH_DELTA_MAX_US) {
			LOG_ERR("Duplicate packet detected for loc 0x%08x, timestamp %u",
				(uint32_t)loc, ref_ts_us);
			return true;
		}
	}

	return false;
}

/** Count how many occupied slots carry @p loc. */
static int slot_depth_for_loc(enum bt_audio_location loc)
{
	int cnt = 0;

	for (int i = 0; i < ARRAY_SIZE(slots); i++) {
		struct audio_metadata const *meta;

		if (slots[i].buf == NULL) {
			continue;
		}

		meta = net_buf_user_data(slots[i].buf);

		if (meta->locations & loc) {
			cnt++;
		}
	}

	return cnt;
}

/* ---------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------------
 */

int le_audio_rx_buffer_packet_add(struct net_buf const *audio_frame_rx,
				  struct audio_metadata const *meta, uint8_t num_active_streams,
				  enum bt_audio_location stream_locations)
{
	int ret;
	int slot_idx;
	struct audio_metadata *slot_meta;

	if (audio_frame_rx == NULL || meta == NULL) {
		return -EINVAL;
	}

	/* Flush buffer if the stream topology (number of channels or their
	 * locations) has changed since the buffered frames were received.
	 */
	for (int i = 0; i < ARRAY_SIZE(slots); i++) {
		if (slots[i].buf == NULL) {
			continue;
		}
		if (slots[i].expected_streams != num_active_streams ||
		    slots[i].stream_locations != stream_locations) {
			LOG_WRN("Stream topology changed, flushing RX pair buffer");
			le_audio_rx_buffer_reset();
			break;
		}
	}

	/* Silently drop exact duplicates (same location, close timestamp). */
	if (slot_is_duplicate(meta->locations, meta->ref_ts_us)) {
		return -EALREADY;
	}

	slot_idx = slot_match_find(meta->locations, meta->ref_ts_us);

	if (slot_idx < 0) {
		/* No matching slot found — need to open a new one.
		 * Enforce the per-channel depth limit first.
		 */
		int depth = slot_depth_for_loc(meta->locations);

		if (depth >= CONFIG_LE_AUDIO_RX_BUFFERED_PACKETS_PER_CHANNEL + 1) {
			int oldest = slot_oldest_for_loc_find(meta->locations);

			if (oldest >= 0) {
				LOG_WRN("RX pair buffer depth limit reached for "
					"loc 0x%08x, dropping oldest",
					(uint32_t)meta->locations);
				slot_free(&slots[oldest]);
			}
		}

		slot_idx = slot_free_find();

		if (slot_idx < 0) {
			/* All slots full despite depth-limit enforcement; drop
			 * the globally oldest slot as a last resort.
			 */
			int oldest = slot_oldest_find();

			if (oldest < 0) {
				return -ENOMEM;
			}

			LOG_ERR("RX pair buffer saturated, dropping oldest frame");
			slot_free(&slots[oldest]);
			slot_idx = oldest;
		}

		/* Allocate the combined net_buf for this new slot. */
		slots[slot_idx].buf = net_buf_alloc(&ble_rx_pool, K_NO_WAIT);
		if (slots[slot_idx].buf == NULL) {
			LOG_WRN("Out of RX pair buffers");
			return -ENOMEM;
		}

		slots[slot_idx].expected_streams = num_active_streams;
		slots[slot_idx].stream_locations = stream_locations;

		/* Store the first channel's metadata as the slot metadata
		 * (ref_ts_us, data_len_us, sample_rate_hz, etc.).
		 */
		memcpy(net_buf_user_data(slots[slot_idx].buf), meta, sizeof(struct audio_metadata));

		/* Reserve space for all channel payloads. */
		net_buf_add(slots[slot_idx].buf, meta->bytes_per_location * num_active_streams);
	} else {
		/* Merge this channel into the existing slot. */
		slot_meta = net_buf_user_data(slots[slot_idx].buf);
		slot_meta->locations |= meta->locations;
		slot_meta->bad_data |= meta->bad_data;
	}

	ret = frame_data_copy(slots[slot_idx].buf, audio_frame_rx, meta, stream_locations);
	if (ret) {
		LOG_ERR("Failed to copy channel data into RX pair slot: %d", ret);
		slot_free(&slots[slot_idx]);
		return ret;
	}

	return 0;
}

int le_audio_rx_buffer_ready_get(struct net_buf **audio_frame)
{
	int best = -1;

	if (audio_frame == NULL) {
		return -EINVAL;
	}

	/* Find the oldest complete (all channels received) slot. */
	for (int i = 0; i < ARRAY_SIZE(slots); i++) {
		struct audio_metadata const *current_match;
		struct audio_metadata const *best_match;

		if (!slot_is_complete(&slots[i])) {
			continue;
		}

		if (best < 0) {
			best = i;
			continue;
		}

		current_match = net_buf_user_data(slots[i].buf);
		best_match = net_buf_user_data(slots[best].buf);

		if ((int32_t)(current_match->ref_ts_us - best_match->ref_ts_us) < 0) {
			best = i;
		}
	}

	if (best < 0) {
		return -ENOENT;
	}

	*audio_frame = slots[best].buf;
	slots[best].buf = NULL;

	return 0;
}

void le_audio_rx_buffer_reset(void)
{
	for (int i = 0; i < ARRAY_SIZE(slots); i++) {
		slot_free(&slots[i]);
	}
}
