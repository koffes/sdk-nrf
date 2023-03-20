#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/kernel.h>
#include <string.h>



#include "tmp_cap.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tmp_cap, 4);

static int tmp_cap_empty_index_get(struct tmp_cap_storage temp_cap[], struct bt_conn *conn, uint8_t *index)
{
	if (conn == NULL) {
		LOG_ERR("No conn provided");
		return -EINVAL;
	}

	for (int i = 0; i < 2; i++) {
		if (temp_cap[i].conn == conn) {
			LOG_ERR("This conn already has an entry");
			return -EACCES;
		}
	}

	/* Connection not found in temp_cap, searching for empty slot */
	for (int i = 0; i < 2; i++) {
		if (temp_cap[i].conn == NULL) {
			temp_cap[i].conn = conn;
			*index = i;
			return 0;
		}
	}

	LOG_ERR("No more space in temp_cap");

	return -ECANCELED;
}

static int tmp_cap_filled_index_get(struct tmp_cap_storage temp_cap[], struct bt_conn const * conn, uint8_t *index)
{
	if (conn == NULL) {
		LOG_ERR("No conn provided");
		return -EINVAL;
	}

	for (int i = 0; i < 2; i++) {
		if (temp_cap[i].conn == conn) {
			*index = i;
			return 0;
		}
	}

	return -ENOMEM;
}

int tmp_cap_push(struct tmp_cap_storage temp_cap[], struct bt_conn* conn, uint8_t num_caps, struct bt_codec const *const codec_in)
{
	int ret;
	uint8_t tmp_cap_index = UINT8_MAX;

	ret = tmp_cap_empty_index_get(temp_cap, conn, &tmp_cap_index);
	if (ret) {
		return ret;
	}

	if (num_caps < ARRAY_SIZE(temp_cap[tmp_cap_index].codec)) {
		struct bt_codec *tmp_cap_loc =
			&temp_cap[tmp_cap_index].codec[num_caps];
		memcpy(tmp_cap_loc, codec_in, sizeof(struct bt_codec));

		for (int i = 0; i < codec_in->data_count; i++) {
			tmp_cap_loc->data[i].data.data = tmp_cap_loc->data[i].value;
		}

		for (int i = 0; i < codec_in->meta_count; i++) {
			tmp_cap_loc->meta[i].data.data = tmp_cap_loc->meta[i].value;
		}
		LOG_WRN("pushed to index %d", tmp_cap_index);
	} else {
		LOG_WRN("No more space for storing capabilities");
	}

	return 0;
}

int tmp_cap_peek(struct tmp_cap_storage temp_cap[], struct bt_conn const* conn, uint8_t num_caps, struct bt_codec *codec_in)
{
	int ret;
	uint8_t tmp_cap_index = UINT8_MAX;

	ret = tmp_cap_filled_index_get(temp_cap, conn, &tmp_cap_index);
	if (ret) {
		return ret;
	}

	LOG_WRN("Peek at index %d", tmp_cap_index);
	codec_in = &temp_cap[tmp_cap_index].codec[num_caps];
	return 0;
}

int tmp_cap_erase(struct tmp_cap_storage temp_cap[], struct bt_conn const *conn)
{
	int ret;
	uint8_t tmp_cap_index = UINT8_MAX;

	ret = tmp_cap_filled_index_get(temp_cap, conn, &tmp_cap_index);
	if (ret) {
		return ret;
	}

	memset(temp_cap[tmp_cap_index].codec, 0, sizeof(temp_cap[tmp_cap_index].codec));
	temp_cap[tmp_cap_index].conn = NULL;
	return 0;
}
