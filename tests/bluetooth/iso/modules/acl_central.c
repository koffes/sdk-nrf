/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include "acl_central.h"

#include <stdlib.h>
#include <ctype.h>
#include <getopt.h>
#include <unistd.h>

#include <zephyr/shell/shell.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/services/nus.h>
#include <bluetooth/services/nus_client.h>
#include <bluetooth/gatt_dm.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(central, CONFIG_ACL_TEST_LOG_LEVEL);

#define REMOTE_DEVICE_NAME_PEER	    CONFIG_BT_DEVICE_NAME
#define REMOTE_DEVICE_NAME_PEER_LEN (sizeof(REMOTE_DEVICE_NAME_PEER) - 1)

static bool running;
static uint32_t acl_dummy_data_send_interval_ms = 1000;
static uint32_t acl_dummy_data_size = 20;
struct peer_device {
	struct bt_conn *conn;
	struct k_work_delayable dummy_data_send_work;
};
static struct peer_device remote_peer[CONFIG_BT_MAX_CONN];
static struct bt_nus_client *nus[CONFIG_BT_MAX_CONN];
static struct bt_nus_client nus_client[CONFIG_BT_MAX_CONN];
/* Default ACL connection parameter configuration */
static struct bt_le_conn_param conn_param = {
	.interval_min = 40, /* ACL interval min (x 1.25 ms) */
	.interval_max = 40, /* ACL interval max (x 1.25 ms) */
	.latency = 0,	    /* ACL slave latency = 0 */
	.timeout = 100	    /* ACL supervision timeout = 1000 ms (x 10 ms)*/
};

static bool initialized;

static int scan_start(void);

static bool all_peers_connected(void)
{
	for (int i = 0; i < ARRAY_SIZE(remote_peer); i++) {
		if (remote_peer[i].conn == NULL) {
			return false;
		}
	}

	return true;
}

static int channel_index_get(const struct bt_conn *conn, uint8_t *index)
{
	if (conn == NULL) {
		LOG_ERR("No connection provided");
		return -EINVAL;
	}

	for (int i = 0; i < ARRAY_SIZE(remote_peer); i++) {
		if (remote_peer[i].conn == conn) {
			*index = i;
			return 0;
		}
	}

	LOG_WRN("Connection not found");

	return -EINVAL;
}

static void work_dummy_data_send(struct k_work *work)
{
	int ret;
	char dummy_string[CONFIG_BT_MAX_CONN][CONFIG_BT_L2CAP_TX_MTU] = {0};
	static uint8_t channel_index;
	static uint32_t acl_send_count[CONFIG_BT_MAX_CONN];

	struct peer_device *peer =
		CONTAINER_OF(work, struct peer_device, dummy_data_send_work.work);

	ret = channel_index_get(peer->conn, &channel_index);
	if (ret) {
		LOG_ERR("Channel index not found");
		return;
	}
	acl_send_count[channel_index]++;
	sys_put_le32(acl_send_count[channel_index], dummy_string[channel_index]);

	ret = bt_nus_client_send(&nus_client[channel_index], dummy_string[channel_index],
				 acl_dummy_data_size);
	if (ret) {
		LOG_WRN("Failed to send, ret = %d", ret);
	}

	if ((acl_send_count[channel_index] % 1) == 0) {
		LOG_INF("\t ACL TX: Count: %7u ch index: %u", acl_send_count[channel_index],
			channel_index);
	}

	k_work_reschedule(&remote_peer[channel_index].dummy_data_send_work,
			  K_MSEC(acl_dummy_data_send_interval_ms));
}

static void discovery_complete(struct bt_gatt_dm *dm, void *context)
{
	int ret;
	uint8_t channel_index;

	ret = channel_index_get(bt_gatt_dm_conn_get(dm), &channel_index);
	if (ret) {
		LOG_ERR("Channel index not found");
	}

	nus[channel_index] = context;

	ret = bt_nus_handles_assign(dm, nus[channel_index]);
	if (ret) {
		LOG_ERR("NUS handle assigne failed, ret = %d", ret);
	}
	bt_nus_subscribe_receive(nus[channel_index]);

	ret = bt_gatt_dm_data_release(dm);
	if (ret) {
		LOG_ERR("Release service discovery data failed, ret = %d", ret);
	}

	k_work_schedule(&remote_peer[channel_index].dummy_data_send_work, K_MSEC(100));

	if (all_peers_connected()) {
		LOG_INF("All devices connected");
	} else {
		LOG_INF("Not all devices connected");
		scan_start();
	}
}

static void discovery_service_not_found(struct bt_conn *conn, void *context)
{
	LOG_ERR("NUS service not found");
}

static void discovery_error(struct bt_conn *conn, int err, void *context)
{
	LOG_ERR("Error while discovering GATT database:, err = %d", err);
}

struct bt_gatt_dm_cb discovery_cb = {
	.completed = discovery_complete,
	.service_not_found = discovery_service_not_found,
	.error_found = discovery_error,
};

static int gatt_discover(struct bt_conn *conn)
{
	int ret;
	uint8_t channel_index;

	ret = channel_index_get(conn, &channel_index);
	if (ret) {
		LOG_ERR("Channel index not found");
	}

	ret = bt_gatt_dm_start(conn, BT_UUID_NUS_SERVICE, &discovery_cb,
			       &nus_client[channel_index]);
	return ret;
}

static uint8_t ble_data_received(struct bt_nus_client *nus, const uint8_t *data, uint16_t len)
{
	LOG_HEXDUMP_INF(data, len, "NUS received:");
	return BT_GATT_ITER_CONTINUE;
}

static int nus_client_init(void)
{
	int ret;
	struct bt_nus_client_init_param init = {.cb = {
							.received = ble_data_received,
						}};

	for (int i = 0; i < ARRAY_SIZE(nus_client); i++) {
		ret = bt_nus_client_init(&nus_client[i], &init);
		if (ret) {
			LOG_ERR("NUS Client [%d] initialization failed, ret = %d", i, ret);
			return ret;
		}
	}

	return 0;
}

static int device_found(uint8_t type, const uint8_t *data, uint8_t data_len,
			const bt_addr_le_t *addr)
{
	int ret;
	struct bt_conn *conn = NULL;

	if (all_peers_connected()) {
		LOG_DBG("All peripherals connected");
		return 0;
	}

	if ((data_len == REMOTE_DEVICE_NAME_PEER_LEN) &&
	    (strncmp(REMOTE_DEVICE_NAME_PEER, data, REMOTE_DEVICE_NAME_PEER_LEN) == 0)) {
		LOG_DBG("Device found");

		ret = bt_le_scan_stop();
		if (ret && ret != -EALREADY) {
			LOG_WRN("Stop scan failed: %d", ret);
		}

		ret = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, &conn_param, &conn);
		if (ret) {
			LOG_ERR("Could not init connection");
			scan_start();
			return ret;
		}

		for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
			if (remote_peer[i].conn == NULL) {
				remote_peer[i].conn = conn;
				break;
			}
		}

		return 0;
	}

	return -ENOENT;
}

/** @brief  Parse BLE advertisement package.
 */
static void ad_parse(struct net_buf_simple *p_ad, const bt_addr_le_t *addr)
{
	while (p_ad->len > 1) {
		uint8_t len = net_buf_simple_pull_u8(p_ad);
		uint8_t type;

		/* Check for early termination */
		if (len == 0) {
			return;
		}

		if (len > p_ad->len) {
			LOG_ERR("AD malformed");
			return;
		}

		type = net_buf_simple_pull_u8(p_ad);

		if (device_found(type, p_ad->data, len - 1, addr) == 0) {
			return;
		}

		(void)net_buf_simple_pull(p_ad, len - 1);
	}
}

static void on_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			    struct net_buf_simple *p_ad)
{
	if ((type == BT_GAP_ADV_TYPE_ADV_IND || type == BT_GAP_ADV_TYPE_EXT_ADV)) {
		/* Note: May lead to connection creation */
		ad_parse(p_ad, addr);
	}
}

static int scan_start(void)
{
	int ret;

	ret = bt_le_scan_start(BT_LE_SCAN_PASSIVE, on_device_found);
	if (ret && ret != -EALREADY) {
		LOG_WRN("Scanning failed to start: %d", ret);
		return ret;
	}

	LOG_INF("Scanning successfully started");
	return 0;
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	int ret;
	char addr[BT_ADDR_LE_STR_LEN];

	(void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		LOG_ERR("ACL connection to %s failed, error %d", addr, err);
		bt_conn_unref(conn);
		scan_start();

		return;
	}

	ret = gatt_discover(conn);
	if (ret) {
		LOG_ERR("Failed to start the discovery procedure, ret = %d", ret);
	}

	/* ACL connection established */
	LOG_INF("Connected: %s", addr);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	int ret;
	uint8_t channel_index;
	char addr[BT_ADDR_LE_STR_LEN];

	(void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason 0x%02x)", addr, reason);
	bt_conn_unref(conn);

	ret = channel_index_get(conn, &channel_index);
	if (ret) {
		LOG_WRN("Unknown connection");
	} else {
		remote_peer[channel_index].conn = NULL;
	}

	k_work_cancel_delayable(&remote_peer[channel_index].dummy_data_send_work);
	if (running) {
		scan_start();
	}
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
};

int acl_central_start(void)
{
	int ret = 0;

	if (!initialized) {
		LOG_INF("Peripheral not initialized. Running init");

		ret = acl_central_init();
		if (ret) {
			return ret;
		}
	}

	ret = scan_start();
	if (ret) {
		return ret;
	}
	running = true;

	LOG_INF("Central started");

	return 0;
}

int acl_central_stop(void)
{
	int ret;

	ret = bt_le_scan_stop();
	if (ret && ret != -EALREADY) {
		LOG_WRN("Stop scan failed: %d", ret);
	}

	for (int i = 0; i < ARRAY_SIZE(remote_peer); i++) {
		if (remote_peer[i].conn != NULL) {
			LOG_WRN("Disconnecting from remote peer %d", i);
			ret = bt_conn_disconnect(remote_peer[i].conn,
						 BT_HCI_ERR_REMOTE_USER_TERM_CONN);
			if (ret) {
				LOG_ERR("Disconnected with remote peer failed %d", ret);
			}
		}
	}

	running = false;

	LOG_INF("Central stopped");

	return 0;
}

static int acl_central_print_cfg(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	LOG_INF("\n\tacl_int_min(x0.625ms): %d\n\tacl_int_max(x0.625ms): %d\n\tacl_latency: "
		"%d\n\tacl_timeout(x100ms): %d\n\tacl_payload_size: %d\n\tacl_send_int(ms): %d\n",
		conn_param.interval_min, conn_param.interval_max, conn_param.latency,
		conn_param.timeout, acl_dummy_data_size, acl_dummy_data_send_interval_ms);
	return 0;
}

static int argument_check(const struct shell *shell, uint8_t const *const input)
{
	char *end;

	if (running) {
		LOG_ERR("Arguments can not be changed while running");
		return -EACCES;
	}

	int arg_val = strtol(input, &end, 10);

	if (*end != '\0' || (uint8_t *)end == input || (arg_val == 0 && !isdigit(input[0])) ||
	    arg_val < 0) {
		LOG_ERR("Argument must be a positive integer %s", input);
		return -EINVAL;
	}

	return arg_val;
}

static int acl_central_int_min_set(const struct shell *shell, size_t argc, char **argv)
{
	int arg = argument_check(shell, argv[1]);

	if (arg < 0) {
		return arg;
	}

	conn_param.interval_min = arg;
	acl_central_print_cfg(shell, 0, NULL);
	return 0;
}

static int acl_central_int_max_set(const struct shell *shell, size_t argc, char **argv)
{
	int arg = argument_check(shell, argv[1]);

	if (arg < 0) {
		return arg;
	}

	conn_param.interval_max = arg;
	acl_central_print_cfg(shell, 0, NULL);
	return 0;
}

static int acl_central_latency_set(const struct shell *shell, size_t argc, char **argv)
{
	int arg = argument_check(shell, argv[1]);

	if (arg < 0) {
		return arg;
	}

	conn_param.latency = arg;
	acl_central_print_cfg(shell, 0, NULL);
	return 0;
}

static int acl_central_timeout_set(const struct shell *shell, size_t argc, char **argv)
{
	int arg = argument_check(shell, argv[1]);

	if (arg < 0) {
		return arg;
	}

	conn_param.timeout = arg;
	acl_central_print_cfg(shell, 0, NULL);
	return 0;
}

static int acl_central_send_int_set(const struct shell *shell, size_t argc, char **argv)
{
	int arg = argument_check(shell, argv[1]);

	if (arg < 0) {
		return arg;
	}

	acl_dummy_data_send_interval_ms = arg;
	acl_central_print_cfg(shell, 0, NULL);
	return 0;
}

static int acl_central_payload_size_set(const struct shell *shell, size_t argc, char **argv)
{
	int arg = argument_check(shell, argv[1]);

	if (arg < 0) {
		return arg;
	}

	acl_dummy_data_size = arg;
	acl_central_print_cfg(shell, 0, NULL);
	return 0;
}

int acl_central_init(void)
{
	int ret;

	ret = nus_client_init();
	bt_conn_cb_register(&conn_callbacks);
	for (int i = 0; i < ARRAY_SIZE(remote_peer); i++) {
		k_work_init_delayable(&remote_peer[i].dummy_data_send_work, work_dummy_data_send);
	}

	initialized = true;

	LOG_INF("ACL Central initialized");

	return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	central_cmd, SHELL_CMD(init, NULL, "Init central NUS.", acl_central_init),
	SHELL_CMD(start, NULL, "Start central NUS.", acl_central_start),
	SHELL_CMD(stop, NULL, "Stop central NUS.", acl_central_stop),
	SHELL_CMD(cfg, NULL, "Print config.", acl_central_print_cfg),
	SHELL_CMD_ARG(int_min_set, NULL, "set", acl_central_int_min_set, 2, 0),
	SHELL_CMD_ARG(int_max_set, NULL, "set", acl_central_int_max_set, 2, 0),
	SHELL_CMD_ARG(latency_set, NULL, "set", acl_central_latency_set, 2, 0),
	SHELL_CMD_ARG(timeout_set, NULL, "set", acl_central_timeout_set, 2, 0),
	SHELL_CMD_ARG(send_int_set, NULL, "set", acl_central_send_int_set, 2, 0),
	SHELL_CMD_ARG(payload_size_set, NULL, "set", acl_central_payload_size_set, 2, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(acl_central, &central_cmd, "Central NUS commands", NULL);
