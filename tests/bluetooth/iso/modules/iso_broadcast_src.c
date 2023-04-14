/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/gpio.h>
#include <stdlib.h>
#include <ctype.h>
#include <getopt.h>
#include <unistd.h>

#include "iso_broadcast_src.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(broadcast_src, CONFIG_ISO_TEST_LOG_LEVEL);

K_THREAD_STACK_DEFINE(broadcaster_thread_stack, 4096);
static struct k_thread broadcaster_thread;

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

#define BUF_ALLOC_TIMEOUT (10) /* milliseconds */
#define BIG_SDU_INTERVAL_US (10000)

NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool, CONFIG_BIS_ISO_CHAN_COUNT_MAX,
			  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU), 8, NULL);

static K_SEM_DEFINE(sem_big_cmplt, 0, CONFIG_BIS_ISO_CHAN_COUNT_MAX);
static K_SEM_DEFINE(sem_big_term, 0, CONFIG_BIS_ISO_CHAN_COUNT_MAX);

static uint16_t seq_num;
static bool running;
K_SEM_DEFINE(tx_sent, 0, 1);

static void iso_connected(struct bt_iso_chan *chan)
{
	LOG_INF("ISO Channel %p connected", chan);

	seq_num = 0U;

	k_sem_give(&sem_big_cmplt);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	LOG_INF("ISO Channel %p disconnected with reason 0x%02x", chan, reason);
	k_sem_give(&sem_big_term);
}

static void iso_sent(struct bt_iso_chan *chan)
{
	k_sem_give(&tx_sent);
}

static struct bt_iso_chan_ops iso_ops = {
	.connected = iso_connected,
	.disconnected = iso_disconnected,
	.sent = iso_sent,
};

static struct bt_iso_chan_io_qos iso_tx_qos = {
	.sdu = sizeof(uint32_t), /* bytes */
	.rtn = 1,
	.phy = BT_GAP_LE_PHY_2M,
};

static struct bt_iso_chan_qos bis_iso_qos = {
	.tx = &iso_tx_qos,
};

static struct bt_iso_chan bis_iso_chan[] = {
	{
		.ops = &iso_ops,
		.qos = &bis_iso_qos,
	},
	{
		.ops = &iso_ops,
		.qos = &bis_iso_qos,
	},
};

static struct bt_iso_chan *bis[] = {
	&bis_iso_chan[0],
	&bis_iso_chan[1],
};

static struct bt_iso_big_create_param big_create_param = {
	.num_bis = 1,
	.bis_channels = bis,
	.interval = BIG_SDU_INTERVAL_US, /* in microseconds */
	.latency = 10, /* in milliseconds */
	.packing = 0, /* 0 - sequential, 1 - interleaved */
	.framing = 0, /* 0 - unframed, 1 - framed */
};

static struct bt_le_ext_adv *adv;
static struct bt_iso_big *big;
static uint32_t iso_send_count;
static uint8_t iso_data[sizeof(iso_send_count)] = { 0 };

struct broadcaster_params broadcast_params = { .sdu_size = 10,
					       .phy = BT_GAP_LE_PHY_2M,
					       .rtn = 1,
					       .num_bis = 1,
					       .sdu_interval_us = 10000,
					       .latency_ms = 10,
					       .packing = 0,
					       .framing = 0 };

static void broadcaster_t(void *arg1, void *arg2, void *arg3)
{
	static uint8_t initial_send = 2;

	while (1) {
		int ret;

		k_sleep(K_USEC(big_create_param.interval));
		if (!running) {
			k_msleep(100);
			initial_send = 2;
			continue;
		}

		if (!initial_send) {
			ret = k_sem_take(&tx_sent, K_USEC(big_create_param.interval * 2));
			if (ret) {
				LOG_ERR("Sent semaphore timed out");
			}
		}

		for (uint8_t chan = 0U; chan < broadcast_params.num_bis; chan++) {
			struct net_buf *buf;

			buf = net_buf_alloc(&bis_tx_pool, K_MSEC(BUF_ALLOC_TIMEOUT));
			if (!buf) {
				LOG_ERR("Data buffer allocate timeout on channel %u", chan);
			}

			net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
			sys_put_le32(iso_send_count, iso_data);
			net_buf_add_mem(buf, iso_data, sizeof(iso_data));
			ret = bt_iso_chan_send(&bis_iso_chan[chan], buf, seq_num,
					       BT_ISO_TIMESTAMP_NONE);
			if (ret < 0) {
				LOG_ERR("Unable to broadcast data on channel %u"
					" : %d",
					chan, ret);
				net_buf_unref(buf);
			}
		}

		iso_send_count++;
		seq_num++;
		if (initial_send) {
			initial_send--;
		}

		if ((iso_send_count % CONFIG_PRINT_CONN_INTERVAL) == 0) {
			LOG_INF("Sending value %u", iso_send_count);
			if ((iso_send_count / CONFIG_PRINT_CONN_INTERVAL) % 2 == 0) {
				ret = gpio_pin_set_dt(&led, 1);
			} else {
				ret = gpio_pin_set_dt(&led, 0);
			}
		}
	}
}

int iso_broadcaster_stop(const struct shell *shell, size_t argc, char **argv)
{
	int err;

	running = false;

	LOG_INF("BIG Terminate");
	err = bt_iso_big_terminate(big);
	if (err) {
		LOG_ERR("failed (err %d)", err);
		return err;
	}

	for (uint8_t chan = 0U; chan < broadcast_params.num_bis; chan++) {
		LOG_INF("Waiting for BIG terminate complete"
			" chan %u...",
			chan);
		err = k_sem_take(&sem_big_term, K_MSEC(100));
		if (err) {
			LOG_ERR("failed (err %d)", err);
			return err;
		}
		LOG_INF("BIG terminate complete chan %u.", chan);
	}

	err = bt_le_ext_adv_stop(adv);
	if (err) {
		LOG_ERR("failed (err %d)", err);
		return err;
	}

	err = bt_le_per_adv_stop(adv);
	if (err) {
		LOG_ERR("failed (err %d)", err);
		return err;
	}

	err = bt_le_ext_adv_delete(adv);
	if (err) {
		LOG_ERR("Failed to delete advertising set (err %d)\n", err);
		return err;
	}

	return 0;
}

int iso_broadcaster_start(const struct shell *shell, size_t argc, char **argv)
{
	int err;

	err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (err < 0) {
		return err;
	}

	/* Create a non-connectable non-scannable advertising set */
	err = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN_NAME, NULL, &adv);
	if (err) {
		LOG_ERR("Failed to create advertising set (err %d)", err);
		return err;
	}

	/* Set periodic advertising parameters */
	err = bt_le_per_adv_set_param(adv, BT_LE_PER_ADV_DEFAULT);
	if (err) {
		LOG_ERR("Failed to set periodic advertising parameters"
			" (err %d)",
			err);
		return err;
	}

	/* Enable Periodic Advertising */
	err = bt_le_per_adv_start(adv);
	if (err) {
		LOG_ERR("Failed to enable periodic advertising (err %d)", err);
		return err;
	}

	/* Start extended advertising */
	err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		LOG_ERR("Failed to start extended advertising (err %d)", err);
		return err;
	}

	/* Create BIG */
	err = bt_iso_big_create(adv, &big_create_param, &big);
	if (err) {
		LOG_ERR("Failed to create BIG (err %d)", err);
		return err;
	}

	for (uint8_t chan = 0U; chan < broadcast_params.num_bis; chan++) {
		LOG_INF("Waiting for BIG complete chan %u...", chan);
		err = k_sem_take(&sem_big_cmplt, K_FOREVER);
		if (err) {
			LOG_ERR("failed (err %d)", err);
			return err;
		}
		LOG_INF("BIG create complete chan %u.", chan);
	}

	running = true;

	return 0;
}

int broadcaster_print_cfg(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(
		shell,
		"sdu_size %d\n phy %d\n rtn %d\n num_bis %d\n sdu_interval_us %d\n" \
		"latency_ms %d\n packing %d\n framing %d",
		broadcast_params.sdu_size, broadcast_params.phy, broadcast_params.rtn,
		broadcast_params.num_bis, broadcast_params.sdu_interval_us,
		broadcast_params.latency_ms, broadcast_params.packing, broadcast_params.framing);

	return 0;
}

static int argument_check(const struct shell *shell, uint8_t const *const input)
{
	char *end;
	int arg_val = strtol(input, &end, 10);

	if (*end != '\0' || (uint8_t *)end == input || (arg_val == 0 && !isdigit(input[0])) ||
	    arg_val < 0) {
		shell_error(shell, "Argument must be a positive integer %s", input);
		return -EINVAL;
	}

	if (running) {
		shell_error(shell, "Arguments can not be changed while running");
		return -EACCES;
	}

	return arg_val;
}

static struct option long_options[] = { { "sdu_size", required_argument, NULL, 's' },
					{ "phy", required_argument, NULL, 'p' },
					{ "rtn", required_argument, NULL, 'r' },
					{ "num_bis:", required_argument, NULL, 'n' },
					{ "sdu_int_us", required_argument, NULL, 'S' },
					{ "latency_ms", required_argument, NULL, 'l' },
					{ "packing", required_argument, NULL, 'P' },
					{ "framing", required_argument, NULL, 'f' },
					{ 0, 0, 0, 0 } };

static const char short_options[] = "s:p:r:n:S:l:P:f:";

static int set_param(const struct shell *shell, size_t argc, char **argv)
{
	int result = argument_check(shell, argv[2]);

	if (result < 0) {
		return result;
	}

	if (running) {
		shell_error(shell, "Stop src before changing parameters");
		return -EPERM;
	}

	int long_index = 0;
	int opt;

	optreset = 1;
	optind = 1;

	while ((opt = getopt_long(argc, argv, short_options, long_options, &long_index)) != -1) {
		switch (opt) {
		case 's':
			broadcast_params.sdu_size = result;
			broadcaster_print_cfg(shell, 0, NULL);
			break;
		case 'p':
			broadcast_params.phy = result;
			broadcaster_print_cfg(shell, 0, NULL);
			break;
		case 'r':
			broadcast_params.rtn = result;
			broadcaster_print_cfg(shell, 0, NULL);
			break;
		case 'n':
			broadcast_params.num_bis = result;
			broadcaster_print_cfg(shell, 0, NULL);
			break;
		case 'S':
			broadcast_params.sdu_interval_us = result;
			broadcaster_print_cfg(shell, 0, NULL);
			break;
		case 'l':
			broadcast_params.latency_ms = result;
			broadcaster_print_cfg(shell, 0, NULL);
			break;
		case 'P':
			broadcast_params.packing = result;
			broadcaster_print_cfg(shell, 0, NULL);
			break;
		case 'f':
			broadcast_params.framing = result;
			broadcaster_print_cfg(shell, 0, NULL);
			break;
		case ':':
			shell_error(shell, "Missing option parameter");
			break;
		case '?':
			shell_error(shell, "Unknown option: %c", opt);
			break;
		default:
			shell_error(shell, "Invalid option: %c", opt);
			break;
		}
	}

	return 0;
}

int iso_broadcast_src_init(void)
{
	running = false;

	if (!gpio_is_ready_dt(&led)) {
		return -EBUSY;
	}

	k_thread_create(&broadcaster_thread, broadcaster_thread_stack,
			K_THREAD_STACK_SIZEOF(broadcaster_thread_stack), broadcaster_t, NULL, NULL,
			NULL, 5, K_USER, K_NO_WAIT);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	broadcaster_cmd, SHELL_CMD(start, NULL, "Start ISO broadcaster.", iso_broadcaster_start),
	SHELL_CMD(stop, NULL, "Stop ISO broadcaster.", iso_broadcaster_stop),
	SHELL_CMD(cfg, NULL, "Print config.", broadcaster_print_cfg),
	SHELL_CMD_ARG(set, NULL, "set", set_param, 3, 0), SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(brcast_src, &broadcaster_cmd, "ISO Broadcast source commands", NULL);
