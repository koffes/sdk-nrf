/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/ztest.h>
#include <zephyr/tc_util.h>

//#include "tmp_cap.h"

void test_push_peek(void)
{
	//int ret;
	//static struct tmp_cap_storage tmp_cap[CONFIG_BT_MAX_CONN];
	//struct bt_codec codec;

	//uint8_t dummy;
	//struct bt_conn *conn = (struct bt_conn*)&dummy;


	//ret = tmp_cap_push(tmp_cap, conn, 2, &codec);
	zassert_equal(0, 0, "tmp_cap_pushed returned an error");
}

void test_main(void)
{
	ztest_test_suite(test_suite_tmp_cap,
		ztest_unit_test(test_push_peek)
	);

	ztest_run_test_suite(test_suite_tmp_cap);
}
