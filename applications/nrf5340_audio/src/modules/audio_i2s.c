/*
 *  Copyright (c) 2021, PACKETCRAFT, INC.
 *
 *  SPDX-License-Identifier: LicenseRef-PCFT
 */

#include "audio_i2s.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <nrfx_i2s.h>
#include <nrfx_clock.h>
#include <zephyr/drivers/gpio.h>

#include "audio_sync_timer.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(audio_i2s, 4);

#define I2S_NL DT_NODELABEL(i2s0)

#define HFCLKAUDIO_12_288_MHZ 0x9BAE

enum audio_i2s_state {
	AUDIO_I2S_STATE_UNINIT,
	AUDIO_I2S_STATE_IDLE,
	AUDIO_I2S_STATE_STARTED,
};

static enum audio_i2s_state state = AUDIO_I2S_STATE_UNINIT;

PINCTRL_DT_DEFINE(I2S_NL);

#if CONFIG_AUDIO_SAMPLE_RATE_16000_HZ
#define CONFIG_AUDIO_RATIO NRF_I2S_RATIO_384X
#elif CONFIG_AUDIO_SAMPLE_RATE_24000_HZ
#define CONFIG_AUDIO_RATIO NRF_I2S_RATIO_256X
#elif CONFIG_AUDIO_SAMPLE_RATE_48000_HZ
#define CONFIG_AUDIO_RATIO NRF_I2S_RATIO_128X
#else
#error "Current AUDIO_SAMPLE_RATE_HZ setting not supported"
#endif

static nrfx_i2s_t i2s_inst = NRFX_I2S_INSTANCE(0);

static nrfx_i2s_config_t cfg = {
	/* Pins are configured by pinctrl. */
	.skip_gpio_cfg = true,
	.skip_psel_cfg = true,
	.irq_priority = DT_IRQ(I2S_NL, priority),
	.mode = NRF_I2S_MODE_MASTER,
	.format = NRF_I2S_FORMAT_I2S,
	.alignment = NRF_I2S_ALIGN_LEFT,
	.ratio = CONFIG_AUDIO_RATIO,
	.mck_setup = 0x66666000,
#if (CONFIG_AUDIO_BIT_DEPTH_16)
	.sample_width = NRF_I2S_SWIDTH_16BIT,
#elif (CONFIG_AUDIO_BIT_DEPTH_32)
	.sample_width = NRF_I2S_SWIDTH_32BIT,
#else
#error Invalid bit depth selected
#endif /* (CONFIG_AUDIO_BIT_DEPTH_16) */
	.channels = NRF_I2S_CHANNELS_STEREO,
	.clksrc = NRF_I2S_CLKSRC_ACLK,
	.enable_bypass = false,
};

static i2s_blk_comp_callback_t i2s_blk_comp_callback;

static uint32_t global_fs;
uint32_t fsync_ret_ts_buf[10];
uint32_t frame_start_ts_buf[10];

#if (CONFIG_AUDIO_DEV == HEADSET) /* TODO: must be removed */
static uint32_t iter;
const static struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));

static void fsync_pin_int_handler(const struct device *gpio_port, struct gpio_callback *cb,
				  uint32_t pins)
{
	if (pins != BIT(16)) {
		return;
	}

	int ret = gpio_pin_set_raw(gpio_dev, 7, 1);

	if (ret) {
		LOG_ERR("Failed to set gpio state, ret: %d", ret);
	}

	ret = gpio_pin_set_raw(gpio_dev, 7, 0);

	if (ret) {
		LOG_ERR("Failed to set gpio state, ret: %d", ret);
	}
}

static void cs47l63_gpio9_pin_int_handler(const struct device *gpio_port, struct gpio_callback *cb,
					  uint32_t pins)
{
	if (pins != BIT(20)) {
		return;
	}

	int ret = gpio_pin_set_raw(gpio_dev, 25, 1);

	if (ret) {
		LOG_ERR("Failed to set gpio state, ret: %d", ret);
	}

	ret = gpio_pin_set_raw(gpio_dev, 25, 0);

	if (ret) {
		LOG_ERR("Failed to set gpio state, ret: %d", ret);
	}

	if (iter <= 9) {
		frame_start_ts_buf[iter] = global_fs;
		fsync_ret_ts_buf[iter] = audio_sync_timer_capture();
	}

	if (iter == 9) {
		/* unsigned int key = irq_lock(); */
		/* irq_unlock(key); */

		LOG_INF("diff %d %d %d", fsync_ret_ts_buf[0] - frame_start_ts_buf[0],
			fsync_ret_ts_buf[1] - frame_start_ts_buf[1],
			fsync_ret_ts_buf[2] - frame_start_ts_buf[2]);
		LOG_INF("fsync_ret_ts_buf[1] , frame_start_ts_buf[1] %d %d", fsync_ret_ts_buf[1],
			frame_start_ts_buf[1]);
		LOG_INF("fsync_ret_ts_buf[2] , frame_start_ts_buf[2] %d %d", fsync_ret_ts_buf[2],
			frame_start_ts_buf[2]);
		/* gpio_pin_interrupt_configure(gpio_dev, 20, GPIO_INT_DISABLE); */
	}
	iter++;
	if (iter >= 48000) {
		iter = 0;
		LOG_WRN("iter reset");
	}
}
#endif

static void i2s_comp_handler(nrfx_i2s_buffers_t const *released_bufs, uint32_t status)
{
	uint32_t frame_start_ts = audio_sync_timer_capture_get();
	global_fs = audio_sync_timer_capture(); /* This cannot be used as this is called every ms*/

	if ((status == NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED) && released_bufs &&
	    i2s_blk_comp_callback && (released_bufs->p_rx_buffer || released_bufs->p_tx_buffer)) {
		i2s_blk_comp_callback(frame_start_ts, released_bufs->p_rx_buffer,
				      released_bufs->p_tx_buffer);
	}
}

void audio_i2s_set_next_buf(const uint8_t *tx_buf, uint32_t *rx_buf)
{
	__ASSERT_NO_MSG(state == AUDIO_I2S_STATE_STARTED);
	if (IS_ENABLED(CONFIG_STREAM_BIDIRECTIONAL) || (CONFIG_AUDIO_DEV == GATEWAY)) {
		__ASSERT_NO_MSG(rx_buf != NULL);
	}

	if (IS_ENABLED(CONFIG_STREAM_BIDIRECTIONAL) || (CONFIG_AUDIO_DEV == HEADSET)) {
		__ASSERT_NO_MSG(tx_buf != NULL);
	}

	const nrfx_i2s_buffers_t i2s_buf = {.p_rx_buffer = rx_buf,
					    .p_tx_buffer = (uint32_t *)tx_buf};

	nrfx_err_t ret;

	ret = nrfx_i2s_next_buffers_set(&i2s_inst, &i2s_buf);
	__ASSERT_NO_MSG(ret == NRFX_SUCCESS);
}

void audio_i2s_start(const uint8_t *tx_buf, uint32_t *rx_buf)
{
	__ASSERT_NO_MSG(state == AUDIO_I2S_STATE_IDLE);
	if (IS_ENABLED(CONFIG_STREAM_BIDIRECTIONAL) || (CONFIG_AUDIO_DEV == GATEWAY)) {
		__ASSERT_NO_MSG(rx_buf != NULL);
	}

	if (IS_ENABLED(CONFIG_STREAM_BIDIRECTIONAL) || (CONFIG_AUDIO_DEV == HEADSET)) {
		__ASSERT_NO_MSG(tx_buf != NULL);
	}

	const nrfx_i2s_buffers_t i2s_buf = {.p_rx_buffer = rx_buf,
					    .p_tx_buffer = (uint32_t *)tx_buf};

	nrfx_err_t ret;

	/* Buffer size in 32-bit words */
	ret = nrfx_i2s_start(&i2s_inst, &i2s_buf, I2S_SAMPLES_NUM, 0);
	__ASSERT_NO_MSG(ret == NRFX_SUCCESS);

	state = AUDIO_I2S_STATE_STARTED;
}

void audio_i2s_stop(void)
{
	__ASSERT_NO_MSG(state == AUDIO_I2S_STATE_STARTED);

	nrfx_i2s_stop(&i2s_inst);

	state = AUDIO_I2S_STATE_IDLE;
}

void audio_i2s_blk_comp_cb_register(i2s_blk_comp_callback_t blk_comp_callback)
{
	i2s_blk_comp_callback = blk_comp_callback;
}

#if (CONFIG_AUDIO_DEV == HEADSET) /* TODO: must be removed */
static struct gpio_callback gpio_cb;
static struct gpio_callback gpio_cb_2;
#endif

void audio_i2s_init(void)
{
	__ASSERT_NO_MSG(state == AUDIO_I2S_STATE_UNINIT);

	nrfx_err_t ret;

	nrfx_clock_hfclkaudio_config_set(HFCLKAUDIO_12_288_MHZ);

	NRF_CLOCK->TASKS_HFCLKAUDIOSTART = 1;

	/* Wait for ACLK to start */
	while (!NRF_CLOCK_EVENT_HFCLKAUDIOSTARTED) {
		k_sleep(K_MSEC(1));
	}

	ret = pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(I2S_NL), PINCTRL_STATE_DEFAULT);
	__ASSERT_NO_MSG(ret == 0);

	IRQ_CONNECT(DT_IRQN(I2S_NL), DT_IRQ(I2S_NL, priority), nrfx_isr, nrfx_i2s_0_irq_handler, 0);
	irq_enable(DT_IRQN(I2S_NL));

	ret = nrfx_i2s_init(&i2s_inst, &cfg, i2s_comp_handler);
	__ASSERT_NO_MSG(ret == NRFX_SUCCESS);

	state = AUDIO_I2S_STATE_IDLE;

#if (CONFIG_AUDIO_DEV == HEADSET) /* TODO: must be removed */

	gpio_init_callback(&gpio_cb, cs47l63_gpio9_pin_int_handler, BIT(20));
	ret = gpio_add_callback(gpio_dev, &gpio_cb);
	if (ret) {
		LOG_ERR("fail");
	}
	ret = gpio_pin_interrupt_configure(gpio_dev, 20, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		LOG_ERR("fail");
	}

	/* gpio_init_callback(&gpio_cb_2, fsync_pin_int_handler, BIT(16));
	 *
	 * ret = gpio_add_callback(gpio_dev, &gpio_cb_2);
	 * if (ret) {
	 *	LOG_ERR("fail");
	 *}
	 *
	 * ret = gpio_pin_interrupt_configure(gpio_dev, 16, GPIO_INT_EDGE_TO_ACTIVE);
	 * if (ret) {
	 *	LOG_ERR("fail");
	 *}
	 */

#endif
}
