/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "audio_system.h"

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/gpio.h>
#include <nrfx_timer.h>
#include <stdio.h>

#include "macros_common.h"
#include "sw_codec_select.h"
#include "audio_datapath.h"
#include "audio_i2s.h"
#include "data_fifo.h"
#include "led.h"
#include "hw_codec.h"
#include "tone.h"
#include "contin_array.h"
#include "pcm_stream_channel_modifier.h"
#include "audio_usb.h"
#include "streamctrl.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(audio_system, CONFIG_AUDIO_SYSTEM_LOG_LEVEL);

#define FIFO_TX_BLOCK_COUNT (CONFIG_FIFO_FRAME_SPLIT_NUM * CONFIG_FIFO_TX_FRAME_COUNT)
#define FIFO_RX_BLOCK_COUNT (CONFIG_FIFO_FRAME_SPLIT_NUM * CONFIG_FIFO_RX_FRAME_COUNT)

#define DEBUG_INTERVAL_NUM 1000

K_THREAD_STACK_DEFINE(encoder_thread_stack, CONFIG_ENCODER_STACK_SIZE);

DATA_FIFO_DEFINE(fifo_tx, FIFO_TX_BLOCK_COUNT, WB_UP(BLOCK_SIZE_BYTES));
DATA_FIFO_DEFINE(fifo_rx, FIFO_RX_BLOCK_COUNT, WB_UP(BLOCK_SIZE_BYTES));

static struct k_thread encoder_thread_data;
static k_tid_t encoder_thread_id;

static struct sw_codec_config sw_codec_cfg;
/* Buffer which can hold max 1 period test tone at 1000 Hz */
static int16_t test_tone_buf[CONFIG_AUDIO_SAMPLE_RATE_HZ / 1000];
static size_t test_tone_size;

static void audio_gateway_configure(void)
{
	if (IS_ENABLED(CONFIG_SW_CODEC_LC3)) {
		sw_codec_cfg.sw_codec = SW_CODEC_LC3;
	} else {
		ERR_CHK_MSG(-EINVAL, "No codec selected");
	}

#if (CONFIG_STREAM_BIDIRECTIONAL)
	sw_codec_cfg.decoder.enabled = true;
	sw_codec_cfg.decoder.num_ch = SW_CODEC_MONO;
#endif /* (CONFIG_STREAM_BIDIRECTIONAL) */

	if (IS_ENABLED(CONFIG_SW_CODEC_LC3)) {
		sw_codec_cfg.encoder.bitrate = CONFIG_LC3_BITRATE;
	} else {
		ERR_CHK_MSG(-EINVAL, "No codec selected");
	}

	if (IS_ENABLED(CONFIG_MONO_TO_ALL_RECEIVERS)) {
		sw_codec_cfg.encoder.num_ch = SW_CODEC_MONO;
	} else {
		sw_codec_cfg.encoder.num_ch = SW_CODEC_STEREO;
	}

	sw_codec_cfg.encoder.enabled = true;
}

static void audio_headset_configure(void)
{
	if (IS_ENABLED(CONFIG_SW_CODEC_LC3)) {
		sw_codec_cfg.sw_codec = SW_CODEC_LC3;
	} else {
		ERR_CHK_MSG(-EINVAL, "No codec selected");
	}

#if (CONFIG_STREAM_BIDIRECTIONAL)
	sw_codec_cfg.encoder.enabled = true;
	sw_codec_cfg.encoder.num_ch = SW_CODEC_MONO;

	if (IS_ENABLED(CONFIG_SW_CODEC_LC3)) {
		sw_codec_cfg.encoder.bitrate = CONFIG_LC3_BITRATE;
	} else {
		ERR_CHK_MSG(-EINVAL, "No codec selected");
	}
#endif /* (CONFIG_STREAM_BIDIRECTIONAL) */

	sw_codec_cfg.decoder.num_ch = SW_CODEC_MONO;
	sw_codec_cfg.decoder.enabled = true;
}

static void encoder_thread(void *arg1, void *arg2, void *arg3)
{
	int ret;
	uint32_t blocks_alloced_num;
	uint32_t blocks_locked_num;

	int debug_trans_count = 0;
	size_t encoded_data_size = 0;

	void *tmp_pcm_raw_data[CONFIG_FIFO_FRAME_SPLIT_NUM];
	char pcm_raw_data[FRAME_SIZE_BYTES];

	static uint8_t *encoded_data;
	static size_t pcm_block_size;
	static uint32_t test_tone_finite_pos;

	while (1) {
		/* Get PCM data from I2S */
		/* Since one audio frame is divided into a number of
		 * blocks, we need to fetch the pointers to all of these
		 * blocks before copying it to a continuous area of memory
		 * before sending it to the encoder
		 */
		for (int i = 0; i < CONFIG_FIFO_FRAME_SPLIT_NUM; i++) {
			ret = data_fifo_pointer_last_filled_get(&fifo_rx, &tmp_pcm_raw_data[i],
								&pcm_block_size, K_FOREVER);
			ERR_CHK(ret);
			memcpy(pcm_raw_data + (i * BLOCK_SIZE_BYTES), tmp_pcm_raw_data[i],
			       pcm_block_size);

			data_fifo_block_free(&fifo_rx, &tmp_pcm_raw_data[i]);
		}

		if (sw_codec_cfg.encoder.enabled) {
			if (test_tone_size) {
				/* Test tone takes over audio stream */
				uint32_t num_bytes;
				char tmp[FRAME_SIZE_BYTES / 2];

				ret = contin_array_create(tmp, FRAME_SIZE_BYTES / 2, test_tone_buf,
							  test_tone_size, &test_tone_finite_pos);
				ERR_CHK(ret);

				ret = pscm_copy_pad(tmp, FRAME_SIZE_BYTES / 2,
						    CONFIG_AUDIO_BIT_DEPTH_BITS, pcm_raw_data,
						    &num_bytes);
				ERR_CHK(ret);
			}

			ret = sw_codec_encode(pcm_raw_data, FRAME_SIZE_BYTES, &encoded_data,
					      &encoded_data_size);

			ERR_CHK_MSG(ret, "Encode failed");
		}

		/* Print block usage */
		if (debug_trans_count == DEBUG_INTERVAL_NUM) {
			ret = data_fifo_num_used_get(&fifo_rx, &blocks_alloced_num,
						     &blocks_locked_num);
			ERR_CHK(ret);
			LOG_DBG(COLOR_CYAN "RX alloced: %d, locked: %d" COLOR_RESET,
				blocks_alloced_num, blocks_locked_num);
			debug_trans_count = 0;
		} else {
			debug_trans_count++;
		}

		if (sw_codec_cfg.encoder.enabled) {
			streamctrl_encoded_data_send(encoded_data, encoded_data_size,
						     sw_codec_cfg.encoder.num_ch);
		}
		STACK_USAGE_PRINT("encoder_thread", &encoder_thread_data);
	}
}

int audio_encode_test_tone_set(uint32_t freq)
{
	int ret;

	if (freq == 0) {
		test_tone_size = 0;
		return 0;
	}

	ret = tone_gen(test_tone_buf, &test_tone_size, freq, CONFIG_AUDIO_SAMPLE_RATE_HZ, 1);
	ERR_CHK(ret);

	if (test_tone_size > sizeof(test_tone_buf)) {
		return -ENOMEM;
	}

	return 0;
}

/* This function is only used on gateway using USB as audio source and bidirectional stream */
int audio_decode(void const *const encoded_data, size_t encoded_data_size, bool bad_frame)
{
	int ret;
	uint32_t blocks_alloced_num;
	uint32_t blocks_locked_num;
	static int debug_trans_count;
	static void *tmp_pcm_raw_data[CONFIG_FIFO_FRAME_SPLIT_NUM];
	static void *pcm_raw_data;
	size_t pcm_block_size;

	if (!sw_codec_cfg.initialized) {
		/* Throw away data */
		/* This can happen when using play/pause since there might be
		 * some packages left in the buffers
		 */
		LOG_DBG("Trying to decode while codec is not initialized");
		return -EPERM;
	}

	ret = data_fifo_num_used_get(&fifo_tx, &blocks_alloced_num, &blocks_locked_num);
	if (ret) {
		return ret;
	}

	uint8_t free_blocks_num = FIFO_TX_BLOCK_COUNT - blocks_locked_num;

	/* If not enough space for a full frame, remove oldest samples to make room */
	if (free_blocks_num < CONFIG_FIFO_FRAME_SPLIT_NUM) {
		void *old_data;
		size_t size;

		for (int i = 0; i < (CONFIG_FIFO_FRAME_SPLIT_NUM - free_blocks_num); i++) {
			ret = data_fifo_pointer_last_filled_get(&fifo_tx, &old_data, &size,
								K_NO_WAIT);
			if (ret == -ENOMSG) {
				/* If there are no more blocks in FIFO, break */
				break;
			}

			data_fifo_block_free(&fifo_tx, &old_data);
		}
	}

	for (int i = 0; i < CONFIG_FIFO_FRAME_SPLIT_NUM; i++) {
		ret = data_fifo_pointer_first_vacant_get(&fifo_tx, &tmp_pcm_raw_data[i], K_FOREVER);
		if (ret) {
			return ret;
		}
	}

	ret = sw_codec_decode(encoded_data, encoded_data_size, bad_frame, &pcm_raw_data,
			      &pcm_block_size);
	if (ret) {
		LOG_ERR("Failed to decode");
		return ret;
	}

	/* Split decoded frame into CONFIG_FIFO_FRAME_SPLIT_NUM blocks */
	for (int i = 0; i < CONFIG_FIFO_FRAME_SPLIT_NUM; i++) {
		memcpy(tmp_pcm_raw_data[i], (char *)pcm_raw_data + (i * (BLOCK_SIZE_BYTES)),
		       BLOCK_SIZE_BYTES);

		ret = data_fifo_block_lock(&fifo_tx, &tmp_pcm_raw_data[i], BLOCK_SIZE_BYTES);
		if (ret) {
			LOG_ERR("Failed to lock block");
			return ret;
		}
	}
	if (debug_trans_count == DEBUG_INTERVAL_NUM) {
		ret = data_fifo_num_used_get(&fifo_tx, &blocks_alloced_num, &blocks_locked_num);
		if (ret) {
			return ret;
		}
		LOG_DBG(COLOR_MAGENTA "TX alloced: %d, locked: %d" COLOR_RESET, blocks_alloced_num,
			blocks_locked_num);
		debug_trans_count = 0;
	} else {
		debug_trans_count++;
	}

	return 0;
}

const static struct device *gpio_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpio0));

static struct gpio_callback pwm_cb;
/*
static nrfx_timer_config_t cfg = {.frequency = NRF_TIMER_FREQ_1MHz,
				  .mode = NRF_TIMER_MODE_TIMER,
				  .bit_width = NRF_TIMER_BIT_WIDTH_32,
				  .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
				  .p_context = NULL};
*/

#define NUM_DELTAS 300
static const nrfx_timer_t pwm_timer = NRFX_TIMER_INSTANCE(1);
static uint32_t deltas[NUM_DELTAS];
static uint32_t counter;

static uint32_t captured_pwm_transition_ts;

static void print_results_worker(struct k_work *work)
{

	for (int i = 0; i < NUM_DELTAS; i = i + 10) {

		LOG_WRN("Result: %d %d %d %d %d %d %d %d %d %d", deltas[i + 0], deltas[i + 1],
			deltas[i + 2], deltas[i + 3], deltas[i + 4], deltas[i + 5], deltas[i + 6],
			deltas[i + 7], deltas[i + 8], deltas[i + 9]);
	}

	uint32_t *frame_start_ts_array;
	uint32_t num_timestamps = audio_i2s_frame_start_ts_array_get(&frame_start_ts_array);

	LOG_WRN("frame start ts array size %d:", num_timestamps);
	LOG_WRN("%d %d %d %d %d %d %d", frame_start_ts_array[0], frame_start_ts_array[1],
		frame_start_ts_array[2], frame_start_ts_array[3], frame_start_ts_array[4],
		frame_start_ts_array[5], frame_start_ts_array[6]);

	LOG_WRN("HW CODEC LATENCY us: %d", captured_pwm_transition_ts - frame_start_ts_array[2]);
}

K_WORK_DEFINE(print_results, print_results_worker);

ISR_DIRECT_DECLARE(pin_isr)
{

	static bool first = true;
	if (first) {
		// LOG_WRN("direct ISR");
		first = false;
	}

	ISR_DIRECT_PM(); /* PM done after servicing interrupt for best latency */
	return 1;	 /* We should check if scheduling decision should be made */
}

static void pwm_int_handler(const struct device *gpio_dev, struct gpio_callback *cb, uint32_t pins)
{
	int ret;
	static uint32_t last_ts;
	uint32_t delta;
	uint32_t ts = nrfx_timer_capture(&pwm_timer, 0);
	delta = ts - last_ts;
	static bool captured = false;

	if (counter < ARRAY_SIZE(deltas)) {
		deltas[counter] = delta;
	}

	if ((delta < 13 && delta > 1) || (delta > 19 && delta < 35)) {
		if (captured == false) {
			LOG_WRN("Captured time out: %d", ts);
			captured = true;
			captured_pwm_transition_ts = ts;
		}
	}

	if (counter == NUM_DELTAS) {
		k_work_submit(&print_results);
	}

	/*
		if ((counter % 50) == 0) {
			ret = gpio_pin_set_raw(gpio_dev, 4, 1);
			if (ret) {
				LOG_ERR("Failed to set gpio state, ret: %d", ret);
				return;
			}
		} else if ((counter % 50) == 1) {
			ret = gpio_pin_set_raw(gpio_dev, 4, 0);
			if (ret) {
				LOG_ERR("Failed to set gpio state, ret: %d", ret);
				return;
			}
		}
	*/
	// Set pin to get timings

	counter++;
	last_ts = ts;
}

static void event_handler(nrf_timer_event_t event_type, void *ctx)
{
}

#define MY_DEV_IRQ  GPIOTE1_IRQn
#define MY_DEV_PRIO 2 /* device uses interrupt priority 2 */
/* argument passed to my_isr(), in this case a pointer to the device */

static int pwm_detection_start()
{

	/*
	ret = nrfx_timer_init(&pwm_timer, &cfg, event_handler);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx timer init error - Return value: %d", ret - NRFX_ERROR_BASE_NUM);
		return ret;
	}

	nrfx_timer_enable(&pwm_timer);
*/

	/* // This hangs for some reason
		IRQ_DIRECT_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_GPIOTE), IRQ_PRIO_LOWEST, pin_isr,
				   0); // IRQ_ZERO_LATENCY can be added
		irq_enable(NRFX_IRQ_NUMBER_GET(NRF_GPIOTE));
	*/

	gpio_port_pins_t pin_mask = BIT(20);
	gpio_pin_configure(gpio_dev, 4, GPIO_OUTPUT);
	gpio_pin_configure(gpio_dev, 20, GPIO_INPUT);
	gpio_init_callback(&pwm_cb, pwm_int_handler, pin_mask);
	gpio_pin_set_raw(gpio_dev, 4, 0);

	return 0;
}

/* Called once when the first I2S block has been sent*/
static void first_blck_tx_cb(void)
{
	int ret;
	LOG_WRN("First TX done");
	ret = gpio_add_callback(gpio_dev, &pwm_cb);
	if (ret) {
		LOG_ERR("error on the add callback");
	}

	ret = gpio_pin_interrupt_configure(gpio_dev, 20, GPIO_INT_EDGE_FALLING);
	if (ret) {
		LOG_ERR("error on the add callback");
	}
}

/**@brief Initializes the FIFOs, the codec, and starts the I2S
 */
void audio_system_start(void)
{
	int ret;

	if (CONFIG_AUDIO_DEV == HEADSET) {
		audio_headset_configure();
	} else if (CONFIG_AUDIO_DEV == GATEWAY) {
		audio_gateway_configure();
	} else {
		LOG_ERR("Invalid CONFIG_AUDIO_DEV: %d", CONFIG_AUDIO_DEV);
		ERR_CHK(-EINVAL);
	}

	if (!fifo_tx.initialized) {
		ret = data_fifo_init(&fifo_tx);
		ERR_CHK_MSG(ret, "Failed to set up tx FIFO");
	}

	if (!fifo_rx.initialized) {
		ret = data_fifo_init(&fifo_rx);
		ERR_CHK_MSG(ret, "Failed to set up rx FIFO");
	}

	ret = sw_codec_init(sw_codec_cfg);
	ERR_CHK_MSG(ret, "Failed to set up codec");

	sw_codec_cfg.initialized = true;

	if (sw_codec_cfg.encoder.enabled && encoder_thread_id == NULL) {
		encoder_thread_id = k_thread_create(
			&encoder_thread_data, encoder_thread_stack, CONFIG_ENCODER_STACK_SIZE,
			(k_thread_entry_t)encoder_thread, NULL, NULL, NULL,
			K_PRIO_PREEMPT(CONFIG_ENCODER_THREAD_PRIO), 0, K_NO_WAIT);
		ret = k_thread_name_set(encoder_thread_id, "ENCODER");
		ERR_CHK(ret);
	}

#if ((CONFIG_AUDIO_SOURCE_USB) && (CONFIG_AUDIO_DEV == GATEWAY))
	ret = audio_usb_start(&fifo_tx, &fifo_rx);
	ERR_CHK(ret);
#else
	ret = hw_codec_default_conf_enable();
	ERR_CHK(ret);

#if (CONFIG_AUDIO_DEV == HEADSET) /* TODO: must be removed */

	ret = pwm_detection_start();
	ERR_CHK(ret);

	uint32_t ts;
	ret = audio_datapath_play_square_i2s_ts_get(&ts, first_blck_tx_cb);
	ERR_CHK(ret);
	LOG_WRN("Ts is %d", ts);

#endif

	ret = audio_datapath_start(&fifo_rx);
	ERR_CHK(ret);
#endif /* ((CONFIG_AUDIO_SOURCE_USB) && (CONFIG_AUDIO_DEV == GATEWAY))) */
}

void audio_system_stop(void)
{
	int ret;

	if (!sw_codec_cfg.initialized) {
		LOG_WRN("Codec already unitialized");
		return;
	}

	LOG_DBG("Stopping codec");

#if ((CONFIG_AUDIO_DEV == GATEWAY) && CONFIG_AUDIO_SOURCE_USB)
	audio_usb_stop();
#else
	ret = hw_codec_soft_reset();
	ERR_CHK(ret);

	ret = audio_datapath_stop();
	ERR_CHK(ret);
#endif /* ((CONFIG_AUDIO_DEV == GATEWAY) && CONFIG_AUDIO_SOURCE_USB) */

	ret = sw_codec_uninit(sw_codec_cfg);
	ERR_CHK_MSG(ret, "Failed to uninit codec");
	sw_codec_cfg.initialized = false;

	data_fifo_empty(&fifo_rx);
	data_fifo_empty(&fifo_tx);
}

int audio_system_fifo_rx_block_drop(void)
{
	int ret;
	void *temp;
	size_t temp_size;

	ret = data_fifo_pointer_last_filled_get(&fifo_rx, &temp, &temp_size, K_NO_WAIT);
	if (ret) {
		LOG_WRN("Failed to get last filled block");
		return -ECANCELED;
	}

	data_fifo_block_free(&fifo_rx, &temp);

	LOG_DBG("Block dropped");
	return 0;
}

void audio_system_init(void)
{
	int ret;

#if ((CONFIG_AUDIO_DEV == GATEWAY) && (CONFIG_AUDIO_SOURCE_USB))
	ret = audio_usb_init();
	ERR_CHK(ret);
#else
	ret = audio_datapath_init();
	ERR_CHK(ret);
	ret = hw_codec_init();
	ERR_CHK(ret);
#endif
}

static int cmd_audio_system_start(const struct shell *shell, size_t argc, const char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	audio_system_start();

	shell_print(shell, "Audio system started");

	return 0;
}

static int cmd_audio_system_stop(const struct shell *shell, size_t argc, const char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	audio_system_stop();

	shell_print(shell, "Audio system stopped");

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(audio_system_cmd,
			       SHELL_COND_CMD(CONFIG_SHELL, start, NULL, "Start the audio system",
					      cmd_audio_system_start),
			       SHELL_COND_CMD(CONFIG_SHELL, stop, NULL, "Stop the audio system",
					      cmd_audio_system_stop),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(audio_system, &audio_system_cmd, "Audio system commands", NULL);
