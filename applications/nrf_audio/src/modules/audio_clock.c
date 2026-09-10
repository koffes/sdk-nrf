/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "audio_clock.h"

#include <errno.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#if !defined(CONFIG_SOC_SERIES_BSIM_NRF53X)
#include <nrfx_clock_hfclkaudio.h>
#endif

int audio_clock_set(uint16_t freq_value)
{
	freq_value = CLAMP(freq_value, APLL_FREQ_MIN, APLL_FREQ_MAX);

#if defined(CONFIG_SOC_SERIES_BSIM_NRF53X)
	/* BabbleSim models no APLL/HFCLKAUDIO peripheral; nothing to configure. */
	return 0;
#elif NRF_CLOCK_HAS_HFCLKAUDIO
	nrfx_clock_hfclkaudio_config_set(freq_value);

	return 0;
#else
	return -ENOTSUP;
#endif /* NRF_CLOCK_HAS_HFCLKAUDIO */
}

int audio_clock_init(void)
{
#if defined(CONFIG_SOC_SERIES_BSIM_NRF53X)
	return 0;
#elif NRF_CLOCK_HAS_HFCLKAUDIO
	int ret;

	ret = nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);
	if (ret) {
		return ret;
	}

	ret = audio_clock_set(APLL_FREQ_CENTER);
	if (ret) {
		return ret;
	}

	NRF_CLOCK->TASKS_HFCLKAUDIOSTART = 1;

	/* Wait for ACLK to start */
	while (!NRF_CLOCK_EVENT_HFCLKAUDIOSTARTED) {
		k_sleep(K_MSEC(1));
	}

	return 0;
#else
	return -ENOTSUP;
#endif /* NRF_CLOCK_HAS_HFCLKAUDIO */
}
