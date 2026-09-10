/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "current_time.h"
#include "current_time_grtc.h"
#include "audio_sync_timer.h"

#if defined(CONFIG_SOC_SERIES_BSIM_NRF53X)
#include <nsi_hw_scheduler.h>
#endif

uint32_t current_time_us_get(void)
{
#if defined(CONFIG_HAS_HW_NRF_GRTC)
	return (uint32_t)current_time_grtc_us_get();
#elif defined(CONFIG_AUDIO_SYNC_TIMER_USES_RTC)
	return audio_sync_timer_capture();
#elif defined(CONFIG_SOC_SERIES_BSIM_NRF53X)
	return (uint32_t)nsi_hws_get_time();
#else
#error "No audio time source defined"
#endif
}
