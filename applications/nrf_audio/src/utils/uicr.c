/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "uicr.h"

#include <stdint.h>
#include <errno.h>
#if defined(CONFIG_NRFX_NVMC)
#include <nrfx_nvmc.h>
#elif defined(CONFIG_NRFX_RRAMC)
#include <nrfx_rramc.h>
#endif /* CONFIG_NRFX_NVMC */

#if !defined(CONFIG_SOC_SERIES_BSIM_NRF53X)
/* Memory address to store segger number of the board */
#define MEM_ADDR_UICR_SNR UICR_APP_BASE_ADDR
/* Memory address to store the location intended to be used for this board */
#define MEM_ADDR_UICR_LOC (MEM_ADDR_UICR_SNR + sizeof(uint32_t))
#endif

uint32_t uicr_location_get(void)
{
#if defined(CONFIG_SOC_SERIES_BSIM_NRF53X)
	/* BabbleSim has no UICR to read a persisted location from. */
	return 0xFFFFFFFF;
#else
	return *(uint32_t *)MEM_ADDR_UICR_LOC;
#endif
}

int uicr_location_set(uint32_t location)
{
#if defined(CONFIG_SOC_SERIES_BSIM_NRF53X)
	(void)location;

	return -ENOTSUP;
#else
	if (location == *(uint32_t *)MEM_ADDR_UICR_LOC) {
		return 0;
	} else if (*(uint32_t *)MEM_ADDR_UICR_LOC != 0xFFFFFFFF) {
		return -EROFS;
	}

#if defined(CONFIG_NRFX_NVMC)
	nrfx_nvmc_word_write(MEM_ADDR_UICR_LOC, location);
	while (!nrfx_nvmc_write_done_check()) {
	}
#elif defined(CONFIG_NRFX_RRAMC)
	nrfx_rramc_word_write(MEM_ADDR_UICR_LOC, location);
#else
	return -ENOTSUP;
#endif /* CONFIG_NRFX_NVMC */

	if (location == *(uint32_t *)MEM_ADDR_UICR_LOC) {
		return 0;
	} else {
		return -EIO;
	}

#endif /* CONFIG_SOC_SERIES_BSIM_NRF53X */
}

uint64_t uicr_snr_get(void)
{
#if defined(CONFIG_SOC_SERIES_BSIM_NRF53X)
	return 0;
#else
	return *(uint64_t *)MEM_ADDR_UICR_SNR;
#endif
}
