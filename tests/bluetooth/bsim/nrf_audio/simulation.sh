#!/usr/bin/env bash
# Copyright 2026 Nordic Semiconductor ASA
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause

BOARD=nrf5340bsim/nrf5340/cpuapp
set -ue

: "${ZEPHYR_BASE:?ZEPHYR_BASE must be set to point to the zephyr root directory}"

SIM_ID="${SIM_ID:-nrf_audio_broadcast}"
VERBOSITY="${VERBOSITY:-2}"
SIM_LENGTH_US="${SIM_LENGTH_US:-60e6}"

source ${ZEPHYR_BASE}/tests/bsim/sh_common.source
cd ${BSIM_OUT_PATH}/bin

Execute ./bs_nrf5340bsim_nrf5340_cpuapp____nrf_applications_nrf_audio_prj_conf_overlay-broadcast_source_conf \
  -v=${VERBOSITY} -s=${SIM_ID} -d=0 -RealEncryption=1

Execute ./bs_nrf5340bsim_nrf5340_cpuapp____nrf_applications_nrf_audio_prj_conf_overlay-broadcast_sink_conf \
  -v=${VERBOSITY} -s=${SIM_ID} -d=1 -RealEncryption=1

Execute ./bs_2G4_phy_v1 -v=${VERBOSITY} -s=${SIM_ID} -D=2 -sim_length=${SIM_LENGTH_US}

wait_for_background_jobs # Wait for all programs in background and return != 0 if any fails
