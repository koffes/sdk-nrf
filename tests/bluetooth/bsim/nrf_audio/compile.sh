#!/usr/bin/env bash
# Copyright 2024 Nordic Semiconductor ASA
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause

BOARD=nrf5340bsim/nrf5340/cpuapp
set -ue

: "${ZEPHYR_BASE:?ZEPHYR_BASE must be set to point to the zephyr root directory}"

source ${ZEPHYR_BASE}/tests/bsim/compile.source
app=${ZEPHYR_NRF_MODULE_DIR}applications/nrf_audio conf_overlay=/home/krs1-wsl/ncs/main/nrf/applications/nrf_audio/broadcast_source/overlay-broadcast_source.conf \
cmake_extra_args="-Dnrf_audio_CONFIG_SW_CODEC_LC3_GOOGLE=y -Dnrf_audio_CONFIG_AUDIO_I2S_FILE_BACKEND=y -DCONFIG_AUDIO_SOURCE_I2S=y" \
sysbuild=1 compile

app=${ZEPHYR_NRF_MODULE_DIR}applications/nrf_audio conf_overlay=/home/krs1-wsl/ncs/main/nrf/applications/nrf_audio/broadcast_sink/overlay-broadcast_sink.conf \
cmake_extra_args="-Dnrf_audio_CONFIG_SW_CODEC_LC3_GOOGLE=y -Dnrf_audio_CONFIG_AUDIO_I2S_FILE_BACKEND=y -DCONFIG_AUDIO_SOURCE_I2S=y" \
sysbuild=1 compile

wait_for_background_jobs
