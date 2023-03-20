#
# Copyright (c) 2022 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

#cmake_minimum_required(VERSION 3.20.0)
#
#find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
#project(tmp_cap)
#
#FILE(GLOB app_sources src/*.c)
##target_sources(app PRIVATE ${ZEPHYR_NRF_MODULE_DIR}/applications/nrf5340_audio/src/bluetooth/tmp_cap.c)
##${ZEPHYR_NRF_MODULE_DIR}/applications/nrf5340_audio/src/utils/macros/
#
##target_include_directories(app
##  PRIVATE
##  ${ZEPHYR_NRF_MODULE_DIR}/applications/nrf5340_audio/src/bluetooth/
##  )
#
#target_sources(app
#  PRIVATE main.c
#  )


cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(NONE)

target_sources(app
  PRIVATE main.c
  )

target_include_directories(app
  PRIVATE
  ${ZEPHYR_NRF_MODULE_DIR}/applications/nrf5340_audio/src/utils/macros/
  )
