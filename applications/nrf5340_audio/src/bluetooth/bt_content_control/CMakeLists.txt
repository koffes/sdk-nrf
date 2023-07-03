#
# Copyright (c) 2023 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

zephyr_library_include_directories(
    content
)

target_sources(app PRIVATE
	       ${CMAKE_CURRENT_SOURCE_DIR}/bt_cont_ctrl.c
	       ${CMAKE_CURRENT_SOURCE_DIR}/content/ble_audio_services.c
)



#TODO: Notes for PR: Should we have subfolders for e.g. volume and mcs?
# Should the server and clients be in the same files?
#
#ZBUS_CHAN_DEFINE(volume_chan, struct volume_msg, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
#		 ZBUS_MSG_INIT(0)); should be rend_volume_chan