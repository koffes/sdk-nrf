#
# Copyright (c) 2023 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

target_sources(app PRIVATE
	       ${CMAKE_CURRENT_SOURCE_DIR}/bt_mgmt.c
)

if (CONFIG_BT_CENTRAL OR CONFIG_BT_OBSERVER)
	target_sources(app PRIVATE
	       ${CMAKE_CURRENT_SOURCE_DIR}/bt_mgmt_scan.c)
endif()

if (CONFIG_BT_PERIPHERAL OR CONFIG_BT_BROADCASTER)
	target_sources(app PRIVATE
	       ${CMAKE_CURRENT_SOURCE_DIR}/bt_mgmt_adv.c)
endif()