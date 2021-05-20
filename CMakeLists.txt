# Copyright (c) 2021, ATL Electronics
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_FAMILY_GD32)

	zephyr_library()

	string(TOUPPER ${CONFIG_SOC_SUB_SERIES} GD32_SERIE)
	zephyr_compile_definitions(${CONFIG_SOC_PART_NUMBER})

	add_subdirectory_ifdef(CONFIG_SOC_SERIES_GD32E10X GD32E10X)
	add_subdirectory_ifdef(CONFIG_SOC_SERIES_GD32F403 GD32F403)
	add_subdirectory_ifdef(CONFIG_SOC_SERIES_GD32VF103 GD32VF103)

endif()
