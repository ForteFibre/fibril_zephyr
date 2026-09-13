# Copyright (c) 2024 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_MOTOR_DJI_ROBOMASTER robomaster.c)
zephyr_library_sources_ifdef(CONFIG_MOTOR_ROBSTRIDE robstride.c)
