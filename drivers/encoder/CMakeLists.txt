# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

# Shared by every encoder driver, and the only source this library is
# guaranteed to have: CONFIG_ENCODER can be selected without any encoder present
# in the devicetree.
zephyr_library_sources(encoder_accum.c)

zephyr_library_sources_ifdef(CONFIG_ENCODER_AMT21 amt21.c)
zephyr_library_sources_ifdef(CONFIG_ENCODER_AMT21_SHELL amt21_shell.c)
zephyr_library_sources_ifdef(CONFIG_ENCODER_QDEC_STM32 qdec_stm32.c)
