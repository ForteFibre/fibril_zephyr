# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_ENCODER_AMT21 amt21.c)
zephyr_library_sources_ifdef(CONFIG_ENCODER_AMT21_SHELL amt21_shell.c)

# CONFIG_ENCODER can be selected without any encoder present in the devicetree,
# which would otherwise leave this library without any source file.
zephyr_library_sources_ifndef(CONFIG_ENCODER_AMT21 ${ZEPHYR_BASE}/misc/empty_file.c)
