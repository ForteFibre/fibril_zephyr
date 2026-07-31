# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

# No encoder driver selected yet, which would otherwise leave this library
# without any source file.
zephyr_library_sources(${ZEPHYR_BASE}/misc/empty_file.c)
