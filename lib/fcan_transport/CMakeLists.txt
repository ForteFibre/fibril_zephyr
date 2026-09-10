# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

# Exactly one backend is built. The two Kconfig symbols are mutually exclusive
# by construction, so an application never has to say which it wants.
zephyr_library_sources_ifdef(CONFIG_FCAN_TRANSPORT_DIRECT direct.c)
zephyr_library_sources_ifdef(CONFIG_FCAN_TRANSPORT_HUB hub.c)

# Orthogonal to the backend: it dresses whichever external port exists.
zephyr_library_sources_ifdef(CONFIG_FCAN_TRANSPORT_GS_USB gs_usb.c)
