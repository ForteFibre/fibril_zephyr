# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_CUSTOM custom)
add_subdirectory_ifdef(CONFIG_FIBRIL_CAN_NODE fibril_can_node)
