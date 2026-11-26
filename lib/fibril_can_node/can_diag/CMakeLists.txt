# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_FIBRIL_CAN_NODE_CAN_DIAG can_diag.cpp)

# The block types are fibril_can's, not this repository's: the bridge, the
# diagnostics node and every firmware share them, so the definition lives
# next to the ROS messages it maps onto. The module directory variable is set
# before any module's CMake runs; fibril_can's FCAN_CAN_DIAG_SCHEMA may not be
# yet.
set_property(GLOBAL APPEND PROPERTY fibril_can_node_type_schemas
             "${ZEPHYR_FIBRIL_CAN_MODULE_DIR}/zephyr/schema/can_diag.yaml")

# The transport, not a devicetree node, decides what there is to report on.
if(CONFIG_FIBRIL_CAN_NODE_CAN_DIAG)
  if(CONFIG_FCAN_TRANSPORT_HUB)
    # One port is the hub's external face, which has no controller of its own.
    math(EXPR peers_max "${CONFIG_FCAN_HUB_MAX_PORTS} - 1")
    fibril_can_node_builtin_instances(TYPE CanPortDiag NS "can{i}" MAX ${peers_max})
    fibril_can_node_builtin_instances(TYPE CanHubDiag NS "can_hub" MAX 1)
  else()
    fibril_can_node_builtin_instances(TYPE CanPortDiag NS "can{i}" MAX 1)
  endif()
endif()
