# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_FIBRIL_CAN_NODE_SOLENOID solenoid.c)

if(CONFIG_FIBRIL_CAN_NODE_SOLENOID)
  # The block type definition travels with its implementation: adding a topic
  # changes both files, and separating them invites changing only one.
  set_property(GLOBAL APPEND PROPERTY fibril_can_node_type_schemas
               "${CMAKE_CURRENT_SOURCE_DIR}/type.yaml")

  fibril_can_node_instances(
    TYPE       Solenoid
    COMPATIBLE "fibril,fcan-solenoid"
    MAX        ${CONFIG_FIBRIL_CAN_NODE_SOLENOID_MAX})
endif()
