# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_FIBRIL_CAN_NODE_ROBSTRIDE robstride.cpp)

# The block type definition travels with its implementation: adding a topic
# changes both files, and separating them invites changing only one.
#
# Registered unconditionally, unlike the instances below: the application
# offers every implemented type to the codegen and lets it drop the ones no
# instance references.
set_property(GLOBAL APPEND PROPERTY fibril_can_node_type_schemas
             "${CMAKE_CURRENT_SOURCE_DIR}/type.yaml")

# Guarded because MAX reads a Kconfig that only exists with the function
# enabled, and the helper refuses an empty argument.
if(CONFIG_FIBRIL_CAN_NODE_ROBSTRIDE)
  fibril_can_node_instances(
    TYPE       RobstrideMotor
    COMPATIBLE "fibril,fcan-robstride"
    MAX        ${CONFIG_FIBRIL_CAN_NODE_ROBSTRIDE_MAX})
endif()
