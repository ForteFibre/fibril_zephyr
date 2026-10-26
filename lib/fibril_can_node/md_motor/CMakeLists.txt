# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_FIBRIL_CAN_NODE_MD_MOTOR md_motor.cpp)

# Registered unconditionally, like every block type: the application offers
# them all to the codegen and lets it drop the ones no instance references.
set_property(GLOBAL APPEND PROPERTY fibril_can_node_type_schemas
             "${CMAKE_CURRENT_SOURCE_DIR}/type.yaml")

if(CONFIG_FIBRIL_CAN_NODE_MD_MOTOR)
  fibril_can_node_instances(
    TYPE       MdMotor
    COMPATIBLE "fibril,fcan-md-motor"
    MAX        ${CONFIG_FIBRIL_CAN_NODE_MD_MOTOR_MAX})
endif()
