# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_FIBRIL_CAN_NODE_ODOMETRY odometry.cpp)

# Registered unconditionally, like every block type: the application offers
# them all to the codegen and lets it drop the ones no instance references.
set_property(GLOBAL APPEND PROPERTY fibril_can_node_type_schemas
             "${CMAKE_CURRENT_SOURCE_DIR}/type.yaml")

# One instance per node: the topics carry exactly the two encoders and the
# gyro a dead reckoning reads, so there is nothing to have a second of.
if(CONFIG_FIBRIL_CAN_NODE_ODOMETRY)
  fibril_can_node_instances(
    TYPE       Odometry
    COMPATIBLE "fibril,fcan-odometry"
    MAX        1)
endif()
