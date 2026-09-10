# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

include(${CMAKE_CURRENT_SOURCE_DIR}/instance_schema.cmake)

add_subdirectory(solenoid)

# Each block type contributes two things on global properties, which the
# application reads and hands to the codegen: its type.yaml, and the
# `instances:` fragment derived from its devicetree node. The devicetree is
# then the only place that decides which functions this image carries.
get_property(_types GLOBAL PROPERTY fibril_can_node_type_schemas)
get_property(_instances GLOBAL PROPERTY fibril_can_node_instance_schemas)
message(STATUS "[fibril_can_node] block types: ${_types}")
message(STATUS "[fibril_can_node] instances:   ${_instances}")
