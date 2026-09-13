# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0
#
# The fibril_can slave application. One image serves every board that is a
# slave on the bus: which block types it carries comes from the node schema a
# snippet selects, and which hardware they drive comes from the devicetree.

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(fibril_node LANGUAGES C)

if(NOT ZEPHYR_FIBRIL_CAN_MODULE_DIR)
  message(FATAL_ERROR
    "ZEPHYR_FIBRIL_CAN_MODULE_DIR is not set — is the fibril_can module on "
    "the west manifest and is CONFIG_FIBRIL_CAN=y?")
endif()

if(NOT CONFIG_FIBRIL_NODE_SCHEMA)
  message(FATAL_ERROR
    "CONFIG_FIBRIL_NODE_SCHEMA is empty. Select a node with a snippet, e.g. "
    "`west build -b <board> apps/node -S rc26-air`.")
endif()

include(${ZEPHYR_FIBRIL_CAN_MODULE_DIR}/zephyr/cmake/fcan_invoke_codegen.cmake)

# Kconfig, not a CMake argument: the schema belongs to the same per-robot
# choice as the overlay and the node id, and a snippet can only append
# Kconfig and devicetree fragments.
set(NODE_SCHEMA "${ZEPHYR_FIBRIL_ZEPHYR_MODULE_DIR}/${CONFIG_FIBRIL_NODE_SCHEMA}")

if(NOT EXISTS "${NODE_SCHEMA}")
  message(FATAL_ERROR "CONFIG_FIBRIL_NODE_SCHEMA points at a missing file: ${NODE_SCHEMA}")
endif()

# Every block type this repository implements is offered to the codegen, not
# just the ones this node uses: an unreferenced type is dropped before the
# blob is built, so the list costs nothing and no one has to curate it. What
# the image carries is decided by the node schema's `instances:` alone.
get_property(FCAN_TYPE_SCHEMAS GLOBAL PROPERTY fibril_can_node_type_schemas)

fcan_invoke_codegen(
  SCHEMA     "${NODE_SCHEMA}" ${FCAN_TYPE_SCHEMAS}
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/schema_gen"
  BASE_NAME  schema_gen)

# Backing section for FIBRIL_FCAN_FUNC_DEFINE (include/fibril_can_node/func.h).
# Both calls are needed and neither covers the other: the .ld fragment feeds
# the linker template that ARM builds actually use, and zephyr_iterable_section
# feeds the CMake linker generator. Declaring only the second one links with
# the list symbols undefined.
zephyr_linker_sources(DATA_SECTIONS sections-ram.ld)
zephyr_iterable_section(NAME fibril_fcan_func GROUP DATA_REGION ${XIP_ALIGN_WITH_INPUT})

target_sources(app PRIVATE src/main.c)
