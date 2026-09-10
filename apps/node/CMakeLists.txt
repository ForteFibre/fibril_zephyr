# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0
#
# The fibril_can slave application. One image serves every board that is a
# slave on the bus: the devicetree says which functions it carries and what
# hardware they drive, and a hand-written header names the node.

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
    "CONFIG_FIBRIL_NODE_SCHEMA is empty. A build needs a transport snippet and "
    "a deployment snippet, e.g. "
    "`west build -b <board> apps/node -S rc26-mainair-usb -S rc26-air`.")
endif()

include(${ZEPHYR_FIBRIL_CAN_MODULE_DIR}/zephyr/cmake/fcan_invoke_codegen.cmake)

# Kconfig, not a CMake argument: the node header belongs to the same
# per-robot choice as the overlay and the node id, and a snippet can only
# append Kconfig and devicetree fragments.
set(NODE_SCHEMA "${ZEPHYR_FIBRIL_ZEPHYR_MODULE_DIR}/${CONFIG_FIBRIL_NODE_SCHEMA}")

if(NOT EXISTS "${NODE_SCHEMA}")
  message(FATAL_ERROR "CONFIG_FIBRIL_NODE_SCHEMA points at a missing file: ${NODE_SCHEMA}")
endif()

# The schema is assembled from three kinds of fragment, which the codegen
# merges deterministically:
#
#   NODE_SCHEMA           the node's identity and limits, written by hand
#   FCAN_TYPE_SCHEMAS     every block type this repository implements
#   FCAN_INSTANCE_SCHEMAS the instances each enabled function derived from
#                         its devicetree node
#
# Types are offered wholesale rather than curated: an unreferenced one is
# dropped before the blob is built, so the list costs nothing. What the image
# carries is decided by the devicetree alone.
get_property(FCAN_TYPE_SCHEMAS GLOBAL PROPERTY fibril_can_node_type_schemas)
get_property(FCAN_INSTANCE_SCHEMAS GLOBAL PROPERTY fibril_can_node_instance_schemas)

if(NOT FCAN_INSTANCE_SCHEMAS)
  message(FATAL_ERROR
    "No function is enabled: the devicetree describes no fibril,fcan-* node, "
    "so this image would announce nothing. Add a deployment snippet.")
endif()

fcan_invoke_codegen(
  SCHEMA     "${NODE_SCHEMA}" ${FCAN_TYPE_SCHEMAS} ${FCAN_INSTANCE_SCHEMAS}
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
