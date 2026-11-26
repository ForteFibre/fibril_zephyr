# SPDX-License-Identifier: Apache-2.0
#
# Zephyr sample: fibril_can hub + self node driven over CANnectivity's gs_usb
# external port. Codegen produces the schema wrappers at build time from
# fibril_can_example/schema/example_node.yaml (same schema as example_node so
# the C service handlers and app_logic can be shared verbatim).
#
# Prerequisites: see the fibril_can module's zephyr/cmake/fcan_invoke_codegen.cmake
# for the fcan_codegen CLI resolution, and the message it prints if the binary
# is not on PATH.

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(fibril_can_hub_gs_usb_self LANGUAGES C)

if(NOT ZEPHYR_FIBRIL_CAN_MODULE_DIR)
  message(FATAL_ERROR
    "ZEPHYR_FIBRIL_CAN_MODULE_DIR is not set — is the fibril_can module on "
    "the west manifest and is CONFIG_FIBRIL_CAN=y?")
endif()

include(${ZEPHYR_FIBRIL_CAN_MODULE_DIR}/zephyr/cmake/fcan_invoke_codegen.cmake)

set(FCAN_EXAMPLE_SCHEMA
  "${ZEPHYR_FIBRIL_CAN_MODULE_DIR}/fibril_can_example/schema/example_node.yaml"
  CACHE FILEPATH "Schema YAML to feed into fcan_codegen")

fcan_invoke_codegen(
  SCHEMA     "${FCAN_EXAMPLE_SCHEMA}"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/schema_gen"
  BASE_NAME  schema_gen)

target_sources(app PRIVATE
  src/main.c
  src/app_logic.c
  src/rgb_state.c
  src/usbd_setup.c)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src)
