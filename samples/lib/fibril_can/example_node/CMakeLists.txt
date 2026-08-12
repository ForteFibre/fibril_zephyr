# SPDX-License-Identifier: Apache-2.0
#
# Zephyr sample: fibril_can codegen-driven slave node.
#
# Runs fcan_codegen against fibril_can_example/schema/example_node.yaml and
# builds a Zephyr application around fcan_runtime + the C99 generated wrappers.
# The application publishes S2M telemetry on a Zephyr CAN device (native_sim
# uses zephyr,can-loopback) and prints state / TX / RX transitions.
#
# Prerequisites: see the fibril_can module's zephyr/cmake/fcan_invoke_codegen.cmake
# for the fcan_codegen CLI resolution, and the message it prints if the binary
# is not on PATH.

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(fibril_can_example_node LANGUAGES C)

# The Zephyr module system exports ZEPHYR_FIBRIL_CAN_MODULE_DIR when the
# fibril_can module is on the west manifest — same variable the module's
# own zephyr/CMakeLists.txt uses (FCAN_ROOT).
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
  src/zephyr_can_hal.c
  src/app_logic.c
  src/rgb_state.c)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src)
