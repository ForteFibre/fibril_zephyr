# SPDX-License-Identifier: Apache-2.0
#
# Zephyr sample: fibril_can_benchmark latency-probe slave node.
#
# Runs fcan_codegen against fibril_can_benchmark/schema/latency_probe.yaml and
# builds a Zephyr application around fcan_runtime + the generated C99 wrappers.
# The application is the MCU-side counterpart of the reference host slave in
# fibril_can_benchmark/src/host_latency_slave.cpp; both must obey the contract
# in fibril_can_benchmark/docs/firmware-requirements.md.

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(fibril_can_latency_probe_node LANGUAGES C)

if(NOT ZEPHYR_FIBRIL_CAN_MODULE_DIR)
  message(FATAL_ERROR
    "ZEPHYR_FIBRIL_CAN_MODULE_DIR is not set — is the fibril_can module on "
    "the west manifest and is CONFIG_FIBRIL_CAN=y?")
endif()

include(${ZEPHYR_FIBRIL_CAN_MODULE_DIR}/zephyr/cmake/fcan_invoke_codegen.cmake)

set(FCAN_LATENCY_PROBE_SCHEMA
  "${ZEPHYR_FIBRIL_CAN_MODULE_DIR}/fibril_can_benchmark/schema/latency_probe.yaml"
  CACHE FILEPATH "Schema YAML to feed into fcan_codegen")

fcan_invoke_codegen(
  SCHEMA     "${FCAN_LATENCY_PROBE_SCHEMA}"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/schema_gen"
  BASE_NAME  schema_gen)

target_sources(app PRIVATE
  src/main.c
  src/probe_logic.c)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src)
