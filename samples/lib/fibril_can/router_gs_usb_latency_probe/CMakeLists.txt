# SPDX-License-Identifier: Apache-2.0
#
# Zephyr sample: fibril_can router + latency-probe self node driven over
# CANnectivity's gs_usb uplink. Same wire contract as
# fibril_zephyr/samples/lib/fibril_can/latency_probe_node, but the probe self
# node sits inside a fcan_router device — uplink is USB (gs_usb) and downlink
# is fdcan1. Use this variant when the same MCU also has to gateway a
# physical CAN bus to the host.
#
# Prerequisites: see the fibril_can module's zephyr/cmake/fcan_invoke_codegen.cmake
# for the fcan_codegen CLI resolution.

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(fibril_can_router_gs_usb_latency_probe LANGUAGES C)

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
  src/probe_logic.c
  src/usbd_setup.c)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src)
