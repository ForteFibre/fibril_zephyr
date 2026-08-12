# SPDX-License-Identifier: Apache-2.0
#
# fcan_invoke_codegen(SCHEMA <yaml> OUTPUT_DIR <dir> BASE_NAME <name>)
#
# Runs an externally-built fcan_codegen CLI at build time and attaches the
# generated <base>.c and schema_blob.c to the Zephyr `app` target, plus the
# output directory as an include path.
#
# Resolution order for the CLI:
#   1. -DFCAN_CODEGEN=/abs/path/fcan_codegen (CMake cache; wins)
#   2. $ENV{FCAN_CODEGEN}
#   3. find_program(fcan_codegen) on PATH (works if a ROS 2 workspace with
#      fibril_can_codegen installed has been sourced)
#
# Escape hatches:
#   -DFCAN_PREGENERATED_DIR=/path/to/gen  Skip codegen entirely and pull
#                                         schema_gen.{h,c} + schema_blob.c
#                                         from the given directory. Useful in
#                                         CI environments without protobuf /
#                                         yaml-cpp on the host.
#
# The helper avoids replicating the ament-side fcan_generate_schema helper's
# limits-sidecar handling: the Zephyr sample only wires the C99 slave runtime
# through the generated register_all/apply_schema_capacities helpers, and does
# not need FCAN_ENABLE_SEGMENTATION toggled from a sidecar (service_buffer=0
# in the example schema keeps the segmentation path off already; add-side
# handling can move here later if required).

function(fcan_invoke_codegen)
  set(_one_value SCHEMA OUTPUT_DIR BASE_NAME)
  cmake_parse_arguments(FCAN "" "${_one_value}" "" ${ARGN})

  if(NOT FCAN_SCHEMA)
    message(FATAL_ERROR "fcan_invoke_codegen: SCHEMA is required")
  endif()
  if(NOT FCAN_OUTPUT_DIR)
    set(FCAN_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/schema_gen")
  endif()
  if(NOT FCAN_BASE_NAME)
    set(FCAN_BASE_NAME "schema_gen")
  endif()

  get_filename_component(_schema_abs "${FCAN_SCHEMA}" ABSOLUTE)

  set(_hdr  "${FCAN_OUTPUT_DIR}/${FCAN_BASE_NAME}.h")
  set(_src  "${FCAN_OUTPUT_DIR}/${FCAN_BASE_NAME}.c")
  set(_blob "${FCAN_OUTPUT_DIR}/schema_blob.c")

  # --- escape hatch: pre-generated tree
  if(FCAN_PREGENERATED_DIR)
    if(NOT EXISTS "${FCAN_PREGENERATED_DIR}/${FCAN_BASE_NAME}.c"
        OR NOT EXISTS "${FCAN_PREGENERATED_DIR}/schema_blob.c")
      message(FATAL_ERROR
        "FCAN_PREGENERATED_DIR=${FCAN_PREGENERATED_DIR} is missing "
        "${FCAN_BASE_NAME}.c or schema_blob.c")
    endif()
    target_sources(app PRIVATE
      "${FCAN_PREGENERATED_DIR}/${FCAN_BASE_NAME}.c"
      "${FCAN_PREGENERATED_DIR}/schema_blob.c")
    target_include_directories(app PRIVATE "${FCAN_PREGENERATED_DIR}")
    message(STATUS "[fibril_can] using pre-generated schema from ${FCAN_PREGENERATED_DIR}")
    return()
  endif()

  # --- resolve fcan_codegen CLI (case-insensitive: cache > env > PATH)
  set(_fcan_codegen "")
  if(FCAN_CODEGEN)
    set(_fcan_codegen "${FCAN_CODEGEN}")
  elseif(DEFINED ENV{FCAN_CODEGEN})
    set(_fcan_codegen "$ENV{FCAN_CODEGEN}")
  else()
    find_program(_fcan_codegen_path NAMES fcan_codegen)
    if(_fcan_codegen_path)
      set(_fcan_codegen "${_fcan_codegen_path}")
    endif()
  endif()

  if(NOT _fcan_codegen OR NOT EXISTS "${_fcan_codegen}")
    message(FATAL_ERROR
      "fcan_invoke_codegen: fcan_codegen executable not found.\n"
      "Build it once, then pass its path via -DFCAN_CODEGEN=<abs-path> or\n"
      "export FCAN_CODEGEN before west build. Two easy ways:\n"
      "  A. From a ROS 2 workspace containing fibril_can:\n"
      "       colcon build --packages-up-to fibril_can_codegen\n"
      "       source install/setup.bash   # puts fcan_codegen on PATH\n"
      "  B. Standalone CMake build (needs libprotobuf-dev, yaml-cpp,\n"
      "     fibril_can_core installed):\n"
      "       cmake -S <fibril_can>/fibril_can_codegen -B /tmp/fcan_cg\n"
      "       cmake --build /tmp/fcan_cg\n"
      "       west build ... -- -DFCAN_CODEGEN=/tmp/fcan_cg/fcan_codegen\n"
      "\n"
      "Alternatively, pass -DFCAN_PREGENERATED_DIR=<dir> to skip codegen\n"
      "and consume already-generated sources.")
  endif()

  add_custom_command(
    OUTPUT  "${_hdr}" "${_src}" "${_blob}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${FCAN_OUTPUT_DIR}"
    COMMAND "${_fcan_codegen}" "${_schema_abs}"
            --out "${FCAN_OUTPUT_DIR}"
            --base "${FCAN_BASE_NAME}"
            --lang c
    DEPENDS "${_schema_abs}" "${_fcan_codegen}"
    COMMENT "[fibril_can] codegen ${FCAN_SCHEMA}"
    VERBATIM)

  target_sources(app PRIVATE "${_src}" "${_blob}")
  target_include_directories(app PRIVATE "${FCAN_OUTPUT_DIR}")

  # Reconfigure when the YAML changes so downstream logic (limits, macro
  # definitions consumed at configure time in the future) stays in sync.
  set_property(DIRECTORY APPEND
    PROPERTY CMAKE_CONFIGURE_DEPENDS "${_schema_abs}")

  message(STATUS "[fibril_can] codegen CLI: ${_fcan_codegen}")
  message(STATUS "[fibril_can] schema:      ${_schema_abs}")
  message(STATUS "[fibril_can] output dir:  ${FCAN_OUTPUT_DIR}")
endfunction()
