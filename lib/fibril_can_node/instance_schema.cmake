# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

# fibril_can_node_instances(TYPE <BlockType> COMPATIBLE <compat> MAX <n>)
#
# Emit the `instances:` fragment for one block type from the devicetree node
# that describes its hardware, and register it for the application to hand to
# the codegen.
#
# The devicetree is the only place a deployment says which functions a board
# carries. Writing the same fact again in a node schema is what this replaces.
function(fibril_can_node_instances)
  set(single_args TYPE COMPATIBLE MAX)
  cmake_parse_arguments(ARG "" "${single_args}" "" ${ARGN})

  foreach(arg ${single_args})
    if(NOT ARG_${arg})
      message(FATAL_ERROR "fibril_can_node_instances() missing required argument: ${arg}")
    endif()
  endforeach()

  dt_comp_path(paths COMPATIBLE "${ARG_COMPATIBLE}")

  # dt_comp_path lists every node with the compatible, disabled ones included
  # (gen_dts_cmake.py walks edt.nodes, not the okay subset), so the status has
  # to be checked here.
  set(enabled "")
  foreach(path ${paths})
    dt_node_has_status(ok PATH "${path}" STATUS okay)
    if(ok)
      list(APPEND enabled "${path}")
    endif()
  endforeach()

  list(LENGTH enabled count)
  if(count EQUAL 0)
    return()
  endif()

  # One node per block type. A second one would produce a second instance
  # group that no handler serves, and the failure would surface as a runtime
  # silence rather than a build error.
  if(count GREATER 1)
    message(FATAL_ERROR
      "${ARG_COMPATIBLE}: ${count} enabled nodes (${enabled}). "
      "One node describes every instance of ${ARG_TYPE}; use its per-instance "
      "property instead of a second node.")
  endif()

  dt_prop(ns PATH "${enabled}" PROPERTY "fcan-ns")
  if(NOT ns)
    message(FATAL_ERROR "${enabled}: fcan-ns is required to name the instances on the bus.")
  endif()

  set(fragment "${CMAKE_CURRENT_BINARY_DIR}/fcan_instances.yaml")
  set(staging "${fragment}.in")

  # max_count is a static-allocation ceiling, not a count: it never reaches
  # the schema blob, so the bus and the ROS graph see the runtime instance
  # count regardless. Overshooting costs sizeof(state) bytes of .bss per
  # unused instance, which is why a Kconfig default beats duplicating the
  # devicetree's length here — CMake cannot read a phandle-array anyway.
  file(WRITE "${staging}"
       "# Generated from ${enabled}. Edit the devicetree, not this file.\n"
       "instances:\n"
       "  - { type: ${ARG_TYPE}, max_count: ${ARG_MAX}, ns: \"${ns}\" }\n")

  # COPYONLY leaves the mtime alone when the content is unchanged, so a
  # re-configure does not make the codegen look out of date.
  configure_file("${staging}" "${fragment}" COPYONLY)

  set_property(GLOBAL APPEND PROPERTY fibril_can_node_instance_schemas "${fragment}")
endfunction()
