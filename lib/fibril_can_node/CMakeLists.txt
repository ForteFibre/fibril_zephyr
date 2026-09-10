# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

add_subdirectory(solenoid)

# Every block type's sources are compiled unconditionally and guard their own
# body on the codegen's FCAN_<TYPE>_MAX_COUNT. The schema is then the only
# place that decides which functions this image carries; a Kconfig per block
# type would be a second answer to the same question and could disagree with
# the first.
#
# The list of type.yaml files each subdirectory contributes is collected on a
# global property, which the application reads and passes to the codegen.
get_property(_schemas GLOBAL PROPERTY fibril_can_node_type_schemas)
message(STATUS "[fibril_can_node] block type schemas: ${_schemas}")
