zephyr_library_sources_ifdef(CONFIG_FIBRIL_CAN_NODE_VESC vesc.cpp)
set_property(GLOBAL APPEND PROPERTY fibril_can_node_type_schemas
             "${CMAKE_CURRENT_SOURCE_DIR}/type.yaml")
if(CONFIG_FIBRIL_CAN_NODE_VESC)
  fibril_can_node_instances(
    TYPE VescMotor COMPATIBLE "fibril,fcan-vesc" MAX ${CONFIG_FIBRIL_CAN_NODE_VESC_MAX})
  fibril_can_node_builtin_instances(TYPE VescBus NS "vesc_bus" MAX 1)
endif()
