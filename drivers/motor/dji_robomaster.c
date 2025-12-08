#include <cstdint>

#include "syscalls/can.h"
#include "zephyr/sys/bitarray.h"
#include "zephyr/sys/util_macro.h"
#define DT_DRV_COMPAT dji_robomaster

#include <drivers/motor.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(dji_robomaster, CONFIG_MOTOR_LOG_LEVEL);

#define DJI_ROBOMASTER_DEFAULT_BITRATE 1000000U
#define DJI_ROBOMASTER_DEFAULT_SAMPLE_POINT 750U

struct robomaster_feedback
{
  uint16_t mechanical_angle;
  uint16_t rotational_speed;
  uint16_t torque_current;
  uint8_t temperature;
};

typedef struct robomaster_feedback robomaster_feedback_t;

struct robomaster_config
{
  const struct device ** canbus;
  const size_t canbus_count;
  const bool auto_probe;
};

struct robomaster_data
{
  const struct robomaster_config * config;
  robomaster_feedback_t feedback[8];
  int16_t outputs[8];
  // Mapping for 4 motors
  // 0-7: 1st CAN bus
  // 8-15: 2nd CAN bus
  // ...
  uint32_t mappings;

  struct can_frame tx_msg;
};

static void dji_robomaster_rx_callback(
  const struct device * dev, struct can_frame * frame, void * user_data)
{
  struct robomaster_data * data = (struct robomaster_data *)user_data;

  size_t motor_id = frame->id - 0x201;

  if (frame->id < 0x201 || frame->id > 0x208) {
    LOG_WRN("Received CAN frame with unexpected ID: 0x%03X", frame->id);
    return;
  }

  if (data->config->auto_probe) {
    size_t dev_index = SIZE_MAX;
    for (size_t i = 0; i < data->config->canbus_count; i++) {
      if (dev == data->config->canbus[i]) {
        dev_index = i;
        break;
      }
    }
    if (dev_index == SIZE_MAX) {
      LOG_ERR("Received CAN frame from unknown device");
      return;
    }

    size_t bit_offset = dev_index * 8 + motor_id;
    data->mappings |= BIT(bit_offset);
  }

  robomaster_feedback_t * feedback = &data->feedback[motor_id];
  feedback->mechanical_angle = (frame->data[0] << 8) | frame->data[1];
  feedback->rotational_speed = (frame->data[2] << 8) | frame->data[3];
  feedback->torque_current = (frame->data[4] << 8) | frame->data[5];
  feedback->temperature = frame->data[6];
}

static void dji_robomaster_send_outputs(const struct device * dev)
{
  struct robomaster_data * data = (struct robomaster_data *)dev->data;

  for (size_t i = 0; i < data->config->canbus_count; i++) {
    bool has_first_data = (data->mappings >> (i * 8)) & BIT_MASK(4);
    bool has_latter_data = (data->mappings >> (i * 8 + 4)) & BIT_MASK(4);
  }
}
static int dji_robomaster_init(const struct device * dev)
{
  int ret;
  const struct robomaster_config * config = (const struct robomaster_config *)dev->config;
  struct robomaster_data * data = (struct robomaster_data *)dev->data;

  if (config->canbus_count > 1 && !config->auto_probe) {
    LOG_ERR("Multiple CAN buses configured without auto-probe enabled");
    return -EINVAL;
  }

  const struct can_filter filter = {
    .id = 0x20F,
    .mask = 0x7F0,
  };

  for (size_t i = 0; i < config->canbus_count; i++) {
    if (!device_is_ready(config->canbus[i])) {
      LOG_ERR("CAN bus %d not ready", i);
      return -ENODEV;
    }

    struct can_timing timing = {0};

    ret = can_calc_timing(
      config->canbus[i], &timing, DJI_ROBOMASTER_DEFAULT_BITRATE,
      DJI_ROBOMASTER_DEFAULT_SAMPLE_POINT);
    if (ret != 0) {
      LOG_ERR("Failed to calculate CAN timing for bus %d: %d", i, ret);
      return ret;
    }

    ret = can_set_timing_data(config->canbus[i], &timing);
    if (ret != 0) {
      LOG_ERR("Failed to set CAN timing for bus %d: %d", i, ret);
      return ret;
    }

    ret = can_add_rx_filter(config->canbus[i], dji_robomaster_rx_callback, data, &filter);
    if (ret != 0) {
      LOG_ERR("Failed to add CAN RX filter for bus %d: %d", i, ret);
      return ret;
    }

    ret = can_start(config->canbus[i]);
    if (ret != 0) {
      LOG_ERR("Failed to start CAN bus %d: %d", i, ret);
      return ret;
    }
  }
}

static DEVICE_API(motor, dji_robomaster_driver_api) = {};

#define GET_CANBUS(node_id, prop, index) DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, index))

#define CREATE_DJI_ROBOMASTER(inst)                                           \
  static const struct device * dji_robomaster_##inst##_canbus[] = {           \
    DT_INST_FOREACH_PROP_ELEM(inst, canbus, GET_CANBUS)};                     \
  static const struct robomaster_config dji_robomaster_##inst##_config = {    \
    .canbus = dji_robomaster_##inst##_canbus,                                 \
    .canbus_count = DT_INST_PROP_LEN(inst, canbus),                           \
    .auto_probe = DT_INST_PROP_OR(inst, auto_probe, false),                   \
  };                                                                          \
  static struct robomaster_data dji_robomaster_##inst##_data = {              \
    .config = &dji_robomaster_##inst##_config};                               \
  DEVICE_DT_INST_DEFINE(                                                      \
    inst, &dji_robomaster_init, NULL, &dji_robomaster_##inst##_data,          \
    &dji_robomaster_##inst##_config, POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY, \
    &dji_robomaster_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CREATE_DJI_ROBOMASTER)
