/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/motor.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(motor_dji_robomaster, CONFIG_MOTOR_LOG_LEVEL);

#define ROBOMASTER_MAX_MOTORS 8
#define ROBOMASTER_GROUP_SIZE 4
#define ROBOMASTER_GROUP_COUNT 2
#define ROBOMASTER_MAX_CANS 4
#define ROBOMASTER_RX_ID_BASE 0x200U
#define ROBOMASTER_TX_ID_GROUP0 0x200U
#define ROBOMASTER_TX_ID_GROUP1 0x1FFU
#define ROBOMASTER_RX_FILTER_MASK 0x7F0U
#define ROBOMASTER_ENCODER_WRAP 8192
#define ROBOMASTER_ENCODER_HALF_WRAP (ROBOMASTER_ENCODER_WRAP / 2)
#define ROBOMASTER_DEFAULT_MAX_CURRENT 10000
#define ROBOMASTER_CANBUS_UNKNOWN 0xFF

enum robomaster_model {
  ROBOMASTER_MODEL_C610 = 0,
  ROBOMASTER_MODEL_C620 = 1,
};

struct robomaster_transport_config
{
  const struct device * const * can_devs;
  size_t can_count;
  uint32_t feedback_timeout_ms;
};

struct robomaster_transport_data
{
  struct k_spinlock lock;
  const struct device * dev;
  const struct device * motors[ROBOMASTER_MAX_MOTORS];
  struct k_work tx_work;
  struct k_timer tx_timer;
};

struct robomaster_motor_config
{
  const struct device * transport;
  uint8_t motor_id;
  enum robomaster_model model;
  int16_t max_current;
};

struct robomaster_motor_data
{
  struct k_spinlock lock;
  bool enabled;
  bool has_last_orientation;
  uint8_t detected_can_bus;
  uint16_t last_orientation_raw;
  int16_t requested_current;
  struct motor_feedback feedback;
};

static int16_t robomaster_motor_get_effective_current(
  const struct device * motor_dev, uint8_t can_bus)
{
  struct robomaster_motor_data * motor = motor_dev->data;
  int16_t current = 0;
  k_spinlock_key_t key = k_spin_lock(&motor->lock);

  if (motor->detected_can_bus == can_bus && motor->enabled) {
    current = motor->requested_current;
  }

  k_spin_unlock(&motor->lock, key);

  return current;
}

static bool robomaster_motor_is_on_can_bus(const struct device * motor_dev, uint8_t can_bus)
{
  struct robomaster_motor_data * motor = motor_dev->data;
  bool matches;
  k_spinlock_key_t key = k_spin_lock(&motor->lock);

  matches = motor->detected_can_bus == can_bus;

  k_spin_unlock(&motor->lock, key);

  return matches;
}

static int robomaster_transport_find_can_bus(
  const struct robomaster_transport_config * config, const struct device * can_dev)
{
  for (size_t i = 0; i < config->can_count; ++i) {
    if (config->can_devs[i] == can_dev) {
      return (int)i;
    }
  }

  return -ENODEV;
}

static void robomaster_transport_tx_work_handler(struct k_work * work)
{
  struct robomaster_transport_data * transport =
    CONTAINER_OF(work, struct robomaster_transport_data, tx_work);
  const struct device * transport_dev = transport->dev;
  const struct robomaster_transport_config * config = transport_dev->config;
  struct can_frame frame = {
    .flags = 0U,
    .dlc = can_bytes_to_dlc(8U),
  };

  for (size_t can_bus = 0; can_bus < config->can_count; ++can_bus) {
    for (size_t group = 0; group < ROBOMASTER_GROUP_COUNT; ++group) {
      const struct device * motor_devs[ROBOMASTER_GROUP_SIZE];
      k_spinlock_key_t key = k_spin_lock(&transport->lock);
      for (size_t slot = 0; slot < ROBOMASTER_GROUP_SIZE; ++slot) {
        motor_devs[slot] = transport->motors[group * ROBOMASTER_GROUP_SIZE + slot];
      }
      k_spin_unlock(&transport->lock, key);

      bool has_motor_on_bus = false;
      for (size_t slot = 0; slot < ROBOMASTER_GROUP_SIZE; ++slot) {
        int16_t current = 0;

        if (
          (motor_devs[slot] != NULL) &&
          robomaster_motor_is_on_can_bus(motor_devs[slot], (uint8_t)can_bus)) {
          has_motor_on_bus = true;
          current = robomaster_motor_get_effective_current(motor_devs[slot], (uint8_t)can_bus);
        }

        sys_put_be16((uint16_t)current, &frame.data[slot * sizeof(int16_t)]);
      }

      if (!has_motor_on_bus) {
        continue;
      }

      frame.id = (group == 0U) ? ROBOMASTER_TX_ID_GROUP0 : ROBOMASTER_TX_ID_GROUP1;
      (void)can_send(config->can_devs[can_bus], &frame, K_NO_WAIT, NULL, NULL);
    }
  }
}

static void robomaster_transport_tx_timer_handler(struct k_timer * timer)
{
  struct robomaster_transport_data * transport = k_timer_user_data_get(timer);

  k_work_submit(&transport->tx_work);
}

static void robomaster_rx_callback(
  const struct device * can_dev, struct can_frame * frame, void * user_data)
{
  const struct device * transport_dev = user_data;
  const struct robomaster_transport_config * config = transport_dev->config;
  struct robomaster_transport_data * transport = transport_dev->data;
  const struct device * motor_dev;
  struct robomaster_motor_data * motor;
  uint8_t motor_id;
  int can_bus;
  uint16_t orientation_raw;
  int16_t velocity;
  int16_t current;
  uint8_t temperature;
  int32_t delta;
  k_spinlock_key_t key;

  ARG_UNUSED(can_dev);

  if (
    (frame->id < (ROBOMASTER_RX_ID_BASE + 1U)) ||
    (frame->id > (ROBOMASTER_RX_ID_BASE + ROBOMASTER_MAX_MOTORS))) {
    return;
  }

  if (can_dlc_to_bytes(frame->dlc) < 7U) {
    return;
  }

  motor_id = (uint8_t)(frame->id - ROBOMASTER_RX_ID_BASE);
  if ((motor_id < 1U) || (motor_id > ROBOMASTER_MAX_MOTORS)) {
    return;
  }

  can_bus = robomaster_transport_find_can_bus(config, can_dev);
  if (can_bus < 0) {
    return;
  }

  key = k_spin_lock(&transport->lock);
  motor_dev = transport->motors[motor_id - 1U];
  k_spin_unlock(&transport->lock, key);

  if (motor_dev == NULL) {
    return;
  }

  motor = motor_dev->data;
  orientation_raw = sys_get_be16(&frame->data[0]);
  velocity = (int16_t)sys_get_be16(&frame->data[2]);
  current = (int16_t)sys_get_be16(&frame->data[4]);
  temperature = frame->data[6];

  key = k_spin_lock(&motor->lock);

  if (motor->detected_can_bus != ROBOMASTER_CANBUS_UNKNOWN && motor->detected_can_bus != can_bus) {
    k_spin_unlock(&motor->lock, key);
    return;
  } else {
    motor->detected_can_bus = (uint8_t)can_bus;
  }

  if (motor->has_last_orientation) {
    delta = (int32_t)orientation_raw - (int32_t)motor->last_orientation_raw;
    if (delta > ROBOMASTER_ENCODER_HALF_WRAP) {
      delta -= ROBOMASTER_ENCODER_WRAP;
    } else if (delta < -ROBOMASTER_ENCODER_HALF_WRAP) {
      delta += ROBOMASTER_ENCODER_WRAP;
    }
    motor->feedback.position += delta;
  } else {
    motor->feedback.position = orientation_raw;
    motor->has_last_orientation = true;
  }

  motor->last_orientation_raw = orientation_raw;
  motor->feedback.valid_mask = MOTOR_FEEDBACK_CURRENT | MOTOR_FEEDBACK_VELOCITY |
                               MOTOR_FEEDBACK_POSITION | MOTOR_FEEDBACK_ORIENTATION |
                               MOTOR_FEEDBACK_TEMPERATURE;
  motor->feedback.current = current;
  motor->feedback.velocity = velocity;
  motor->feedback.orientation = orientation_raw;
  motor->feedback.temperature = temperature;
  motor->feedback.online = true;
  motor->feedback.stale = false;
  motor->feedback.timestamp_ms = k_uptime_get();
  k_spin_unlock(&motor->lock, key);
}

static int robomaster_transport_init(const struct device * dev)
{
  const struct robomaster_transport_config * config = dev->config;
  struct robomaster_transport_data * data = dev->data;
  const struct can_filter filter = {
    .id = ROBOMASTER_RX_ID_BASE,
    .mask = ROBOMASTER_RX_FILTER_MASK,
    .flags = 0U,
  };

  if (config->can_count > ROBOMASTER_MAX_CANS) {
    LOG_ERR("Too many CAN devices (%u)", (unsigned int)config->can_count);
    return -EINVAL;
  }

  data->dev = dev;
  k_work_init(&data->tx_work, robomaster_transport_tx_work_handler);
  k_timer_init(&data->tx_timer, robomaster_transport_tx_timer_handler, NULL);
  k_timer_user_data_set(&data->tx_timer, data);

  for (size_t i = 0; i < config->can_count; ++i) {
    if (!device_is_ready(config->can_devs[i])) {
      LOG_ERR("CAN bus %u not ready", (unsigned int)i);
      return -ENODEV;
    }

    int filter_id =
      can_add_rx_filter(config->can_devs[i], robomaster_rx_callback, (void *)dev, &filter);
    if (filter_id < 0) {
      LOG_ERR("Failed to add RX filter on CAN bus %u (%d)", (unsigned int)i, filter_id);
      return filter_id;
    }

    int ret = can_start(config->can_devs[i]);
    if ((ret < 0) && (ret != -EALREADY)) {
      LOG_ERR("Failed to start CAN bus %u (%d)", (unsigned int)i, ret);
      return ret;
    }
  }

  k_timer_start(&data->tx_timer, K_MSEC(1), K_MSEC(1));

  return 0;
}

static int robomaster_transport_register_motor(
  const struct device * transport_dev, const struct device * motor_dev, uint8_t motor_id)
{
  struct robomaster_transport_data * transport = transport_dev->data;
  k_spinlock_key_t key;

  if ((motor_id < 1U) || (motor_id > ROBOMASTER_MAX_MOTORS)) {
    return -EINVAL;
  }

  key = k_spin_lock(&transport->lock);
  if (transport->motors[motor_id - 1U] != NULL) {
    k_spin_unlock(&transport->lock, key);
    return -EALREADY;
  }
  struct robomaster_motor_data * motor_data = motor_dev->data;
  memset(motor_data, 0, sizeof(struct robomaster_motor_data));
  motor_data->detected_can_bus = ROBOMASTER_CANBUS_UNKNOWN;
  transport->motors[motor_id - 1U] = motor_dev;
  k_spin_unlock(&transport->lock, key);

  return 0;
}

static int robomaster_motor_enable(const struct device * dev)
{
  struct robomaster_motor_data * data = dev->data;
  k_spinlock_key_t key;

  key = k_spin_lock(&data->lock);
  data->enabled = true;
  k_spin_unlock(&data->lock, key);

  return 0;
}

static int robomaster_motor_disable(const struct device * dev)
{
  struct robomaster_motor_data * data = dev->data;
  k_spinlock_key_t key;

  key = k_spin_lock(&data->lock);
  data->enabled = false;
  k_spin_unlock(&data->lock, key);

  return 0;
}

static int robomaster_motor_set_output(
  const struct device * dev, const enum motor_output_mode mode, const int16_t output)
{
  const struct robomaster_motor_config * config = dev->config;
  struct robomaster_motor_data * data = dev->data;
  int16_t clamped;
  k_spinlock_key_t key;

  if ((mode != MOTOR_OUTPUT_MODE_CURRENT) && (mode != MOTOR_OUTPUT_MODE_TORQUE)) {
    return -ENOTSUP;
  }

  clamped = CLAMP(output, -config->max_current, config->max_current);

  key = k_spin_lock(&data->lock);
  data->requested_current = clamped;
  k_spin_unlock(&data->lock, key);

  return 0;
}

static int robomaster_motor_get_feedback(const struct device * dev, void * feedback)
{
  const struct robomaster_motor_config * config = dev->config;
  const struct robomaster_transport_config * transport_config = config->transport->config;
  struct robomaster_motor_data * data = dev->data;
  struct motor_feedback * out = feedback;
  k_spinlock_key_t key;
  int ret = 0;

  if (out == NULL) {
    return -EINVAL;
  }

  key = k_spin_lock(&data->lock);
  *out = data->feedback;

  if (!data->feedback.online) {
    ret = -ENODATA;
  } else if (
    (transport_config->feedback_timeout_ms > 0U) &&
    ((k_uptime_get() - data->feedback.timestamp_ms) > transport_config->feedback_timeout_ms)) {
    out->stale = true;
    ret = -EAGAIN;
  }

  k_spin_unlock(&data->lock, key);

  return ret;
}

static int robomaster_motor_init(const struct device * dev)
{
  const struct robomaster_motor_config * config = dev->config;

  if (!device_is_ready(config->transport)) {
    LOG_ERR("Transport not ready for motor %u", config->motor_id);
    return -ENODEV;
  }

  if (config->motor_id < 1U || config->motor_id > ROBOMASTER_MAX_MOTORS) {
    LOG_ERR("Invalid motor ID %u", config->motor_id);
    return -EINVAL;
  }

  return robomaster_transport_register_motor(config->transport, dev, config->motor_id);
}

static const struct motor_driver_api robomaster_motor_api = {
  .enable = robomaster_motor_enable,
  .disable = robomaster_motor_disable,
  .set_output = robomaster_motor_set_output,
  .get_feedback = robomaster_motor_get_feedback,
};

#define DT_DRV_COMPAT dji_robomaster

#define ROBOMASTER_CAN_DEV_ELEM(idx, inst) DEVICE_DT_GET(DT_INST_PHANDLE_BY_IDX(inst, cans, idx))

#define ROBOMASTER_TRANSPORT_DEFINE(inst)                                                \
  BUILD_ASSERT(                                                                          \
    DT_INST_PROP_LEN(inst, cans) <= ROBOMASTER_MAX_CANS,                                 \
    "Too many CAN devices configured for RoboMaster transport");                         \
  static const struct device * const robomaster_can_devs_##inst[] = {                    \
    LISTIFY(DT_INST_PROP_LEN(inst, cans), ROBOMASTER_CAN_DEV_ELEM, (, ), inst)};         \
  static struct robomaster_transport_data robomaster_transport_data_##inst;              \
  static const struct robomaster_transport_config robomaster_transport_config_##inst = { \
    .can_devs = robomaster_can_devs_##inst,                                              \
    .can_count = DT_INST_PROP_LEN(inst, cans),                                           \
    .feedback_timeout_ms = DT_INST_PROP_OR(inst, feedback_timeout_ms, 100),              \
  };                                                                                     \
  DEVICE_DT_INST_DEFINE(                                                                 \
    inst, robomaster_transport_init, NULL, &robomaster_transport_data_##inst,            \
    &robomaster_transport_config_##inst, POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(ROBOMASTER_TRANSPORT_DEFINE)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT dji_robomaster_motor

#define ROBOMASTER_MODEL_INIT(inst) DT_INST_ENUM_IDX(inst, model)

#define ROBOMASTER_MAX_CURRENT_INIT(inst) \
  DT_INST_PROP_OR(inst, max_current, ROBOMASTER_DEFAULT_MAX_CURRENT)

#define ROBOMASTER_MOTOR_DEFINE(inst)                                            \
  static struct robomaster_motor_data robomaster_motor_data_##inst;              \
  static const struct robomaster_motor_config robomaster_motor_config_##inst = { \
    .transport = DEVICE_DT_GET(DT_INST_PARENT(inst)),                            \
    .motor_id = DT_INST_REG_ADDR(inst),                                          \
    .model = ROBOMASTER_MODEL_INIT(inst),                                        \
    .max_current = ROBOMASTER_MAX_CURRENT_INIT(inst),                            \
  };                                                                             \
  DEVICE_DT_INST_DEFINE(                                                         \
    inst, robomaster_motor_init, NULL, &robomaster_motor_data_##inst,            \
    &robomaster_motor_config_##inst, POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY,    \
    &robomaster_motor_api);

DT_INST_FOREACH_STATUS_OKAY(ROBOMASTER_MOTOR_DEFINE)
