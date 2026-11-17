/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT vesc_encoder

#include <drivers/encoder.h>
#include <drivers/motor.h>
#include <drivers/motor/vesc.h>
#include <drivers/motor/vesc_protocol.h>

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

struct vesc_encoder_config {
  const struct device * motor;
};

struct vesc_encoder_data {
  struct k_spinlock lock;
  struct encoder_feedback feedback;
  int64_t position;
  int64_t bias;
  int32_t previous;
  uint32_t previous_cycle;
  bool valid;
  bool has_interval;
};

static void on_position(
  const struct device * motor, int32_t position_counts, uint32_t cycle, void * user_data)
{
  const struct device * dev = user_data;
  struct vesc_encoder_data * data = dev->data;
  k_spinlock_key_t key = k_spin_lock(&data->lock);
  ARG_UNUSED(motor);

  if (!data->valid) {
    data->position = position_counts;
    data->bias = 0;
    data->feedback.position_epoch++;
    data->valid = true;
    data->has_interval = false;
  } else {
    const int64_t delta = vesc_position_wrap_delta(position_counts, data->previous);
    data->position += delta;
    if (data->has_interval) {
      const uint32_t interval_us = (uint32_t)k_cyc_to_us_floor64(cycle - data->previous_cycle);
      if (interval_us > 0U) {
        const int64_t velocity = delta * USEC_PER_SEC / interval_us;
        data->feedback.velocity = (int32_t)CLAMP(velocity, INT32_MIN, INT32_MAX);
        data->feedback.sample_interval_us = interval_us;
        data->feedback.valid_mask |= ENCODER_FEEDBACK_VELOCITY;
      }
    }
  }
  data->previous = position_counts;
  data->previous_cycle = cycle;
  data->has_interval = true;
  data->feedback.position = data->position + data->bias;
  data->feedback.single_turn = (uint32_t)position_counts;
  data->feedback.valid_mask |= ENCODER_FEEDBACK_POSITION | ENCODER_FEEDBACK_SINGLE_TURN;
  data->feedback.timestamp_ms = k_uptime_get();
  k_spin_unlock(&data->lock, key);
}

static int get_feedback(const struct device * dev, void * feedback_ptr)
{
  const struct vesc_encoder_config * config = dev->config;
  struct vesc_encoder_data * data = dev->data;
  struct encoder_feedback * feedback = feedback_ptr;
  struct motor_feedback motor_feedback;
  if (feedback == NULL) return -EINVAL;
  k_spinlock_key_t key = k_spin_lock(&data->lock);
  if (!data->valid) {
    k_spin_unlock(&data->lock, key);
    return -ENODATA;
  }
  *feedback = data->feedback;
  k_spin_unlock(&data->lock, key);
  if (motor_get_feedback(config->motor, &motor_feedback) != 0) return -EIO;
  feedback->online = motor_feedback.online;
  feedback->stale = motor_feedback.stale;
  if (!feedback->online) return -EIO;
  if (feedback->stale) return -EAGAIN;
  return 0;
}

static int get_resolution(const struct device * dev, uint8_t * resolution)
{
  ARG_UNUSED(dev);
  ARG_UNUSED(resolution);
  /* STATUS_4 uses 18000 codes/rev, not a binary counter width. */
  return -ENOTSUP;
}

static int set_zero(const struct device * dev)
{
  ARG_UNUSED(dev);
  return -ENOTSUP;
}

static int set_position(const struct device * dev, int64_t position)
{
  struct vesc_encoder_data * data = dev->data;
  if ((position > (INT64_C(1) << 62)) || (position < -(INT64_C(1) << 62))) return -EINVAL;
  k_spinlock_key_t key = k_spin_lock(&data->lock);
  if (!data->valid) {
    k_spin_unlock(&data->lock, key);
    return -ENODATA;
  }
  data->bias = position - data->position;
  data->feedback.position = position;
  k_spin_unlock(&data->lock, key);
  return 0;
}

static int reset(const struct device * dev)
{
  ARG_UNUSED(dev);
  return -ENOTSUP;
}

static int init(const struct device * dev)
{
  const struct vesc_encoder_config * config = dev->config;
  if (!device_is_ready(config->motor)) return -ENODEV;
  return vesc_set_position_listener(config->motor, on_position, (void *)dev);
}

static const struct encoder_driver_api api = {
  .get_feedback = get_feedback, .get_resolution = get_resolution,
  .set_zero = set_zero, .set_position = set_position, .reset = reset,
};

#define DEFINE(inst)                                                                           \
  static struct vesc_encoder_data data_##inst;                                                  \
  static const struct vesc_encoder_config config_##inst = {                                     \
    .motor = DEVICE_DT_GET(DT_INST_PHANDLE(inst, motor))};                                      \
  DEVICE_DT_INST_DEFINE(inst, init, NULL, &data_##inst, &config_##inst, POST_KERNEL,             \
                        CONFIG_ENCODER_INIT_PRIORITY, &api);

DT_INST_FOREACH_STATUS_OKAY(DEFINE)
