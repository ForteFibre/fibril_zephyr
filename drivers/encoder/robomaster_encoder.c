/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * The rotor angle a DJI RoboMaster ESC reports, seen as an encoder.
 *
 * The motor driver already resolves the angle's wrap into
 * motor_feedback.position, but a control loop written against the encoder
 * class needs the velocity over a measured interval and an epoch that says
 * when the position stopped meaning anything. Both come from following every
 * frame as it arrives, which is why this device listens to the motor driver's
 * receive path rather than polling the motor's snapshot.
 */

#define DT_DRV_COMPAT dji_robomaster_encoder

#include <errno.h>

#include <drivers/encoder.h>
#include <drivers/motor.h>
#include <drivers/motor/robomaster.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "encoder_accum.h"

LOG_MODULE_REGISTER(encoder_robomaster, CONFIG_ENCODER_LOG_LEVEL);

/* The ESC reports the rotor angle as 0..8191. */
#define ROBOMASTER_ENCODER_BITS 13U

struct robomaster_encoder_config
{
  const struct device * motor;
  bool invert;
};

struct robomaster_encoder_data
{
  struct k_spinlock lock;
  struct encoder_feedback feedback;
  struct encoder_accum accum;
};

static void robomaster_encoder_on_rotor(
  const struct device * motor, const struct robomaster_rotor_sample * rotor, void * user_data)
{
  const struct device * dev = user_data;
  const struct robomaster_encoder_config * config = dev->config;
  struct robomaster_encoder_data * data = dev->data;
  struct encoder_accum_sample sample;
  k_spinlock_key_t key;

  ARG_UNUSED(motor);

  key = k_spin_lock(&data->lock);

  if (!rotor->continuous) {
    encoder_accum_invalidate(&data->accum);
  }

  if (!encoder_accum_is_valid(&data->accum)) {
    /* The rotor angle is absolute within a turn, so it is where the position
     * starts, as on any absolute encoder. */
    const int64_t start = config->invert ? -(int64_t)rotor->orientation : rotor->orientation;

    encoder_accum_reset(&data->accum, start, rotor->orientation);
    data->feedback.position_epoch++;
  }

  encoder_accum_update(&data->accum, rotor->orientation, rotor->cycle, &sample);

  data->feedback.valid_mask = ENCODER_FEEDBACK_POSITION | ENCODER_FEEDBACK_SINGLE_TURN;
  data->feedback.position = sample.position;
  data->feedback.single_turn = rotor->orientation;

  if (sample.has_velocity) {
    data->feedback.valid_mask |= ENCODER_FEEDBACK_VELOCITY;
    data->feedback.velocity = sample.velocity;
    data->feedback.sample_interval_us = sample.sample_interval_us;
  }

  data->feedback.online = true;
  data->feedback.stale = false;
  data->feedback.timestamp_ms = k_uptime_get();

  k_spin_unlock(&data->lock, key);
}

static int robomaster_encoder_get_feedback(const struct device * dev, void * feedback)
{
  const struct robomaster_encoder_config * config = dev->config;
  struct robomaster_encoder_data * data = dev->data;
  struct encoder_feedback * out = feedback;
  struct motor_feedback motor_feedback;
  k_spinlock_key_t key;
  int ret;

  if (out == NULL) {
    return -EINVAL;
  }

  /* Freshness is the motor driver's call, so this answers -EAGAIN exactly
   * when the motor's own feedback does. */
  ret = motor_get_feedback(config->motor, &motor_feedback);

  key = k_spin_lock(&data->lock);

  *out = data->feedback;

  if (data->feedback.valid_mask == 0U) {
    ret = -ENODATA;
  } else if (ret == -EAGAIN) {
    out->stale = true;
  }

  k_spin_unlock(&data->lock, key);

  return ret;
}

static int robomaster_encoder_get_resolution(const struct device * dev, uint8_t * resolution)
{
  ARG_UNUSED(dev);

  if (resolution == NULL) {
    return -EINVAL;
  }

  *resolution = ROBOMASTER_ENCODER_BITS;

  return 0;
}

static int robomaster_encoder_set_zero(const struct device * dev)
{
  ARG_UNUSED(dev);

  /* The ESC has nowhere to store a zero point; encoder_set_position() is the
   * way to make the current position read as zero. */
  return -ENOTSUP;
}

static int robomaster_encoder_set_position(const struct device * dev, int64_t position)
{
  struct robomaster_encoder_data * data = dev->data;
  k_spinlock_key_t key;
  int ret;

  key = k_spin_lock(&data->lock);

  ret = encoder_accum_set_position(&data->accum, position);

  if (ret == 0) {
    data->feedback.position = position;
  }

  k_spin_unlock(&data->lock, key);

  return ret;
}

static int robomaster_encoder_reset(const struct device * dev)
{
  struct robomaster_encoder_data * data = dev->data;
  k_spinlock_key_t key;

  /* There is nothing on the ESC to reset. Starting over from the next frame
   * is what a reset means here: the position returns to the rotor angle and
   * the epoch advances. */
  key = k_spin_lock(&data->lock);

  encoder_accum_invalidate(&data->accum);
  data->feedback.valid_mask = 0U;

  k_spin_unlock(&data->lock, key);

  return 0;
}

static const struct encoder_driver_api robomaster_encoder_api = {
  .get_feedback = robomaster_encoder_get_feedback,
  .get_resolution = robomaster_encoder_get_resolution,
  .set_zero = robomaster_encoder_set_zero,
  .set_position = robomaster_encoder_set_position,
  .reset = robomaster_encoder_reset,
};

static int robomaster_encoder_init(const struct device * dev)
{
  const struct robomaster_encoder_config * config = dev->config;
  struct robomaster_encoder_data * data = dev->data;
  int ret;

  /* The phandle makes this device depend on the motor, so the motor has run
   * its init by now; a failed one shows here rather than as silence. */
  if (!device_is_ready(config->motor)) {
    LOG_ERR("%s: motor %s not ready", dev->name, config->motor->name);
    return -ENODEV;
  }

  encoder_accum_init(&data->accum, ROBOMASTER_ENCODER_BITS, config->invert);

  ret = robomaster_motor_set_rotor_callback(config->motor, robomaster_encoder_on_rotor, (void *)dev);
  if (ret < 0) {
    LOG_ERR("%s: could not listen to %s (%d)", dev->name, config->motor->name, ret);
    return ret;
  }

  return 0;
}

#define ROBOMASTER_ENCODER_INIT(inst)                                                     \
  static const struct robomaster_encoder_config robomaster_encoder_config_##inst = {      \
    .motor = DEVICE_DT_GET(DT_INST_PHANDLE(inst, motor)),                                 \
    .invert = DT_INST_PROP(inst, invert_direction),                                      \
  };                                                                                      \
                                                                                          \
  static struct robomaster_encoder_data robomaster_encoder_data_##inst;                   \
                                                                                          \
  DEVICE_DT_INST_DEFINE(                                                                  \
    inst, robomaster_encoder_init, NULL, &robomaster_encoder_data_##inst,                 \
    &robomaster_encoder_config_##inst, POST_KERNEL, CONFIG_ENCODER_INIT_PRIORITY,         \
    &robomaster_encoder_api);

DT_INST_FOREACH_STATUS_OKAY(ROBOMASTER_ENCODER_INIT)
