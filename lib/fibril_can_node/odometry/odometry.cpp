/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * The inputs of a dead reckoning on the bus. Binds the schema's Odometry block
 * type to the two encoders and the gyro the devicetree names, and publishes
 * what PCMainFW sent its host for fibril_localization.
 */

#define DT_DRV_COMPAT fibril_fcan_odometry

#include <cmath>
#include <cstdint>

#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <drivers/encoder.h>
#include <drivers/imu.h>
#include <fibril_can_node/func.h>

#include "schema_gen.hpp"

LOG_MODULE_REGISTER(fcan_odometry, CONFIG_FIBRIL_CAN_NODE_LOG_LEVEL);

BUILD_ASSERT(
  DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
  "exactly one enabled fibril,fcan-odometry node");

#define DEV_BY_IDX(node_id, prop, idx) DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx))

static const struct device * const encoders[] = {
  DT_INST_FOREACH_PROP_ELEM_SEP(0, encoders, DEV_BY_IDX, (, ))
};

/* OdometryEncoder has exactly encoder0 and encoder1. */
BUILD_ASSERT(ARRAY_SIZE(encoders) == 2, "fibril,fcan-odometry takes exactly two encoders");

static const struct device * const imu = DEVICE_DT_GET(DT_INST_PHANDLE(0, imu));

/* The count published for one encoder. It follows the encoder's position by
 * difference, so a rebuilt accumulator (position_epoch) does not show as a
 * jump that a dead reckoning would read as travel. */
struct count_state
{
  bool known;
  uint32_t epoch;
  int64_t position;
  int64_t count;
};

static count_state counts[ARRAY_SIZE(encoders)];

static void follow_encoder(uint8_t i)
{
  count_state & s = counts[i];
  struct encoder_feedback fb;
  const int ret = encoder_get_feedback(encoders[i], &fb);

  /* A stale reading still carries the last position, which is the best
   * there is; only an encoder that never read leaves the count alone. */
  if (((ret != 0) && (ret != -EAGAIN)) || ((fb.valid_mask & ENCODER_FEEDBACK_POSITION) == 0U)) {
    return;
  }

  if (s.known && (fb.position_epoch == s.epoch)) {
    s.count += fb.position - s.position;
  } else if (s.known) {
    LOG_WRN("encoder %u: position rebuilt, count continues from %lld", i, (long long)s.count);
  }

  s.known = true;
  s.epoch = fb.position_epoch;
  s.position = fb.position;
}

/* Roll, then pitch, then yaw about the fixed axes, as PCMainFW built it. */
static void orientation(float roll, float pitch, float yaw, float (&q)[4])
{
  const float cr = cosf(roll * 0.5F);
  const float sr = sinf(roll * 0.5F);
  const float cp = cosf(pitch * 0.5F);
  const float sp = sinf(pitch * 0.5F);
  const float cy = cosf(yaw * 0.5F);
  const float sy = sinf(yaw * 0.5F);

  q[0] = sr * cp * cy - cr * sp * sy;
  q[1] = cr * sp * cy + sr * cp * sy;
  q[2] = cr * cp * sy - sr * sp * cy;
  q[3] = cr * cp * cy + sr * sp * sy;
}

static void publish(void)
{
  struct imu_feedback fb = {};
  const int ret = imu_get_feedback(imu, &fb);
  float q[4];

  /* Before the first sample every value is zero, which still makes a valid
   * orientation; is_connecting says not to trust it. */
  orientation(fb.roll, fb.pitch, fb.yaw, q);

  if (auto pub = fcan_gen::odometry(0).publish_encoder()) {
    pub->encoder0 = counts[0].count;
    pub->encoder1 = counts[1].count;
    pub->is_connecting = (ret == 0);
    pub->roll = fb.roll;
    pub->pitch = fb.pitch;
    pub->yaw = fb.yaw;
  }

  if (auto pub = fcan_gen::odometry(0).publish_imu()) {
    for (size_t i = 0; i < 4; i++) {
      pub->orientation[i] = q[i];
    }
    for (size_t i = 0; i < 3; i++) {
      pub->angular_velocity[i] = fb.gyro[i];
      pub->linear_acceleration[i] = fb.accel[i];
    }
  }
}

static int odometry_func_init(void)
{
  for (size_t i = 0; i < ARRAY_SIZE(encoders); i++) {
    if (!device_is_ready(encoders[i])) {
      LOG_ERR("encoder %u: %s not ready", (unsigned)i, encoders[i]->name);
      return -ENODEV;
    }
  }

  if (!device_is_ready(imu)) {
    LOG_ERR("imu: %s not ready", imu->name);
    return -ENODEV;
  }

  return 0;
}

/* Every tick, so the periodic frames carry a sample at most a tick old. */
static void odometry_func_tick(void)
{
  for (uint8_t i = 0; i < (uint8_t)ARRAY_SIZE(encoders); i++) {
    follow_encoder(i);
  }

  publish();
}

FIBRIL_FCAN_FUNC_DEFINE(
  odometry,
  .array = fcan_gen::odometry::block_array_index,
  .count = 1,
  .init = odometry_func_init,
  .tick = odometry_func_tick);
