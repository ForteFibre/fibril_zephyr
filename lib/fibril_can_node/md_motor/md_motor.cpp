/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Motors closed around encoders on the board. Binds the schema's MdMotor
 * block type to the motor and encoder devices the devicetree pairs, one
 * instance per pair, and runs lib/motor_control for each of them in the tick.
 *
 * Everything that touches a motor or an encoder happens in the tick. The
 * service and parameter handlers run on whichever thread drives fcan_poll,
 * so they raise a request and return; the tick consumes it.
 */

#define DT_DRV_COMPAT fibril_fcan_md_motor

#include <cmath>
#include <cstdint>

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <drivers/encoder.h>
#include <drivers/motor.h>
#include <motor_control/motor_control.hpp>

#include <fibril_can_node/func.h>

#include "schema_gen.hpp"

LOG_MODULE_REGISTER(fcan_md_motor, CONFIG_FIBRIL_CAN_NODE_LOG_LEVEL);

using reset_encoder_req = fcan_gen::mdmotor::reset_encoder_req;
using reset_encoder_resp = fcan_gen::mdmotor::reset_encoder_resp;
using reset_safety_req = fcan_gen::mdmotor::reset_safety_req;
using reset_safety_resp = fcan_gen::mdmotor::reset_safety_resp;

BUILD_ASSERT(
  DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
  "exactly one enabled fibril,fcan-md-motor node; put every motor in its motors");

#define DEV_BY_IDX(node_id, prop, idx) DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx))

static const struct device * const motors[] = {
  DT_INST_FOREACH_PROP_ELEM_SEP(0, motors, DEV_BY_IDX, (, ))
};

static const struct device * const encoders[] = {
  DT_INST_FOREACH_PROP_ELEM_SEP(0, encoders, DEV_BY_IDX, (, ))
};

BUILD_ASSERT(ARRAY_SIZE(motors) == ARRAY_SIZE(encoders),
             "fibril,fcan-md-motor pairs motors and encoders by position; the lists differ in length");
BUILD_ASSERT(ARRAY_SIZE(motors) <= fcan_gen::mdmotor::max_count,
             "more motors wired than CONFIG_FIBRIL_CAN_NODE_MD_MOTOR_MAX");

static constexpr float duty_full_scale = DT_INST_PROP(0, duty_full_scale);

/* fibril_control_msgs/msg/Target's constants. */
enum target_type {
  TARGET_DUTY = 0,
  TARGET_VELOCITY = 1,
  TARGET_TORQUE = 2,
  TARGET_POSITION = 3,
  TARGET_TRAJECTORY = 4,
};

/*
 * Raised on the poll thread — by a service handler, or by the runtime's
 * parameter callback — and consumed by the tick.
 */
struct requests
{
  bool reset_encoder;
  bool reset_encoder_offset;
  float reset_encoder_value;
  bool reset_safety;
  /* Any bit at all re-reads every parameter; there are few enough of them
   * that tracking which changed would cost more than reading them. */
  uint32_t params_changed;
};

struct motor_state
{
  motor_control::MotorControl control;

  struct requests req;

  /* Parameters the control core does not hold, read with the rest. */
  float sign;
  float encoder_gain;
  float accel_ff_gain;
  bool cascade;

  /* Generations of the last command applied from each M2S topic. The topics
   * keep their last value, so only a change here says a new frame arrived. */
  uint32_t target_seq;
  uint32_t trajectory_seq;

  /* The encoder's accumulator generation, to notice a rebuilt position. */
  uint32_t position_epoch;
  bool epoch_known;

  bool encoder_valid;
};

static struct motor_state states[ARRAY_SIZE(motors)];

/* Guards the request blocks, the whole of what the poll thread writes. */
static struct k_spinlock lock;

static void apply_params(uint8_t inst)
{
  const fcan_gen::mdmotor m(inst);
  struct motor_state * s = &states[inst];
  motor_control::MotorControl & c = s->control;

  s->sign = m.param_invert() ? -1.0F : 1.0F;
  s->encoder_gain = m.param_encoder__gain();
  s->accel_ff_gain = m.param_accel_ff_gain();
  s->cascade = m.param_use_cascade_position();

  c.encoder_gain(s->encoder_gain);
  /* The schema's min and max already bound this, so a refusal here means the
   * two disagree. */
  if (!c.speed_filter_coefficient(m.param_velocity__filter_coe())) {
    LOG_WRN("motor %u: velocity/filter_coe out of range, kept", inst);
  }
  c.speed_controller()
    .kp(m.param_velocity__kp())
    .ki(m.param_velocity__ki())
    .kd(m.param_velocity__kd())
    .max(m.param_velocity__max())
    .i_saturation(m.param_velocity__i_sat());
  c.position_controller()
    .kp(m.param_position__kp())
    .ki(m.param_position__ki())
    .kd(m.param_position__kd())
    .max(m.param_position__max())
    .i_saturation(m.param_position__i_sat());
  c.friction_coulomb(m.param_friction__coulomb());
  c.friction_viscous(m.param_friction__viscous());
  c.duty_slew_limit(m.param_duty__slew_rate());
  c.stall_timeout_ticks(
    m.param_stall__timeout_ms() * (1000U / motor_control::MotorControl::tick_us));
}

/*
 * One encoder reading in the control core's terms, with `invert` applied. A
 * rebuilt accumulator reads as invalid for the tick it is noticed on: the
 * position jumped, so the closed loop drops to DUTY 0 and waits for the
 * master to command again rather than chase the jump.
 */
static motor_control::EncoderSample sample_encoder(uint8_t inst)
{
  struct motor_state * s = &states[inst];
  struct encoder_feedback fb;
  const uint32_t needed = ENCODER_FEEDBACK_POSITION | ENCODER_FEEDBACK_VELOCITY;

  if ((encoder_get_feedback(encoders[inst], &fb) != 0) || ((fb.valid_mask & needed) != needed)) {
    return {false, 0, 0.0F};
  }

  const bool rebuilt = s->epoch_known && (fb.position_epoch != s->position_epoch);

  s->position_epoch = fb.position_epoch;
  s->epoch_known = true;

  if (rebuilt) {
    LOG_WRN("motor %u: encoder position rebuilt, output stopped", inst);
    return {false, 0, 0.0F};
  }

  const bool inverted = s->sign < 0.0F;

  return {
    true,
    inverted ? -fb.position : fb.position,
    inverted ? -(float)fb.velocity : (float)fb.velocity,
  };
}

/* Changes the encoder's reported position; the control core follows on the
 * next sample. */
static void reset_encoder_position(uint8_t inst, bool offset, float value)
{
  struct motor_state * s = &states[inst];
  struct encoder_feedback fb;
  int ret;

  if (s->encoder_gain == 0.0F) {
    LOG_WRN("motor %u: reset_encoder ignored, encoder/gain is 0", inst);
    return;
  }

  /* User units back to raw counts, undoing `invert` as the sample does. */
  const int64_t counts = (int64_t)std::llround(value * s->sign / s->encoder_gain);

  if (!offset) {
    ret = encoder_set_position(encoders[inst], counts);
  } else {
    ret = encoder_get_feedback(encoders[inst], &fb);
    if (ret == 0) {
      ret = encoder_set_position(encoders[inst], fb.position - counts);
    }
  }

  if (ret != 0) {
    LOG_WRN("motor %u: reset_encoder failed (%d)", inst, ret);
  }
}

static void drain_requests(uint8_t inst)
{
  struct motor_state * s = &states[inst];
  struct requests req;
  k_spinlock_key_t key;

  key = k_spin_lock(&lock);
  req = s->req;
  s->req = requests{};
  k_spin_unlock(&lock, key);

  if (req.params_changed != 0U) {
    apply_params(inst);
  }

  if (req.reset_safety) {
    s->control.reset_safety();
  }

  if (req.reset_encoder) {
    reset_encoder_position(inst, req.reset_encoder_offset, req.reset_encoder_value);
  }
}

static void apply_target(uint8_t inst, const fcan_gen::mdmotor::target & cmd)
{
  struct motor_state * s = &states[inst];

  switch (cmd.type) {
    case TARGET_DUTY:
      s->control.set_duty(cmd.value);
      break;
    case TARGET_VELOCITY:
      s->control.set_speed(cmd.value);
      break;
    case TARGET_POSITION:
      if (s->cascade) {
        s->control.set_position_speed(cmd.value);
      } else {
        s->control.set_position(cmd.value);
      }
      break;
    default:
      LOG_WRN("motor %u: unsupported target type %u", inst, cmd.type);
      break;
  }
}

static void apply_trajectory(uint8_t inst, const fcan_gen::mdmotor::trajectory & point)
{
  struct motor_state * s = &states[inst];

  s->control.set_position_speed(point.positions[0]);
  s->control.set_velocity_ff(point.velocities[0]);
  s->control.set_duty_ff(s->accel_ff_gain * point.accelerations[0]);
}

/*
 * Applies whichever command arrived since the last tick. Both arriving in one
 * tick means the master switched between them within a millisecond; the
 * trajectory point wins because it is the one that has to keep streaming.
 */
static void apply_commands(uint8_t inst)
{
  struct motor_state * s = &states[inst];
  const fcan_gen::mdmotor m(inst);
  uint32_t target_seq = s->target_seq;
  uint32_t trajectory_seq = s->trajectory_seq;
  const auto target = m.read_target(target_seq);
  const auto trajectory = m.read_trajectory(trajectory_seq);
  const bool new_target = target && (target_seq != s->target_seq);
  const bool new_trajectory = trajectory && (trajectory_seq != s->trajectory_seq);

  if (target) {
    s->target_seq = target_seq;
  }
  if (trajectory) {
    s->trajectory_seq = trajectory_seq;
  }

  if (new_trajectory) {
    apply_trajectory(inst, *trajectory);
  } else if (new_target) {
    apply_target(inst, *target);
  }
}

static void drive(uint8_t inst, float duty)
{
  const float raw = duty * states[inst].sign * duty_full_scale;
  const int16_t output = (int16_t)CLAMP(std::lround(raw), INT16_MIN, INT16_MAX);
  const int ret = motor_set_output(motors[inst], MOTOR_OUTPUT_MODE_CURRENT, output);

  if (ret != 0) {
    LOG_WRN_ONCE("motor %u: set_output failed (%d)", inst, ret);
  }
}

static void publish_feedback(uint8_t inst)
{
  const struct motor_state * s = &states[inst];
  auto pub = fcan_gen::mdmotor(inst).publish_feedback();

  if (!pub) {
    return;
  }

  /* While the encoder is out, the last position and speed stay on the wire
   * and is_ready says not to trust them, as CanMotorMbed's status bits did. */
  pub->is_ready = s->encoder_valid && !s->control.stalled();
  pub->velocity = s->control.speed();
  pub->position = s->control.position();
  pub->output = s->control.duty();
}

static void params_changed(uint8_t inst, uint32_t changed_mask)
{
  k_spinlock_key_t key = k_spin_lock(&lock);

  states[inst].req.params_changed |= changed_mask;
  k_spin_unlock(&lock, key);
}

static fcan_gen::svc_status reset_encoder(
  uint8_t inst, const reset_encoder_req & req, reset_encoder_resp & resp)
{
  struct encoder_feedback fb;
  k_spinlock_key_t key;

  /* Refused here rather than dropped in the tick, so the reply says whether
   * there was a position to move. The snapshot never blocks. */
  if (encoder_get_feedback(encoders[inst], &fb) != 0) {
    LOG_WRN("motor %u: reset_encoder refused, no position yet", inst);
    resp.success = false;
    return FCAN_SVC_APP_ERROR;
  }

  key = k_spin_lock(&lock);
  states[inst].req.reset_encoder = true;
  states[inst].req.reset_encoder_offset = req.offset;
  states[inst].req.reset_encoder_value = req.value;
  k_spin_unlock(&lock, key);

  resp.success = true;

  return FCAN_SVC_OK;
}

static fcan_gen::svc_status reset_safety(uint8_t inst, reset_safety_resp & resp)
{
  k_spinlock_key_t key = k_spin_lock(&lock);

  states[inst].req.reset_safety = true;
  k_spin_unlock(&lock, key);

  resp.success = true;

  return FCAN_SVC_OK;
}

static int md_motor_func_init(void)
{
  for (size_t i = 0; i < ARRAY_SIZE(motors); i++) {
    if (!device_is_ready(motors[i])) {
      LOG_ERR("motor %u: %s not ready", (unsigned)i, motors[i]->name);
      return -ENODEV;
    }
    if (!device_is_ready(encoders[i])) {
      LOG_ERR("motor %u: encoder %s not ready", (unsigned)i, encoders[i]->name);
      return -ENODEV;
    }
  }

  return 0;
}

static int md_motor_func_start(void)
{
  for (uint8_t i = 0; i < (uint8_t)ARRAY_SIZE(motors); i++) {
    fcan_gen::mdmotor m(i);

    m.on_reset_encoder(
      [i](const reset_encoder_req & req, reset_encoder_resp & resp,
          fcan_gen::call_handle) noexcept { return reset_encoder(i, req, resp); });
    m.on_reset_safety(
      [i](const reset_safety_req &, reset_safety_resp & resp, fcan_gen::call_handle) noexcept {
        return reset_safety(i, resp);
      });
    m.on_params_changed([i](uint32_t changed_mask) noexcept { params_changed(i, changed_mask); });

    apply_params(i);

    /* The output is gated by the control core, which starts at DUTY 0, so the
     * motor can be enabled from the start, as CanMotorMbed drove it. */
    const int ret = motor_enable(motors[i]);

    if (ret != 0) {
      LOG_ERR("motor %u: enable failed (%d)", i, ret);
    }
  }

  return 0;
}

static void md_motor_func_tick(void)
{
  for (uint8_t i = 0; i < (uint8_t)ARRAY_SIZE(motors); i++) {
    struct motor_state * s = &states[i];

    /* Requests first, so new gains and a reset encoder are in effect for the
     * sample and the command this tick reads. */
    drain_requests(i);

    const motor_control::EncoderSample sample = sample_encoder(i);

    s->encoder_valid = sample.valid;
    apply_commands(i);
    drive(i, s->control.update(sample));
    publish_feedback(i);
  }
}

FIBRIL_FCAN_FUNC_DEFINE(
  md_motor,
  .array = fcan_gen::mdmotor::block_array_index,
  .count = (uint8_t)ARRAY_SIZE(motors),
  .init = md_motor_func_init,
  .start = md_motor_func_start,
  .tick = md_motor_func_tick);
