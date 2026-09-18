/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * RobStride actuators on the bus. Binds the schema's RobstrideMotor block
 * type to the motor devices the devicetree names, one instance per entry of
 * `motors`.
 *
 * Everything that touches the motor happens in the tick. The service
 * handlers run on whichever thread drives fcan_poll, which behind a CAN hub
 * is the driver's, so they raise a request and return; the tick consumes it.
 * That keeps the RobStride bus driven from one thread without the handlers
 * having to know which one they are on.
 */

#define DT_DRV_COMPAT fibril_fcan_robstride

#include <math.h>

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/motor.h>
#include <drivers/motor/robstride.h>

#include <fibril_can_node/func.h>

#include "schema_gen.h"

LOG_MODULE_REGISTER(fcan_robstride, CONFIG_FIBRIL_CAN_NODE_LOG_LEVEL);

/* One node names every actuator this board exposes, and the list below reads
 * instance 0 only. A second enabled node would build and then be invisible
 * from the bus.
 */
BUILD_ASSERT(
  DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
  "exactly one enabled fibril,fcan-robstride node; put every actuator in its motors");

#define MOTOR_DEV_BY_IDX(node_id, prop, idx) DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx))

static const struct device * const motors[] = {
  DT_INST_FOREACH_PROP_ELEM_SEP(0, motors, MOTOR_DEV_BY_IDX, (, ))
};

/* max_count is a Kconfig ceiling rather than the devicetree's length, because
 * CMake cannot read a phandle list. An extra actuator would otherwise be
 * silently unreachable from the bus.
 */
BUILD_ASSERT(ARRAY_SIZE(motors) <= FCAN_ROBSTRIDEMOTOR_MAX_COUNT,
             "more motors wired than CONFIG_FIBRIL_CAN_NODE_ROBSTRIDE_MAX");

/* fibril_control_msgs/msg/Target's constants. The message carries the output
 * domain as a number, so this table has to agree with that package.
 */
enum target_type {
  TARGET_DUTY = 0,
  TARGET_VELOCITY = 1,
  TARGET_TORQUE = 2,
  TARGET_POSITION = 3,
  TARGET_TRAJECTORY = 4,
};

/* Bit positions of the mask robstridemotor_on_params_changed() receives. The
 * codegen assigns them in the declaration order of `params:` in type.yaml, so
 * reordering there moves every bit here without breaking the build.
 */
enum param_bit {
  PARAM_BIT_GEAR_RATIO,
  PARAM_BIT_INVERT,
  PARAM_BIT_MAX_TEMP_C,
  PARAM_BIT_COMMAND_TIMEOUT_MS,
  PARAM_BIT_MOTION_KP,
  PARAM_BIT_MOTION_KD,
  PARAM_BIT_GAINS_POSITION_KP,
  PARAM_BIT_GAINS_VELOCITY_KP,
  PARAM_BIT_GAINS_VELOCITY_KI,
  PARAM_BIT_GAINS_CURRENT_KP,
  PARAM_BIT_GAINS_CURRENT_KI,
};

#define PARAM_MASK_GAINS                                                                 \
  (BIT(PARAM_BIT_GAINS_POSITION_KP) | BIT(PARAM_BIT_GAINS_VELOCITY_KP) |                 \
   BIT(PARAM_BIT_GAINS_VELOCITY_KI) | BIT(PARAM_BIT_GAINS_CURRENT_KP) |                  \
   BIT(PARAM_BIT_GAINS_CURRENT_KI))

/*
 * Snapshot of the parameters a conversion needs. Taken as a set rather than
 * read one call at a time, so a parameter write landing between two lines
 * cannot scale a position by one gear ratio and the velocity beside it by
 * another.
 */
struct tunable
{
  float gear_ratio;
  float sign;
  float max_temp_c;
  uint32_t command_timeout_ms;
  float motion_kp;
  float motion_kd;
};

/* Raised by a service handler, consumed by the tick. */
struct requests
{
  bool arm;
  bool clear_trip;
  bool clear_faults;
  bool save;
  bool offset;
  bool offset_absolute;
  float offset_value;
};

struct motor_state
{
  struct tunable tune;

  struct requests req;

  /* What the master last asked for through the enable service. */
  bool enabled;

  /* Shift from the motor's own origin to the reported one, on the motor side
   * and in rad. Held here rather than written into the motor, so that a
   * re-zero costs no bus traffic and survives nothing.
   */
  float position_offset;

  int64_t last_command_ms;
  bool commanded;

  /* What the driver was last told, so a disable is not re-sent every tick. */
  bool output_on;

  /* Software overtemperature trip. Latched: cooling down is not on its own a
   * reason to start driving a joint again.
   */
  bool overtemp;
};

static struct motor_state states[ARRAY_SIZE(motors)];

/* Guards what a service handler writes — the request block and `enabled` —
 * against the tick that consumes it. Held only around the handoff; every call
 * into the motor driver is made outside it.
 */
static struct k_spinlock lock;

static float to_motor(const struct tunable * t, float load)
{
  return load * t->gear_ratio * t->sign;
}

static float to_load(const struct tunable * t, float motor)
{
  return (motor / t->gear_ratio) * t->sign;
}

static float torque_to_motor(const struct tunable * t, float load)
{
  return (load / t->gear_ratio) * t->sign;
}

static float torque_to_load(const struct tunable * t, float motor)
{
  return motor * t->gear_ratio * t->sign;
}

static void tunable_read(uint8_t inst, struct tunable * out)
{
  out->gear_ratio = robstridemotor_param_gear_ratio(inst);
  out->sign = robstridemotor_param_invert(inst) ? -1.0F : 1.0F;
  out->max_temp_c = robstridemotor_param_thermal__max_temp_c(inst);
  out->command_timeout_ms = robstridemotor_param_command_timeout_ms(inst);
  out->motion_kp = robstridemotor_param_motion__kp(inst);
  out->motion_kd = robstridemotor_param_motion__kd(inst);
}

static void apply_gains(uint8_t inst)
{
  const struct robstride_gains gains = {
    .position_kp = robstridemotor_param_gains__position_kp(inst),
    .velocity_kp = robstridemotor_param_gains__velocity_kp(inst),
    .velocity_ki = robstridemotor_param_gains__velocity_ki(inst),
    .current_kp = robstridemotor_param_gains__current_kp(inst),
    .current_ki = robstridemotor_param_gains__current_ki(inst),
  };

  (void)robstride_set_gains(motors[inst], &gains);
}

static void mark_commanded(uint8_t inst, int64_t now)
{
  states[inst].commanded = true;
  states[inst].last_command_ms = now;
}

static void set_output(uint8_t inst, bool on)
{
  struct motor_state * s = &states[inst];
  int ret;

  if (s->output_on == on) {
    return;
  }

  ret = on ? motor_enable(motors[inst]) : motor_disable(motors[inst]);

  if (ret != 0) {
    LOG_ERR("motor %u: %s failed (%d)", inst, on ? "enable" : "disable", ret);
  }

  /* Recorded as done even when the frame could not be queued. The driver
   * drops its own enabled flag before it tries to send and re-sends the stop
   * every interval, so retrying from here would add a second retry loop —
   * and on a board whose motor supply is off, one that logs at the tick
   * rate for as long as it stays off.
   */
  s->output_on = on;
}

/*
 * The single place that decides whether the output may be driven. Called after
 * anything that can change one of the three answers, so a stale reason never
 * survives a tick.
 */
static void refresh_output(uint8_t inst, int64_t now)
{
  const struct motor_state * s = &states[inst];
  const uint32_t timeout = s->tune.command_timeout_ms;
  bool fresh = s->commanded;

  if (fresh && (timeout != 0U)) {
    fresh = (now - s->last_command_ms) <= (int64_t)timeout;
  }

  set_output(inst, s->enabled && !s->overtemp && fresh);
}

static void apply_target(uint8_t inst, const robstridemotor_target_t * cmd, int64_t now)
{
  struct motor_state * s = &states[inst];
  const struct tunable * t = &s->tune;
  const struct device * dev = motors[inst];

  switch (cmd->type) {
    case TARGET_POSITION:
      (void)robstride_set_position_csp(dev, to_motor(t, cmd->value) - s->position_offset);
      break;
    case TARGET_TRAJECTORY:
      (void)robstride_set_position(dev, to_motor(t, cmd->value) - s->position_offset);
      break;
    case TARGET_VELOCITY:
      (void)robstride_set_velocity(dev, to_motor(t, cmd->value));
      break;
    case TARGET_TORQUE: {
      /* Operation control with both gains at zero is a pure torque command,
       * which keeps the newton metre the master sent a newton metre all the
       * way to the motor. */
      const struct robstride_motion_target target = {
        .torque = torque_to_motor(t, cmd->value),
      };

      (void)robstride_set_motion_target(dev, &target);
      break;
    }
    case TARGET_DUTY: {
      /* Full scale is the motor's own current limit, which is what the
       * devicetree's max-current-ma set. There is no other denominator the
       * node can name without holding a copy of the model table. */
      struct robstride_limits limits;

      if (robstride_get_limits(dev, &limits) != 0) {
        return;
      }

      (void)robstride_set_current(dev, cmd->value * limits.current * t->sign);
      break;
    }
    default:
      LOG_WRN("motor %u: unknown target type %u", inst, cmd->type);
      return;
  }

  mark_commanded(inst, now);
}

static void apply_motion_target(
  uint8_t inst, const robstridemotor_motion_target_t * cmd, int64_t now)
{
  struct motor_state * s = &states[inst];
  const struct tunable * t = &s->tune;
  const struct robstride_motion_target target = {
    .position = to_motor(t, cmd->position) - s->position_offset,
    .velocity = to_motor(t, cmd->velocity),
    /* NaN is how a consumer that does not hold this joint's gains asks for
     * whatever it is tuned to. */
    .kp = isnan(cmd->kp) ? t->motion_kp : cmd->kp,
    .kd = isnan(cmd->kd) ? t->motion_kd : cmd->kd,
    .torque = torque_to_motor(t, cmd->torque_ff),
  };

  (void)robstride_set_motion_target(motors[inst], &target);
  mark_commanded(inst, now);
}

/* Consumes what the service handlers left behind. */
static void drain_requests(uint8_t inst, const struct robstride_feedback * fb, bool measured,
                           int64_t now)
{
  struct motor_state * s = &states[inst];
  struct requests req;
  k_spinlock_key_t key;

  key = k_spin_lock(&lock);
  req = s->req;
  s->req = (struct requests){0};
  k_spin_unlock(&lock, key);

  if (req.clear_trip) {
    s->overtemp = false;
  }

  if (req.arm) {
    mark_commanded(inst, now);
  }

  if (req.clear_faults) {
    int ret = robstride_clear_faults(motors[inst]);

    if (ret != 0) {
      LOG_ERR("motor %u: fault clear failed (%d)", inst, ret);
    }
  }

  if (req.save) {
    int ret = robstride_save_parameters(motors[inst]);

    if (ret != 0) {
      LOG_ERR("motor %u: save failed (%d)", inst, ret);
    }
  }

  if (!req.offset) {
    return;
  }

  if (req.offset_absolute) {
    s->position_offset = to_motor(&s->tune, req.offset_value);
    return;
  }

  /* Making the current position read as a given value needs a current
   * position to subtract, so this one waits for the motor to have answered. */
  if (!measured) {
    LOG_WRN("motor %u: reset_encoder ignored, no position yet", inst);
    return;
  }

  s->position_offset = to_motor(&s->tune, req.offset_value) - fb->position;
}

static void guard_temperature(uint8_t inst, const struct robstride_feedback * fb, bool measured)
{
  struct motor_state * s = &states[inst];

  if (!measured || (fb->temperature <= s->tune.max_temp_c)) {
    return;
  }

  if (!s->overtemp) {
    LOG_ERR(
      "motor %u: over temperature %d C > %d C, output off", inst, (int)fb->temperature,
      (int)s->tune.max_temp_c);
  }

  s->overtemp = true;
}

static void publish_feedback(uint8_t inst, const struct robstride_feedback * fb, bool measured)
{
  const struct motor_state * s = &states[inst];
  const struct tunable * t = &s->tune;
  robstridemotor_feedback_t * out = robstridemotor_feedback_begin(inst);

  if (out == NULL) {
    return;
  }

  /* A motor with nothing to report publishes zeros rather than the last
   * measurement, so that a consumer watching position sees the link drop
   * instead of a value frozen at whatever it was. */
  if (!measured) {
    out->is_ready = false;
    out->position = 0.0F;
    out->velocity = 0.0F;
    out->output = 0.0F;
    robstridemotor_feedback_commit(inst);
    return;
  }

  out->is_ready = s->output_on && !s->overtemp &&
                  (fb->run_state == ROBSTRIDE_RUN_STATE_RUNNING) && (fb->error_code == 0U) &&
                  (fb->fault_bits == 0U);
  out->position = to_load(t, fb->position + s->position_offset);
  out->velocity = to_load(t, fb->velocity);
  out->output = torque_to_load(t, fb->torque);

  robstridemotor_feedback_commit(inst);
}

static void publish_diagnostics(uint8_t inst, const struct robstride_feedback * fb)
{
  const struct motor_state * s = &states[inst];
  robstridemotor_diagnostics_t * out = robstridemotor_diagnostics_begin(inst);

  if (out == NULL) {
    return;
  }

  /* Unlike the feedback above, every field here stays readable when the motor
   * has gone quiet: they are what says why. */
  out->online = fb->online;
  out->stale = fb->stale;
  out->enabled = s->output_on;
  out->mode = (uint8_t)fb->mode;
  out->run_state = (uint8_t)fb->run_state;
  out->error_code = fb->error_code;
  out->fault_bits = fb->fault_bits;
  out->warning_bits = fb->warning_bits;
  out->temperature = fb->temperature;

  robstridemotor_diagnostics_commit(inst);
}

static int robstride_func_init(void)
{
  for (size_t i = 0; i < ARRAY_SIZE(motors); i++) {
    if (!device_is_ready(motors[i])) {
      LOG_ERR("motor %u: %s not ready", (unsigned)i, motors[i]->name);
      return -ENODEV;
    }
  }

  return 0;
}

static int robstride_func_start(void)
{
  for (uint8_t i = 0; i < (uint8_t)ARRAY_SIZE(motors); i++) {
    tunable_read(i, &states[i].tune);

    /* Written unconditionally, so that a motor whose gains were edited over
     * its own USB link comes back to the schema's values rather than
     * behaving differently from the identical joint beside it. */
    apply_gains(i);

    /* The ROS 2 original starts with the motor enabled and only a manual
     * disable holding it back. Nothing is driven yet: the output stays off
     * until the first command arrives. */
    states[i].enabled = true;
  }

  return 0;
}

static void robstride_func_tick(void)
{
  const int64_t now = k_uptime_get();

  for (uint8_t i = 0; i < (uint8_t)ARRAY_SIZE(motors); i++) {
    struct robstride_feedback fb;
    const bool measured = robstride_get_feedback(motors[i], &fb) == 0;
    robstridemotor_target_t target;
    robstridemotor_motion_target_t motion;

    /* Order matters twice here. The requests come first so that an enable
     * arriving with a target is already in effect when the target lands. The
     * trip comes after them, so that an enable clearing the latch while the
     * motor is still too hot re-trips before anything is driven, rather than
     * energising for one tick. */
    drain_requests(i, &fb, measured, now);
    guard_temperature(i, &fb, measured);

    if (robstridemotor_target_read(i, &target)) {
      apply_target(i, &target, now);
    }

    if (robstridemotor_motion_target_read(i, &motion)) {
      apply_motion_target(i, &motion, now);
    }

    refresh_output(i, now);
    publish_feedback(i, &fb, measured);
    publish_diagnostics(i, &fb);
  }
}

void robstridemotor_on_params_changed(uint8_t inst, uint32_t changed_mask)
{
  if (inst >= (uint8_t)ARRAY_SIZE(motors)) {
    return;
  }

  tunable_read(inst, &states[inst].tune);

  if ((changed_mask & PARAM_MASK_GAINS) != 0U) {
    apply_gains(inst);
  }
}

fcan_svc_status_t robstridemotor_enable(
  uint8_t inst, const robstridemotor_enable_req_t * req, robstridemotor_enable_resp_t * resp,
  fcan_call_handle_t h)
{
  k_spinlock_key_t key;

  ARG_UNUSED(h);

  if (inst >= (uint8_t)ARRAY_SIZE(motors)) {
    resp->success = false;
    return FCAN_SVC_APP_ERROR;
  }

  key = k_spin_lock(&lock);
  states[inst].enabled = req->on;
  /* Enabling is also the way back from an overtemperature stop, and it counts
   * as traffic in its own right: a master that enables a joint and then says
   * nothing has still been heard from within the deadline. */
  states[inst].req.clear_trip = req->on;
  states[inst].req.arm = req->on;
  k_spin_unlock(&lock, key);

  resp->success = true;

  LOG_INF("motor %u -> %s", inst, req->on ? "enabled" : "disabled");

  return FCAN_SVC_OK;
}

fcan_svc_status_t robstridemotor_fault_clear(
  uint8_t inst, const robstridemotor_fault_clear_req_t * req,
  robstridemotor_fault_clear_resp_t * resp, fcan_call_handle_t h)
{
  k_spinlock_key_t key;

  ARG_UNUSED(h);

  if (inst >= (uint8_t)ARRAY_SIZE(motors)) {
    resp->success = false;
    return FCAN_SVC_APP_ERROR;
  }

  if (req->on) {
    key = k_spin_lock(&lock);
    states[inst].req.clear_faults = true;
    k_spin_unlock(&lock, key);
  }

  resp->success = true;

  return FCAN_SVC_OK;
}

fcan_svc_status_t robstridemotor_reset_encoder(
  uint8_t inst, const robstridemotor_reset_encoder_req_t * req,
  robstridemotor_reset_encoder_resp_t * resp, fcan_call_handle_t h)
{
  k_spinlock_key_t key;

  ARG_UNUSED(h);

  if (inst >= (uint8_t)ARRAY_SIZE(motors)) {
    resp->success = false;
    return FCAN_SVC_APP_ERROR;
  }

  key = k_spin_lock(&lock);
  states[inst].req.offset_absolute = req->offset;
  states[inst].req.offset_value = req->value;
  states[inst].req.offset = true;
  k_spin_unlock(&lock, key);

  /* Reaches the wire but has nowhere to land: ResetEncoder.srv answers with
   * nothing. See the note in type.yaml for why the field exists at all. */
  resp->success = true;

  return FCAN_SVC_OK;
}

fcan_svc_status_t robstridemotor_save_parameters(
  uint8_t inst, const robstridemotor_save_parameters_req_t * req,
  robstridemotor_save_parameters_resp_t * resp, fcan_call_handle_t h)
{
  k_spinlock_key_t key;

  ARG_UNUSED(req);
  ARG_UNUSED(h);

  if (inst >= (uint8_t)ARRAY_SIZE(motors)) {
    resp->success = false;
    return FCAN_SVC_APP_ERROR;
  }

  key = k_spin_lock(&lock);
  states[inst].req.save = true;
  k_spin_unlock(&lock, key);

  /* The motor does not acknowledge a save, so this says the request was taken,
   * not that the parameters reached its memory. */
  resp->success = true;

  return FCAN_SVC_OK;
}

FIBRIL_FCAN_FUNC_DEFINE(
  robstride,
  .array = FCAN_ARRAY_ROBSTRIDEMOTOR,
  .count = (uint8_t)ARRAY_SIZE(motors),
  .init = robstride_func_init,
  .start = robstride_func_start,
  .tick = robstride_func_tick);
