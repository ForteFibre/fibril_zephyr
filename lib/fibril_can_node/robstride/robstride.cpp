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

#include <cmath>

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/motor.h>
#include <drivers/motor/robstride.h>

#include <fibril_can_node/func.h>

#include "schema_gen.hpp"

LOG_MODULE_REGISTER(fcan_robstride, CONFIG_FIBRIL_CAN_NODE_LOG_LEVEL);

using enable_req = fcan_gen::robstridemotor::enable_req;
using enable_resp = fcan_gen::robstridemotor::enable_resp;
using fault_clear_req = fcan_gen::robstridemotor::fault_clear_req;
using fault_clear_resp = fcan_gen::robstridemotor::fault_clear_resp;
using reset_encoder_req = fcan_gen::robstridemotor::reset_encoder_req;
using reset_encoder_resp = fcan_gen::robstridemotor::reset_encoder_resp;
using save_parameters_req = fcan_gen::robstridemotor::save_parameters_req;
using save_parameters_resp = fcan_gen::robstridemotor::save_parameters_resp;

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
BUILD_ASSERT(ARRAY_SIZE(motors) <= fcan_gen::robstridemotor::max_count,
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

/* Bit positions of the mask the params-changed handler receives. The codegen
 * assigns them in the declaration order of `params:` in type.yaml, so
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

/*
 * Raised on the poll thread — by a service handler, or by the runtime's
 * parameter callback — and consumed by the tick.
 *
 * `enable` goes through here rather than being written straight into
 * motor_state, so that the state the output decision reads is only ever
 * touched by the tick. Reading it under the lock instead would not be
 * enough: a disable landing between that read and the driver call would
 * still be acted on one tick late, with the output turned on in between.
 */
struct requests
{
  bool enable;
  bool enable_value;
  bool arm;
  bool clear_trip;
  bool clear_faults;
  bool save;
  bool offset;
  bool offset_absolute;
  float offset_value;
  /* Bits of the codegen's changed mask, accumulated until the tick reads the
   * parameters they stand for. */
  uint32_t params_changed;
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

/* Guards the request block, which is the whole of what the poll thread
 * writes. Everything else in motor_state belongs to the tick. Held only
 * around the handoff; every call into the motor driver is made outside it.
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
  const fcan_gen::robstridemotor m(inst);

  out->gear_ratio = m.param_gear_ratio();
  out->sign = m.param_invert() ? -1.0F : 1.0F;
  out->max_temp_c = m.param_thermal__max_temp_c();
  out->command_timeout_ms = m.param_command_timeout_ms();
  out->motion_kp = m.param_motion__kp();
  out->motion_kd = m.param_motion__kd();
}

static void apply_gains(uint8_t inst)
{
  const fcan_gen::robstridemotor m(inst);
  const struct robstride_gains gains = {
    .position_kp = m.param_gains__position_kp(),
    .velocity_kp = m.param_gains__velocity_kp(),
    .velocity_ki = m.param_gains__velocity_ki(),
    .current_kp = m.param_gains__current_kp(),
    .current_ki = m.param_gains__current_ki(),
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

static void apply_target(uint8_t inst, const fcan_gen::robstridemotor::target & cmd, int64_t now)
{
  struct motor_state * s = &states[inst];
  const struct tunable * t = &s->tune;
  const struct device * dev = motors[inst];

  switch (cmd.type) {
    case TARGET_POSITION:
      (void)robstride_set_position_csp(dev, to_motor(t, cmd.value) - s->position_offset);
      break;
    case TARGET_TRAJECTORY:
      (void)robstride_set_position(dev, to_motor(t, cmd.value) - s->position_offset);
      break;
    case TARGET_VELOCITY:
      (void)robstride_set_velocity(dev, to_motor(t, cmd.value));
      break;
    case TARGET_TORQUE: {
      /* Operation control with both gains at zero is a pure torque command,
       * which keeps the newton metre the master sent a newton metre all the
       * way to the motor. */
      const struct robstride_motion_target target = {
        .torque = torque_to_motor(t, cmd.value),
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

      (void)robstride_set_current(dev, cmd.value * limits.current * t->sign);
      break;
    }
    default:
      LOG_WRN("motor %u: unknown target type %u", inst, cmd.type);
      return;
  }

  mark_commanded(inst, now);
}

static void apply_motion_target(
  uint8_t inst, const fcan_gen::robstridemotor::motion_target & cmd, int64_t now)
{
  struct motor_state * s = &states[inst];
  const struct tunable * t = &s->tune;
  const struct robstride_motion_target target = {
    .position = to_motor(t, cmd.position) - s->position_offset,
    .velocity = to_motor(t, cmd.velocity),
    /* NaN is how a consumer that does not hold this joint's gains asks for
     * whatever it is tuned to. */
    .kp = std::isnan(cmd.kp) ? t->motion_kp : cmd.kp,
    .kd = std::isnan(cmd.kd) ? t->motion_kd : cmd.kd,
    .torque = torque_to_motor(t, cmd.torque_ff),
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
  s->req = requests{};
  k_spin_unlock(&lock, key);

  if (req.params_changed != 0U) {
    tunable_read(inst, &s->tune);

    if ((req.params_changed & PARAM_MASK_GAINS) != 0U) {
      apply_gains(inst);
    }
  }

  if (req.enable) {
    s->enabled = req.enable_value;
  }

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
  auto pub = fcan_gen::robstridemotor(inst).publish_feedback();

  if (!pub) {
    return;
  }

  /* A motor with nothing to report publishes zeros rather than the last
   * measurement, so that a consumer watching position sees the link drop
   * instead of a value frozen at whatever it was. */
  if (!measured) {
    pub->is_ready = false;
    pub->position = 0.0F;
    pub->velocity = 0.0F;
    pub->output = 0.0F;
    return;
  }

  pub->is_ready = s->output_on && !s->overtemp &&
                  (fb->run_state == ROBSTRIDE_RUN_STATE_RUNNING) && (fb->error_code == 0U) &&
                  (fb->fault_bits == 0U);
  pub->position = to_load(t, fb->position + s->position_offset);
  pub->velocity = to_load(t, fb->velocity);
  pub->output = torque_to_load(t, fb->torque);
}

static void publish_diagnostics(uint8_t inst, const struct robstride_feedback * fb)
{
  const struct motor_state * s = &states[inst];
  auto pub = fcan_gen::robstridemotor(inst).publish_diagnostics();

  if (!pub) {
    return;
  }

  /* Unlike the feedback above, every field here stays readable when the motor
   * has gone quiet: they are what says why. */
  pub->online = fb->online;
  pub->stale = fb->stale;
  pub->enabled = s->output_on;
  pub->mode = (uint8_t)fb->mode;
  pub->run_state = (uint8_t)fb->run_state;
  pub->error_code = fb->error_code;
  pub->fault_bits = fb->fault_bits;
  pub->warning_bits = fb->warning_bits;
  pub->temperature = fb->temperature;
}

/* Fires from fcan_poll, so it only records what changed; the tick is what
 * re-reads the parameters and writes the gains, for the same reason the
 * service handlers defer.
 */
static void params_changed(uint8_t inst, uint32_t changed_mask)
{
  k_spinlock_key_t key = k_spin_lock(&lock);

  states[inst].req.params_changed |= changed_mask;
  k_spin_unlock(&lock, key);
}

static fcan_gen::svc_status enable_motor(uint8_t inst, const enable_req & req, enable_resp & resp)
{
  k_spinlock_key_t key = k_spin_lock(&lock);

  states[inst].req.enable = true;
  states[inst].req.enable_value = req.on;
  /* Enabling is also the way back from an overtemperature stop, and it counts
   * as traffic in its own right: a master that enables a joint and then says
   * nothing has still been heard from within the deadline. */
  states[inst].req.clear_trip = req.on;
  states[inst].req.arm = req.on;
  k_spin_unlock(&lock, key);

  resp.success = true;

  LOG_INF("motor %u -> %s", inst, req.on ? "enabled" : "disabled");

  return FCAN_SVC_OK;
}

static fcan_gen::svc_status clear_faults(
  uint8_t inst, const fault_clear_req & req, fault_clear_resp & resp)
{
  if (req.on) {
    k_spinlock_key_t key = k_spin_lock(&lock);

    states[inst].req.clear_faults = true;
    k_spin_unlock(&lock, key);
  }

  resp.success = true;

  return FCAN_SVC_OK;
}

static fcan_gen::svc_status reset_encoder(
  uint8_t inst, const reset_encoder_req & req, reset_encoder_resp & resp)
{
  k_spinlock_key_t key;

  /* Making the current position read as a given value needs a current
   * position to subtract from. Refusing here rather than letting the tick
   * drop the request is what keeps the reply honest; the snapshot never
   * blocks, so it is safe to take on this thread. A motor that goes stale
   * between here and the tick is still dropped there.
   */
  if (!req.offset) {
    struct robstride_feedback fb;

    if (robstride_get_feedback(motors[inst], &fb) != 0) {
      LOG_WRN("motor %u: reset_encoder refused, no position yet", inst);
      resp.success = false;
      return FCAN_SVC_APP_ERROR;
    }
  }

  key = k_spin_lock(&lock);
  states[inst].req.offset_absolute = req.offset;
  states[inst].req.offset_value = req.value;
  states[inst].req.offset = true;
  k_spin_unlock(&lock, key);

  /* Reaches the wire but has nowhere to land: ResetEncoder.srv answers with
   * nothing. See the note in type.yaml for why the field exists at all. */
  resp.success = true;

  return FCAN_SVC_OK;
}

static fcan_gen::svc_status save_parameters(uint8_t inst, save_parameters_resp & resp)
{
  k_spinlock_key_t key = k_spin_lock(&lock);

  states[inst].req.save = true;
  k_spin_unlock(&lock, key);

  /* The motor does not acknowledge a save, so this says the request was taken,
   * not that the parameters reached its memory. */
  resp.success = true;

  return FCAN_SVC_OK;
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

/* Handlers are registered per instance, so the bound index is the only thing
 * a handler captures and an out-of-range call never reaches one — the runtime
 * answers BAD_INDEX for a slot nobody claimed.
 */
static int robstride_func_start(void)
{
  for (uint8_t i = 0; i < (uint8_t)ARRAY_SIZE(motors); i++) {
    fcan_gen::robstridemotor m(i);

    m.on_enable([i](const enable_req & req, enable_resp & resp, fcan_gen::call_handle) noexcept {
      return enable_motor(i, req, resp);
    });
    m.on_fault_clear(
      [i](const fault_clear_req & req, fault_clear_resp & resp, fcan_gen::call_handle) noexcept {
        return clear_faults(i, req, resp);
      });
    m.on_reset_encoder(
      [i](const reset_encoder_req & req, reset_encoder_resp & resp,
          fcan_gen::call_handle) noexcept { return reset_encoder(i, req, resp); });
    m.on_save_parameters(
      [i](const save_parameters_req &, save_parameters_resp & resp,
          fcan_gen::call_handle) noexcept { return save_parameters(i, resp); });
    m.on_params_changed([i](uint32_t changed_mask) noexcept { params_changed(i, changed_mask); });

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
    const fcan_gen::robstridemotor m(i);
    struct robstride_feedback fb;
    const bool measured = robstride_get_feedback(motors[i], &fb) == 0;

    /* Order matters twice here. The requests come first so that an enable
     * arriving with a target is already in effect when the target lands. The
     * trip comes after them, so that an enable clearing the latch while the
     * motor is still too hot re-trips before anything is driven, rather than
     * energising for one tick. */
    drain_requests(i, &fb, measured, now);
    guard_temperature(i, &fb, measured);

    if (const auto target = m.read_target()) {
      apply_target(i, *target, now);
    }

    if (const auto motion = m.read_motion_target()) {
      apply_motion_target(i, *motion, now);
    }

    refresh_output(i, now);
    publish_feedback(i, &fb, measured);
    publish_diagnostics(i, &fb);
  }
}

FIBRIL_FCAN_FUNC_DEFINE(
  robstride,
  .array = fcan_gen::robstridemotor::block_array_index,
  .count = (uint8_t)ARRAY_SIZE(motors),
  .init = robstride_func_init,
  .start = robstride_func_start,
  .tick = robstride_func_tick);
