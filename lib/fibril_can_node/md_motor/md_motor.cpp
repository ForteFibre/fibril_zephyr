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
 * so they raise a request and return; the tick consumes it. The deferred
 * replies, calibrate_current_baseline's and trigger's, are handed to the
 * runtime from the tick when they are due, and fcan_poll sends them (ADR 0013,
 * ADR 0018).
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
#include <motor_control/sensing.hpp>

#include <fibril_can_node/adc_port.hpp>
#include <fibril_can_node/func.h>

#include "schema_gen.hpp"

LOG_MODULE_REGISTER(fcan_md_motor, CONFIG_FIBRIL_CAN_NODE_LOG_LEVEL);

using reset_encoder_req = fcan_gen::mdmotor::reset_encoder_req;
using reset_encoder_resp = fcan_gen::mdmotor::reset_encoder_resp;
using reset_safety_req = fcan_gen::mdmotor::reset_safety_req;
using reset_safety_resp = fcan_gen::mdmotor::reset_safety_resp;
using sensor_state_req = fcan_gen::mdmotor::sensor_state_req;
using sensor_state_resp = fcan_gen::mdmotor::sensor_state_resp;
using baseline_req = fcan_gen::mdmotor::calibrate_current_baseline_req;
using baseline_resp = fcan_gen::mdmotor::calibrate_current_baseline_resp;
using trigger_req = fcan_gen::mdmotor::trigger_req;
using trigger_resp = fcan_gen::mdmotor::trigger_resp;
using trigger_cancel_req = fcan_gen::mdmotor::trigger_cancel_req;
using trigger_cancel_resp = fcan_gen::mdmotor::trigger_cancel_resp;

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

/* CanMotorMbed's RoboMaster read_current(). */
static constexpr float current_full_scale = 1000.0F;

/* `adc/port` naming the motor's own index. */
static constexpr uint8_t adc_port_own = 255;

/* The current baseline only averages while the motor holds still: DUTY 0, and
 * slower than CanMotorMbed's 0.1 user units per 1 ms tick. */
static constexpr float still_duty = 1e-3F;
static constexpr float still_speed = 0.1F / motor_control::MotorControl::tick_s;

/* What current/baseline_duration_s is clamped to. The upper bound dates from
 * when the bridge timed out deferred replies; see type.yaml. */
static constexpr float baseline_min_s = 0.05F;
static constexpr float baseline_max_s = 1.0F;

/* adc/direction, and fibril_control_msgs/msg/SensorState's direction. */
enum adc_direction {
  ADC_FALL = 0,
  ADC_RISE = 1,
  ADC_EITHER = 2,
};

/* A trigger replaced by a second one and the second's own answer can both
 * wait for a staging slot at once. */
static constexpr uint8_t trigger_replies_max = 2;

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
  bool sensor_state;
  bool sensor_state_on;
  bool baseline;
  fcan_gen::call_handle baseline_call;
  bool trigger;
  fcan_gen::call_handle trigger_call;
  /* trigger_cancel before the trigger above arrived, so it ends the call
   * already waiting; after it, so it ends that new one instead. */
  bool trigger_cancel;
  bool trigger_cancel_new;
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
  uint8_t adc_port;
  uint8_t adc_direction;
  float current_polarity;
  float current_offset;
  uint32_t baseline_ticks;
  /* command_timeout_ms in ticks; 0 is off. */
  uint32_t command_timeout_ticks;

  /* Generations of the last command applied from each M2S topic. The topics
   * keep their last value, so only a change here says a new frame arrived. */
  uint32_t target_seq;
  uint32_t trajectory_seq;

  /* Ticks since the last new command, saturating. */
  uint32_t command_age;
  /* The timeout has stopped the motor and waits for a new command, so it
   * stops it once rather than on every tick. */
  bool command_lapsed;

  /* The encoder's accumulator generation, to notice a rebuilt position. */
  uint32_t position_epoch;
  bool epoch_known;

  bool encoder_valid;

  /* feedback.current, and whether the motor reported one this tick. */
  float current;
  bool current_valid;

  motor_control::LevelWatch sensor;
  uint32_t adc_generation;
  /* The port `sensor` remembers a level for. */
  uint8_t sensor_port;

  motor_control::CurrentBaseline baseline;
  fcan_gen::call_handle baseline_call;
  /* The run has ended but the runtime has not taken the reply yet. */
  bool baseline_replying;
  bool baseline_ok;
  /* Set by the handler, cleared by the tick once it has replied, so a second
   * call cannot overwrite the handle of one still running. Under `lock`. */
  bool baseline_busy;

  /* A trigger waiting for its edge. It has a watch of its own, so the levels
   * sensor_state reports do not move it, and it skips the first event, which
   * is the level the port starts at rather than a crossing. */
  bool trigger_waiting;
  fcan_gen::call_handle trigger_call;
  motor_control::LevelWatch trigger_watch;
  bool trigger_primed;
  /* The port trigger_watch remembers a level for; sensor_port is the same for
   * `sensor`, and the two are reset separately. */
  uint8_t trigger_port;

  /* Answers the runtime has not taken yet, oldest first. */
  struct
  {
    fcan_gen::call_handle call;
    float position;
    bool success;
  } trigger_replies[trigger_replies_max];
  uint8_t trigger_reply_count;
  /* Under `lock`: a trigger request not drained yet, or answers still queued.
   * The handler refuses another trigger meanwhile rather than lose a handle. */
  bool trigger_busy;
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
  s->adc_port = m.param_adc__port();
  s->adc_direction = m.param_adc__direction();
  s->current_polarity = (m.param_current__polarity() < 0.0F) ? -1.0F : 1.0F;
  s->current_offset = m.param_current__offset();

  /* The runtime does not enforce the schema's min and max. */
  float baseline_s = m.param_current__baseline_duration_s();

  if (!std::isfinite(baseline_s)) {
    baseline_s = baseline_min_s;
  }
  baseline_s = CLAMP(baseline_s, baseline_min_s, baseline_max_s);
  s->baseline_ticks = (uint32_t)std::lround(baseline_s / motor_control::MotorControl::tick_s);
  s->command_timeout_ticks =
    m.param_command_timeout_ms() * (1000U / motor_control::MotorControl::tick_us);

  c.encoder_gain(s->encoder_gain);
  /* The runtime stores parameters without checking the schema's min and max,
   * so this is the check that keeps the filter stable. */
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

/* `port` is the adc/port parameter. */
static uint8_t watched_port(uint8_t inst, uint8_t port)
{
  return (port == adc_port_own) ? inst : port;
}

static void queue_trigger_reply(uint8_t inst, bool success, float position)
{
  struct motor_state * s = &states[inst];

  /* The handler refuses a trigger while answers are queued, so at most the
   * replaced call and the current one are here. */
  if (s->trigger_reply_count >= trigger_replies_max) {
    LOG_ERR("motor %u: trigger reply dropped", inst);
    return;
  }

  s->trigger_replies[s->trigger_reply_count++] = {s->trigger_call, position, success};
  s->trigger_waiting = false;
}

/* Takes the starting level from the port's latest sample, so an edge before
 * the next sample still counts. Without one, the next sample is the start. */
static void prime_trigger(uint8_t inst, uint8_t port)
{
  struct motor_state * s = &states[inst];
  float value;

  s->trigger_watch.start();
  s->trigger_port = port;
  s->trigger_primed = false;

  if (adc_port::read(port, value)) {
    const auto event = s->trigger_watch.update(value, adc_port::threshold(port));

    s->trigger_primed = event != motor_control::LevelWatch::Event::NONE;
  }
}

static void start_trigger(uint8_t inst, fcan_gen::call_handle call)
{
  struct motor_state * s = &states[inst];

  /* A second trigger replaces the first, as can_md_controller did. */
  if (s->trigger_waiting) {
    queue_trigger_reply(inst, false, 0.0F);
  }

  s->trigger_call = call;
  s->trigger_waiting = true;
  prime_trigger(inst, watched_port(inst, s->adc_port));
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

  if (req.sensor_state) {
    if (req.sensor_state_on) {
      s->sensor.start();
      s->sensor_port = watched_port(inst, s->adc_port);
    } else {
      s->sensor.stop();
    }
  }

  if (req.baseline) {
    s->baseline_call = req.baseline_call;
    s->baseline.start(s->baseline_ticks);
  }

  if (req.trigger_cancel && s->trigger_waiting) {
    queue_trigger_reply(inst, false, 0.0F);
  }

  if (req.trigger) {
    start_trigger(inst, req.trigger_call);
    if (req.trigger_cancel_new) {
      queue_trigger_reply(inst, false, 0.0F);
    }
  }
}

/* False when the command was refused, so it does not count as the master
 * still commanding. */
static bool apply_target(uint8_t inst, const fcan_gen::mdmotor::target & cmd)
{
  struct motor_state * s = &states[inst];

  if (!std::isfinite(cmd.value)) {
    LOG_WRN("motor %u: non-finite target ignored", inst);
    return false;
  }

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
      return false;
  }

  return true;
}

static bool apply_trajectory(uint8_t inst, const fcan_gen::mdmotor::trajectory & point)
{
  struct motor_state * s = &states[inst];

  if (!std::isfinite(point.positions[0]) || !std::isfinite(point.velocities[0]) ||
      !std::isfinite(point.accelerations[0])) {
    LOG_WRN("motor %u: non-finite trajectory point ignored", inst);
    return false;
  }

  s->control.set_position_speed(point.positions[0]);
  s->control.set_velocity_ff(point.velocities[0]);
  s->control.set_duty_ff(s->accel_ff_gain * point.accelerations[0]);
  return true;
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

  bool applied = false;

  if (new_trajectory) {
    applied = apply_trajectory(inst, *trajectory);
  } else if (new_target) {
    applied = apply_target(inst, *target);
  }

  if (applied) {
    s->command_age = 0;
    s->command_lapsed = false;
  }
}

/*
 * The open-loop and speed modes need the master to keep sending: a duty or a
 * speed held past the master's attention runs the mechanism into whatever is
 * in the way. A position is a place to stay, so it is held (doc/adr/0016).
 */
static void watch_command(uint8_t inst)
{
  struct motor_state * s = &states[inst];
  const motor_control::Mode mode = s->control.mode();

  if (s->command_age < UINT32_MAX) {
    s->command_age++;
  }

  if (s->command_lapsed || (s->command_timeout_ticks == 0U) ||
      (s->command_age < s->command_timeout_ticks)) {
    return;
  }

  if ((mode != motor_control::Mode::DUTY) && (mode != motor_control::Mode::SPEED)) {
    return;
  }

  s->command_lapsed = true;
  s->control.stop();
}

/*
 * The commands are checked on the way in, but a parameter can still carry a
 * NaN into the loops (the runtime stores parameters without checking the
 * schema's min/max), and lround
 * of a non-finite or out-of-range value is unspecified. Anything that is not
 * a finite duty stops the motor rather than reaching the integer conversion.
 */
static void drive(uint8_t inst, float duty)
{
  float raw = duty * states[inst].sign * duty_full_scale;

  if (!std::isfinite(raw)) {
    LOG_WRN_ONCE("motor %u: non-finite duty, output 0", inst);
    raw = 0.0F;
  }

  raw = CLAMP(raw, (float)INT16_MIN, (float)INT16_MAX);

  const int16_t output = (int16_t)std::lround(raw);
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
  pub->current = s->current_valid ? s->current : 0.0F;
}

/*
 * The current as feedback.current reports it, before `invert`: polarity and
 * the manual offset applied, the measured baseline not yet. The baseline is
 * averaged over this, so it is what remains after the manual offset.
 */
static bool sample_current(uint8_t inst, float & current)
{
  const struct motor_state * s = &states[inst];
  struct motor_feedback fb;

  if ((motor_get_feedback(motors[inst], &fb) != 0) ||
      ((fb.valid_mask & MOTOR_FEEDBACK_CURRENT) == 0U) || !fb.online || fb.stale) {
    return false;
  }

  current = s->current_polarity * (float)fb.current / current_full_scale - s->current_offset;

  /* current/offset is any f32 the host sends, NaN included. */
  return std::isfinite(current);
}

/* FCAN_E_BUSY means every staging slot still waits for fcan_poll, as when
 * several motors finish in the same tick; the next tick tries again. */
static void reply_baseline(uint8_t inst)
{
  struct motor_state * s = &states[inst];
  const baseline_resp resp{s->baseline_ok};
  k_spinlock_key_t key;

  if (
    fcan_gen::mdmotor::calibrate_current_baseline_complete(
      s->baseline_call, s->baseline_ok ? FCAN_SVC_OK : FCAN_SVC_APP_ERROR, resp) == FCAN_E_BUSY) {
    return;
  }

  s->baseline_replying = false;

  key = k_spin_lock(&lock);
  s->baseline_busy = false;
  k_spin_unlock(&lock, key);
}

static void finish_baseline(uint8_t inst, bool success)
{
  struct motor_state * s = &states[inst];

  if (success) {
    LOG_INF("motor %u: current baseline %.4f", inst, (double)s->baseline.offset());
  } else {
    LOG_WRN("motor %u: current baseline aborted, the motor moved or lost its current", inst);
  }

  s->baseline_ok = success;
  s->baseline_replying = true;
  reply_baseline(inst);
}

static void update_current(uint8_t inst)
{
  struct motor_state * s = &states[inst];
  float current = 0.0F;

  s->current_valid = sample_current(inst, current);

  if (s->baseline_replying) {
    reply_baseline(inst);
  }

  if (s->baseline.running()) {
    const bool still = s->encoder_valid && (s->control.mode() == motor_control::Mode::DUTY) &&
                       (std::fabs(s->control.duty()) < still_duty) &&
                       (std::fabs(s->control.speed()) < still_speed);

    switch (s->baseline.update(still, s->current_valid, current)) {
      case motor_control::CurrentBaseline::State::DONE:
        finish_baseline(inst, true);
        break;
      case motor_control::CurrentBaseline::State::ABORTED:
        finish_baseline(inst, false);
        break;
      default:
        break;
    }
  }

  /* `invert` flips the current with the rest of the feedback, as
   * can_md_controller did on the host. */
  s->current = s->sign * (current - s->baseline.offset());
}

static void watch_trigger(uint8_t inst, float value, float threshold)
{
  struct motor_state * s = &states[inst];
  const auto event = s->trigger_watch.update(value, threshold);

  if (event == motor_control::LevelWatch::Event::NONE) {
    return;
  }

  if (!s->trigger_primed) {
    s->trigger_primed = true;
    return;
  }

  const uint8_t direction = (event == motor_control::LevelWatch::Event::HIGH) ? ADC_RISE : ADC_FALL;

  if ((s->adc_direction == ADC_EITHER) || (s->adc_direction == direction)) {
    queue_trigger_reply(inst, true, s->control.position());
  }
}

/* Hands queued answers to the runtime in order. FCAN_E_BUSY keeps the rest for
 * the next tick; FCAN_E_STALE means the runtime already dropped the call. */
static void reply_triggers(uint8_t inst)
{
  struct motor_state * s = &states[inst];

  /* The node stopped serving calls: the runtime dropped the call and the
   * master gave up on it, so there is nothing left to wait for. */
  if (s->trigger_waiting && !fibril_fcan_call_live(s->trigger_call)) {
    s->trigger_waiting = false;
  }

  while (s->trigger_reply_count > 0U) {
    const auto & r = s->trigger_replies[0];
    trigger_resp resp{};

    resp.triggered_position = r.position;
    resp.success = r.success;

    if (fcan_gen::mdmotor::trigger_complete(r.call, FCAN_SVC_OK, resp) == FCAN_E_BUSY) {
      break;
    }

    for (uint8_t i = 1; i < s->trigger_reply_count; i++) {
      s->trigger_replies[i - 1] = s->trigger_replies[i];
    }
    s->trigger_reply_count--;
  }

  k_spinlock_key_t key = k_spin_lock(&lock);

  s->trigger_busy = s->req.trigger || (s->trigger_reply_count > 0U);
  k_spin_unlock(&lock, key);
}

/* Runs once per AdcPort sampling, so a port is compared once per sample. */
static void watch_sensor(uint8_t inst)
{
  struct motor_state * s = &states[inst];
  const uint32_t generation = adc_port::generation();
  float value;

  if (generation == s->adc_generation) {
    return;
  }
  s->adc_generation = generation;

  const uint8_t port = watched_port(inst, s->adc_port);
  const bool sensing = s->sensor.active();

  if (!sensing && !s->trigger_waiting) {
    return;
  }

  /* A level remembered for another port says nothing about this one, so start
   * over: the next sample reports the new port's level, as sensor_state does,
   * and a trigger waits for a crossing on the new port. */
  if (sensing && (port != s->sensor_port)) {
    s->sensor.start();
    s->sensor_port = port;
  }
  if (s->trigger_waiting && (port != s->trigger_port)) {
    prime_trigger(inst, port);
  }

  /* An edge is only worth its position. While the encoder is out the watcher
   * keeps its level, so a crossing during the outage is reported on the first
   * valid sample, with that sample's position. */
  if (!s->encoder_valid || !adc_port::read(port, value)) {
    return;
  }

  const float threshold = adc_port::threshold(port);

  if (s->trigger_waiting) {
    watch_trigger(inst, value, threshold);
  }

  if (!sensing) {
    return;
  }

  const auto event = s->sensor.update(value, threshold);

  if (event == motor_control::LevelWatch::Event::NONE) {
    return;
  }

  if (auto pub = fcan_gen::mdmotor(inst).publish_sensor()) {
    pub->direction = (event == motor_control::LevelWatch::Event::HIGH) ? ADC_RISE : ADC_FALL;
    pub->edge_position = s->control.position();
  }
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

static fcan_gen::svc_status sensor_state(
  uint8_t inst, const sensor_state_req & req, sensor_state_resp & resp)
{
  k_spinlock_key_t key;

  /* Checked here so the reply says whether there is anything to watch. The
   * port list is fixed at build time, so the answer cannot go stale. */
  const uint8_t port = watched_port(inst, fcan_gen::mdmotor(inst).param_adc__port());

  if (req.data && (port >= adc_port::count())) {
    LOG_WRN("motor %u: sensor_state refused, no AdcPort %u", inst, port);
    resp.success = false;
    return FCAN_SVC_APP_ERROR;
  }

  key = k_spin_lock(&lock);
  states[inst].req.sensor_state = true;
  states[inst].req.sensor_state_on = req.data;
  k_spin_unlock(&lock, key);

  resp.success = true;

  return FCAN_SVC_OK;
}

/* Replied to from the tick, by finish_baseline(), once the run ends. */
static fcan_gen::svc_status calibrate_current_baseline(
  uint8_t inst, baseline_resp & resp, fcan_gen::call_handle call)
{
  struct motor_state * s = &states[inst];
  k_spinlock_key_t key = k_spin_lock(&lock);

  if (s->baseline_busy) {
    k_spin_unlock(&lock, key);
    resp.success = false;
    return FCAN_SVC_BUSY;
  }

  s->baseline_busy = true;
  s->req.baseline = true;
  s->req.baseline_call = call;
  k_spin_unlock(&lock, key);

  return FCAN_SVC_ACCEPTED;
}

/* Answered from the tick, by reply_triggers(), once the edge comes. */
static fcan_gen::svc_status trigger(uint8_t inst, trigger_resp & resp, fcan_gen::call_handle call)
{
  struct motor_state * s = &states[inst];
  const uint8_t port = watched_port(inst, fcan_gen::mdmotor(inst).param_adc__port());

  resp.triggered_position = 0.0F;
  resp.success = false;

  if (port >= adc_port::count()) {
    LOG_WRN("motor %u: trigger refused, no AdcPort %u", inst, port);
    return FCAN_SVC_APP_ERROR;
  }

  k_spinlock_key_t key = k_spin_lock(&lock);

  if (s->trigger_busy) {
    k_spin_unlock(&lock, key);
    return FCAN_SVC_BUSY;
  }

  s->trigger_busy = true;
  s->req.trigger = true;
  s->req.trigger_call = call;
  k_spin_unlock(&lock, key);

  return FCAN_SVC_ACCEPTED;
}

static fcan_gen::svc_status trigger_cancel(uint8_t inst, trigger_cancel_resp & resp)
{
  k_spinlock_key_t key = k_spin_lock(&lock);
  struct requests & req = states[inst].req;

  /* A trigger not drained yet came first, so this cancel is meant for it. */
  if (req.trigger) {
    req.trigger_cancel_new = true;
  } else {
    req.trigger_cancel = true;
  }
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
    m.on_sensor_state(
      [i](const sensor_state_req & req, sensor_state_resp & resp, fcan_gen::call_handle) noexcept {
        return sensor_state(i, req, resp);
      });
    m.on_calibrate_current_baseline(
      [i](const baseline_req &, baseline_resp & resp, fcan_gen::call_handle call) noexcept {
        return calibrate_current_baseline(i, resp, call);
      });
    m.on_trigger([i](const trigger_req &, trigger_resp & resp, fcan_gen::call_handle call) noexcept {
      return trigger(i, resp, call);
    });
    m.on_trigger_cancel(
      [i](const trigger_cancel_req &, trigger_cancel_resp & resp, fcan_gen::call_handle) noexcept {
        return trigger_cancel(i, resp);
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
  /* Read once, so every motor acts on the same state this tick. */
  static bool was_running;
  const bool running = fibril_fcan_node_state() == FCAN_STATE_RUNNING;
  const bool master_lost = was_running && !running;

  was_running = running;

  if (master_lost) {
    LOG_WRN("master lost, every motor stopped until commanded again");
  }

  for (uint8_t i = 0; i < (uint8_t)ARRAY_SIZE(motors); i++) {
    struct motor_state * s = &states[i];

    /* Requests first, so new gains and a reset encoder are in effect for the
     * sample and the command this tick reads. */
    drain_requests(i);

    const motor_control::EncoderSample sample = sample_encoder(i);

    s->encoder_valid = sample.valid;

    /* Stopped on the edge only, and before the commands: a frame landing in
     * the same tick is the one that brings the node back to RUNNING, and
     * dropping it would lose a one-shot position target for good. */
    if (master_lost) {
      s->control.stop();
    }

    apply_commands(i);
    watch_command(i);
    drive(i, s->control.update(sample));
    update_current(i);
    watch_sensor(i);
    reply_triggers(i);
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
