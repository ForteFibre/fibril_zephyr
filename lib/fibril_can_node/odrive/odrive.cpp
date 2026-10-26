/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * ODrive axes on the bus. Binds the schema's ODriveMotor block type to the
 * axis devices the devicetree names, one instance per entry of `axes`, and
 * runs one lib/odrive_sequencer per axis in the tick.
 *
 * Everything that touches the axis happens in the tick. The only thing the
 * poll thread hands over is that the parameters changed.
 */

#define DT_DRV_COMPAT fibril_fcan_odrive

#include <cmath>
#include <cstring>

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/motor.h>
#include <drivers/motor/odrive.h>
#include <odrive_sequencer/odrive_sequencer.hpp>

#include <fibril_can_node/func.h>

#include "schema_gen.hpp"

LOG_MODULE_REGISTER(fcan_odrive, CONFIG_FIBRIL_CAN_NODE_LOG_LEVEL);

using odrive_sequencer::Sequencer;
using odrive_sequencer::Setpoint;

/* One node names every axis this board exposes, and the list below reads
 * instance 0 only. A second enabled node would build and then be invisible
 * from the bus.
 */
BUILD_ASSERT(
  DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
  "exactly one enabled fibril,fcan-odrive node; put every axis in its axes");

#define AXIS_DEV_BY_IDX(node_id, prop, idx) DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx))

static const struct device * const axes[] = {
  DT_INST_FOREACH_PROP_ELEM_SEP(0, axes, AXIS_DEV_BY_IDX, (, ))
};

/* max_count is a Kconfig ceiling rather than the devicetree's length, because
 * CMake cannot read a phandle list. An extra axis would otherwise be silently
 * unreachable from the bus.
 */
BUILD_ASSERT(ARRAY_SIZE(axes) <= fcan_gen::odrivemotor::max_count,
             "more axes wired than CONFIG_FIBRIL_CAN_NODE_ODRIVE_MAX");

/* fibril_control_msgs/msg/Target's constants. */
enum target_type {
  TARGET_DUTY = 0,
  TARGET_VELOCITY = 1,
  TARGET_TORQUE = 2,
  TARGET_POSITION = 3,
  TARGET_TRAJECTORY = 4,
};

/*
 * Snapshot of the parameters, taken as a set so that a write landing between
 * two lines cannot scale a position by one gain and the velocity beside it by
 * another.
 */
struct tunable
{
  float encoder_gain;
  float sign;
  bool use_input_vel_ramp;
  float duty_full_scale;
  uint32_t command_timeout_ms;
};

struct axis_state
{
  struct tunable tune;
  Sequencer sequencer;

  /* Raised by the poll thread, consumed by the tick. */
  bool params_changed;

  /* Generation of the last target read; the topic keeps its last value. */
  uint32_t target_seq;
  int64_t last_command_ms;
  bool commanded;

  /* The target type last refused, so a master repeating it at 100 Hz is
   * reported once. */
  int refused_type;
};

static struct axis_state states[ARRAY_SIZE(axes)];

/* Guards params_changed, the whole of what the poll thread writes. */
static struct k_spinlock lock;

/* The sequencer's view of one axis, forwarded to the driver. */
class DriverPort : public odrive_sequencer::Port
{
public:
  explicit DriverPort(const struct device * dev) : _dev(dev) {}

  int enable() override { return motor_enable(_dev); }

  int disable() override { return motor_disable(_dev); }

  int request_state(uint8_t state) override
  {
    return odrive_request_axis_state(_dev, (enum odrive_axis_state)state);
  }

  int clear_errors() override { return odrive_clear_errors(_dev); }

  int apply(const Setpoint & setpoint) override
  {
    const int ret = odrive_set_input_mode(_dev, setpoint.input_mode);

    if (ret != 0) {
      return ret;
    }

    switch (setpoint.control_mode) {
      case ODRIVE_CONTROL_MODE_POSITION:
        return odrive_set_position(_dev, setpoint.value, 0.0F, 0.0F);
      case ODRIVE_CONTROL_MODE_VELOCITY:
        return odrive_set_velocity(_dev, setpoint.value, 0.0F);
      case ODRIVE_CONTROL_MODE_TORQUE:
        return odrive_set_torque(_dev, setpoint.value);
      default:
        return -EINVAL;
    }
  }

private:
  const struct device * _dev;
};

static void tunable_read(uint8_t inst)
{
  const fcan_gen::odrivemotor m(inst);
  struct axis_state * s = &states[inst];
  struct tunable * t = &s->tune;
  const float gain = m.param_encoder_gain();
  const float full_scale = m.param_duty_full_scale();

  /* The runtime does not hold parameters to their min and max, and a zero
   * gain would divide every target into infinity. */
  if (std::isfinite(gain) && (gain != 0.0F)) {
    t->encoder_gain = gain;
  } else {
    LOG_WRN("axis %u: encoder_gain must be finite and non-zero, kept", inst);
  }

  t->sign = m.param_invert() ? -1.0F : 1.0F;
  t->use_input_vel_ramp = m.param_use_input_vel_ramp();
  t->duty_full_scale = (std::isfinite(full_scale) && (full_scale > 0.0F)) ? full_scale : 0.0F;
  t->command_timeout_ms = m.param_command_timeout_ms();

  odrive_sequencer::Settings & settings = s->sequencer.settings();

  settings.full_calibration = m.param_use_full_calibration();
  settings.switch_stop_velocity = m.param_switch_stop_velocity_threshold();
}

/* The adapter's publish_target(), from the joint's units into the ODrive's. */
static bool to_setpoint(
  const struct tunable * t, const fcan_gen::odrivemotor::target & cmd, Setpoint * out)
{
  if (!std::isfinite(cmd.value)) {
    return false;
  }

  out->type = cmd.type;
  out->input_mode = ODRIVE_INPUT_MODE_PASSTHROUGH;

  switch (cmd.type) {
    case TARGET_DUTY:
      if (t->duty_full_scale == 0.0F) {
        return false;
      }

      out->control_mode = ODRIVE_CONTROL_MODE_TORQUE;
      out->value = cmd.value * t->duty_full_scale * t->sign;
      return true;
    case TARGET_TORQUE:
      out->control_mode = ODRIVE_CONTROL_MODE_TORQUE;
      out->value = cmd.value * t->sign;
      return true;
    case TARGET_VELOCITY:
      out->control_mode = ODRIVE_CONTROL_MODE_VELOCITY;
      out->input_mode =
        t->use_input_vel_ramp ? ODRIVE_INPUT_MODE_VEL_RAMP : ODRIVE_INPUT_MODE_PASSTHROUGH;
      out->value = cmd.value / t->encoder_gain * t->sign;
      return true;
    case TARGET_POSITION:
      out->control_mode = ODRIVE_CONTROL_MODE_POSITION;
      out->value = cmd.value / t->encoder_gain * t->sign;
      return true;
    case TARGET_TRAJECTORY:
      out->control_mode = ODRIVE_CONTROL_MODE_POSITION;
      out->input_mode = ODRIVE_INPUT_MODE_TRAP_TRAJ;
      out->value = cmd.value / t->encoder_gain * t->sign;
      return true;
    default:
      return false;
  }
}

/*
 * Hands the sequencer the target that arrived since the last tick, once.
 * Handing it the held value every tick would count as fresh traffic and keep
 * command_timeout_ms from ever lapsing.
 */
static void apply_target(uint8_t inst, odrive_sequencer::Port & port, int64_t now)
{
  struct axis_state * s = &states[inst];
  uint32_t seq = s->target_seq;
  const auto target = fcan_gen::odrivemotor(inst).read_target(seq);
  Setpoint setpoint;

  if (!target || (seq == s->target_seq)) {
    return;
  }

  s->target_seq = seq;

  if (!to_setpoint(&s->tune, *target, &setpoint)) {
    if (s->refused_type != target->type) {
      s->refused_type = target->type;
      LOG_WRN("axis %u: target type %u refused (DUTY needs duty_full_scale)", inst,
              target->type);
    }
    return;
  }

  s->refused_type = -1;
  s->commanded = true;
  s->last_command_ms = now;
  s->sequencer.command(setpoint, now, port);
}

static void publish_feedback(uint8_t inst, const struct odrive_feedback * fb, bool measured)
{
  const struct axis_state * s = &states[inst];
  const struct tunable * t = &s->tune;
  auto pub = fcan_gen::odrivemotor(inst).publish_feedback();

  if (!pub) {
    return;
  }

  /* An axis with nothing to report publishes zeros rather than the last
   * measurement, so that a consumer watching position sees the link drop
   * instead of a value frozen at whatever it was. */
  if (!measured) {
    pub->is_ready = false;
    pub->position = 0.0F;
    pub->velocity = 0.0F;
    pub->output = 0.0F;
    pub->current = 0.0F;
    return;
  }

  const float scale = t->encoder_gain * t->sign;

  pub->is_ready = s->sequencer.ready();
  pub->position = fb->position * scale;
  pub->velocity = fb->velocity * scale;
  /* The adapter left output at 0. The torque estimate is what RobstrideMotor
   * reports there, so it is filled when the ODrive sends Get_Torques. */
  pub->output = ((fb->valid_mask & ODRIVE_FEEDBACK_TORQUES) != 0U)
                  ? fb->torque_estimate * t->sign
                  : 0.0F;
  pub->current = ((fb->valid_mask & ODRIVE_FEEDBACK_IQ) != 0U) ? fb->iq_measured * t->sign
                                                                : 0.0F;
}

static void publish_diagnostics(uint8_t inst, const struct odrive_feedback * fb)
{
  const struct axis_state * s = &states[inst];
  auto pub = fcan_gen::odrivemotor(inst).publish_diagnostics();

  if (!pub) {
    return;
  }

  /* Unlike the feedback above, every field here stays readable when the axis
   * has gone quiet: they are what says why. */
  pub->online = fb->online;
  pub->stale = fb->stale;
  pub->calibrated = s->sequencer.calibrated();
  pub->state = (uint8_t)s->sequencer.state();
  pub->axis_state = (uint8_t)fb->axis_state;
  pub->procedure_result = (uint8_t)fb->procedure_result;
  pub->active_errors = fb->active_errors;
  pub->disarm_reason = fb->disarm_reason;
  pub->bus_voltage = fb->bus_voltage;
  pub->fet_temperature = fb->fet_temperature;
  pub->motor_temperature = fb->motor_temperature;
}

/* Fires from fcan_poll, so it only records that something changed. */
static void params_changed(uint8_t inst)
{
  k_spinlock_key_t key = k_spin_lock(&lock);

  states[inst].params_changed = true;
  k_spin_unlock(&lock, key);
}

static int odrive_func_init(void)
{
  for (size_t i = 0; i < ARRAY_SIZE(axes); i++) {
    if (!device_is_ready(axes[i])) {
      LOG_ERR("axis %u: %s not ready", (unsigned)i, axes[i]->name);
      return -ENODEV;
    }
  }

  return 0;
}

static int odrive_func_start(void)
{
  for (uint8_t i = 0; i < (uint8_t)ARRAY_SIZE(axes); i++) {
    fcan_gen::odrivemotor m(i);

    m.on_params_changed([i](uint32_t) noexcept { params_changed(i); });

    states[i].tune.encoder_gain = 1.0F;
    states[i].refused_type = -1;
    tunable_read(i);
  }

  return 0;
}

static void odrive_func_tick(void)
{
  const int64_t now = k_uptime_get();
  /* Read once, so every axis acts on the same state this tick. */
  static bool was_running;
  const bool running = fibril_fcan_node_state() == FCAN_STATE_RUNNING;
  const bool master_lost = was_running && !running;

  was_running = running;

  if (master_lost) {
    LOG_WRN("master lost, every axis disarmed until commanded again");
  }

  for (uint8_t i = 0; i < (uint8_t)ARRAY_SIZE(axes); i++) {
    struct axis_state * s = &states[i];
    DriverPort port(axes[i]);
    struct odrive_feedback fb;
    bool reread;
    k_spinlock_key_t key;

    /* -ENODATA and -EAGAIN still fill the snapshot; only the estimates are
     * missing or old, which `measured` below is about. */
    const int ret = odrive_get_feedback(axes[i], &fb);

    if ((ret != 0) && (ret != -ENODATA) && (ret != -EAGAIN)) {
      memset(&fb, 0, sizeof(fb));
    }

    key = k_spin_lock(&lock);
    reread = s->params_changed;
    s->params_changed = false;
    k_spin_unlock(&lock, key);

    if (reread) {
      tunable_read(i);
    }

    /* Before the target: a frame landing in the same tick is the one that
     * brings the node back to RUNNING, and counts as the new command. */
    if (master_lost) {
      s->commanded = false;
      s->sequencer.release(port);
    }

    apply_target(i, port, now);

    const uint32_t timeout = s->tune.command_timeout_ms;

    if (s->commanded && (timeout != 0U) && ((now - s->last_command_ms) > (int64_t)timeout)) {
      LOG_WRN("axis %u: no target for %u ms, disarmed", i, (unsigned)timeout);
      s->commanded = false;
      s->sequencer.release(port);
    }

    s->sequencer.step(fb, running, now, port);

    /* 0 also comes back for an axis whose encoder_msg_rate_ms is off, with
     * the estimates never having arrived. */
    publish_feedback(i, &fb, (ret == 0) && ((fb.valid_mask & ODRIVE_FEEDBACK_ESTIMATES) != 0U));
    publish_diagnostics(i, &fb);
  }
}

FIBRIL_FCAN_FUNC_DEFINE(
  odrive,
  .array = fcan_gen::odrivemotor::block_array_index,
  .count = (uint8_t)ARRAY_SIZE(axes),
  .init = odrive_func_init,
  .start = odrive_func_start,
  .tick = odrive_func_tick);
