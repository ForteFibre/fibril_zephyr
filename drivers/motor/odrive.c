/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/motor.h>
#include <drivers/motor/odrive.h>
#include <math.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(motor_odrive, CONFIG_MOTOR_LOG_LEVEL);

#define ODRIVE_START_RETRY_MS 500

/*
 * The identifier is node_id in bits 10..5 and cmd_id in bits 4..0, so one
 * filter per axis takes every message that axis sends.
 */
#define ODRIVE_NODE_SHIFT 5U
#define ODRIVE_CMD_MASK 0x1FU
#define ODRIVE_NODE_MASK 0x7E0U

/* 63 is what an unconfigured ODrive answers to, and it broadcasts nothing. */
#define ODRIVE_NODE_ID_MAX 62U

/* Command identifiers, carried in bits 4..0. */
#define ODRIVE_CMD_HEARTBEAT 0x001U
#define ODRIVE_CMD_ESTOP 0x002U
#define ODRIVE_CMD_GET_ERROR 0x003U
#define ODRIVE_CMD_SET_AXIS_STATE 0x007U
#define ODRIVE_CMD_GET_ENCODER_ESTIMATES 0x009U
#define ODRIVE_CMD_SET_CONTROLLER_MODE 0x00BU
#define ODRIVE_CMD_SET_INPUT_POS 0x00CU
#define ODRIVE_CMD_SET_INPUT_VEL 0x00DU
#define ODRIVE_CMD_SET_INPUT_TORQUE 0x00EU
#define ODRIVE_CMD_SET_LIMITS 0x00FU
#define ODRIVE_CMD_SET_TRAJ_VEL_LIMIT 0x011U
#define ODRIVE_CMD_SET_TRAJ_ACCEL_LIMITS 0x012U
#define ODRIVE_CMD_GET_IQ 0x014U
#define ODRIVE_CMD_GET_TEMPERATURE 0x015U
#define ODRIVE_CMD_GET_BUS_VOLTAGE_CURRENT 0x017U
#define ODRIVE_CMD_CLEAR_ERRORS 0x018U
#define ODRIVE_CMD_SET_ABSOLUTE_POSITION 0x019U
#define ODRIVE_CMD_SET_POS_GAIN 0x01AU
#define ODRIVE_CMD_SET_VEL_GAINS 0x01BU
#define ODRIVE_CMD_GET_TORQUES 0x01CU

/*
 * One revolution is 65536 counts in motor_feedback.position. Pos_Estimate is a
 * float32, whose step reaches 1/65536 rev at 128 rev, so counts and estimates
 * correspond one to one up to there and coarsen by a factor of two per
 * doubling beyond it. See ADR 0022.
 */
#define ODRIVE_COUNTS_PER_REV 65536.0F

/*
 * Vel_FF and Torque_FF of Set_Input_Pos are int16 scaled by the ODrive's
 * input_vel_scale and input_torque_scale, both of which default to 0.001. The
 * driver assumes the defaults rather than writing them.
 */
#define ODRIVE_FF_SCALE 1000.0F

/* Arrival time of a message that has not arrived. 0 is a real uptime. */
#define ODRIVE_NEVER INT64_MIN

#define ODRIVE_PENDING_FEEDBACK BIT(0)
#define ODRIVE_PENDING_STATE BIT(1)

/* Which frame of the command interval a transmission carried. */
enum odrive_tx_stage {
  ODRIVE_TX_STAGE_NONE = 0,
  ODRIVE_TX_STAGE_CLEAR_ERRORS,
  ODRIVE_TX_STAGE_CONTROLLER_MODE,
  ODRIVE_TX_STAGE_AXIS_STATE,
  ODRIVE_TX_STAGE_SETPOINT,
  ODRIVE_TX_STAGE_IDLE,
  ODRIVE_TX_STAGE_COUNT,
};

struct odrive_axis_data;

/*
 * Handed to can_send() as the completion context. One per axis and stage,
 * filled once at init, because a completion may land after the next stage has
 * already been built and has to name the stage it belonged to.
 */
struct odrive_tx_ctx
{
  struct odrive_axis_data * data;
  enum odrive_tx_stage stage;
};

struct odrive_bus_config
{
  const struct device * can_dev;
  k_thread_stack_t * workq_stack;
  size_t workq_stack_size;
  uint32_t heartbeat_timeout_ms;
  uint32_t estimate_timeout_ms;
  uint32_t measurement_timeout_ms;
};

struct odrive_bus_data
{
  const struct device * dev;
  const struct device * axes[CONFIG_MOTOR_ODRIVE_MAX_AXES];
  struct k_spinlock lock;
  struct k_work tx_work;
  struct k_timer tx_timer;
  /* Runs the application callbacks, so a slow one cannot delay the commands. */
  struct k_work_q workq;
  struct k_work dispatch_work;
  bool started;
  int64_t last_start_retry;
  /*
   * Axis the command interval starts from. It moves to the first axis the
   * controller refused, so that with more axes than TX mailboxes the one left
   * out changes from interval to interval.
   */
  size_t tx_start;
};

struct odrive_axis_config
{
  const struct device * bus;
  uint8_t node_id;
  bool has_motor_thermistor;
};

struct odrive_axis_data
{
  struct k_spinlock lock;

  /* What the application asked for. */
  bool enabled;
  enum odrive_control_mode control_mode;
  enum odrive_input_mode input_mode;
  float target_position;
  float target_velocity;
  float target_torque;
  float velocity_ff;
  float torque_ff;
  /*
   * TRAP_TRAJ re-plans on every position frame it receives, so an unchanged
   * target must not be re-sent in that mode.
   */
  bool setpoint_dirty;
  /* A target has been given since the last enable. */
  bool has_target;

  /*
   * Arming progress. Cleared on enable and whenever the commanded mode
   * changes, because the ODrive has to be told the mode before the state.
   */
  bool clear_errors_pending;
  bool mode_sent;
  bool state_sent;
  int64_t state_sent_ms;
  /* The closed loop request built for state_sent was reported sent. */
  bool state_on_wire;
  /* Set once the heartbeat has confirmed closed loop, so that a later drop
   * out of it is a fault rather than the arming still being in progress. */
  bool armed;
  /*
   * The axis may still be in closed loop after the driver stopped commanding
   * it, on a disable, an estop or a heartbeat timeout, so it is owed an idle
   * request until the heartbeat shows it out of closed loop.
   */
  bool idle_pending;
  /* Schedules the idle retries. Moved back when one fails, to retry at once. */
  int64_t idle_sent_ms;
  /*
   * Idle and estop requests queued but not yet reported sent. A count, because
   * they share one completion context and a late report for an older one must
   * not clear a newer one.
   */
  uint8_t idle_in_flight;
  /* When the newest of them was queued, which bounds how long arming waits. */
  int64_t stop_queued_ms;
  struct odrive_tx_ctx tx_ctx[ODRIVE_TX_STAGE_COUNT];

  /* What the axis reported. */
  enum odrive_axis_state axis_state;
  enum odrive_procedure_result procedure_result;
  uint32_t active_errors;
  uint32_t disarm_reason;
  bool trajectory_done;
  float position;
  float velocity;
  float iq_setpoint;
  float iq_measured;
  float fet_temperature;
  float motor_temperature;
  float torque_target;
  float torque_estimate;
  float bus_voltage;
  float bus_current;

  bool online;
  int64_t heartbeat_ms;
  int64_t estimates_ms;
  int64_t iq_ms;
  int64_t temperature_ms;
  int64_t torques_ms;
  int64_t bus_ms;
  int64_t error_ms;
  int64_t timestamp_ms;

  /* Deferred notification. */
  uint8_t pending;
  odrive_callback_t state_cb;
  void * state_cb_data;
  odrive_callback_t feedback_cb;
  void * feedback_cb_data;
};

static int64_t odrive_round(float value)
{
  return (int64_t)(value + ((value >= 0.0F) ? 0.5F : -0.5F));
}

/*
 * Returns false for an estimate with no int64 count: NaN or infinity from an
 * encoder fault, or a magnitude past 2^63 counts. Converting those is undefined.
 */
static bool odrive_to_counts(float rev, int64_t * counts)
{
  const float scaled = rev * ODRIVE_COUNTS_PER_REV;

  if (!(fabsf(scaled) < 9223372036854775808.0F)) {
    return false;
  }

  *counts = odrive_round(scaled);

  return true;
}

static void odrive_put_float(uint8_t * dst, float value)
{
  uint32_t bits;

  memcpy(&bits, &value, sizeof(bits));
  sys_put_le32(bits, dst);
}

static float odrive_get_float(const uint8_t * src)
{
  const uint32_t bits = sys_get_le32(src);
  float value;

  memcpy(&value, &bits, sizeof(value));

  return value;
}

static void odrive_frame_init(
  struct can_frame * frame, uint8_t node_id, uint16_t cmd, uint8_t len)
{
  memset(frame, 0, sizeof(*frame));
  frame->id = ((uint32_t)node_id << ODRIVE_NODE_SHIFT) | cmd;
  frame->dlc = can_bytes_to_dlc(len);
  frame->flags = 0;
}

static const struct device * odrive_find_axis(struct odrive_bus_data * bus, uint8_t node_id)
{
  for (size_t i = 0; i < ARRAY_SIZE(bus->axes); ++i) {
    const struct device * dev = bus->axes[i];

    if (dev == NULL) {
      continue;
    }

    const struct odrive_axis_config * config = dev->config;

    if (config->node_id == node_id) {
      return dev;
    }
  }

  return NULL;
}

/* An idle or estop request is being queued. Caller holds the axis lock. */
static void odrive_idle_queued(struct odrive_axis_data * data, int64_t now)
{
  if (data->idle_in_flight < UINT8_MAX) {
    data->idle_in_flight++;
  }

  data->stop_queued_ms = now;
}

/* One idle or estop request is no longer queued. Caller holds the axis lock. */
static void odrive_idle_settled(struct odrive_axis_data * data)
{
  if (data->idle_in_flight > 0U) {
    data->idle_in_flight--;
  }
}

/*
 * Undo a stage whose frame never reached the bus, so the command interval
 * builds it again. Caller holds the axis lock. Each case re-checks that the
 * stage still applies: the failure may be reported after the application has
 * moved the axis on.
 */
static void odrive_tx_failed(struct odrive_axis_data * data, enum odrive_tx_stage stage)
{
  switch (stage) {
    case ODRIVE_TX_STAGE_CLEAR_ERRORS:
      /* An axis whose errors were never cleared would refuse closed loop. */
      if (data->enabled && !data->armed) {
        data->clear_errors_pending = true;
        data->mode_sent = false;
      }
      break;
    case ODRIVE_TX_STAGE_CONTROLLER_MODE:
      if (data->enabled) {
        data->mode_sent = false;
      }
      break;
    case ODRIVE_TX_STAGE_AXIS_STATE:
      /* Also on an armed axis, which re-requests closed loop after a mode change. */
      if (data->enabled) {
        data->state_sent = false;
        data->state_on_wire = false;
      }
      break;
    case ODRIVE_TX_STAGE_SETPOINT:
      data->setpoint_dirty = true;
      break;
    case ODRIVE_TX_STAGE_IDLE:
      /* Due again at once rather than after a full retry interval. */
      odrive_idle_settled(data);
      data->idle_sent_ms -= CONFIG_MOTOR_ODRIVE_STATE_RETRY_MS;
      break;
    default:
      break;
  }
}

static void odrive_tx_done(const struct device * can_dev, int error, void * user_data)
{
  const struct odrive_tx_ctx * ctx = user_data;

  ARG_UNUSED(can_dev);

  if (ctx == NULL) {
    return;
  }

  /* Arming waits on these two being sent, so their success matters too. */
  if ((error == 0) && (ctx->stage != ODRIVE_TX_STAGE_IDLE) &&
      (ctx->stage != ODRIVE_TX_STAGE_AXIS_STATE)) {
    return;
  }

  k_spinlock_key_t key = k_spin_lock(&ctx->data->lock);

  if (error != 0) {
    odrive_tx_failed(ctx->data, ctx->stage);
  } else if (ctx->stage == ODRIVE_TX_STAGE_IDLE) {
    odrive_idle_settled(ctx->data);
  } else if (ctx->data->state_sent) {
    ctx->data->state_on_wire = true;
  }

  k_spin_unlock(&ctx->data->lock, key);
}

/*
 * The result covers queueing only. can_send() with no completion callback
 * waits for the frame to reach the wire with K_FOREVER, which would park the
 * transmit work, and the whole system work queue with it, on a bus nobody
 * acknowledges. Delivery is reported through odrive_tx_done() instead; pass
 * NULL for a frame whose loss needs no follow-up.
 */
static int odrive_axis_send(
  const struct device * axis_dev, const struct can_frame * frame, struct odrive_tx_ctx * ctx)
{
  const struct odrive_axis_config * config = axis_dev->config;
  const struct odrive_bus_config * bus_config = config->bus->config;
  struct odrive_bus_data * bus = config->bus->data;

  if (!bus->started) {
    return -EIO;
  }

  return can_send(bus_config->can_dev, frame, K_NO_WAIT, odrive_tx_done, ctx);
}

/*
 * Bring up a bus that would not start earlier. Throttled, because can_start()
 * on a controller stuck in initialisation mode costs a hardware timeout.
 */
static void odrive_bus_retry_start(const struct device * dev)
{
  const struct odrive_bus_config * config = dev->config;
  struct odrive_bus_data * data = dev->data;
  const int64_t now = k_uptime_get();

  if (data->started || ((now - data->last_start_retry) < ODRIVE_START_RETRY_MS)) {
    return;
  }

  data->last_start_retry = now;

  const int ret = can_start(config->can_dev);

  if ((ret == 0) || (ret == -EALREADY)) {
    data->started = true;
    LOG_INF("CAN bus started");
  }
}

static void odrive_build_set_controller_mode(
  struct can_frame * frame, uint8_t node_id, enum odrive_control_mode control_mode,
  enum odrive_input_mode input_mode)
{
  odrive_frame_init(frame, node_id, ODRIVE_CMD_SET_CONTROLLER_MODE, 8U);
  sys_put_le32((uint32_t)control_mode, &frame->data[0]);
  sys_put_le32((uint32_t)input_mode, &frame->data[4]);
}

static void odrive_build_set_axis_state(
  struct can_frame * frame, uint8_t node_id, enum odrive_axis_state state)
{
  /* The payload is one uint32; the reference implementation sends dlc 4. */
  odrive_frame_init(frame, node_id, ODRIVE_CMD_SET_AXIS_STATE, 4U);
  sys_put_le32((uint32_t)state, &frame->data[0]);
}

static void odrive_build_clear_errors(struct can_frame * frame, uint8_t node_id)
{
  /* The single byte is the identify flag, which we never want to set. */
  odrive_frame_init(frame, node_id, ODRIVE_CMD_CLEAR_ERRORS, 1U);
}

/* Returns false when the current mode has no setpoint frame to send. */
static bool odrive_build_setpoint(
  struct can_frame * frame, uint8_t node_id, const struct odrive_axis_data * data)
{
  switch (data->control_mode) {
    case ODRIVE_CONTROL_MODE_POSITION: {
      const float vel_ff = CLAMP(data->velocity_ff * ODRIVE_FF_SCALE, INT16_MIN, INT16_MAX);
      const float torque_ff = CLAMP(data->torque_ff * ODRIVE_FF_SCALE, INT16_MIN, INT16_MAX);

      odrive_frame_init(frame, node_id, ODRIVE_CMD_SET_INPUT_POS, 8U);
      odrive_put_float(&frame->data[0], data->target_position);
      sys_put_le16((uint16_t)(int16_t)odrive_round(vel_ff), &frame->data[4]);
      sys_put_le16((uint16_t)(int16_t)odrive_round(torque_ff), &frame->data[6]);
      break;
    }
    case ODRIVE_CONTROL_MODE_VELOCITY:
      odrive_frame_init(frame, node_id, ODRIVE_CMD_SET_INPUT_VEL, 8U);
      odrive_put_float(&frame->data[0], data->target_velocity);
      odrive_put_float(&frame->data[4], data->torque_ff);
      break;
    case ODRIVE_CONTROL_MODE_TORQUE:
      odrive_frame_init(frame, node_id, ODRIVE_CMD_SET_INPUT_TORQUE, 4U);
      odrive_put_float(&frame->data[0], data->target_torque);
      break;
    default:
      return false;
  }

  return true;
}

/*
 * Produce the frame this axis owes the bus in this interval. One arming stage
 * per interval keeps the burst short; the axis is armed a few intervals after
 * it is enabled. The stage is marked done here, before the frame is sent, and
 * odrive_tx_failed() takes it back if the frame does not go out.
 */
static enum odrive_tx_stage odrive_build_tx(
  const struct device * axis_dev, struct can_frame * frame, int64_t now)
{
  const struct odrive_axis_config * config = axis_dev->config;
  struct odrive_axis_data * data = axis_dev->data;
  const uint8_t node_id = config->node_id;

  if (!data->enabled) {
    /*
     * Only while the axis is still seen in closed loop. Once it reports
     * anything else the idle request is done; while it is offline the last
     * state it reported stands, so the request keeps going out in case only
     * the receive side was lost.
     */
    if (data->idle_pending && (data->axis_state == ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) &&
        ((now - data->idle_sent_ms) >= CONFIG_MOTOR_ODRIVE_STATE_RETRY_MS)) {
      odrive_build_set_axis_state(frame, node_id, ODRIVE_AXIS_STATE_IDLE);
      data->idle_sent_ms = now;
      odrive_idle_queued(data, now);
      return ODRIVE_TX_STAGE_IDLE;
    }

    return ODRIVE_TX_STAGE_NONE;
  }

  /*
   * Wait for the stop that preceded this enable to reach the wire. A heartbeat
   * the ODrive sent before acting on it still shows closed loop, and arriving
   * after the request below it would arm the axis. Bounded, because a bus
   * nobody acknowledges never reports the frame sent.
   */
  if ((data->idle_in_flight > 0U) &&
      ((now - data->stop_queued_ms) < CONFIG_MOTOR_ODRIVE_STATE_RETRY_MS)) {
    return ODRIVE_TX_STAGE_NONE;
  }

  if (data->clear_errors_pending) {
    odrive_build_clear_errors(frame, node_id);
    data->clear_errors_pending = false;
    return ODRIVE_TX_STAGE_CLEAR_ERRORS;
  }

  if (!data->mode_sent) {
    odrive_build_set_controller_mode(frame, node_id, data->control_mode, data->input_mode);
    data->mode_sent = true;
    data->state_sent = false;
    data->state_on_wire = false;
    return ODRIVE_TX_STAGE_CONTROLLER_MODE;
  }

  if (!data->state_sent) {
    odrive_build_set_axis_state(frame, node_id, ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL);
    data->state_sent = true;
    data->state_sent_ms = now;
    return ODRIVE_TX_STAGE_AXIS_STATE;
  }

  if (!data->armed) {
    /*
     * Ask again, but never clear errors from here: a tripped axis would then
     * be cycled through its fault for as long as it stays enabled.
     */
    if ((now - data->state_sent_ms) >= CONFIG_MOTOR_ODRIVE_STATE_RETRY_MS) {
      data->mode_sent = false;
    }

    return ODRIVE_TX_STAGE_NONE;
  }

  if (!data->has_target) {
    return ODRIVE_TX_STAGE_NONE;
  }

  if ((data->control_mode == ODRIVE_CONTROL_MODE_POSITION) &&
      (data->input_mode == ODRIVE_INPUT_MODE_TRAP_TRAJ) && !data->setpoint_dirty) {
    return ODRIVE_TX_STAGE_NONE;
  }

  if (!odrive_build_setpoint(frame, node_id, data)) {
    return ODRIVE_TX_STAGE_NONE;
  }

  data->setpoint_dirty = false;

  return ODRIVE_TX_STAGE_SETPOINT;
}

/* Drop an axis that stopped answering, and notice when it comes back. */
static void odrive_check_timeout(const struct device * axis_dev, int64_t now)
{
  const struct odrive_axis_config * config = axis_dev->config;
  const struct odrive_bus_config * bus_config = config->bus->config;
  struct odrive_axis_data * data = axis_dev->data;

  if (!data->online || (bus_config->heartbeat_timeout_ms == 0U)) {
    return;
  }

  if ((now - data->heartbeat_ms) <= (int64_t)bus_config->heartbeat_timeout_ms) {
    return;
  }

  data->online = false;
  data->armed = false;

  /*
   * Without a heartbeat there is no confirmation that the axis is still in
   * closed loop, so this is the same situation as a self-disarm: stop
   * commanding and let the application re-arm. Staying enabled would make the
   * driver silently re-arm the axis as soon as frames resumed. Unlike a
   * self-disarm the axis may still be running on its last setpoint, so it is
   * also told to go idle.
   */
  if (data->enabled) {
    data->enabled = false;
    data->idle_pending = true;
    data->idle_sent_ms = now - CONFIG_MOTOR_ODRIVE_STATE_RETRY_MS;
  }

  data->pending |= ODRIVE_PENDING_STATE;
}

static void odrive_bus_tx_work_handler(struct k_work * work)
{
  struct odrive_bus_data * bus = CONTAINER_OF(work, struct odrive_bus_data, tx_work);
  const int64_t now = k_uptime_get();
  const size_t start = bus->tx_start;
  bool refused = false;
  bool notify = false;

  odrive_bus_retry_start(bus->dev);

  for (size_t n = 0; n < ARRAY_SIZE(bus->axes); ++n) {
    const size_t i = (start + n) % ARRAY_SIZE(bus->axes);
    const struct device * axis_dev = bus->axes[i];

    if (axis_dev == NULL) {
      continue;
    }

    struct odrive_axis_data * data = axis_dev->data;
    struct can_frame frame;
    k_spinlock_key_t key;
    enum odrive_tx_stage stage = ODRIVE_TX_STAGE_NONE;

    key = k_spin_lock(&data->lock);
    odrive_check_timeout(axis_dev, now);

    /* Once the controller refuses one frame it refuses the rest of the burst. */
    if (!refused) {
      stage = odrive_build_tx(axis_dev, &frame, now);
    }

    notify = notify || (data->pending != 0U);
    k_spin_unlock(&data->lock, key);

    if (stage == ODRIVE_TX_STAGE_NONE) {
      continue;
    }

    if (odrive_axis_send(axis_dev, &frame, &data->tx_ctx[stage]) != 0) {
      key = k_spin_lock(&data->lock);
      odrive_tx_failed(data, stage);
      k_spin_unlock(&data->lock, key);

      refused = true;
      bus->tx_start = i;
    }
  }

  if (notify) {
    k_work_submit_to_queue(&bus->workq, &bus->dispatch_work);
  }
}

static void odrive_bus_tx_timer_handler(struct k_timer * timer)
{
  struct odrive_bus_data * bus = k_timer_user_data_get(timer);

  k_work_submit(&bus->tx_work);
}

/* A timeout of 0 means the value never expires. */
static bool odrive_is_recent(int64_t stamp, uint32_t timeout, int64_t now)
{
  if (stamp == ODRIVE_NEVER) {
    return false;
  }

  return (timeout == 0U) || ((now - stamp) <= (int64_t)timeout);
}

/* Caller holds the axis lock. */
static void odrive_fill_feedback(
  const struct device * axis_dev, struct odrive_feedback * out, int64_t now)
{
  const struct odrive_axis_config * config = axis_dev->config;
  const struct odrive_bus_config * bus_config = config->bus->config;
  const struct odrive_axis_data * data = axis_dev->data;

  memset(out, 0, sizeof(*out));

  out->position = data->position;
  out->velocity = data->velocity;
  out->iq_setpoint = data->iq_setpoint;
  out->iq_measured = data->iq_measured;
  out->fet_temperature = data->fet_temperature;
  out->motor_temperature = data->motor_temperature;
  out->torque_target = data->torque_target;
  out->torque_estimate = data->torque_estimate;
  out->bus_voltage = data->bus_voltage;
  out->bus_current = data->bus_current;

  out->axis_state = data->axis_state;
  out->procedure_result = data->procedure_result;
  out->active_errors = data->active_errors;
  out->disarm_reason = data->disarm_reason;
  out->trajectory_done = data->trajectory_done;
  out->control_mode = data->control_mode;
  out->input_mode = data->input_mode;
  out->enabled = data->enabled;
  out->online = data->online;
  out->timestamp_ms = data->timestamp_ms;

  /*
   * A measurement counts as present while its own cyclic message keeps
   * arriving. Most of those messages are disabled in the ODrive's default
   * configuration, so a clear bit usually means a rate was never enabled.
   * The estimates have a timeout of their own because they also decide
   * whether the snapshot is stale.
   */
  const uint32_t estimate_timeout = bus_config->estimate_timeout_ms;
  const uint32_t measurement_timeout = bus_config->measurement_timeout_ms;
  const struct {
    int64_t stamp;
    uint32_t bit;
    uint32_t timeout;
  } fields[] = {
    {data->estimates_ms, ODRIVE_FEEDBACK_ESTIMATES, estimate_timeout},
    {data->iq_ms, ODRIVE_FEEDBACK_IQ, measurement_timeout},
    {data->temperature_ms, ODRIVE_FEEDBACK_TEMPERATURE, measurement_timeout},
    {data->torques_ms, ODRIVE_FEEDBACK_TORQUES, measurement_timeout},
    {data->bus_ms, ODRIVE_FEEDBACK_BUS, measurement_timeout},
    {data->error_ms, ODRIVE_FEEDBACK_ERROR, measurement_timeout},
  };

  for (size_t i = 0; i < ARRAY_SIZE(fields); ++i) {
    if (odrive_is_recent(fields[i].stamp, fields[i].timeout, now)) {
      out->valid_mask |= fields[i].bit;
    }
  }

  out->stale = (data->estimates_ms != ODRIVE_NEVER) &&
               ((out->valid_mask & ODRIVE_FEEDBACK_ESTIMATES) == 0U);
}

static void odrive_bus_dispatch_work_handler(struct k_work * work)
{
  struct odrive_bus_data * bus = CONTAINER_OF(work, struct odrive_bus_data, dispatch_work);
  const int64_t now = k_uptime_get();

  for (size_t i = 0; i < ARRAY_SIZE(bus->axes); ++i) {
    const struct device * axis_dev = bus->axes[i];

    if (axis_dev == NULL) {
      continue;
    }

    struct odrive_axis_data * data = axis_dev->data;
    struct odrive_feedback feedback;
    odrive_callback_t state_cb;
    void * state_cb_data;
    odrive_callback_t feedback_cb;
    void * feedback_cb_data;
    k_spinlock_key_t key;
    uint8_t pending;

    key = k_spin_lock(&data->lock);
    pending = data->pending;
    data->pending = 0U;

    if (pending != 0U) {
      odrive_fill_feedback(axis_dev, &feedback, now);
    }

    state_cb = data->state_cb;
    state_cb_data = data->state_cb_data;
    feedback_cb = data->feedback_cb;
    feedback_cb_data = data->feedback_cb_data;
    k_spin_unlock(&data->lock, key);

    if (pending == 0U) {
      continue;
    }

    /* Outside the lock, so a callback may call back into this driver. */
    if (((pending & ODRIVE_PENDING_STATE) != 0U) && (state_cb != NULL)) {
      state_cb(axis_dev, &feedback, state_cb_data);
    }

    if (((pending & ODRIVE_PENDING_FEEDBACK) != 0U) && (feedback_cb != NULL)) {
      feedback_cb(axis_dev, &feedback, feedback_cb_data);
    }
  }
}

/* Returns true when the application has something new to be told about. */
static bool odrive_handle_heartbeat(
  const struct device * axis_dev, const struct can_frame * frame, int64_t now)
{
  struct odrive_axis_data * data = axis_dev->data;
  const uint32_t active_errors = sys_get_le32(&frame->data[0]);
  const enum odrive_axis_state axis_state = (enum odrive_axis_state)frame->data[4];
  const enum odrive_procedure_result procedure_result =
    (enum odrive_procedure_result)frame->data[5];
  const bool trajectory_done = (frame->data[6] & 0x01U) != 0U;
  k_spinlock_key_t key = k_spin_lock(&data->lock);

  const bool changed = !data->online || (data->axis_state != axis_state) ||
                       (data->procedure_result != procedure_result) ||
                       (data->active_errors != active_errors) ||
                       (data->trajectory_done != trajectory_done);

  data->axis_state = axis_state;
  data->procedure_result = procedure_result;
  data->active_errors = active_errors;
  data->trajectory_done = trajectory_done;
  data->online = true;
  data->heartbeat_ms = now;
  data->timestamp_ms = now;

  if (axis_state == ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) {
    /*
     * Only a heartbeat after this enable's request reached the wire counts.
     * One still showing the closed loop from before a quick disable and enable
     * would otherwise arm the axis, and the idle that follows would read as a
     * fault.
     */
    if (data->enabled && data->state_on_wire) {
      data->armed = true;
    }
  } else if (data->armed) {
    /*
     * The axis was armed and dropped out on its own, which means it tripped.
     * Recovering needs the errors cleared, and doing that here would clear
     * them without anyone having looked, so stop commanding and let the
     * application decide through the state callback.
     */
    data->armed = false;
    data->enabled = false;
    LOG_WRN("axis disarmed (state %u, errors 0x%08x)", (unsigned int)axis_state,
            (unsigned int)active_errors);
  }

  if (axis_state != ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) {
    data->idle_pending = false;
  }

  if (changed) {
    data->pending |= ODRIVE_PENDING_STATE;
  }

  k_spin_unlock(&data->lock, key);

  return changed;
}

static bool odrive_handle_frame(
  const struct device * axis_dev, uint16_t cmd, const struct can_frame * frame, int64_t now)
{
  struct odrive_axis_data * data = axis_dev->data;
  const float first = odrive_get_float(&frame->data[0]);
  const float second = odrive_get_float(&frame->data[4]);
  k_spinlock_key_t key = k_spin_lock(&data->lock);
  int64_t * stamp;
  bool notify = false;

  switch (cmd) {
    case ODRIVE_CMD_GET_ENCODER_ESTIMATES:
      data->position = first;
      data->velocity = second;
      stamp = &data->estimates_ms;
      data->pending |= ODRIVE_PENDING_FEEDBACK;
      notify = true;
      break;
    case ODRIVE_CMD_GET_IQ:
      data->iq_setpoint = first;
      data->iq_measured = second;
      stamp = &data->iq_ms;
      break;
    case ODRIVE_CMD_GET_TEMPERATURE:
      data->fet_temperature = first;
      data->motor_temperature = second;
      stamp = &data->temperature_ms;
      break;
    case ODRIVE_CMD_GET_TORQUES:
      data->torque_target = first;
      data->torque_estimate = second;
      stamp = &data->torques_ms;
      break;
    case ODRIVE_CMD_GET_BUS_VOLTAGE_CURRENT:
      data->bus_voltage = first;
      data->bus_current = second;
      stamp = &data->bus_ms;
      break;
    case ODRIVE_CMD_GET_ERROR: {
      const uint32_t active_errors = sys_get_le32(&frame->data[0]);
      const uint32_t disarm_reason = sys_get_le32(&frame->data[4]);

      /*
       * The next heartbeat compares against what is stored here, so a change
       * not reported now would never be.
       */
      if ((data->active_errors != active_errors) || (data->disarm_reason != disarm_reason)) {
        data->pending |= ODRIVE_PENDING_STATE;
        notify = true;
      }

      data->active_errors = active_errors;
      data->disarm_reason = disarm_reason;
      stamp = &data->error_ms;
      break;
    }
    default:
      k_spin_unlock(&data->lock, key);
      return false;
  }

  *stamp = now;
  data->timestamp_ms = now;
  k_spin_unlock(&data->lock, key);

  return notify;
}

static void odrive_rx_callback(
  const struct device * can_dev, struct can_frame * frame, void * user_data)
{
  const struct device * bus_dev = user_data;
  struct odrive_bus_data * bus = bus_dev->data;
  const int64_t now = k_uptime_get();

  ARG_UNUSED(can_dev);

  /* CANSimple is 11-bit classic CAN; anything else is not ours. */
  if ((frame->flags & (CAN_FRAME_IDE | CAN_FRAME_RTR)) != 0U) {
    return;
  }

  const uint16_t cmd = (uint16_t)(frame->id & ODRIVE_CMD_MASK);
  const uint8_t node_id = (uint8_t)((frame->id >> ODRIVE_NODE_SHIFT) & 0x3FU);
  const struct device * axis_dev = odrive_find_axis(bus, node_id);

  if (axis_dev == NULL) {
    return;
  }

  bool notify;

  if (cmd == ODRIVE_CMD_HEARTBEAT) {
    /* Trajectory_Done_Flag sits in byte 6, so a shorter frame is malformed. */
    if (can_dlc_to_bytes(frame->dlc) < 7U) {
      return;
    }

    notify = odrive_handle_heartbeat(axis_dev, frame, now);
  } else {
    if (can_dlc_to_bytes(frame->dlc) < 8U) {
      return;
    }

    notify = odrive_handle_frame(axis_dev, cmd, frame, now);
  }

  /*
   * Submitting an already-pending work is a no-op, which is what makes the
   * callbacks coalesce: a burst of frames produces one call carrying the
   * latest snapshot.
   */
  if (notify) {
    k_work_submit_to_queue(&bus->workq, &bus->dispatch_work);
  }
}

static int odrive_bus_init(const struct device * dev)
{
  const struct odrive_bus_config * config = dev->config;
  struct odrive_bus_data * data = dev->data;

  data->dev = dev;
  k_work_init(&data->tx_work, odrive_bus_tx_work_handler);
  k_work_init(&data->dispatch_work, odrive_bus_dispatch_work_handler);
  k_timer_init(&data->tx_timer, odrive_bus_tx_timer_handler, NULL);
  k_timer_user_data_set(&data->tx_timer, data);

  if (!device_is_ready(config->can_dev)) {
    LOG_ERR("CAN bus not ready");
    return -ENODEV;
  }

  k_work_queue_start(
    &data->workq, config->workq_stack, config->workq_stack_size,
    CONFIG_MOTOR_ODRIVE_WORKQ_PRIORITY, NULL);
  k_thread_name_set(&data->workq.thread, "odrive_cb");

  /*
   * A bus whose transceiver has no power holds RX dominant, and a controller
   * then never leaves initialisation mode. That is the normal state of a board
   * powered from the debug probe alone, so it must not be fatal. Retry the
   * start from the transmit work, so the bus comes up whenever the motor
   * supply does.
   */
  const int ret = can_start(config->can_dev);

  if ((ret < 0) && (ret != -EALREADY)) {
    LOG_WRN("CAN bus not startable yet (%d), retrying", ret);
  } else {
    data->started = true;
  }

  data->last_start_retry = k_uptime_get();

  /*
   * The timer runs even with no axis enabled: it also evaluates the heartbeat
   * timeout, which is how an application learns that an axis it has not armed
   * yet is present or gone.
   */
  k_timer_start(
    &data->tx_timer, K_MSEC(CONFIG_MOTOR_ODRIVE_TX_INTERVAL_MS),
    K_MSEC(CONFIG_MOTOR_ODRIVE_TX_INTERVAL_MS));

  return 0;
}

static int odrive_bus_register_axis(
  const struct device * bus_dev, const struct device * axis_dev, uint8_t node_id)
{
  const struct odrive_bus_config * bus_config = bus_dev->config;
  struct odrive_bus_data * bus = bus_dev->data;
  k_spinlock_key_t key;
  int ret = -ENOSPC;

  if (node_id > ODRIVE_NODE_ID_MAX) {
    LOG_ERR("Node ID %u is out of range", (unsigned int)node_id);
    return -EINVAL;
  }

  key = k_spin_lock(&bus->lock);

  if (odrive_find_axis(bus, node_id) != NULL) {
    k_spin_unlock(&bus->lock, key);
    LOG_ERR("Node ID %u is already registered", (unsigned int)node_id);
    return -EALREADY;
  }

  for (size_t i = 0; i < ARRAY_SIZE(bus->axes); ++i) {
    if (bus->axes[i] == NULL) {
      bus->axes[i] = axis_dev;
      ret = 0;
      break;
    }
  }

  k_spin_unlock(&bus->lock, key);

  if (ret != 0) {
    return ret;
  }

  const struct can_filter filter = {
    .id = (uint32_t)node_id << ODRIVE_NODE_SHIFT,
    /* Every message this axis sends shares the node ID in bits 10..5. */
    .mask = ODRIVE_NODE_MASK,
    .flags = 0,
  };

  const int filter_id =
    can_add_rx_filter(bus_config->can_dev, odrive_rx_callback, (void *)bus_dev, &filter);

  if (filter_id < 0) {
    LOG_ERR("Failed to add RX filter for node %u (%d)", (unsigned int)node_id, filter_id);

    /* Otherwise the command interval would drive an axis it cannot hear. */
    key = k_spin_lock(&bus->lock);

    for (size_t i = 0; i < ARRAY_SIZE(bus->axes); ++i) {
      if (bus->axes[i] == axis_dev) {
        bus->axes[i] = NULL;
      }
    }

    k_spin_unlock(&bus->lock, key);

    return filter_id;
  }

  return 0;
}

static int odrive_axis_enable(const struct device * dev)
{
  struct odrive_axis_data * data = dev->data;
  k_spinlock_key_t key = k_spin_lock(&data->lock);

  /*
   * Restart the whole sequence even if the axis is already enabled. This is
   * the only path that clears errors, and it is what an application calls to
   * recover an axis that disarmed on its own. The stored target is dropped;
   * doc/drivers/odrive.md says why.
   */
  data->enabled = true;
  data->armed = false;
  data->idle_pending = false;
  data->clear_errors_pending = true;
  data->mode_sent = false;
  data->state_sent = false;
  data->state_on_wire = false;
  data->has_target = false;

  k_spin_unlock(&data->lock, key);

  return 0;
}

/*
 * Stop commanding the axis and send frame, an idle or estop request, at once.
 * The command interval repeats the idle request for as long as the heartbeat
 * still shows closed loop, so a frame that is refused or lost does not leave
 * the axis running on its last setpoint.
 */
static int odrive_axis_stop(const struct device * dev, const struct can_frame * frame)
{
  const struct odrive_axis_config * config = dev->config;
  struct odrive_bus_data * bus = config->bus->data;
  struct odrive_axis_data * data = dev->data;
  const int64_t now = k_uptime_get();
  k_spinlock_key_t key = k_spin_lock(&data->lock);
  const bool was_enabled = data->enabled;
  int ret;

  data->enabled = false;
  data->armed = false;
  data->idle_pending = true;
  data->idle_sent_ms = now;
  odrive_idle_queued(data, now);
  data->clear_errors_pending = false;
  data->mode_sent = false;
  data->state_sent = false;
  data->state_on_wire = false;

  if (was_enabled) {
    data->pending |= ODRIVE_PENDING_STATE;
  }

  k_spin_unlock(&data->lock, key);

  if (was_enabled) {
    k_work_submit_to_queue(&bus->workq, &bus->dispatch_work);
  }

  ret = odrive_axis_send(dev, frame, &data->tx_ctx[ODRIVE_TX_STAGE_IDLE]);

  if (ret != 0) {
    key = k_spin_lock(&data->lock);
    odrive_tx_failed(data, ODRIVE_TX_STAGE_IDLE);
    k_spin_unlock(&data->lock, key);
  }

  return ret;
}

static int odrive_axis_disable(const struct device * dev)
{
  const struct odrive_axis_config * config = dev->config;
  struct can_frame frame;

  /* Stopping should not wait for the next command interval. */
  odrive_build_set_axis_state(&frame, config->node_id, ODRIVE_AXIS_STATE_IDLE);

  return odrive_axis_stop(dev, &frame);
}

static int odrive_axis_set_output(
  const struct device * dev, const enum motor_output_mode mode, const int16_t output)
{
  ARG_UNUSED(dev);
  ARG_UNUSED(mode);
  ARG_UNUSED(output);

  /*
   * An ODrive setpoint is bounded by the drive's own configuration rather than
   * by the protocol, so there is no unit this class-wide call could carry
   * without changing meaning between drivers. Use
   * include/drivers/motor/odrive.h instead.
   */
  return -ENOTSUP;
}

static int odrive_axis_get_feedback(const struct device * dev, void * feedback)
{
  const struct odrive_axis_config * config = dev->config;
  struct odrive_axis_data * data = dev->data;
  struct motor_feedback * out = feedback;
  const int64_t now = k_uptime_get();
  struct odrive_feedback snapshot;
  int64_t estimates_ms;
  int64_t counts;
  k_spinlock_key_t key;

  if (out == NULL) {
    return -EINVAL;
  }

  memset(out, 0, sizeof(*out));

  key = k_spin_lock(&data->lock);
  odrive_fill_feedback(dev, &snapshot, now);
  estimates_ms = data->estimates_ms;
  k_spin_unlock(&data->lock, key);

  /*
   * Only the fields that mean the same thing here as in the other motor
   * drivers are published. Velocity, current and single-turn orientation are
   * SI values with no unit agreed across the class; odrive_get_feedback()
   * reports them.
   */
  if (((snapshot.valid_mask & ODRIVE_FEEDBACK_ESTIMATES) != 0U) &&
      odrive_to_counts(snapshot.position, &counts)) {
    out->valid_mask |= MOTOR_FEEDBACK_POSITION;
    out->position = counts;
  }

  /*
   * The class reads a successful return as a position to use, and its
   * timestamp as the age of that position. Heartbeats and slow measurements
   * keep the snapshot's own timestamp fresh, so it would not do here.
   */
  out->online = snapshot.online;
  out->stale = snapshot.online && ((out->valid_mask & MOTOR_FEEDBACK_POSITION) == 0U);
  out->timestamp_ms = (estimates_ms != ODRIVE_NEVER) ? estimates_ms : snapshot.timestamp_ms;

  /*
   * Without a thermistor the ODrive reports a motor temperature of 0, which
   * would read as a real measurement rather than as a missing sensor.
   */
  if (config->has_motor_thermistor &&
      ((snapshot.valid_mask & ODRIVE_FEEDBACK_TEMPERATURE) != 0U) &&
      isfinite(snapshot.motor_temperature)) {
    out->valid_mask |= MOTOR_FEEDBACK_TEMPERATURE;
    out->temperature = (int16_t)CLAMP(snapshot.motor_temperature, INT16_MIN, INT16_MAX);
  }

  if (!out->online) {
    return -ENODATA;
  }

  return out->stale ? -EAGAIN : 0;
}

static int odrive_axis_init(const struct device * dev)
{
  const struct odrive_axis_config * config = dev->config;
  struct odrive_axis_data * data = dev->data;

  if (!device_is_ready(config->bus)) {
    LOG_ERR("Bus not ready for node %u", (unsigned int)config->node_id);
    return -ENODEV;
  }

  data->control_mode = ODRIVE_CONTROL_MODE_TORQUE;
  data->input_mode = ODRIVE_INPUT_MODE_PASSTHROUGH;
  data->estimates_ms = ODRIVE_NEVER;
  data->iq_ms = ODRIVE_NEVER;
  data->temperature_ms = ODRIVE_NEVER;
  data->torques_ms = ODRIVE_NEVER;
  data->bus_ms = ODRIVE_NEVER;
  data->error_ms = ODRIVE_NEVER;

  for (size_t i = 0; i < ARRAY_SIZE(data->tx_ctx); ++i) {
    data->tx_ctx[i].data = data;
    data->tx_ctx[i].stage = (enum odrive_tx_stage)i;
  }

  return odrive_bus_register_axis(config->bus, dev, config->node_id);
}

static const struct motor_driver_api odrive_axis_api = {
  .enable = odrive_axis_enable,
  .disable = odrive_axis_disable,
  .set_output = odrive_axis_set_output,
  .get_feedback = odrive_axis_get_feedback,
};

static bool odrive_is_axis(const struct device * dev)
{
  return (dev != NULL) && (dev->api == &odrive_axis_api);
}

/* The ODrive has to be told the new mode before the setpoint means it. */
static void odrive_mode_changed(struct odrive_axis_data * data)
{
  data->mode_sent = false;
  data->setpoint_dirty = true;
}

/*
 * Selecting a target also selects the mode it belongs to. An unchanged target
 * in an unchanged mode leaves nothing to send, which is what keeps a control
 * loop re-applying the same TRAP_TRAJ target from re-planning the trajectory.
 * The first target after an enable counts as changed even if it repeats the
 * dropped one.
 */
static void odrive_select_mode(
  struct odrive_axis_data * data, enum odrive_control_mode mode, bool target_changed)
{
  if (data->control_mode != mode) {
    data->control_mode = mode;
    odrive_mode_changed(data);
  }

  if (target_changed || !data->has_target) {
    data->setpoint_dirty = true;
  }

  data->has_target = true;
}

int odrive_set_position(
  const struct device * dev, float position, float velocity_ff, float torque_ff)
{
  if (!odrive_is_axis(dev) || !isfinite(position) || !isfinite(velocity_ff) ||
      !isfinite(torque_ff)) {
    return -EINVAL;
  }

  struct odrive_axis_data * data = dev->data;
  k_spinlock_key_t key = k_spin_lock(&data->lock);
  const bool changed = (data->target_position != position) ||
                       (data->velocity_ff != velocity_ff) || (data->torque_ff != torque_ff);

  data->target_position = position;
  data->velocity_ff = velocity_ff;
  data->torque_ff = torque_ff;
  odrive_select_mode(data, ODRIVE_CONTROL_MODE_POSITION, changed);

  k_spin_unlock(&data->lock, key);

  return 0;
}

int odrive_set_velocity(const struct device * dev, float velocity, float torque_ff)
{
  if (!odrive_is_axis(dev) || !isfinite(velocity) || !isfinite(torque_ff)) {
    return -EINVAL;
  }

  struct odrive_axis_data * data = dev->data;
  k_spinlock_key_t key = k_spin_lock(&data->lock);
  const bool changed = (data->target_velocity != velocity) || (data->torque_ff != torque_ff);

  data->target_velocity = velocity;
  data->torque_ff = torque_ff;
  odrive_select_mode(data, ODRIVE_CONTROL_MODE_VELOCITY, changed);

  k_spin_unlock(&data->lock, key);

  return 0;
}

int odrive_set_torque(const struct device * dev, float torque)
{
  if (!odrive_is_axis(dev) || !isfinite(torque)) {
    return -EINVAL;
  }

  struct odrive_axis_data * data = dev->data;
  k_spinlock_key_t key = k_spin_lock(&data->lock);
  const bool changed = data->target_torque != torque;

  data->target_torque = torque;
  odrive_select_mode(data, ODRIVE_CONTROL_MODE_TORQUE, changed);

  k_spin_unlock(&data->lock, key);

  return 0;
}

static bool odrive_input_mode_is_valid(enum odrive_input_mode mode)
{
  switch (mode) {
    case ODRIVE_INPUT_MODE_PASSTHROUGH:
    case ODRIVE_INPUT_MODE_VEL_RAMP:
    case ODRIVE_INPUT_MODE_POS_FILTER:
    case ODRIVE_INPUT_MODE_TRAP_TRAJ:
    case ODRIVE_INPUT_MODE_TORQUE_RAMP:
      return true;
    default:
      return false;
  }
}

int odrive_set_input_mode(const struct device * dev, enum odrive_input_mode mode)
{
  if (!odrive_is_axis(dev) || !odrive_input_mode_is_valid(mode)) {
    return -EINVAL;
  }

  struct odrive_axis_data * data = dev->data;
  k_spinlock_key_t key = k_spin_lock(&data->lock);

  if (data->input_mode != mode) {
    data->input_mode = mode;
    odrive_mode_changed(data);
  }

  k_spin_unlock(&data->lock, key);

  return 0;
}

int odrive_set_state_callback(
  const struct device * dev, odrive_callback_t callback, void * user_data)
{
  if (!odrive_is_axis(dev)) {
    return -EINVAL;
  }

  struct odrive_axis_data * data = dev->data;
  k_spinlock_key_t key = k_spin_lock(&data->lock);

  data->state_cb = callback;
  data->state_cb_data = user_data;

  k_spin_unlock(&data->lock, key);

  return 0;
}

int odrive_set_feedback_callback(
  const struct device * dev, odrive_callback_t callback, void * user_data)
{
  if (!odrive_is_axis(dev)) {
    return -EINVAL;
  }

  struct odrive_axis_data * data = dev->data;
  k_spinlock_key_t key = k_spin_lock(&data->lock);

  data->feedback_cb = callback;
  data->feedback_cb_data = user_data;

  k_spin_unlock(&data->lock, key);

  return 0;
}

int odrive_get_feedback(const struct device * dev, struct odrive_feedback * feedback)
{
  if (!odrive_is_axis(dev) || (feedback == NULL)) {
    return -EINVAL;
  }

  struct odrive_axis_data * data = dev->data;
  const int64_t now = k_uptime_get();
  k_spinlock_key_t key = k_spin_lock(&data->lock);

  odrive_fill_feedback(dev, feedback, now);

  k_spin_unlock(&data->lock, key);

  if (!feedback->online) {
    return -ENODATA;
  }

  return feedback->stale ? -EAGAIN : 0;
}

int odrive_set_limits(const struct device * dev, float velocity_limit, float current_limit)
{
  if (!odrive_is_axis(dev) || !isfinite(velocity_limit) || !isfinite(current_limit)) {
    return -EINVAL;
  }

  const struct odrive_axis_config * config = dev->config;
  struct can_frame frame;

  odrive_frame_init(&frame, config->node_id, ODRIVE_CMD_SET_LIMITS, 8U);
  odrive_put_float(&frame.data[0], velocity_limit);
  odrive_put_float(&frame.data[4], current_limit);

  return odrive_axis_send(dev, &frame, NULL);
}

int odrive_set_traj_limits(
  const struct device * dev, float velocity_limit, float accel_limit, float decel_limit)
{
  if (!odrive_is_axis(dev) || !isfinite(velocity_limit) || !isfinite(accel_limit) ||
      !isfinite(decel_limit)) {
    return -EINVAL;
  }

  const struct odrive_axis_config * config = dev->config;
  struct can_frame frame;
  int ret;

  odrive_frame_init(&frame, config->node_id, ODRIVE_CMD_SET_TRAJ_VEL_LIMIT, 4U);
  odrive_put_float(&frame.data[0], velocity_limit);

  ret = odrive_axis_send(dev, &frame, NULL);

  if (ret != 0) {
    return ret;
  }

  odrive_frame_init(&frame, config->node_id, ODRIVE_CMD_SET_TRAJ_ACCEL_LIMITS, 8U);
  odrive_put_float(&frame.data[0], accel_limit);
  odrive_put_float(&frame.data[4], decel_limit);

  return odrive_axis_send(dev, &frame, NULL);
}

int odrive_set_gains(
  const struct device * dev, float pos_gain, float vel_gain, float vel_integrator_gain)
{
  if (!odrive_is_axis(dev) || !isfinite(pos_gain) || !isfinite(vel_gain) ||
      !isfinite(vel_integrator_gain)) {
    return -EINVAL;
  }

  const struct odrive_axis_config * config = dev->config;
  struct can_frame frame;
  int ret;

  odrive_frame_init(&frame, config->node_id, ODRIVE_CMD_SET_POS_GAIN, 4U);
  odrive_put_float(&frame.data[0], pos_gain);

  ret = odrive_axis_send(dev, &frame, NULL);

  if (ret != 0) {
    return ret;
  }

  odrive_frame_init(&frame, config->node_id, ODRIVE_CMD_SET_VEL_GAINS, 8U);
  odrive_put_float(&frame.data[0], vel_gain);
  odrive_put_float(&frame.data[4], vel_integrator_gain);

  return odrive_axis_send(dev, &frame, NULL);
}

int odrive_set_absolute_position(const struct device * dev, float position)
{
  if (!odrive_is_axis(dev) || !isfinite(position)) {
    return -EINVAL;
  }

  const struct odrive_axis_config * config = dev->config;
  struct can_frame frame;

  odrive_frame_init(&frame, config->node_id, ODRIVE_CMD_SET_ABSOLUTE_POSITION, 4U);
  odrive_put_float(&frame.data[0], position);

  return odrive_axis_send(dev, &frame, NULL);
}

int odrive_clear_errors(const struct device * dev)
{
  if (!odrive_is_axis(dev)) {
    return -EINVAL;
  }

  const struct odrive_axis_config * config = dev->config;
  struct can_frame frame;

  odrive_build_clear_errors(&frame, config->node_id);

  return odrive_axis_send(dev, &frame, NULL);
}

int odrive_estop(const struct device * dev)
{
  if (!odrive_is_axis(dev)) {
    return -EINVAL;
  }

  const struct odrive_axis_config * config = dev->config;
  struct can_frame frame;

  odrive_frame_init(&frame, config->node_id, ODRIVE_CMD_ESTOP, 0U);

  return odrive_axis_stop(dev, &frame);
}

int odrive_request_axis_state(const struct device * dev, enum odrive_axis_state state)
{
  if (!odrive_is_axis(dev)) {
    return -EINVAL;
  }

  const struct odrive_axis_config * config = dev->config;
  struct odrive_axis_data * data = dev->data;
  struct can_frame frame;
  k_spinlock_key_t key = k_spin_lock(&data->lock);
  const bool busy = data->enabled;

  /* The application has taken the axis state over from the driver. */
  if (!busy) {
    data->idle_pending = false;
  }

  k_spin_unlock(&data->lock, key);

  /* Otherwise the command interval would immediately ask for closed loop. */
  if (busy) {
    return -EBUSY;
  }

  odrive_build_set_axis_state(&frame, config->node_id, state);

  return odrive_axis_send(dev, &frame, NULL);
}

#define DT_DRV_COMPAT odrive_bus

#define ODRIVE_BUS_DEFINE(inst)                                                            \
  static K_KERNEL_STACK_DEFINE(                                                            \
    odrive_workq_stack_##inst, CONFIG_MOTOR_ODRIVE_WORKQ_STACK_SIZE);                      \
  static struct odrive_bus_data odrive_bus_data_##inst;                                    \
  static const struct odrive_bus_config odrive_bus_config_##inst = {                       \
    .can_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, can)),                                  \
    .workq_stack = odrive_workq_stack_##inst,                                              \
    .workq_stack_size = K_KERNEL_STACK_SIZEOF(odrive_workq_stack_##inst),                  \
    .heartbeat_timeout_ms = DT_INST_PROP(inst, heartbeat_timeout_ms),                      \
    .estimate_timeout_ms = DT_INST_PROP(inst, estimate_timeout_ms),                        \
    .measurement_timeout_ms = DT_INST_PROP(inst, measurement_timeout_ms),                  \
  };                                                                                       \
  DEVICE_DT_INST_DEFINE(                                                                   \
    inst, odrive_bus_init, NULL, &odrive_bus_data_##inst, &odrive_bus_config_##inst,       \
    POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(ODRIVE_BUS_DEFINE)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT odrive_axis

#define ODRIVE_AXIS_DEFINE(inst)                                                           \
  BUILD_ASSERT(                                                                            \
    DT_INST_REG_ADDR(inst) <= ODRIVE_NODE_ID_MAX,                                          \
    "ODrive node ID out of range; 63 is the unaddressed value");                           \
  static struct odrive_axis_data odrive_axis_data_##inst;                                  \
  static const struct odrive_axis_config odrive_axis_config_##inst = {                     \
    .bus = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                            \
    .node_id = DT_INST_REG_ADDR(inst),                                                     \
    .has_motor_thermistor = DT_INST_PROP(inst, has_motor_thermistor),                      \
  };                                                                                       \
  DEVICE_DT_INST_DEFINE(                                                                   \
    inst, odrive_axis_init, NULL, &odrive_axis_data_##inst, &odrive_axis_config_##inst,    \
    POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY, &odrive_axis_api);

DT_INST_FOREACH_STATUS_OKAY(ODRIVE_AXIS_DEFINE)
