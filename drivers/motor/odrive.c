/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/motor.h>
#include <drivers/motor/odrive.h>
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
 * doubling beyond it. See ADR 0006.
 */
#define ODRIVE_COUNTS_PER_REV 65536.0F

/*
 * Vel_FF and Torque_FF of Set_Input_Pos are int16 scaled by the ODrive's
 * input_vel_scale and input_torque_scale, both of which default to 0.001. The
 * driver assumes the defaults rather than writing them.
 */
#define ODRIVE_FF_SCALE 1000.0F

#define ODRIVE_PENDING_FEEDBACK BIT(0)
#define ODRIVE_PENDING_STATE BIT(1)

struct odrive_bus_config
{
  const struct device * can_dev;
  k_thread_stack_t * workq_stack;
  size_t workq_stack_size;
  uint32_t heartbeat_timeout_ms;
  uint32_t estimate_timeout_ms;
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

  /*
   * Arming progress. Cleared on enable and whenever the commanded mode
   * changes, because the ODrive has to be told the mode before the state.
   */
  bool clear_errors_pending;
  bool mode_sent;
  bool state_sent;
  int64_t state_sent_ms;
  /* Set once the heartbeat has confirmed closed loop, so that a later drop
   * out of it is a fault rather than the arming still being in progress. */
  bool armed;

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

static int odrive_axis_send(const struct device * axis_dev, const struct can_frame * frame)
{
  const struct odrive_axis_config * config = axis_dev->config;
  const struct odrive_bus_config * bus_config = config->bus->config;
  struct odrive_bus_data * bus = config->bus->data;

  if (!bus->started) {
    return -EIO;
  }

  return can_send(bus_config->can_dev, frame, K_NO_WAIT, NULL, NULL);
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

static void odrive_build_setpoint(
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
    default:
      odrive_frame_init(frame, node_id, ODRIVE_CMD_SET_INPUT_TORQUE, 4U);
      odrive_put_float(&frame->data[0], data->target_torque);
      break;
  }
}

/*
 * Produce the frame this axis owes the bus in this interval. One arming stage
 * per interval keeps the burst short; the axis is armed a few intervals after
 * it is enabled.
 */
static size_t odrive_build_tx(const struct device * axis_dev, struct can_frame * frame, int64_t now)
{
  const struct odrive_axis_config * config = axis_dev->config;
  struct odrive_axis_data * data = axis_dev->data;
  const uint8_t node_id = config->node_id;

  if (!data->enabled) {
    return 0;
  }

  if (data->clear_errors_pending) {
    odrive_build_clear_errors(frame, node_id);
    data->clear_errors_pending = false;
    return 1;
  }

  if (!data->mode_sent) {
    odrive_build_set_controller_mode(frame, node_id, data->control_mode, data->input_mode);
    data->mode_sent = true;
    data->state_sent = false;
    return 1;
  }

  if (!data->state_sent) {
    odrive_build_set_axis_state(frame, node_id, ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL);
    data->state_sent = true;
    data->state_sent_ms = now;
    return 1;
  }

  if (!data->armed) {
    /*
     * Ask again, but never clear errors from here: a tripped axis would then
     * be cycled through its fault for as long as it stays enabled.
     */
    if ((now - data->state_sent_ms) >= CONFIG_MOTOR_ODRIVE_STATE_RETRY_MS) {
      data->mode_sent = false;
    }

    return 0;
  }

  if ((data->control_mode == ODRIVE_CONTROL_MODE_POSITION) &&
      (data->input_mode == ODRIVE_INPUT_MODE_TRAP_TRAJ) && !data->setpoint_dirty) {
    return 0;
  }

  odrive_build_setpoint(frame, node_id, data);
  data->setpoint_dirty = false;

  return 1;
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
  data->pending |= ODRIVE_PENDING_STATE;
}

static void odrive_bus_tx_work_handler(struct k_work * work)
{
  struct odrive_bus_data * bus = CONTAINER_OF(work, struct odrive_bus_data, tx_work);
  const int64_t now = k_uptime_get();
  bool notify = false;

  odrive_bus_retry_start(bus->dev);

  for (size_t i = 0; i < ARRAY_SIZE(bus->axes); ++i) {
    const struct device * axis_dev = bus->axes[i];

    if (axis_dev == NULL) {
      continue;
    }

    struct odrive_axis_data * data = axis_dev->data;
    struct can_frame frame;
    k_spinlock_key_t key;
    size_t count;

    key = k_spin_lock(&data->lock);
    odrive_check_timeout(axis_dev, now);
    count = odrive_build_tx(axis_dev, &frame, now);
    notify = notify || (data->pending != 0U);
    k_spin_unlock(&data->lock, key);

    if (count != 0U) {
      /* A frame lost to a busy bus is re-sent on the next interval. */
      (void)odrive_axis_send(axis_dev, &frame);
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

/* Caller holds the axis lock. */
static void odrive_fill_feedback(
  const struct device * axis_dev, struct odrive_feedback * out, int64_t now)
{
  const struct odrive_axis_config * config = axis_dev->config;
  const struct odrive_bus_config * bus_config = config->bus->config;
  const struct odrive_axis_data * data = axis_dev->data;
  const uint32_t timeout = bus_config->estimate_timeout_ms;

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
   */
#define ODRIVE_MARK_VALID(stamp, bit)                                                     \
  if ((stamp != 0) && ((timeout == 0U) || ((now - stamp) <= (int64_t)timeout))) {         \
    out->valid_mask |= (bit);                                                             \
  }

  ODRIVE_MARK_VALID(data->estimates_ms, ODRIVE_FEEDBACK_ESTIMATES)
  ODRIVE_MARK_VALID(data->iq_ms, ODRIVE_FEEDBACK_IQ)
  ODRIVE_MARK_VALID(data->temperature_ms, ODRIVE_FEEDBACK_TEMPERATURE)
  ODRIVE_MARK_VALID(data->torques_ms, ODRIVE_FEEDBACK_TORQUES)
  ODRIVE_MARK_VALID(data->bus_ms, ODRIVE_FEEDBACK_BUS)
  ODRIVE_MARK_VALID(data->error_ms, ODRIVE_FEEDBACK_ERROR)

#undef ODRIVE_MARK_VALID

  out->stale = (data->estimates_ms != 0) && ((out->valid_mask & ODRIVE_FEEDBACK_ESTIMATES) == 0U);
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
                       (data->active_errors != active_errors);

  data->axis_state = axis_state;
  data->procedure_result = procedure_result;
  data->active_errors = active_errors;
  data->trajectory_done = trajectory_done;
  data->online = true;
  data->heartbeat_ms = now;
  data->timestamp_ms = now;

  if (axis_state == ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) {
    data->armed = true;
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
  k_spinlock_key_t key;
  bool notify = false;

  switch (cmd) {
    case ODRIVE_CMD_GET_ENCODER_ESTIMATES:
      key = k_spin_lock(&data->lock);
      data->position = odrive_get_float(&frame->data[0]);
      data->velocity = odrive_get_float(&frame->data[4]);
      data->estimates_ms = now;
      data->timestamp_ms = now;
      data->pending |= ODRIVE_PENDING_FEEDBACK;
      k_spin_unlock(&data->lock, key);
      notify = true;
      break;
    case ODRIVE_CMD_GET_IQ:
      key = k_spin_lock(&data->lock);
      data->iq_setpoint = odrive_get_float(&frame->data[0]);
      data->iq_measured = odrive_get_float(&frame->data[4]);
      data->iq_ms = now;
      data->timestamp_ms = now;
      k_spin_unlock(&data->lock, key);
      break;
    case ODRIVE_CMD_GET_TEMPERATURE:
      key = k_spin_lock(&data->lock);
      data->fet_temperature = odrive_get_float(&frame->data[0]);
      data->motor_temperature = odrive_get_float(&frame->data[4]);
      data->temperature_ms = now;
      data->timestamp_ms = now;
      k_spin_unlock(&data->lock, key);
      break;
    case ODRIVE_CMD_GET_TORQUES:
      key = k_spin_lock(&data->lock);
      data->torque_target = odrive_get_float(&frame->data[0]);
      data->torque_estimate = odrive_get_float(&frame->data[4]);
      data->torques_ms = now;
      data->timestamp_ms = now;
      k_spin_unlock(&data->lock, key);
      break;
    case ODRIVE_CMD_GET_BUS_VOLTAGE_CURRENT:
      key = k_spin_lock(&data->lock);
      data->bus_voltage = odrive_get_float(&frame->data[0]);
      data->bus_current = odrive_get_float(&frame->data[4]);
      data->bus_ms = now;
      data->timestamp_ms = now;
      k_spin_unlock(&data->lock, key);
      break;
    case ODRIVE_CMD_GET_ERROR:
      key = k_spin_lock(&data->lock);
      data->active_errors = sys_get_le32(&frame->data[0]);
      data->disarm_reason = sys_get_le32(&frame->data[4]);
      data->error_ms = now;
      data->timestamp_ms = now;
      k_spin_unlock(&data->lock, key);
      break;
    default:
      break;
  }

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
   * recover an axis that disarmed on its own.
   */
  data->enabled = true;
  data->armed = false;
  data->clear_errors_pending = true;
  data->mode_sent = false;
  data->state_sent = false;
  data->setpoint_dirty = true;

  k_spin_unlock(&data->lock, key);

  return 0;
}

static int odrive_axis_disable(const struct device * dev)
{
  const struct odrive_axis_config * config = dev->config;
  struct odrive_axis_data * data = dev->data;
  struct can_frame frame;
  k_spinlock_key_t key = k_spin_lock(&data->lock);

  data->enabled = false;
  data->armed = false;
  data->clear_errors_pending = false;
  data->mode_sent = false;
  data->state_sent = false;

  k_spin_unlock(&data->lock, key);

  /* Stopping should not wait for the next command interval. */
  odrive_build_set_axis_state(&frame, config->node_id, ODRIVE_AXIS_STATE_IDLE);

  return odrive_axis_send(dev, &frame);
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
  k_spinlock_key_t key;
  int ret = 0;

  if (out == NULL) {
    return -EINVAL;
  }

  memset(out, 0, sizeof(*out));

  key = k_spin_lock(&data->lock);
  odrive_fill_feedback(dev, &snapshot, now);
  k_spin_unlock(&data->lock, key);

  /*
   * Only the fields that mean the same thing here as in the other motor
   * drivers are published. Velocity, current and single-turn orientation are
   * SI values with no unit agreed across the class; odrive_get_feedback()
   * reports them.
   */
  if ((snapshot.valid_mask & ODRIVE_FEEDBACK_ESTIMATES) != 0U) {
    out->valid_mask |= MOTOR_FEEDBACK_POSITION;
    out->position = odrive_round(snapshot.position * ODRIVE_COUNTS_PER_REV);
  }

  out->online = snapshot.online;
  out->stale = snapshot.stale;
  out->timestamp_ms = snapshot.timestamp_ms;

  /*
   * Without a thermistor the ODrive reports a motor temperature of 0, which
   * would read as a real measurement rather than as a missing sensor.
   */
  if (config->has_motor_thermistor &&
      ((snapshot.valid_mask & ODRIVE_FEEDBACK_TEMPERATURE) != 0U)) {
    out->valid_mask |= MOTOR_FEEDBACK_TEMPERATURE;
    out->temperature = (int16_t)snapshot.motor_temperature;
  }

  if (!snapshot.online) {
    ret = -ENODATA;
  } else if (snapshot.stale) {
    ret = -EAGAIN;
  }

  return ret;
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

/* Selecting a target also selects the mode it belongs to. */
static void odrive_select_mode(struct odrive_axis_data * data, enum odrive_control_mode mode)
{
  if (data->control_mode != mode) {
    data->control_mode = mode;
    /* The ODrive has to be told the new mode before the setpoint means it. */
    data->mode_sent = false;
  }

  data->setpoint_dirty = true;
}

int odrive_set_position(
  const struct device * dev, float position, float velocity_ff, float torque_ff)
{
  if (!odrive_is_axis(dev)) {
    return -EINVAL;
  }

  struct odrive_axis_data * data = dev->data;
  k_spinlock_key_t key = k_spin_lock(&data->lock);

  data->target_position = position;
  data->velocity_ff = velocity_ff;
  data->torque_ff = torque_ff;
  odrive_select_mode(data, ODRIVE_CONTROL_MODE_POSITION);

  k_spin_unlock(&data->lock, key);

  return 0;
}

int odrive_set_velocity(const struct device * dev, float velocity, float torque_ff)
{
  if (!odrive_is_axis(dev)) {
    return -EINVAL;
  }

  struct odrive_axis_data * data = dev->data;
  k_spinlock_key_t key = k_spin_lock(&data->lock);

  data->target_velocity = velocity;
  data->torque_ff = torque_ff;
  odrive_select_mode(data, ODRIVE_CONTROL_MODE_VELOCITY);

  k_spin_unlock(&data->lock, key);

  return 0;
}

int odrive_set_torque(const struct device * dev, float torque)
{
  if (!odrive_is_axis(dev)) {
    return -EINVAL;
  }

  struct odrive_axis_data * data = dev->data;
  k_spinlock_key_t key = k_spin_lock(&data->lock);

  data->target_torque = torque;
  odrive_select_mode(data, ODRIVE_CONTROL_MODE_TORQUE);

  k_spin_unlock(&data->lock, key);

  return 0;
}

int odrive_set_input_mode(const struct device * dev, enum odrive_input_mode mode)
{
  if (!odrive_is_axis(dev)) {
    return -EINVAL;
  }

  struct odrive_axis_data * data = dev->data;
  k_spinlock_key_t key = k_spin_lock(&data->lock);

  if (data->input_mode != mode) {
    data->input_mode = mode;
    data->mode_sent = false;
    data->setpoint_dirty = true;
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

  if (feedback->stale) {
    return -EAGAIN;
  }

  return 0;
}

int odrive_set_limits(const struct device * dev, float velocity_limit, float current_limit)
{
  if (!odrive_is_axis(dev)) {
    return -EINVAL;
  }

  const struct odrive_axis_config * config = dev->config;
  struct can_frame frame;

  odrive_frame_init(&frame, config->node_id, ODRIVE_CMD_SET_LIMITS, 8U);
  odrive_put_float(&frame.data[0], velocity_limit);
  odrive_put_float(&frame.data[4], current_limit);

  return odrive_axis_send(dev, &frame);
}

int odrive_set_traj_limits(
  const struct device * dev, float velocity_limit, float accel_limit, float decel_limit)
{
  if (!odrive_is_axis(dev)) {
    return -EINVAL;
  }

  const struct odrive_axis_config * config = dev->config;
  struct can_frame frame;
  int ret;

  odrive_frame_init(&frame, config->node_id, ODRIVE_CMD_SET_TRAJ_VEL_LIMIT, 4U);
  odrive_put_float(&frame.data[0], velocity_limit);

  ret = odrive_axis_send(dev, &frame);

  if (ret != 0) {
    return ret;
  }

  odrive_frame_init(&frame, config->node_id, ODRIVE_CMD_SET_TRAJ_ACCEL_LIMITS, 8U);
  odrive_put_float(&frame.data[0], accel_limit);
  odrive_put_float(&frame.data[4], decel_limit);

  return odrive_axis_send(dev, &frame);
}

int odrive_set_gains(
  const struct device * dev, float pos_gain, float vel_gain, float vel_integrator_gain)
{
  if (!odrive_is_axis(dev)) {
    return -EINVAL;
  }

  const struct odrive_axis_config * config = dev->config;
  struct can_frame frame;
  int ret;

  odrive_frame_init(&frame, config->node_id, ODRIVE_CMD_SET_POS_GAIN, 4U);
  odrive_put_float(&frame.data[0], pos_gain);

  ret = odrive_axis_send(dev, &frame);

  if (ret != 0) {
    return ret;
  }

  odrive_frame_init(&frame, config->node_id, ODRIVE_CMD_SET_VEL_GAINS, 8U);
  odrive_put_float(&frame.data[0], vel_gain);
  odrive_put_float(&frame.data[4], vel_integrator_gain);

  return odrive_axis_send(dev, &frame);
}

int odrive_set_absolute_position(const struct device * dev, float position)
{
  if (!odrive_is_axis(dev)) {
    return -EINVAL;
  }

  const struct odrive_axis_config * config = dev->config;
  struct can_frame frame;

  odrive_frame_init(&frame, config->node_id, ODRIVE_CMD_SET_ABSOLUTE_POSITION, 4U);
  odrive_put_float(&frame.data[0], position);

  return odrive_axis_send(dev, &frame);
}

int odrive_clear_errors(const struct device * dev)
{
  if (!odrive_is_axis(dev)) {
    return -EINVAL;
  }

  const struct odrive_axis_config * config = dev->config;
  struct can_frame frame;

  odrive_build_clear_errors(&frame, config->node_id);

  return odrive_axis_send(dev, &frame);
}

int odrive_estop(const struct device * dev)
{
  if (!odrive_is_axis(dev)) {
    return -EINVAL;
  }

  const struct odrive_axis_config * config = dev->config;
  struct odrive_axis_data * data = dev->data;
  struct can_frame frame;
  k_spinlock_key_t key = k_spin_lock(&data->lock);

  data->enabled = false;
  data->armed = false;
  data->clear_errors_pending = false;
  data->mode_sent = false;
  data->state_sent = false;

  k_spin_unlock(&data->lock, key);

  odrive_frame_init(&frame, config->node_id, ODRIVE_CMD_ESTOP, 0U);

  return odrive_axis_send(dev, &frame);
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

  k_spin_unlock(&data->lock, key);

  /* Otherwise the command interval would immediately ask for closed loop. */
  if (busy) {
    return -EBUSY;
  }

  odrive_build_set_axis_state(&frame, config->node_id, state);

  return odrive_axis_send(dev, &frame);
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
