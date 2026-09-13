/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/motor.h>
#include <drivers/motor/robstride.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(motor_robstride, CONFIG_MOTOR_LOG_LEVEL);

#define ROBSTRIDE_MAX_CANS 4
#define ROBSTRIDE_CANBUS_UNKNOWN 0xFF
#define ROBSTRIDE_START_RETRY_MS 500
/* A disabled motor is silent, so probe it slowly to keep presence known. */
#define ROBSTRIDE_PROBE_INTERVAL_MS 500
/* Largest number of frames one motor can need in a single interval, which is
 * the stage that writes the five loop gains. */
#define ROBSTRIDE_TX_BURST 5

/* Communication types, carried in bits 28..24 of the identifier. */
#define ROBSTRIDE_TYPE_GET_ID 0x00U
#define ROBSTRIDE_TYPE_OP_CONTROL 0x01U
#define ROBSTRIDE_TYPE_FEEDBACK 0x02U
#define ROBSTRIDE_TYPE_ENABLE 0x03U
#define ROBSTRIDE_TYPE_STOP 0x04U
#define ROBSTRIDE_TYPE_SET_ZERO 0x06U
#define ROBSTRIDE_TYPE_GET_PARAM 0x11U
#define ROBSTRIDE_TYPE_SET_PARAM 0x12U
#define ROBSTRIDE_TYPE_FAULT 0x15U
#define ROBSTRIDE_TYPE_SAVE 0x16U

/* Parameter indices. */
#define ROBSTRIDE_PARAM_RUN_MODE 0x7005U
#define ROBSTRIDE_PARAM_IQ_REF 0x7006U
#define ROBSTRIDE_PARAM_SPD_REF 0x700AU
#define ROBSTRIDE_PARAM_LIMIT_TORQUE 0x700BU
#define ROBSTRIDE_PARAM_CUR_KP 0x7010U
#define ROBSTRIDE_PARAM_CUR_KI 0x7011U
#define ROBSTRIDE_PARAM_LOC_REF 0x7016U
#define ROBSTRIDE_PARAM_LIMIT_SPD 0x7017U
#define ROBSTRIDE_PARAM_LIMIT_CUR 0x7018U
#define ROBSTRIDE_PARAM_LOC_KP 0x701EU
#define ROBSTRIDE_PARAM_SPD_KP 0x701FU
#define ROBSTRIDE_PARAM_SPD_KI 0x7020U
#define ROBSTRIDE_PARAM_EP_SCAN_TIME 0x7026U
#define ROBSTRIDE_PARAM_ZERO_STA 0x7029U

/* Run mode values written to ROBSTRIDE_PARAM_RUN_MODE. */
#define ROBSTRIDE_RUN_MODE_OPERATION 0U
#define ROBSTRIDE_RUN_MODE_POSITION_PP 1U
#define ROBSTRIDE_RUN_MODE_VELOCITY 2U
#define ROBSTRIDE_RUN_MODE_CURRENT 3U
#define ROBSTRIDE_RUN_MODE_POSITION_CSP 5U

/*
 * Position is quantised over +-4 pi with the full 16-bit code space, so the
 * code wraps back through the same physical point after UINT16_MAX steps
 * rather than after 65536. The accumulator below uses that as its wrap.
 */
#define ROBSTRIDE_POSITION_MAX 12.56637F
#define ROBSTRIDE_POSITION_MIN (-ROBSTRIDE_POSITION_MAX)
#define ROBSTRIDE_POSITION_WRAP UINT16_MAX
#define ROBSTRIDE_POSITION_HALF_WRAP (ROBSTRIDE_POSITION_WRAP / 2)

/* Gain ranges of the operation control frame. */
#define ROBSTRIDE_OP_KP_MAX 500.0F
#define ROBSTRIDE_OP_KD_MAX 5.0F

struct robstride_model_limits
{
  float velocity;
  float torque;
  float current;
};

/*
 * Only the models that have been confirmed against hardware are listed. A
 * wrong entry does not fail loudly: it silently rescales every command and
 * every measurement of that motor.
 */
static const struct robstride_model_limits robstride_model_limits[] = {
  [ROBSTRIDE_MODEL_RS00] = {.velocity = 33.0F, .torque = 14.0F, .current = 15.5F},
  [ROBSTRIDE_MODEL_RS05] = {.velocity = 50.0F, .torque = 5.5F, .current = 11.0F},
};

struct robstride_bus_config
{
  const struct device * const * can_devs;
  size_t can_count;
  uint8_t master_can_id;
  uint32_t feedback_timeout_ms;
};

struct robstride_bus_data
{
  const struct device * dev;
  const struct device * motors[CONFIG_MOTOR_ROBSTRIDE_MAX_MOTORS];
  struct k_spinlock lock;
  struct k_work tx_work;
  struct k_timer tx_timer;
  /* Cleared for a bus that has not been started yet; see the retry below. */
  bool started[ROBSTRIDE_MAX_CANS];
  int64_t last_start_retry;
};

struct robstride_motor_config
{
  const struct device * bus;
  uint8_t motor_id;
  enum robstride_model model;
  struct robstride_limits limits;
};

struct robstride_motor_data
{
  struct k_spinlock lock;

  /* What the application asked for. */
  bool enabled;
  enum robstride_mode mode;
  float target;
  struct robstride_motion_target motion;
  struct robstride_limits limits;
  struct robstride_gains gains;

  /*
   * Handshake progress. Every flag is cleared whenever the motor has to be
   * configured again, which is on enable, on a mode change and after a fault,
   * because in all three cases the motor may have dropped what we wrote.
   */
  bool configured;
  bool limits_applied;
  bool gains_applied;
  bool enable_sent;

  /* What the motor reported. */
  bool has_last_position;
  uint16_t last_position_raw;
  int64_t position_counts;
  float velocity;
  float torque;
  float temperature;
  enum robstride_run_state run_state;
  uint8_t error_code;
  uint32_t fault_bits;
  uint32_t warning_bits;
  bool online;
  int64_t timestamp_ms;
  uint8_t can_bus;
  int64_t last_probe_ms;

  /* In-flight parameter read. */
  struct k_sem param_sem;
  uint16_t param_index;
  uint32_t param_raw;
  bool param_pending;
};

static float robstride_u16_to_float(uint16_t raw, float min, float max)
{
  return ((float)raw * (max - min) / (float)UINT16_MAX) + min;
}

static uint16_t robstride_float_to_u16(float value, float min, float max)
{
  const float clamped = CLAMP(value, min, max);

  return (uint16_t)((clamped - min) * (float)UINT16_MAX / (max - min));
}

/* These three indices carry an integer payload rather than a float. */
static bool robstride_param_is_int(uint16_t index)
{
  return (index == ROBSTRIDE_PARAM_RUN_MODE) || (index == ROBSTRIDE_PARAM_EP_SCAN_TIME) ||
         (index == ROBSTRIDE_PARAM_ZERO_STA);
}

static void robstride_frame_init(
  struct can_frame * frame, uint8_t type, uint16_t aux, uint8_t target)
{
  memset(frame, 0, sizeof(*frame));
  frame->id = ((uint32_t)type << 24) | ((uint32_t)aux << 8) | (uint32_t)target;
  frame->dlc = can_bytes_to_dlc(8U);
  frame->flags = CAN_FRAME_IDE;
}

static void robstride_build_set_param(
  struct can_frame * frame, uint8_t master, uint8_t motor_id, uint16_t index, float value)
{
  uint32_t bits;

  robstride_frame_init(frame, ROBSTRIDE_TYPE_SET_PARAM, master, motor_id);
  sys_put_le16(index, &frame->data[0]);

  if (robstride_param_is_int(index)) {
    bits = (uint32_t)value;
  } else {
    memcpy(&bits, &value, sizeof(bits));
  }

  sys_put_le32(bits, &frame->data[4]);
}

static void robstride_build_get_param(
  struct can_frame * frame, uint8_t master, uint8_t motor_id, uint16_t index)
{
  robstride_frame_init(frame, ROBSTRIDE_TYPE_GET_PARAM, master, motor_id);
  sys_put_le16(index, &frame->data[0]);
}

static void robstride_build_operation_control(
  struct can_frame * frame, uint8_t motor_id, const struct robstride_model_limits * limits,
  const struct robstride_motion_target * target)
{
  /*
   * Unlike every other type, the operation control frame spends the 16 bits
   * that normally hold the host identifier on the feed-forward torque.
   */
  const uint16_t torque_raw =
    robstride_float_to_u16(target->torque, -limits->torque, limits->torque);

  robstride_frame_init(frame, ROBSTRIDE_TYPE_OP_CONTROL, torque_raw, motor_id);
  sys_put_be16(
    robstride_float_to_u16(target->position, ROBSTRIDE_POSITION_MIN, ROBSTRIDE_POSITION_MAX),
    &frame->data[0]);
  sys_put_be16(
    robstride_float_to_u16(target->velocity, -limits->velocity, limits->velocity),
    &frame->data[2]);
  sys_put_be16(robstride_float_to_u16(target->kp, 0.0F, ROBSTRIDE_OP_KP_MAX), &frame->data[4]);
  sys_put_be16(robstride_float_to_u16(target->kd, 0.0F, ROBSTRIDE_OP_KD_MAX), &frame->data[6]);
}

static uint32_t robstride_run_mode(enum robstride_mode mode)
{
  switch (mode) {
    case ROBSTRIDE_MODE_POSITION:
      return ROBSTRIDE_RUN_MODE_POSITION_PP;
    case ROBSTRIDE_MODE_POSITION_CSP:
      return ROBSTRIDE_RUN_MODE_POSITION_CSP;
    case ROBSTRIDE_MODE_VELOCITY:
      return ROBSTRIDE_RUN_MODE_VELOCITY;
    case ROBSTRIDE_MODE_CURRENT:
      return ROBSTRIDE_RUN_MODE_CURRENT;
    case ROBSTRIDE_MODE_OPERATION:
    default:
      return ROBSTRIDE_RUN_MODE_OPERATION;
  }
}

static int robstride_find_can_bus(
  const struct robstride_bus_config * config, const struct device * can_dev)
{
  for (size_t i = 0; i < config->can_count; ++i) {
    if (config->can_devs[i] == can_dev) {
      return (int)i;
    }
  }

  return -ENODEV;
}

static const struct device * robstride_find_motor(
  const struct robstride_bus_data * bus, uint8_t motor_id)
{
  for (size_t i = 0; i < ARRAY_SIZE(bus->motors); ++i) {
    const struct device * motor_dev = bus->motors[i];

    if (motor_dev == NULL) {
      continue;
    }

    const struct robstride_motor_config * config = motor_dev->config;

    if (config->motor_id == motor_id) {
      return motor_dev;
    }
  }

  return NULL;
}

/*
 * Send to the bus the motor last answered on. Until it has answered we do not
 * know which one that is, so the frame goes to every started bus.
 */
static int robstride_motor_send(const struct device * motor_dev, const struct can_frame * frame)
{
  const struct robstride_motor_config * config = motor_dev->config;
  const struct robstride_bus_config * bus_config = config->bus->config;
  struct robstride_bus_data * bus = config->bus->data;
  struct robstride_motor_data * data = motor_dev->data;
  k_spinlock_key_t key;
  uint8_t can_bus;
  int ret = -EIO;

  key = k_spin_lock(&data->lock);
  can_bus = data->can_bus;
  k_spin_unlock(&data->lock, key);

  for (size_t i = 0; i < bus_config->can_count; ++i) {
    if (!bus->started[i]) {
      continue;
    }

    if ((can_bus != ROBSTRIDE_CANBUS_UNKNOWN) && (can_bus != i)) {
      continue;
    }

    const int sent = can_send(bus_config->can_devs[i], frame, K_NO_WAIT, NULL, NULL);

    if (sent == 0) {
      ret = 0;
    }
  }

  return ret;
}

/*
 * Bring up any bus that would not start earlier. Called from the transmit work
 * but throttled to ROBSTRIDE_START_RETRY_MS, because can_start() on a
 * controller stuck in initialisation mode costs a hardware timeout.
 */
static void robstride_bus_retry_start(const struct device * dev)
{
  const struct robstride_bus_config * config = dev->config;
  struct robstride_bus_data * data = dev->data;
  const int64_t now = k_uptime_get();

  if ((now - data->last_start_retry) < ROBSTRIDE_START_RETRY_MS) {
    return;
  }

  data->last_start_retry = now;

  for (size_t i = 0; i < config->can_count; ++i) {
    if (data->started[i]) {
      continue;
    }

    const int ret = can_start(config->can_devs[i]);

    if ((ret == 0) || (ret == -EALREADY)) {
      data->started[i] = true;
      LOG_INF("CAN bus %u started", (unsigned int)i);
    }
  }
}

/*
 * Produce the frames this motor owes the bus in this interval. One handshake
 * stage per interval keeps the burst short; the motor is ready a few intervals
 * after it is enabled.
 */
static size_t robstride_build_tx(
  const struct device * motor_dev, struct can_frame * frames, int64_t now)
{
  const struct robstride_motor_config * config = motor_dev->config;
  const struct robstride_bus_config * bus_config = config->bus->config;
  struct robstride_motor_data * data = motor_dev->data;
  const uint8_t master = bus_config->master_can_id;
  const uint8_t motor_id = config->motor_id;
  size_t count = 0;

  if (!data->enabled) {
    if ((now - data->last_probe_ms) < ROBSTRIDE_PROBE_INTERVAL_MS) {
      return 0;
    }

    data->last_probe_ms = now;
    robstride_frame_init(&frames[count++], ROBSTRIDE_TYPE_GET_ID, master, motor_id);
    return count;
  }

  if (!data->configured) {
    /* The motor only accepts a new run mode while it is stopped. */
    robstride_frame_init(&frames[count++], ROBSTRIDE_TYPE_STOP, master, motor_id);
    robstride_build_set_param(
      &frames[count++], master, motor_id, ROBSTRIDE_PARAM_RUN_MODE,
      (float)robstride_run_mode(data->mode));
    data->configured = true;
    data->enable_sent = false;
    return count;
  }

  if (!data->limits_applied) {
    robstride_build_set_param(
      &frames[count++], master, motor_id, ROBSTRIDE_PARAM_LIMIT_CUR, data->limits.current);
    robstride_build_set_param(
      &frames[count++], master, motor_id, ROBSTRIDE_PARAM_LIMIT_SPD, data->limits.velocity);
    robstride_build_set_param(
      &frames[count++], master, motor_id, ROBSTRIDE_PARAM_LIMIT_TORQUE, data->limits.torque);
    data->limits_applied = true;
    return count;
  }

  if (!data->gains_applied) {
    robstride_build_set_param(
      &frames[count++], master, motor_id, ROBSTRIDE_PARAM_LOC_KP, data->gains.position_kp);
    robstride_build_set_param(
      &frames[count++], master, motor_id, ROBSTRIDE_PARAM_SPD_KP, data->gains.velocity_kp);
    robstride_build_set_param(
      &frames[count++], master, motor_id, ROBSTRIDE_PARAM_SPD_KI, data->gains.velocity_ki);
    robstride_build_set_param(
      &frames[count++], master, motor_id, ROBSTRIDE_PARAM_CUR_KP, data->gains.current_kp);
    robstride_build_set_param(
      &frames[count++], master, motor_id, ROBSTRIDE_PARAM_CUR_KI, data->gains.current_ki);
    data->gains_applied = true;
    return count;
  }

  if (!data->enable_sent) {
    robstride_frame_init(&frames[count++], ROBSTRIDE_TYPE_ENABLE, master, motor_id);
    data->enable_sent = true;
    return count;
  }

  switch (data->mode) {
    case ROBSTRIDE_MODE_OPERATION:
      robstride_build_operation_control(
        &frames[count++], motor_id, &robstride_model_limits[config->model], &data->motion);
      break;
    case ROBSTRIDE_MODE_POSITION:
    case ROBSTRIDE_MODE_POSITION_CSP:
      robstride_build_set_param(
        &frames[count++], master, motor_id, ROBSTRIDE_PARAM_LOC_REF, data->target);
      break;
    case ROBSTRIDE_MODE_VELOCITY:
      robstride_build_set_param(
        &frames[count++], master, motor_id, ROBSTRIDE_PARAM_SPD_REF, data->target);
      break;
    case ROBSTRIDE_MODE_CURRENT:
      robstride_build_set_param(
        &frames[count++], master, motor_id, ROBSTRIDE_PARAM_IQ_REF, data->target);
      break;
    default:
      break;
  }

  return count;
}

static void robstride_bus_tx_work_handler(struct k_work * work)
{
  struct robstride_bus_data * bus = CONTAINER_OF(work, struct robstride_bus_data, tx_work);
  const struct device * bus_dev = bus->dev;
  const int64_t now = k_uptime_get();

  robstride_bus_retry_start(bus_dev);

  for (size_t i = 0; i < ARRAY_SIZE(bus->motors); ++i) {
    const struct device * motor_dev = bus->motors[i];

    if (motor_dev == NULL) {
      continue;
    }

    struct robstride_motor_data * data = motor_dev->data;
    struct can_frame frames[ROBSTRIDE_TX_BURST];
    k_spinlock_key_t key;
    size_t count;

    key = k_spin_lock(&data->lock);
    count = robstride_build_tx(motor_dev, frames, now);
    k_spin_unlock(&data->lock, key);

    for (size_t f = 0; f < count; ++f) {
      /* A frame lost to a busy bus is re-sent on the next interval. */
      (void)robstride_motor_send(motor_dev, &frames[f]);
    }
  }
}

static void robstride_bus_tx_timer_handler(struct k_timer * timer)
{
  struct robstride_bus_data * bus = k_timer_user_data_get(timer);

  k_work_submit(&bus->tx_work);
}

static void robstride_mark_seen(struct robstride_motor_data * data, uint8_t can_bus, int64_t now)
{
  data->can_bus = can_bus;
  data->online = true;
  data->timestamp_ms = now;
}

static void robstride_handle_feedback(
  const struct device * motor_dev, uint32_t id, const struct can_frame * frame, uint8_t can_bus)
{
  const struct robstride_motor_config * config = motor_dev->config;
  const struct robstride_model_limits * limits = &robstride_model_limits[config->model];
  struct robstride_motor_data * data = motor_dev->data;
  const uint16_t position_raw = sys_get_be16(&frame->data[0]);
  const uint16_t velocity_raw = sys_get_be16(&frame->data[2]);
  const uint16_t torque_raw = sys_get_be16(&frame->data[4]);
  const int16_t temperature_raw = (int16_t)sys_get_be16(&frame->data[6]);
  k_spinlock_key_t key;

  key = k_spin_lock(&data->lock);

  if (data->has_last_position) {
    int32_t delta = (int32_t)position_raw - (int32_t)data->last_position_raw;

    if (delta > ROBSTRIDE_POSITION_HALF_WRAP) {
      delta -= ROBSTRIDE_POSITION_WRAP;
    } else if (delta < -ROBSTRIDE_POSITION_HALF_WRAP) {
      delta += ROBSTRIDE_POSITION_WRAP;
    }

    data->position_counts += delta;
  } else {
    /* Seed from the absolute reading, so the position at power-on survives. */
    data->position_counts = (int64_t)position_raw;
    data->has_last_position = true;
  }

  data->last_position_raw = position_raw;
  data->velocity = robstride_u16_to_float(velocity_raw, -limits->velocity, limits->velocity);
  data->torque = robstride_u16_to_float(torque_raw, -limits->torque, limits->torque);
  data->temperature = (float)temperature_raw * 0.1F;
  const enum robstride_run_state run_state = (enum robstride_run_state)((id >> 22) & 0x03U);

  /*
   * Only the transition out of running counts. The motor dropped the enable we
   * sent, so the handshake has to run again. A motor that is merely still
   * calibrating has lost nothing, and re-sending enable every interval would
   * add traffic without changing anything.
   */
  if ((data->run_state == ROBSTRIDE_RUN_STATE_RUNNING) &&
      (run_state != ROBSTRIDE_RUN_STATE_RUNNING)) {
    data->enable_sent = false;
  }

  data->run_state = run_state;
  data->error_code = (uint8_t)((id >> 16) & 0x3FU);

  robstride_mark_seen(data, can_bus, k_uptime_get());
  k_spin_unlock(&data->lock, key);
}

static void robstride_handle_fault(
  const struct device * motor_dev, const struct can_frame * frame, uint8_t can_bus)
{
  struct robstride_motor_data * data = motor_dev->data;
  const uint32_t fault_bits = sys_get_le32(&frame->data[0]);
  const uint32_t warning_bits = sys_get_le32(&frame->data[4]);
  k_spinlock_key_t key;

  key = k_spin_lock(&data->lock);
  data->fault_bits = fault_bits;
  data->warning_bits = warning_bits;

  if (fault_bits != 0U) {
    /* A tripped motor disables itself and forgets what we configured. */
    data->configured = false;
    data->limits_applied = false;
    data->enable_sent = false;
  }

  robstride_mark_seen(data, can_bus, k_uptime_get());
  k_spin_unlock(&data->lock, key);
}

static void robstride_handle_param(
  const struct device * motor_dev, const struct can_frame * frame, uint8_t can_bus)
{
  struct robstride_motor_data * data = motor_dev->data;
  const uint16_t index = sys_get_le16(&frame->data[0]);
  const uint32_t raw = sys_get_le32(&frame->data[4]);
  bool complete = false;
  k_spinlock_key_t key;

  key = k_spin_lock(&data->lock);

  if (data->param_pending && (data->param_index == index)) {
    data->param_raw = raw;
    data->param_pending = false;
    complete = true;
  }

  robstride_mark_seen(data, can_bus, k_uptime_get());
  k_spin_unlock(&data->lock, key);

  if (complete) {
    k_sem_give(&data->param_sem);
  }
}

static void robstride_rx_callback(
  const struct device * can_dev, struct can_frame * frame, void * user_data)
{
  const struct device * bus_dev = user_data;
  const struct robstride_bus_config * config = bus_dev->config;
  struct robstride_bus_data * bus = bus_dev->data;

  if ((frame->flags & CAN_FRAME_IDE) == 0U) {
    return;
  }

  if (can_dlc_to_bytes(frame->dlc) < 8U) {
    return;
  }

  const uint32_t id = frame->id & CAN_EXT_ID_MASK;

  if ((uint8_t)(id & 0xFFU) != config->master_can_id) {
    return;
  }

  const struct device * motor_dev = robstride_find_motor(bus, (uint8_t)((id >> 8) & 0xFFU));

  if (motor_dev == NULL) {
    return;
  }

  const int can_bus = robstride_find_can_bus(config, can_dev);

  if (can_bus < 0) {
    return;
  }

  switch ((uint8_t)((id >> 24) & 0x1FU)) {
    case ROBSTRIDE_TYPE_FEEDBACK:
      robstride_handle_feedback(motor_dev, id, frame, (uint8_t)can_bus);
      break;
    case ROBSTRIDE_TYPE_FAULT:
      robstride_handle_fault(motor_dev, frame, (uint8_t)can_bus);
      break;
    case ROBSTRIDE_TYPE_GET_PARAM:
      robstride_handle_param(motor_dev, frame, (uint8_t)can_bus);
      break;
    case ROBSTRIDE_TYPE_GET_ID: {
      struct robstride_motor_data * data = motor_dev->data;
      k_spinlock_key_t key = k_spin_lock(&data->lock);

      robstride_mark_seen(data, (uint8_t)can_bus, k_uptime_get());
      k_spin_unlock(&data->lock, key);
      break;
    }
    default:
      break;
  }
}

static int robstride_bus_init(const struct device * dev)
{
  const struct robstride_bus_config * config = dev->config;
  struct robstride_bus_data * data = dev->data;
  const struct can_filter filter = {
    .id = config->master_can_id,
    /* Every motor-to-host type puts our identifier in the low byte. */
    .mask = 0xFFU,
    .flags = CAN_FILTER_IDE,
  };

  if (config->can_count > ROBSTRIDE_MAX_CANS) {
    LOG_ERR("Too many CAN devices (%u)", (unsigned int)config->can_count);
    return -EINVAL;
  }

  data->dev = dev;
  k_work_init(&data->tx_work, robstride_bus_tx_work_handler);
  k_timer_init(&data->tx_timer, robstride_bus_tx_timer_handler, NULL);
  k_timer_user_data_set(&data->tx_timer, data);

  for (size_t i = 0; i < config->can_count; ++i) {
    if (!device_is_ready(config->can_devs[i])) {
      LOG_ERR("CAN bus %u not ready", (unsigned int)i);
      return -ENODEV;
    }

    const int filter_id =
      can_add_rx_filter(config->can_devs[i], robstride_rx_callback, (void *)dev, &filter);

    if (filter_id < 0) {
      LOG_ERR("Failed to add RX filter on CAN bus %u (%d)", (unsigned int)i, filter_id);
      return filter_id;
    }

    /*
     * A bus whose transceiver has no power holds RX dominant, and a controller
     * then never leaves initialisation mode. That is the normal state of a
     * board powered from the debug probe alone, so it must not be fatal. Keep
     * the other buses and the motor devices usable and retry the start from
     * the transmit work, so the bus comes up whenever the motor supply does.
     */
    const int ret = can_start(config->can_devs[i]);

    if ((ret < 0) && (ret != -EALREADY)) {
      LOG_WRN("CAN bus %u not startable yet (%d), retrying", (unsigned int)i, ret);
      continue;
    }

    data->started[i] = true;
  }

  data->last_start_retry = k_uptime_get();
  k_timer_start(
    &data->tx_timer, K_MSEC(CONFIG_MOTOR_ROBSTRIDE_TX_INTERVAL_MS),
    K_MSEC(CONFIG_MOTOR_ROBSTRIDE_TX_INTERVAL_MS));

  return 0;
}

static int robstride_bus_register_motor(
  const struct device * bus_dev, const struct device * motor_dev, uint8_t motor_id)
{
  const struct robstride_bus_config * config = bus_dev->config;
  struct robstride_bus_data * bus = bus_dev->data;
  k_spinlock_key_t key;
  int ret = -ENOSPC;

  if ((motor_id < 1U) || (motor_id > 127U)) {
    return -EINVAL;
  }

  if (motor_id == config->master_can_id) {
    LOG_ERR("Motor ID %u collides with the master CAN ID", (unsigned int)motor_id);
    return -EINVAL;
  }

  key = k_spin_lock(&bus->lock);

  if (robstride_find_motor(bus, motor_id) != NULL) {
    k_spin_unlock(&bus->lock, key);
    return -EALREADY;
  }

  for (size_t i = 0; i < ARRAY_SIZE(bus->motors); ++i) {
    if (bus->motors[i] == NULL) {
      bus->motors[i] = motor_dev;
      ret = 0;
      break;
    }
  }

  k_spin_unlock(&bus->lock, key);

  return ret;
}

static int robstride_motor_enable(const struct device * dev)
{
  struct robstride_motor_data * data = dev->data;
  k_spinlock_key_t key;

  key = k_spin_lock(&data->lock);

  if (!data->enabled) {
    data->enabled = true;
    /*
     * The motor may have been power-cycled or have tripped while we were not
     * talking to it, so start the handshake from the beginning rather than
     * assuming it still holds what we wrote.
     */
    data->configured = false;
    data->limits_applied = false;
    data->enable_sent = false;
  }

  k_spin_unlock(&data->lock, key);

  return 0;
}

static int robstride_motor_disable(const struct device * dev)
{
  const struct robstride_motor_config * config = dev->config;
  const struct robstride_bus_config * bus_config = config->bus->config;
  struct robstride_motor_data * data = dev->data;
  struct can_frame frame;
  k_spinlock_key_t key;

  key = k_spin_lock(&data->lock);
  data->enabled = false;
  data->enable_sent = false;
  k_spin_unlock(&data->lock, key);

  /* Stopping is the safety-critical direction, so it does not wait for the
   * next transmit interval. */
  robstride_frame_init(&frame, ROBSTRIDE_TYPE_STOP, bus_config->master_can_id, config->motor_id);

  /* The transmit work stops commanding this motor either way, so a bus that
   * cannot take the frame right now does not make disabling fail. */
  (void)robstride_motor_send(dev, &frame);

  return 0;
}

static int robstride_motor_set_output(
  const struct device * dev, const enum motor_output_mode mode, const int16_t output)
{
  ARG_UNUSED(dev);
  ARG_UNUSED(mode);
  ARG_UNUSED(output);

  /*
   * A RobStride target is an SI quantity whose range depends on the model, so
   * there is no unit this class-wide call could carry without changing meaning
   * between drivers. Use include/drivers/motor/robstride.h instead.
   */
  return -ENOTSUP;
}

static int robstride_motor_get_feedback(const struct device * dev, void * feedback)
{
  const struct robstride_motor_config * config = dev->config;
  const struct robstride_bus_config * bus_config = config->bus->config;
  struct robstride_motor_data * data = dev->data;
  struct motor_feedback * out = feedback;
  k_spinlock_key_t key;
  int ret = 0;

  if (out == NULL) {
    return -EINVAL;
  }

  memset(out, 0, sizeof(*out));

  key = k_spin_lock(&data->lock);

  /*
   * Only the two fields that mean the same thing here as in the other motor
   * drivers are published. The rest are SI values with no unit agreed across
   * the class; robstride_get_feedback() reports them.
   */
  out->valid_mask = MOTOR_FEEDBACK_POSITION | MOTOR_FEEDBACK_TEMPERATURE;
  out->position = data->position_counts;
  out->temperature = (int16_t)data->temperature;
  out->online = data->online;
  out->timestamp_ms = data->timestamp_ms;

  if (!data->online) {
    ret = -ENODATA;
  } else if (
    (bus_config->feedback_timeout_ms > 0U) &&
    ((k_uptime_get() - data->timestamp_ms) > (int64_t)bus_config->feedback_timeout_ms)) {
    out->stale = true;
    ret = -EAGAIN;
  }

  k_spin_unlock(&data->lock, key);

  return ret;
}

static int robstride_motor_init(const struct device * dev)
{
  const struct robstride_motor_config * config = dev->config;
  struct robstride_motor_data * data = dev->data;

  if (!device_is_ready(config->bus)) {
    LOG_ERR("Bus not ready for motor %u", (unsigned int)config->motor_id);
    return -ENODEV;
  }

  data->can_bus = ROBSTRIDE_CANBUS_UNKNOWN;
  data->mode = ROBSTRIDE_MODE_CURRENT;
  data->limits = config->limits;
  /* The motor keeps the gains in its own memory unless the application
   * overrides them, so nothing is written until robstride_set_gains(). */
  data->gains_applied = true;
  k_sem_init(&data->param_sem, 0, 1);

  return robstride_bus_register_motor(config->bus, dev, config->motor_id);
}

static const struct motor_driver_api robstride_motor_api = {
  .enable = robstride_motor_enable,
  .disable = robstride_motor_disable,
  .set_output = robstride_motor_set_output,
  .get_feedback = robstride_motor_get_feedback,
};

static bool robstride_is_motor(const struct device * dev)
{
  return (dev != NULL) && (dev->api == &robstride_motor_api);
}

static int robstride_set_target(const struct device * dev, enum robstride_mode mode, float value)
{
  const struct robstride_motor_config * config;
  struct robstride_motor_data * data;
  k_spinlock_key_t key;

  if (!robstride_is_motor(dev)) {
    return -EINVAL;
  }

  config = dev->config;
  data = dev->data;

  key = k_spin_lock(&data->lock);

  if (data->mode != mode) {
    data->mode = mode;
    /* A run mode is only accepted while the motor is stopped, so the
     * handshake has to run again. */
    data->configured = false;
    data->enable_sent = false;
  }

  switch (mode) {
    case ROBSTRIDE_MODE_POSITION:
    case ROBSTRIDE_MODE_POSITION_CSP:
      data->target = CLAMP(value, ROBSTRIDE_POSITION_MIN, ROBSTRIDE_POSITION_MAX);
      break;
    case ROBSTRIDE_MODE_VELOCITY: {
      const float limit = robstride_model_limits[config->model].velocity;

      data->target = CLAMP(value, -limit, limit);
      break;
    }
    case ROBSTRIDE_MODE_CURRENT:
    default: {
      const float limit = robstride_model_limits[config->model].current;

      data->target = CLAMP(value, -limit, limit);
      break;
    }
  }

  k_spin_unlock(&data->lock, key);

  return 0;
}

int robstride_set_position(const struct device * dev, float position)
{
  return robstride_set_target(dev, ROBSTRIDE_MODE_POSITION, position);
}

int robstride_set_position_csp(const struct device * dev, float position)
{
  return robstride_set_target(dev, ROBSTRIDE_MODE_POSITION_CSP, position);
}

int robstride_set_velocity(const struct device * dev, float velocity)
{
  return robstride_set_target(dev, ROBSTRIDE_MODE_VELOCITY, velocity);
}

int robstride_set_current(const struct device * dev, float current)
{
  return robstride_set_target(dev, ROBSTRIDE_MODE_CURRENT, current);
}

int robstride_set_motion_target(
  const struct device * dev, const struct robstride_motion_target * target)
{
  struct robstride_motor_data * data;
  k_spinlock_key_t key;

  if (!robstride_is_motor(dev) || (target == NULL)) {
    return -EINVAL;
  }

  data = dev->data;

  key = k_spin_lock(&data->lock);

  if (data->mode != ROBSTRIDE_MODE_OPERATION) {
    data->mode = ROBSTRIDE_MODE_OPERATION;
    data->configured = false;
    data->enable_sent = false;
  }

  /* Every field is clamped again when the frame is encoded, against the
   * ranges the wire format fixes. */
  data->motion = *target;
  k_spin_unlock(&data->lock, key);

  return 0;
}

int robstride_get_feedback(const struct device * dev, struct robstride_feedback * feedback)
{
  const struct robstride_motor_config * config;
  const struct robstride_bus_config * bus_config;
  struct robstride_motor_data * data;
  k_spinlock_key_t key;
  int ret = 0;

  if (!robstride_is_motor(dev) || (feedback == NULL)) {
    return -EINVAL;
  }

  config = dev->config;
  bus_config = config->bus->config;
  data = dev->data;

  memset(feedback, 0, sizeof(*feedback));

  key = k_spin_lock(&data->lock);
  feedback->position =
    ((float)data->position_counts *
     ((ROBSTRIDE_POSITION_MAX - ROBSTRIDE_POSITION_MIN) / (float)UINT16_MAX)) +
    ROBSTRIDE_POSITION_MIN;
  feedback->velocity = data->velocity;
  feedback->torque = data->torque;
  feedback->temperature = data->temperature;
  feedback->mode = data->mode;
  feedback->run_state = data->run_state;
  feedback->error_code = data->error_code;
  feedback->fault_bits = data->fault_bits;
  feedback->warning_bits = data->warning_bits;
  feedback->online = data->online;
  feedback->timestamp_ms = data->timestamp_ms;

  if (!data->online) {
    ret = -ENODATA;
  } else if (
    (bus_config->feedback_timeout_ms > 0U) &&
    ((k_uptime_get() - data->timestamp_ms) > (int64_t)bus_config->feedback_timeout_ms)) {
    feedback->stale = true;
    ret = -EAGAIN;
  }

  k_spin_unlock(&data->lock, key);

  return ret;
}

int robstride_set_limits(const struct device * dev, const struct robstride_limits * limits)
{
  const struct robstride_motor_config * config;
  struct robstride_motor_data * data;
  k_spinlock_key_t key;

  if (!robstride_is_motor(dev) || (limits == NULL)) {
    return -EINVAL;
  }

  config = dev->config;
  data = dev->data;

  const struct robstride_model_limits * model = &robstride_model_limits[config->model];

  key = k_spin_lock(&data->lock);
  data->limits.current = CLAMP(limits->current, 0.0F, model->current);
  data->limits.velocity = CLAMP(limits->velocity, 0.0F, model->velocity);
  data->limits.torque = CLAMP(limits->torque, 0.0F, model->torque);
  data->limits_applied = false;
  k_spin_unlock(&data->lock, key);

  return 0;
}

int robstride_set_gains(const struct device * dev, const struct robstride_gains * gains)
{
  struct robstride_motor_data * data;
  k_spinlock_key_t key;

  if (!robstride_is_motor(dev) || (gains == NULL)) {
    return -EINVAL;
  }

  data = dev->data;

  key = k_spin_lock(&data->lock);
  data->gains = *gains;
  data->gains_applied = false;
  k_spin_unlock(&data->lock, key);

  return 0;
}

static int robstride_send_simple(const struct device * dev, uint8_t type, uint8_t payload0)
{
  const struct robstride_motor_config * config;
  const struct robstride_bus_config * bus_config;
  struct can_frame frame;

  if (!robstride_is_motor(dev)) {
    return -EINVAL;
  }

  config = dev->config;
  bus_config = config->bus->config;

  robstride_frame_init(&frame, type, bus_config->master_can_id, config->motor_id);
  frame.data[0] = payload0;

  return robstride_motor_send(dev, &frame);
}

int robstride_set_zero(const struct device * dev)
{
  return robstride_send_simple(dev, ROBSTRIDE_TYPE_SET_ZERO, 0x01U);
}

int robstride_save_parameters(const struct device * dev)
{
  return robstride_send_simple(dev, ROBSTRIDE_TYPE_SAVE, 0x01U);
}

int robstride_set_parameter(const struct device * dev, uint16_t index, float value)
{
  const struct robstride_motor_config * config;
  const struct robstride_bus_config * bus_config;
  struct can_frame frame;

  if (!robstride_is_motor(dev)) {
    return -EINVAL;
  }

  config = dev->config;
  bus_config = config->bus->config;

  robstride_build_set_param(
    &frame, bus_config->master_can_id, config->motor_id, index, value);

  return robstride_motor_send(dev, &frame);
}

int robstride_get_parameter(const struct device * dev, uint16_t index, float * value)
{
  const struct robstride_motor_config * config;
  const struct robstride_bus_config * bus_config;
  struct robstride_motor_data * data;
  struct can_frame frame;
  k_spinlock_key_t key;
  uint32_t raw;
  int ret;

  if (!robstride_is_motor(dev) || (value == NULL)) {
    return -EINVAL;
  }

  config = dev->config;
  bus_config = config->bus->config;
  data = dev->data;

  key = k_spin_lock(&data->lock);

  if (data->param_pending) {
    k_spin_unlock(&data->lock, key);
    return -EBUSY;
  }

  data->param_index = index;
  data->param_pending = true;
  k_spin_unlock(&data->lock, key);

  k_sem_reset(&data->param_sem);
  robstride_build_get_param(&frame, bus_config->master_can_id, config->motor_id, index);
  ret = robstride_motor_send(dev, &frame);

  if (ret == 0) {
    ret = k_sem_take(&data->param_sem, K_MSEC(CONFIG_MOTOR_ROBSTRIDE_PARAM_TIMEOUT_MS));

    if (ret != 0) {
      ret = -ETIMEDOUT;
    }
  }

  key = k_spin_lock(&data->lock);
  raw = data->param_raw;
  data->param_pending = false;
  k_spin_unlock(&data->lock, key);

  if (ret != 0) {
    return ret;
  }

  if (robstride_param_is_int(index)) {
    *value = (float)raw;
  } else {
    memcpy(value, &raw, sizeof(*value));
  }

  return 0;
}

#define DT_DRV_COMPAT robstride_bus

#define ROBSTRIDE_CAN_DEV_ELEM(idx, inst) DEVICE_DT_GET(DT_INST_PHANDLE_BY_IDX(inst, cans, idx))

#define ROBSTRIDE_BUS_DEFINE(inst)                                                     \
  BUILD_ASSERT(                                                                        \
    DT_INST_PROP_LEN(inst, cans) <= ROBSTRIDE_MAX_CANS,                                \
    "Too many CAN devices configured for RobStride bus");                              \
  static const struct device * const robstride_can_devs_##inst[] = {                   \
    LISTIFY(DT_INST_PROP_LEN(inst, cans), ROBSTRIDE_CAN_DEV_ELEM, (, ), inst)};        \
  static struct robstride_bus_data robstride_bus_data_##inst;                          \
  static const struct robstride_bus_config robstride_bus_config_##inst = {             \
    .can_devs = robstride_can_devs_##inst,                                             \
    .can_count = DT_INST_PROP_LEN(inst, cans),                                         \
    .master_can_id = DT_INST_PROP(inst, master_can_id),                                \
    .feedback_timeout_ms = DT_INST_PROP(inst, feedback_timeout_ms),                    \
  };                                                                                   \
  DEVICE_DT_INST_DEFINE(                                                               \
    inst, robstride_bus_init, NULL, &robstride_bus_data_##inst,                        \
    &robstride_bus_config_##inst, POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(ROBSTRIDE_BUS_DEFINE)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT robstride_motor

/* A limit left out of the devicetree falls back to the model's own limit. */
#define ROBSTRIDE_LIMIT_INIT(inst, prop, field)                                              \
  COND_CODE_1(                                                                               \
    DT_INST_NODE_HAS_PROP(inst, prop), ((float)DT_INST_PROP(inst, prop) / 1000.0F),           \
    (robstride_model_limits[DT_INST_ENUM_IDX(inst, model)].field))

#define ROBSTRIDE_MOTOR_DEFINE(inst)                                                        \
  BUILD_ASSERT(                                                                             \
    DT_INST_ENUM_IDX(inst, model) < ARRAY_SIZE(robstride_model_limits),                     \
    "Unknown RobStride model");                                                             \
  static struct robstride_motor_data robstride_motor_data_##inst;                           \
  static const struct robstride_motor_config robstride_motor_config_##inst = {              \
    .bus = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                             \
    .motor_id = DT_INST_REG_ADDR(inst),                                                     \
    .model = (enum robstride_model)DT_INST_ENUM_IDX(inst, model),                           \
    .limits =                                                                               \
      {                                                                                     \
        .current = ROBSTRIDE_LIMIT_INIT(inst, max_current_ma, current),                     \
        .velocity = ROBSTRIDE_LIMIT_INIT(inst, max_velocity_mrad_s, velocity),              \
        .torque = ROBSTRIDE_LIMIT_INIT(inst, max_torque_mnm, torque),                       \
      },                                                                                    \
  };                                                                                        \
  DEVICE_DT_INST_DEFINE(                                                                    \
    inst, robstride_motor_init, NULL, &robstride_motor_data_##inst,                         \
    &robstride_motor_config_##inst, POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY,                \
    &robstride_motor_api);

DT_INST_FOREACH_STATUS_OKAY(ROBSTRIDE_MOTOR_DEFINE)
