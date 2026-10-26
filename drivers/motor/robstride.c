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
 * also how many parameters one batch writes or reads back. */
#define ROBSTRIDE_TX_BURST 5

/* Read only with readback on, where Kconfig defines them. */
#define ROBSTRIDE_VERIFY_TIMEOUT_MS                                                         \
  COND_CODE_1(CONFIG_MOTOR_ROBSTRIDE_VERIFY, (CONFIG_MOTOR_ROBSTRIDE_VERIFY_TIMEOUT_MS), (0))
#define ROBSTRIDE_VERIFY_RETRIES                                                            \
  COND_CODE_1(CONFIG_MOTOR_ROBSTRIDE_VERIFY, (CONFIG_MOTOR_ROBSTRIDE_VERIFY_RETRIES), (1))

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
#define ROBSTRIDE_PARAM_ACC_RAD 0x7022U
#define ROBSTRIDE_PARAM_VEL_MAX 0x7024U
#define ROBSTRIDE_PARAM_ACC_SET 0x7025U
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

/*
 * The stage a single interval's burst belongs to. Every stage but the command
 * marks itself done as it is built, so the work has to be able to take that
 * back when the frames do not reach the controller.
 */
enum robstride_tx_stage {
  ROBSTRIDE_TX_STAGE_NONE,
  ROBSTRIDE_TX_STAGE_STOP,
  ROBSTRIDE_TX_STAGE_PROBE,
  ROBSTRIDE_TX_STAGE_PARAM_WRITE,
  ROBSTRIDE_TX_STAGE_PARAM_VERIFY,
  ROBSTRIDE_TX_STAGE_REF,
  ROBSTRIDE_TX_STAGE_ENABLE,
  ROBSTRIDE_TX_STAGE_COMMAND,
};

#define ROBSTRIDE_TX_STAGE_COUNT (ROBSTRIDE_TX_STAGE_COMMAND + 1)

/*
 * Every parameter the handshake writes, in the order it writes them. The run
 * mode comes first because the motor takes it only while stopped, and it goes
 * out alone behind a stop frame.
 */
enum robstride_param_slot {
  ROBSTRIDE_SLOT_RUN_MODE,
  ROBSTRIDE_SLOT_LIMIT_CUR,
  ROBSTRIDE_SLOT_LIMIT_SPD,
  ROBSTRIDE_SLOT_LIMIT_TORQUE,
  ROBSTRIDE_SLOT_VEL_MAX,
  ROBSTRIDE_SLOT_ACC_SET,
  ROBSTRIDE_SLOT_ACC_RAD,
  ROBSTRIDE_SLOT_LOC_KP,
  ROBSTRIDE_SLOT_SPD_KP,
  ROBSTRIDE_SLOT_SPD_KI,
  ROBSTRIDE_SLOT_CUR_KP,
  ROBSTRIDE_SLOT_CUR_KI,
  ROBSTRIDE_SLOT_COUNT,
};

static const uint16_t robstride_slot_index[ROBSTRIDE_SLOT_COUNT] = {
  [ROBSTRIDE_SLOT_RUN_MODE] = ROBSTRIDE_PARAM_RUN_MODE,
  [ROBSTRIDE_SLOT_LIMIT_CUR] = ROBSTRIDE_PARAM_LIMIT_CUR,
  [ROBSTRIDE_SLOT_LIMIT_SPD] = ROBSTRIDE_PARAM_LIMIT_SPD,
  [ROBSTRIDE_SLOT_LIMIT_TORQUE] = ROBSTRIDE_PARAM_LIMIT_TORQUE,
  [ROBSTRIDE_SLOT_VEL_MAX] = ROBSTRIDE_PARAM_VEL_MAX,
  [ROBSTRIDE_SLOT_ACC_SET] = ROBSTRIDE_PARAM_ACC_SET,
  [ROBSTRIDE_SLOT_ACC_RAD] = ROBSTRIDE_PARAM_ACC_RAD,
  [ROBSTRIDE_SLOT_LOC_KP] = ROBSTRIDE_PARAM_LOC_KP,
  [ROBSTRIDE_SLOT_SPD_KP] = ROBSTRIDE_PARAM_SPD_KP,
  [ROBSTRIDE_SLOT_SPD_KI] = ROBSTRIDE_PARAM_SPD_KI,
  [ROBSTRIDE_SLOT_CUR_KP] = ROBSTRIDE_PARAM_CUR_KP,
  [ROBSTRIDE_SLOT_CUR_KI] = ROBSTRIDE_PARAM_CUR_KI,
};

/*
 * How far a value read back may sit from the one written and still count. The
 * same margins as robstride_actuator_bridge_ros2, tighter for the integral and
 * current-loop gains, whose factory values are around 0.01. The run mode is an
 * integer and compared exactly.
 */
static const float robstride_slot_tolerance[ROBSTRIDE_SLOT_COUNT] = {
  [ROBSTRIDE_SLOT_RUN_MODE] = 0.0F,
  [ROBSTRIDE_SLOT_LIMIT_CUR] = 1.0e-3F,
  [ROBSTRIDE_SLOT_LIMIT_SPD] = 1.0e-3F,
  [ROBSTRIDE_SLOT_LIMIT_TORQUE] = 1.0e-3F,
  [ROBSTRIDE_SLOT_VEL_MAX] = 1.0e-3F,
  [ROBSTRIDE_SLOT_ACC_SET] = 1.0e-3F,
  [ROBSTRIDE_SLOT_ACC_RAD] = 1.0e-3F,
  [ROBSTRIDE_SLOT_LOC_KP] = 1.0e-3F,
  [ROBSTRIDE_SLOT_SPD_KP] = 1.0e-3F,
  [ROBSTRIDE_SLOT_SPD_KI] = 1.0e-4F,
  [ROBSTRIDE_SLOT_CUR_KP] = 1.0e-4F,
  [ROBSTRIDE_SLOT_CUR_KI] = 1.0e-4F,
};

/* Written in every mode. CSP also overrides what goes into LIMIT_SPD. */
#define ROBSTRIDE_SLOTS_COMMON                                                    \
  (BIT(ROBSTRIDE_SLOT_RUN_MODE) | BIT(ROBSTRIDE_SLOT_LIMIT_CUR) |                 \
   BIT(ROBSTRIDE_SLOT_LIMIT_SPD) | BIT(ROBSTRIDE_SLOT_LIMIT_TORQUE))

#define ROBSTRIDE_SLOTS_LIMITS                                                    \
  (BIT(ROBSTRIDE_SLOT_LIMIT_CUR) | BIT(ROBSTRIDE_SLOT_LIMIT_SPD) |                \
   BIT(ROBSTRIDE_SLOT_LIMIT_TORQUE))

#define ROBSTRIDE_SLOTS_CURRENT_LOOP (BIT(ROBSTRIDE_SLOT_CUR_KP) | BIT(ROBSTRIDE_SLOT_CUR_KI))

#define ROBSTRIDE_SLOTS_SPEED_LOOP                                                \
  (BIT(ROBSTRIDE_SLOT_SPD_KP) | BIT(ROBSTRIDE_SLOT_SPD_KI) | ROBSTRIDE_SLOTS_CURRENT_LOOP)

/* What each mode writes from its robstride_mode_params, once it has some. */
static const uint16_t robstride_mode_slots[ROBSTRIDE_MODE_COUNT] = {
  [ROBSTRIDE_MODE_OPERATION] = ROBSTRIDE_SLOTS_CURRENT_LOOP,
  [ROBSTRIDE_MODE_POSITION] = BIT(ROBSTRIDE_SLOT_VEL_MAX) | BIT(ROBSTRIDE_SLOT_ACC_SET) |
                              BIT(ROBSTRIDE_SLOT_LOC_KP) | ROBSTRIDE_SLOTS_SPEED_LOOP,
  [ROBSTRIDE_MODE_POSITION_CSP] = BIT(ROBSTRIDE_SLOT_LIMIT_SPD) | BIT(ROBSTRIDE_SLOT_LOC_KP) |
                                  ROBSTRIDE_SLOTS_SPEED_LOOP,
  [ROBSTRIDE_MODE_VELOCITY] = BIT(ROBSTRIDE_SLOT_ACC_RAD) | ROBSTRIDE_SLOTS_SPEED_LOOP,
  [ROBSTRIDE_MODE_CURRENT] = ROBSTRIDE_SLOTS_CURRENT_LOOP,
};

struct robstride_motor_data;

/*
 * Handed to can_send() as the completion context. One per motor and stage,
 * filled once at init and never written again, because a completion may land
 * several intervals after the frame was queued and has to name the stage it
 * actually belonged to.
 */
struct robstride_tx_ctx
{
  struct robstride_motor_data * data;
  enum robstride_tx_stage stage;
  /* Which parameter a PARAM_WRITE frame carried. Unused by the other stages. */
  enum robstride_param_slot slot;
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
  struct robstride_mode_params mode_params[ROBSTRIDE_MODE_COUNT];
  /* Modes robstride_set_mode_params() has supplied values for. A mode outside
   * this runs on what the motor holds in its own memory, and we write none. */
  uint8_t mode_params_set;

  /*
   * Handshake progress, reset whenever the motor has to be configured again:
   * on enable, on a mode change, after a fault and after a failed attempt,
   * because in each case the motor may have dropped what we wrote.
   *
   * A parameter slot is in at most one of the two masks. param_to_write holds
   * the ones still to be sent, param_to_verify the ones sent and not yet read
   * back with the value we wrote. A slot in neither is in the motor.
   */
  uint16_t param_to_write;
  uint16_t param_to_verify;
  /* The reads for param_to_verify have gone out, and verify_deadline_ms is
   * when to stop waiting for their replies. */
  bool verify_sent;
  int64_t verify_deadline_ms;
  /* Writes of the batch being verified, against
   * CONFIG_MOTOR_ROBSTRIDE_VERIFY_RETRIES. */
  uint8_t verify_attempts;
  bool ref_sent;
  bool enable_sent;
  /* A feedback frame has reported running, with no error, since the enable.
   * Commands are held back until then. */
  bool running_confirmed;
  int64_t running_deadline_ms;
  /* Nothing is sent to configure the motor before this, after a failure. */
  int64_t configure_after_ms;
  uint32_t configure_failures;
  /* Set from a failure until the motor next runs, so a motor that keeps
   * failing, such as one without power, is reported once rather than on
   * every attempt. */
  bool configure_failing;
  /* A stop frame the immediate send could not queue, re-sent by the work. */
  bool stop_pending;
  /*
   * Queueing a frame and getting it onto the wire are separate events, and
   * frames from more than one stage can be outstanding at once, so a failed
   * completion names its own stage here rather than whichever one was built
   * most recently. Set from the completion callback, drained by the work.
   */
  uint32_t tx_failed_stages;
  /* The same for parameter writes, one bit per slot, so a late failure undoes
   * the parameter it carried rather than whichever batch is current. */
  uint16_t param_tx_failed;
  struct robstride_tx_ctx tx_ctx[ROBSTRIDE_TX_STAGE_COUNT];
  struct robstride_tx_ctx param_tx_ctx[ROBSTRIDE_SLOT_COUNT];

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
  /*
   * There is one slot, and these three say which part of it is taken.
   * param_pending owns the slot from the claim until the caller that made it
   * releases it, and is what -EBUSY reports; releasing it in the receive path
   * instead would let a second caller in before the first read its result.
   * param_armed says replies may now be matched, and stays clear until the
   * semaphore has been drained, so a reply to a read that already gave up
   * cannot be counted for the one being set up. param_complete says the reply
   * landed, which the caller trusts over the semaphore in the timeout race.
   */
  bool param_pending;
  bool param_armed;
  bool param_complete;
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

/* One past the largest value the uint32 payload of an integer parameter can
 * hold; the conversion is undefined from here up. */
#define ROBSTRIDE_PARAM_INT_LIMIT 4294967296.0F

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

/*
 * Build a parameter write, reporting whether the value could be carried.
 * Only an integer-payload index can fail, and only from
 * robstride_set_parameter(): the transmit path writes either a float payload
 * or the run mode, which robstride_run_mode() keeps in range. The frame is
 * always left well-formed so a caller that ignores the result cannot send
 * uninitialised memory.
 */
static int robstride_build_set_param(
  struct can_frame * frame, uint8_t master, uint8_t motor_id, uint16_t index, float value)
{
  uint32_t bits = 0;
  int ret = 0;

  if (robstride_param_is_int(index)) {
    /*
     * Converting a float that is negative, too large, or not a number to
     * uint32_t is undefined, and this value comes from the caller. NaN fails
     * both comparisons, so the range test rejects it too.
     */
    if ((value >= 0.0F) && (value < ROBSTRIDE_PARAM_INT_LIMIT)) {
      bits = (uint32_t)value;
    } else {
      ret = -EINVAL;
    }
  } else {
    memcpy(&bits, &value, sizeof(bits));
  }

  robstride_frame_init(frame, ROBSTRIDE_TYPE_SET_PARAM, master, motor_id);
  sys_put_le16(index, &frame->data[0]);
  sys_put_le32(bits, &frame->data[4]);

  return ret;
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

/* Every slot the handshake has to get into the motor for the current mode. */
static uint16_t robstride_required_slots(const struct robstride_motor_data * data)
{
  uint16_t slots = ROBSTRIDE_SLOTS_COMMON;

  if ((data->mode_params_set & BIT(data->mode)) != 0U) {
    slots |= robstride_mode_slots[data->mode];
  }

  return slots;
}

/* The value a slot is written with, and compared against when read back. */
static float robstride_slot_value(
  const struct robstride_motor_data * data, enum robstride_param_slot slot)
{
  const struct robstride_mode_params * params = &data->mode_params[data->mode];

  switch (slot) {
    case ROBSTRIDE_SLOT_RUN_MODE:
      return (float)robstride_run_mode(data->mode);
    case ROBSTRIDE_SLOT_LIMIT_CUR:
      return data->limits.current;
    case ROBSTRIDE_SLOT_LIMIT_SPD:
      /* CSP has no velocity cap of its own; robstride_actuator_bridge_ros2
       * puts its max velocity here. It may narrow the limit, never widen it. */
      if (
        (data->mode == ROBSTRIDE_MODE_POSITION_CSP) &&
        ((data->mode_params_set & BIT(ROBSTRIDE_MODE_POSITION_CSP)) != 0U)) {
        return MIN(params->max_velocity, data->limits.velocity);
      }
      return data->limits.velocity;
    case ROBSTRIDE_SLOT_LIMIT_TORQUE:
      return data->limits.torque;
    case ROBSTRIDE_SLOT_VEL_MAX:
      return params->max_velocity;
    case ROBSTRIDE_SLOT_ACC_SET:
    case ROBSTRIDE_SLOT_ACC_RAD:
      return params->acceleration;
    case ROBSTRIDE_SLOT_LOC_KP:
      return params->position_kp;
    case ROBSTRIDE_SLOT_SPD_KP:
      return params->velocity_kp;
    case ROBSTRIDE_SLOT_SPD_KI:
      return params->velocity_ki;
    case ROBSTRIDE_SLOT_CUR_KP:
      return params->current_kp;
    case ROBSTRIDE_SLOT_CUR_KI:
    default:
      return params->current_ki;
  }
}

static bool robstride_slot_matches(
  const struct robstride_motor_data * data, enum robstride_param_slot slot, uint32_t raw)
{
  const float expected = robstride_slot_value(data, slot);
  const float tolerance = robstride_slot_tolerance[slot];
  float actual;

  if (slot == ROBSTRIDE_SLOT_RUN_MODE) {
    return raw == robstride_run_mode(data->mode);
  }

  memcpy(&actual, &raw, sizeof(actual));

  /* Written as two comparisons so that NaN, which fails both, never matches. */
  return ((actual - expected) <= tolerance) && ((expected - actual) <= tolerance);
}

/*
 * Make slots be written again, whatever state they were in. A slot already
 * waiting to be written is unaffected.
 */
static void robstride_requeue_slots(struct robstride_motor_data * data, uint16_t slots)
{
  data->param_to_verify &= (uint16_t)~slots;
  data->param_to_write |= slots;
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
 * Record a frame that the controller accepted but could not get onto the wire,
 * so the transmit work builds its stage again.
 */
static void robstride_tx_done(const struct device * can_dev, int error, void * user_data)
{
  const struct robstride_tx_ctx * ctx = user_data;
  struct robstride_motor_data * data = ctx->data;
  k_spinlock_key_t key;

  ARG_UNUSED(can_dev);

  if (error == 0) {
    return;
  }

  key = k_spin_lock(&data->lock);

  if (ctx->stage == ROBSTRIDE_TX_STAGE_PARAM_WRITE) {
    data->param_tx_failed |= (uint16_t)BIT(ctx->slot);
  } else {
    data->tx_failed_stages |= BIT(ctx->stage);
  }

  k_spin_unlock(&data->lock, key);
}

/*
 * Send to the bus the motor last answered on. Until it has answered we do not
 * know which one that is, so the frame goes to every started bus.
 *
 * The result covers queueing only. can_send() with no completion callback
 * waits for the frame to reach the wire with K_FOREVER, which would park the
 * transmit work, and every caller below it, on a controller that cannot
 * transmit. Delivery is reported through robstride_tx_done() instead.
 */
static int robstride_motor_send_ctx(
  const struct device * motor_dev, const struct can_frame * frame,
  struct robstride_tx_ctx * ctx)
{
  const struct robstride_motor_config * config = motor_dev->config;
  const struct robstride_bus_config * bus_config = config->bus->config;
  struct robstride_bus_data * bus = config->bus->data;
  struct robstride_motor_data * data = motor_dev->data;
  k_spinlock_key_t key;
  uint8_t can_bus;
  bool attempted = false;
  int ret = 0;

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

    attempted = true;

    const int sent = can_send(bus_config->can_devs[i], frame, K_NO_WAIT, robstride_tx_done, ctx);

    if (sent != 0) {
      /*
       * Every controller addressed here has to take the frame. While the motor
       * is unlocated it could be behind any of them, so one that refuses is
       * one that may have been the right one, and reporting success would
       * strand the frame the way a discarded error used to.
       */
      ret = sent;
    }
  }

  return attempted ? ret : -EIO;
}

static int robstride_motor_send(
  const struct device * motor_dev, const struct can_frame * frame,
  enum robstride_tx_stage stage)
{
  struct robstride_motor_data * data = motor_dev->data;

  return robstride_motor_send_ctx(motor_dev, frame, &data->tx_ctx[stage]);
}

/*
 * Make the motor run the whole handshake again. Used wherever the motor may
 * have dropped what we wrote, or may never have received it.
 */
static void robstride_restart_handshake(struct robstride_motor_data * data)
{
  /* Only what we have values for: a mode left to the motor's own memory is
   * not overwritten. */
  data->param_to_write = robstride_required_slots(data);
  data->param_to_verify = 0U;
  data->verify_sent = false;
  data->verify_attempts = 0U;
  data->ref_sent = false;
  data->enable_sent = false;
  data->running_confirmed = false;
}

/*
 * Give up on this attempt and start over once CONFIG_MOTOR_ROBSTRIDE_CONFIGURE_BACKOFF_MS
 * has passed, as robstride_actuator_bridge_ros2 does. Starting over rather than
 * retrying the step that failed is deliberate: a motor that will not take a
 * write or will not run may have reset underneath us.
 */
static void robstride_configure_failed(
  struct robstride_motor_data * data, uint8_t motor_id, int64_t now, const char * what)
{
  data->configure_failures++;
  data->configure_after_ms = now + CONFIG_MOTOR_ROBSTRIDE_CONFIGURE_BACKOFF_MS;
  robstride_restart_handshake(data);

  if (!data->configure_failing) {
    LOG_WRN(
      "Motor %u: %s, configuring again every %d ms until it runs", (unsigned int)motor_id, what,
      CONFIG_MOTOR_ROBSTRIDE_CONFIGURE_BACKOFF_MS);
  }

  data->configure_failing = true;
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
  bool started_any = false;

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
      started_any = true;
      LOG_INF("CAN bus %u started", (unsigned int)i);
    }
  }

  if (!started_any) {
    return;
  }

  /*
   * A frame counts as sent once any started bus takes it, so with a second bus
   * already up the handshake of a motor sitting on the bus that was down ran
   * to completion against nobody. A motor that has answered has a known bus
   * and that bus was up, so only the unlocated ones have to start over.
   */
  for (size_t i = 0; i < ARRAY_SIZE(data->motors); ++i) {
    const struct device * motor_dev = data->motors[i];

    if (motor_dev == NULL) {
      continue;
    }

    struct robstride_motor_data * motor = motor_dev->data;
    k_spinlock_key_t key = k_spin_lock(&motor->lock);

    if (motor->can_bus == ROBSTRIDE_CANBUS_UNKNOWN) {
      robstride_restart_handshake(motor);
    }

    k_spin_unlock(&motor->lock, key);
  }
}

/* The target frame of a parameter-referenced mode. Operation control has a
 * frame type of its own and no reference parameter. */
static void robstride_build_ref(
  struct can_frame * frame, uint8_t master, uint8_t motor_id,
  const struct robstride_motor_data * data)
{
  uint16_t index;

  switch (data->mode) {
    case ROBSTRIDE_MODE_POSITION:
    case ROBSTRIDE_MODE_POSITION_CSP:
      index = ROBSTRIDE_PARAM_LOC_REF;
      break;
    case ROBSTRIDE_MODE_VELOCITY:
      index = ROBSTRIDE_PARAM_SPD_REF;
      break;
    case ROBSTRIDE_MODE_CURRENT:
    default:
      index = ROBSTRIDE_PARAM_IQ_REF;
      break;
  }

  (void)robstride_build_set_param(frame, master, motor_id, index, data->target);
}

/*
 * Produce the frames this motor owes the bus in this interval, and the
 * completion context of each. One handshake step per interval keeps the burst
 * short, and keeps a frame from overtaking the one it depends on: a controller
 * sends the lower identifier first, which would put a read (type 0x11) ahead
 * of the write it checks (0x12) and the enable (0x03) ahead of the target it
 * should follow.
 */
static size_t robstride_build_tx(
  const struct device * motor_dev, struct can_frame * frames, struct robstride_tx_ctx ** ctxs,
  int64_t now, enum robstride_tx_stage * stage, uint16_t * slots)
{
  const struct robstride_motor_config * config = motor_dev->config;
  const struct robstride_bus_config * bus_config = config->bus->config;
  struct robstride_motor_data * data = motor_dev->data;
  const uint8_t master = bus_config->master_can_id;
  const uint8_t motor_id = config->motor_id;
  size_t count = 0;

  *stage = ROBSTRIDE_TX_STAGE_NONE;
  *slots = 0U;

  /* A stop the API could not get out takes priority over everything else,
   * including the handshake a later enable may already have restarted. */
  if (data->stop_pending) {
    ctxs[count] = &data->tx_ctx[ROBSTRIDE_TX_STAGE_STOP];
    robstride_frame_init(&frames[count++], ROBSTRIDE_TYPE_STOP, master, motor_id);
    data->stop_pending = false;
    *stage = ROBSTRIDE_TX_STAGE_STOP;
    return count;
  }

  if (!data->enabled) {
    if ((now - data->last_probe_ms) < ROBSTRIDE_PROBE_INTERVAL_MS) {
      return 0;
    }

    data->last_probe_ms = now;
    ctxs[count] = &data->tx_ctx[ROBSTRIDE_TX_STAGE_PROBE];
    robstride_frame_init(&frames[count++], ROBSTRIDE_TYPE_GET_ID, master, motor_id);
    *stage = ROBSTRIDE_TX_STAGE_PROBE;
    return count;
  }

  if (now < data->configure_after_ms) {
    return 0;
  }

  if (data->param_to_verify != 0U) {
    if (!data->verify_sent) {
      for (int slot = 0; slot < ROBSTRIDE_SLOT_COUNT; ++slot) {
        if ((data->param_to_verify & BIT(slot)) == 0U) {
          continue;
        }

        ctxs[count] = &data->tx_ctx[ROBSTRIDE_TX_STAGE_PARAM_VERIFY];
        robstride_build_get_param(&frames[count++], master, motor_id, robstride_slot_index[slot]);
      }

      data->verify_sent = true;
      data->verify_deadline_ms = now + ROBSTRIDE_VERIFY_TIMEOUT_MS;
      *stage = ROBSTRIDE_TX_STAGE_PARAM_VERIFY;
      return count;
    }

    if (now < data->verify_deadline_ms) {
      return 0;
    }

    /*
     * A reply that never came and one that carried another value are treated
     * alike: either way the motor has not shown it holds what we wrote.
     * Mismatches are not acted on as they arrive, because a reply to an
     * earlier read of the same index is indistinguishable from this one's.
     */
    if (++data->verify_attempts >= ROBSTRIDE_VERIFY_RETRIES) {
      robstride_configure_failed(data, motor_id, now, "parameters not confirmed");
      return 0;
    }

    robstride_requeue_slots(data, data->param_to_verify);
  } else if (data->verify_sent) {
    /* Every read of the batch came back with the value written. */
    data->verify_sent = false;
    data->verify_attempts = 0U;
  }

  if (data->param_to_write != 0U) {
    uint16_t batch = 0U;

    if ((data->param_to_write & BIT(ROBSTRIDE_SLOT_RUN_MODE)) != 0U) {
      /*
       * The motor only accepts a new run mode while it is stopped. The stop
       * shares the run mode's context, so losing either one writes both
       * again. Stopping drops the enable, and the target with it.
       */
      ctxs[count] = &data->param_tx_ctx[ROBSTRIDE_SLOT_RUN_MODE];
      robstride_frame_init(&frames[count++], ROBSTRIDE_TYPE_STOP, master, motor_id);
      ctxs[count] = &data->param_tx_ctx[ROBSTRIDE_SLOT_RUN_MODE];
      (void)robstride_build_set_param(
        &frames[count++], master, motor_id, ROBSTRIDE_PARAM_RUN_MODE,
        robstride_slot_value(data, ROBSTRIDE_SLOT_RUN_MODE));
      batch = BIT(ROBSTRIDE_SLOT_RUN_MODE);
      data->ref_sent = false;
      data->enable_sent = false;
      data->running_confirmed = false;
    } else {
      for (int slot = 0; (slot < ROBSTRIDE_SLOT_COUNT) && (count < ROBSTRIDE_TX_BURST); ++slot) {
        if ((data->param_to_write & BIT(slot)) == 0U) {
          continue;
        }

        ctxs[count] = &data->param_tx_ctx[slot];
        (void)robstride_build_set_param(
          &frames[count++], master, motor_id, robstride_slot_index[slot],
          robstride_slot_value(data, (enum robstride_param_slot)slot));
        batch |= (uint16_t)BIT(slot);
      }
    }

    data->param_to_write &= (uint16_t)~batch;

    if (IS_ENABLED(CONFIG_MOTOR_ROBSTRIDE_VERIFY)) {
      data->param_to_verify |= batch;
      data->verify_sent = false;
    }

    *stage = ROBSTRIDE_TX_STAGE_PARAM_WRITE;
    *slots = batch;
    return count;
  }

  /*
   * The target goes in before the enable, so the motor does not start towards
   * whatever reference it held from before. Operation control carries its
   * target in the command frame and has nothing to write here.
   */
  if (!data->ref_sent) {
    data->ref_sent = true;

    if (data->mode != ROBSTRIDE_MODE_OPERATION) {
      ctxs[count] = &data->tx_ctx[ROBSTRIDE_TX_STAGE_REF];
      robstride_build_ref(&frames[count++], master, motor_id, data);
      *stage = ROBSTRIDE_TX_STAGE_REF;
      return count;
    }
  }

  if (!data->enable_sent) {
    ctxs[count] = &data->tx_ctx[ROBSTRIDE_TX_STAGE_ENABLE];
    robstride_frame_init(&frames[count++], ROBSTRIDE_TYPE_ENABLE, master, motor_id);
    data->enable_sent = true;
    data->running_confirmed = false;
    data->running_deadline_ms = now + CONFIG_MOTOR_ROBSTRIDE_RUNNING_TIMEOUT_MS;
    *stage = ROBSTRIDE_TX_STAGE_ENABLE;
    return count;
  }

  if (!data->running_confirmed) {
    if (now < data->running_deadline_ms) {
      return 0;
    }

    /* Calibrating is the motor working towards running, not refusing to. */
    if (data->run_state == ROBSTRIDE_RUN_STATE_CALIBRATION) {
      data->running_deadline_ms = now + CONFIG_MOTOR_ROBSTRIDE_RUNNING_TIMEOUT_MS;
      return 0;
    }

    robstride_configure_failed(data, motor_id, now, "not running after enable");
    return 0;
  }

  ctxs[count] = &data->tx_ctx[ROBSTRIDE_TX_STAGE_COMMAND];

  if (data->mode == ROBSTRIDE_MODE_OPERATION) {
    robstride_build_operation_control(
      &frames[count++], motor_id, &robstride_model_limits[config->model], &data->motion);
  } else {
    robstride_build_ref(&frames[count++], master, motor_id, data);
  }

  *stage = ROBSTRIDE_TX_STAGE_COMMAND;

  return count;
}

/*
 * Make parameter writes that did not reach the motor go out again. With
 * readback on, a slot already confirmed is left alone: the read proves the
 * motor holds the value, so the failure belonged to an earlier copy of the
 * write. Without it the failure is all there is to go on.
 */
static void robstride_retry_param_writes(struct robstride_motor_data * data, uint16_t slots)
{
  if (IS_ENABLED(CONFIG_MOTOR_ROBSTRIDE_VERIFY)) {
    slots &= data->param_to_verify;
  } else {
    slots &= robstride_required_slots(data);
  }

  if (slots == 0U) {
    return;
  }

  robstride_requeue_slots(data, slots);

  if ((slots & BIT(ROBSTRIDE_SLOT_RUN_MODE)) != 0U) {
    /* Rewriting the run mode stops the motor again. */
    data->ref_sent = false;
    data->enable_sent = false;
    data->running_confirmed = false;
  }
}

/*
 * Undo what robstride_build_tx() marked done, so the step is built again on
 * the next interval. The command repeats on its own and the probe comes round
 * again on its own interval, so neither has anything to take back. A lost read
 * is left to the readback timeout, which writes the batch again.
 */
static void robstride_retry_tx_stage(
  struct robstride_motor_data * data, enum robstride_tx_stage stage, uint16_t slots)
{
  switch (stage) {
    case ROBSTRIDE_TX_STAGE_STOP:
      data->stop_pending = true;
      break;
    case ROBSTRIDE_TX_STAGE_PARAM_WRITE:
      robstride_retry_param_writes(data, slots);
      break;
    case ROBSTRIDE_TX_STAGE_REF:
      data->ref_sent = false;
      break;
    case ROBSTRIDE_TX_STAGE_ENABLE:
      data->enable_sent = false;
      break;
    default:
      break;
  }
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
    struct robstride_tx_ctx * ctxs[ROBSTRIDE_TX_BURST];
    enum robstride_tx_stage stage;
    uint16_t slots;
    k_spinlock_key_t key;
    size_t count;
    bool queued = true;

    key = k_spin_lock(&data->lock);

    /* Frames the controller took but could not transmit are reported here, an
     * interval or more late, and each undoes its own step before the next one
     * is built. */
    uint32_t failed = data->tx_failed_stages;

    data->tx_failed_stages = 0U;

    while (failed != 0U) {
      const int bit = find_lsb_set(failed) - 1;

      failed &= ~BIT(bit);
      robstride_retry_tx_stage(data, (enum robstride_tx_stage)bit, 0U);
    }

    robstride_retry_param_writes(data, data->param_tx_failed);
    data->param_tx_failed = 0U;

    count = robstride_build_tx(motor_dev, frames, ctxs, now, &stage, &slots);
    k_spin_unlock(&data->lock, key);

    for (size_t f = 0; f < count; ++f) {
      if (robstride_motor_send_ctx(motor_dev, &frames[f], ctxs[f]) != 0) {
        queued = false;
      }
    }

    if (queued) {
      continue;
    }

    /*
     * Taking the step back rather than confirming it after a successful send
     * is what keeps this correct against the API calls that clear the same
     * flags: re-clearing a flag another thread has just cleared is a no-op,
     * whereas setting one would swallow the request it stands for.
     */
    key = k_spin_lock(&data->lock);
    robstride_retry_tx_stage(data, stage, slots);
    k_spin_unlock(&data->lock, key);
  }
}

static void robstride_bus_tx_timer_handler(struct k_timer * timer)
{
  struct robstride_bus_data * bus = k_timer_user_data_get(timer);

  k_work_submit(&bus->tx_work);
}

/*
 * Presence only. Every type the motor sends proves it is there and says which
 * controller it is on, but only a feedback frame carries measurements, so the
 * timestamp the staleness check reads is not touched here.
 */
static void robstride_mark_seen(struct robstride_motor_data * data, uint8_t can_bus)
{
  data->can_bus = can_bus;
  data->online = true;
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
  const uint8_t error_code = (uint8_t)((id >> 16) & 0x3FU);

  if ((data->run_state == ROBSTRIDE_RUN_STATE_RUNNING) &&
      (run_state != ROBSTRIDE_RUN_STATE_RUNNING)) {
    data->ref_sent = false;
    data->enable_sent = false;
    data->running_confirmed = false;
  } else if (
    data->enable_sent && (run_state == ROBSTRIDE_RUN_STATE_RUNNING) && (error_code == 0U)) {
    data->running_confirmed = true;
    data->configure_failing = false;
  }

  data->run_state = run_state;
  data->error_code = error_code;
  data->timestamp_ms = k_uptime_get();

  robstride_mark_seen(data, can_bus);
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
    robstride_restart_handshake(data);
  }

  robstride_mark_seen(data, can_bus);
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

  for (int slot = 0; slot < ROBSTRIDE_SLOT_COUNT; ++slot) {
    if (
      ((data->param_to_verify & BIT(slot)) != 0U) && (robstride_slot_index[slot] == index) &&
      robstride_slot_matches(data, (enum robstride_param_slot)slot, raw)) {
      data->param_to_verify &= (uint16_t)~BIT(slot);
    }
  }

  /* The slot stays taken; only the waiting caller gives it up. */
  if (data->param_armed && (data->param_index == index)) {
    data->param_raw = raw;
    data->param_complete = true;
    complete = true;
  }

  robstride_mark_seen(data, can_bus);
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

      robstride_mark_seen(data, (uint8_t)can_bus);
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
    robstride_restart_handshake(data);
    /* An enable asked for is a retry asked for, so it does not wait out the
     * pause after an earlier failure. */
    data->configure_after_ms = 0;
    /* The handshake opens with a stop of its own, so a stop still owed from an
     * earlier disable has nothing left to do. */
    data->stop_pending = false;
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
  int ret;

  key = k_spin_lock(&data->lock);
  /*
   * Dropping this now is what makes the transmit work stop commanding the
   * motor; it must not wait on the bus. The stop frame is a separate promise,
   * and the motor holds its last target until it arrives.
   */
  data->enabled = false;
  data->enable_sent = false;
  data->stop_pending = true;
  k_spin_unlock(&data->lock, key);

  /* Stopping is the safety-critical direction, so it does not wait for the
   * next transmit interval. */
  robstride_frame_init(&frame, ROBSTRIDE_TYPE_STOP, bus_config->master_can_id, config->motor_id);
  ret = robstride_motor_send(dev, &frame, ROBSTRIDE_TX_STAGE_STOP);

  if (ret != 0) {
    /* The work keeps re-sending, but the caller has to know the motor has not
     * been told to stop yet. */
    return ret;
  }

  key = k_spin_lock(&data->lock);
  data->stop_pending = false;
  k_spin_unlock(&data->lock, key);

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
   * drivers are published, and only once a feedback frame has carried them: a
   * motor that has answered nothing but a presence probe is online with no
   * measurement to its name.
   */
  if (data->has_last_position) {
    out->valid_mask = MOTOR_FEEDBACK_POSITION | MOTOR_FEEDBACK_TEMPERATURE;
  }

  out->position = data->position_counts;
  out->temperature = (int16_t)data->temperature;
  out->online = data->online;
  out->timestamp_ms = data->timestamp_ms;

  if (!data->online) {
    ret = -ENODATA;
  } else if (!data->has_last_position) {
    /*
     * Online with nothing measured: either only a presence or parameter reply
     * has come back, or the accumulator was emptied by a zeroing. The
     * timestamp can still be inside the timeout in the second case, so
     * freshness is not what decides this.
     */
    out->stale = true;
    ret = -EAGAIN;
  } else if (
    (bus_config->feedback_timeout_ms > 0U) &&
    ((k_uptime_get() - data->timestamp_ms) > (int64_t)bus_config->feedback_timeout_ms)) {
    out->stale = true;
    ret = -EAGAIN;
  }

  k_spin_unlock(&data->lock, key);

  return ret;
}

/*
 * Resolve one devicetree limit. ROBSTRIDE_LIMIT_ABSENT stands for a property
 * the devicetree left out, which the model's own limit fills in; the model
 * table cannot be read from the static initialiser that builds the config.
 */
static float robstride_resolve_limit(float requested, float model_limit)
{
  if (requested < 0.0F) {
    return model_limit;
  }

  return MIN(requested, model_limit);
}

static int robstride_motor_init(const struct device * dev)
{
  const struct robstride_motor_config * config = dev->config;
  const struct robstride_model_limits * model = &robstride_model_limits[config->model];
  struct robstride_motor_data * data = dev->data;

  if (!device_is_ready(config->bus)) {
    LOG_ERR("Bus not ready for motor %u", (unsigned int)config->motor_id);
    return -ENODEV;
  }

  for (size_t i = 0; i < ARRAY_SIZE(data->tx_ctx); ++i) {
    data->tx_ctx[i].data = data;
    data->tx_ctx[i].stage = (enum robstride_tx_stage)i;
  }

  for (size_t i = 0; i < ARRAY_SIZE(data->param_tx_ctx); ++i) {
    data->param_tx_ctx[i].data = data;
    data->param_tx_ctx[i].stage = ROBSTRIDE_TX_STAGE_PARAM_WRITE;
    data->param_tx_ctx[i].slot = (enum robstride_param_slot)i;
  }

  data->can_bus = ROBSTRIDE_CANBUS_UNKNOWN;
  data->mode = ROBSTRIDE_MODE_CURRENT;
  /* Held to the same range as robstride_set_limits(), so the devicetree cannot
   * put a limit on the wire that the run-time call would have rejected. */
  data->limits.current = robstride_resolve_limit(config->limits.current, model->current);
  data->limits.velocity = robstride_resolve_limit(config->limits.velocity, model->velocity);
  data->limits.torque = robstride_resolve_limit(config->limits.torque, model->torque);
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
    /* A run mode is only accepted while the motor is stopped, and the limits
     * and gains belong to the mode, so the handshake has to run again. */
    robstride_restart_handshake(data);
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
    robstride_restart_handshake(data);
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
  feedback->configured = data->enabled && (data->param_to_write == 0U) &&
                         (data->param_to_verify == 0U) && data->running_confirmed;
  feedback->configure_failures = data->configure_failures;

  if (!data->online) {
    ret = -ENODATA;
  } else if (!data->has_last_position) {
    /* Same rule as the class-wide snapshot: the fault and mode fields above
     * stay readable, but no measurement has arrived to be fresh. */
    feedback->stale = true;
    ret = -EAGAIN;
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
  robstride_requeue_slots(data, ROBSTRIDE_SLOTS_LIMITS);
  k_spin_unlock(&data->lock, key);

  return 0;
}

int robstride_get_limits(const struct device * dev, struct robstride_limits * limits)
{
  struct robstride_motor_data * data;
  k_spinlock_key_t key;

  if (!robstride_is_motor(dev) || (limits == NULL)) {
    return -EINVAL;
  }

  data = dev->data;

  key = k_spin_lock(&data->lock);
  *limits = data->limits;
  k_spin_unlock(&data->lock, key);

  return 0;
}

int robstride_set_mode_params(
  const struct device * dev, enum robstride_mode mode, const struct robstride_mode_params * params)
{
  const struct robstride_motor_config * config;
  struct robstride_motor_data * data;
  k_spinlock_key_t key;

  if (!robstride_is_motor(dev) || ((unsigned int)mode >= ROBSTRIDE_MODE_COUNT) || (params == NULL)) {
    return -EINVAL;
  }

  const float fields[] = {
    params->max_velocity, params->acceleration, params->position_kp, params->velocity_kp,
    params->velocity_ki,  params->current_kp,   params->current_ki,
  };

  for (size_t i = 0; i < ARRAY_SIZE(fields); ++i) {
    /* NaN fails the comparison, so it is rejected with the negatives. */
    if (!(fields[i] >= 0.0F)) {
      return -EINVAL;
    }
  }

  config = dev->config;
  data = dev->data;

  key = k_spin_lock(&data->lock);
  data->mode_params[mode] = *params;
  data->mode_params[mode].max_velocity =
    MIN(params->max_velocity, robstride_model_limits[config->model].velocity);
  data->mode_params_set |= (uint8_t)BIT(mode);

  /* Another mode's values go in when the motor is switched to it. */
  if (data->mode == mode) {
    robstride_requeue_slots(data, robstride_mode_slots[mode]);
  }

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

  return robstride_motor_send(dev, &frame, ROBSTRIDE_TX_STAGE_NONE);
}

int robstride_set_zero(const struct device * dev)
{
  struct robstride_motor_data * data;
  k_spinlock_key_t key;
  const int ret = robstride_send_simple(dev, ROBSTRIDE_TYPE_SET_ZERO, 0x01U);

  if (ret != 0) {
    return ret;
  }

  data = dev->data;

  /*
   * Everything after this counts from the new origin, so the next reading has
   * to seed the accumulator instead of being differenced against a sample that
   * belongs to the old one. Zeroing only takes effect on a stopped motor, and
   * a stopped motor is sent no commands, so no feedback is on its way here.
   */
  key = k_spin_lock(&data->lock);
  data->has_last_position = false;
  k_spin_unlock(&data->lock, key);

  return 0;
}

int robstride_save_parameters(const struct device * dev)
{
  return robstride_send_simple(dev, ROBSTRIDE_TYPE_SAVE, 0x01U);
}

int robstride_clear_faults(const struct device * dev)
{
  /*
   * The stop frame carries the clear in its first payload byte, so this also
   * takes the output down. Nothing here touches the driver's own enabled
   * flag: the motor leaving RUNNING is what restarts the handshake, and that
   * is observed from the feedback rather than assumed at the call.
   */
  return robstride_send_simple(dev, ROBSTRIDE_TYPE_STOP, 0x01U);
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

  const int ret =
    robstride_build_set_param(&frame, bus_config->master_can_id, config->motor_id, index, value);

  if (ret != 0) {
    return ret;
  }

  return robstride_motor_send(dev, &frame, ROBSTRIDE_TX_STAGE_NONE);
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

  /* Take the slot before the semaphore is touched, but leave it unarmed: a
   * reply matched now would be signalled and then thrown away by the reset. */
  data->param_pending = true;
  data->param_armed = false;
  data->param_complete = false;
  k_spin_unlock(&data->lock, key);

  /* Drop a signal left behind by a read whose reply arrived after it gave up. */
  k_sem_reset(&data->param_sem);

  key = k_spin_lock(&data->lock);
  data->param_index = index;
  data->param_armed = true;
  k_spin_unlock(&data->lock, key);

  robstride_build_get_param(&frame, bus_config->master_can_id, config->motor_id, index);
  ret = robstride_motor_send(dev, &frame, ROBSTRIDE_TX_STAGE_NONE);

  if (ret == 0) {
    ret = k_sem_take(&data->param_sem, K_MSEC(CONFIG_MOTOR_ROBSTRIDE_PARAM_TIMEOUT_MS));

    if (ret != 0) {
      ret = -ETIMEDOUT;
    }
  }

  key = k_spin_lock(&data->lock);
  data->param_armed = false;
  raw = data->param_raw;

  /* A reply that landed just as the wait expired is still this read's answer,
   * so the flag decides rather than how the wait ended. */
  if (data->param_complete) {
    ret = 0;
  }

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
  BUILD_ASSERT(                                                                        \
    IN_RANGE(DT_INST_PROP(inst, master_can_id), 0, UINT8_MAX),                         \
    "master-can-id does not fit in the low byte of the identifier");                   \
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

/*
 * A limit left out of the devicetree is carried as ROBSTRIDE_LIMIT_ABSENT and
 * resolved against the model in robstride_motor_init(). Naming the model table
 * here instead would read a const array from a static initialiser, which only
 * the common compilers accept.
 */
#define ROBSTRIDE_LIMIT_ABSENT (-1.0F)

#define ROBSTRIDE_LIMIT_INIT(inst, prop)                                                     \
  COND_CODE_1(                                                                               \
    DT_INST_NODE_HAS_PROP(inst, prop), ((float)DT_INST_PROP(inst, prop) / 1000.0F),           \
    (ROBSTRIDE_LIMIT_ABSENT))

/*
 * A devicetree integer reaches C unsigned, so a negative limit arrives as a
 * value above INT32_MAX rather than as a negative one and would otherwise be
 * taken for a very large limit and quietly cut down to the model's.
 */
#define ROBSTRIDE_LIMIT_ASSERT(inst, prop)          \
  BUILD_ASSERT(                                     \
    DT_INST_PROP_OR(inst, prop, 0) <= INT32_MAX,    \
    #prop " must be positive and within int32 range");

#define ROBSTRIDE_MOTOR_DEFINE(inst)                                                        \
  BUILD_ASSERT(                                                                             \
    DT_INST_ENUM_IDX(inst, model) < ARRAY_SIZE(robstride_model_limits),                     \
    "Unknown RobStride model");                                                             \
  BUILD_ASSERT(                                                                             \
    IN_RANGE(DT_INST_REG_ADDR(inst), 1, 127),                                               \
    "reg is the motor CAN ID and must be in 1..127");                                       \
  ROBSTRIDE_LIMIT_ASSERT(inst, max_current_ma)                                              \
  ROBSTRIDE_LIMIT_ASSERT(inst, max_velocity_mrad_s)                                         \
  ROBSTRIDE_LIMIT_ASSERT(inst, max_torque_mnm)                                              \
  static struct robstride_motor_data robstride_motor_data_##inst;                           \
  static const struct robstride_motor_config robstride_motor_config_##inst = {              \
    .bus = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                             \
    .motor_id = DT_INST_REG_ADDR(inst),                                                     \
    .model = (enum robstride_model)DT_INST_ENUM_IDX(inst, model),                           \
    .limits =                                                                               \
      {                                                                                     \
        .current = ROBSTRIDE_LIMIT_INIT(inst, max_current_ma),                              \
        .velocity = ROBSTRIDE_LIMIT_INIT(inst, max_velocity_mrad_s),                        \
        .torque = ROBSTRIDE_LIMIT_INIT(inst, max_torque_mnm),                               \
      },                                                                                    \
  };                                                                                        \
  DEVICE_DT_INST_DEFINE(                                                                    \
    inst, robstride_motor_init, NULL, &robstride_motor_data_##inst,                         \
    &robstride_motor_config_##inst, POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY,                \
    &robstride_motor_api);

DT_INST_FOREACH_STATUS_OKAY(ROBSTRIDE_MOTOR_DEFINE)
