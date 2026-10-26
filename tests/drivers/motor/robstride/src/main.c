/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can/can_fake.h>
#include <zephyr/fff.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#include <drivers/motor.h>
#include <drivers/motor/robstride.h>

#define TEST_MASTER_ID 0xFDU
#define TEST_MOTOR0_ID 0x7FU
#define TEST_MOTOR1_ID 0x01U
#define TEST_MOTOR2_ID 0x02U
#define TEST_MOTOR4_ID 0x04U

#define TYPE_GET_ID 0x00U
#define TYPE_OP_CONTROL 0x01U
#define TYPE_FEEDBACK 0x02U
#define TYPE_ENABLE 0x03U
#define TYPE_STOP 0x04U
#define TYPE_GET_PARAM 0x11U
#define TYPE_SET_PARAM 0x12U
#define TYPE_FAULT 0x15U

#define PARAM_RUN_MODE 0x7005U
#define PARAM_IQ_REF 0x7006U
#define PARAM_SPD_REF 0x700AU
#define PARAM_LIMIT_TORQUE 0x700BU
#define PARAM_LOC_REF 0x7016U
#define PARAM_LIMIT_SPD 0x7017U
#define PARAM_LIMIT_CUR 0x7018U
#define PARAM_CUR_KP 0x7010U
#define PARAM_CUR_KI 0x7011U
#define PARAM_LOC_KP 0x701EU
#define PARAM_SPD_KP 0x701FU
#define PARAM_SPD_KI 0x7020U
#define PARAM_ACC_RAD 0x7022U
#define PARAM_VEL_MAX 0x7024U
#define PARAM_ACC_SET 0x7025U

#define RS00_VELOCITY_MAX 33.0F
#define RS00_TORQUE_MAX 14.0F
#define RS05_VELOCITY_MAX 50.0F
#define POSITION_MAX 12.56637F

/* Long enough for the driver to work through every handshake step, with each
 * batch of writes read back, but shorter than the running timeout. */
#define HANDSHAKE_MS (CONFIG_MOTOR_ROBSTRIDE_TX_INTERVAL_MS * 16)
BUILD_ASSERT(HANDSHAKE_MS < CONFIG_MOTOR_ROBSTRIDE_RUNNING_TIMEOUT_MS);

/* Lower priority than the test thread, so answering the reader below does not
 * hand the CPU over before the assertion that follows it runs. */
#define READER_PRIORITY (CONFIG_ZTEST_THREAD_PRIORITY + 1)
#define READER_STACK_SIZE 1024

static const struct device * const test_can = DEVICE_DT_GET(DT_NODELABEL(test_can0));
static const struct device * const motor0 = DEVICE_DT_GET(DT_NODELABEL(motor0));
static const struct device * const motor1 = DEVICE_DT_GET(DT_NODELABEL(motor1));
static const struct device * const motor2 = DEVICE_DT_GET(DT_NODELABEL(motor2));
static const struct device * const motor3 = DEVICE_DT_GET(DT_NODELABEL(motor3));
static const struct device * const motor4 = DEVICE_DT_GET(DT_NODELABEL(motor4));

#define CAPTURE_MAX 64

static struct can_frame captured[CAPTURE_MAX];
static size_t captured_count;
static can_rx_callback_t rx_callback;
static void * rx_user_data;
static struct can_filter rx_filter;

/* Makes can_send() refuse the way a controller with a full transmit queue
 * does, for the tests that check what the driver does with the refusal. */
static atomic_t send_fails;

/* Makes can_send() accept the frame and then report that it never reached the
 * wire, the way a controller on a bus that nobody acknowledges does. */
static atomic_t send_completes_with_error;

/* Makes the controller take the frame and never report a completion at all,
 * which is what a bus nobody acknowledges looks like until it goes bus-off. */
static atomic_t send_swallows_completion;

/* Holds completions back so a test can report them after later frames have
 * already gone out, which is the only way to tell whether a failure is
 * attributed to the stage it belonged to. */
static atomic_t send_defers_completion;

#define DEFERRED_MAX 16

struct deferred_completion
{
  can_tx_callback_t callback;
  const struct device * dev;
  void * user_data;
};

static struct deferred_completion deferred[DEFERRED_MAX];
static size_t deferred_count;

/*
 * The motors on the far side of the fake controller. Each keeps what it is
 * written, answers a read with it, and answers an enable with a feedback frame
 * reporting running, which is all the handshake needs to run to the end.
 */
#define SIM_PARAMS 16

struct sim_param
{
  uint16_t index;
  uint32_t raw;
};

struct sim_motor
{
  uint8_t id;
  size_t count;
  struct sim_param params[SIM_PARAMS];
};

static struct sim_motor sims[] = {
  {.id = TEST_MOTOR0_ID},
  {.id = TEST_MOTOR1_ID},
  {.id = TEST_MOTOR2_ID},
};

/* Answers no read. */
static atomic_t sim_silent;
/* Answers the enable with nothing, so the motor never reports running. */
static atomic_t sim_ignores_enable;
/* Answers reads of this index with a value other than the one stored, far
 * enough off to fall outside any readback tolerance. */
static atomic_t sim_skewed_index;

static void release_deferred_completions(int error)
{
  const size_t count = MIN(deferred_count, (size_t)DEFERRED_MAX);

  deferred_count = 0;

  for (size_t i = 0; i < count; ++i) {
    deferred[i].callback(deferred[i].dev, error, deferred[i].user_data);
  }
}

DEFINE_FFF_GLOBALS;

static int test_fake_can_add_rx_filter(
  const struct device * dev, can_rx_callback_t callback, void * user_data,
  const struct can_filter * filter)
{
  ARG_UNUSED(dev);

  rx_callback = callback;
  rx_user_data = user_data;
  rx_filter = *filter;

  return 1;
}

static void inject(uint8_t type, uint8_t aux_high, uint8_t motor_id, const uint8_t data[8]);
static void inject_feedback_in_state(
  uint8_t motor_id, enum robstride_run_state run_state, uint16_t position, uint16_t velocity,
  uint16_t torque, int16_t temperature);

static struct sim_param * sim_find(struct sim_motor * sim, uint16_t index)
{
  for (size_t i = 0; i < sim->count; ++i) {
    if (sim->params[i].index == index) {
      return &sim->params[i];
    }
  }

  if (sim->count >= SIM_PARAMS) {
    return NULL;
  }

  sim->params[sim->count] = (struct sim_param){.index = index};

  return &sim->params[sim->count++];
}

static void sim_receive(const struct can_frame * frame)
{
  const uint32_t id = frame->id & CAN_EXT_ID_MASK;
  const uint8_t type = (uint8_t)((id >> 24) & 0x1FU);
  const uint8_t motor_id = (uint8_t)(id & 0xFFU);
  struct sim_motor * sim = NULL;

  for (size_t i = 0; i < ARRAY_SIZE(sims); ++i) {
    if (sims[i].id == motor_id) {
      sim = &sims[i];
    }
  }

  if (sim == NULL) {
    return;
  }

  const uint16_t index = sys_get_le16(&frame->data[0]);
  struct sim_param * param;
  uint8_t reply[8] = {0};

  switch (type) {
    case 0x12U: /* set parameter */
      param = sim_find(sim, index);
      if (param != NULL) {
        param->raw = sys_get_le32(&frame->data[4]);
      }
      break;
    case 0x11U: /* get parameter */
      param = sim_find(sim, index);
      if ((param == NULL) || atomic_get(&sim_silent)) {
        break;
      }
      sys_put_le16(index, &reply[0]);
      sys_put_le32(
        param->raw ^ ((atomic_get(&sim_skewed_index) == index) ? 0x00400000U : 0U), &reply[4]);
      inject(0x11U, 0U, motor_id, reply);
      break;
    case 0x03U: /* enable */
      if (!atomic_get(&sim_ignores_enable)) {
        inject_feedback_in_state(motor_id, ROBSTRIDE_RUN_STATE_RUNNING, 32767U, 32767U, 32767U, 250);
      }
      break;
    default:
      break;
  }
}

static int test_fake_can_send(
  const struct device * dev, const struct can_frame * frame, k_timeout_t timeout,
  can_tx_callback_t callback, void * user_data)
{
  ARG_UNUSED(timeout);

  if (atomic_get(&send_fails) != 0) {
    return -EAGAIN;
  }

  if (captured_count < CAPTURE_MAX) {
    captured[captured_count] = *frame;
  }

  captured_count++;

  /* A frame reported as never transmitted never reached the motor either. A
   * deferred one did, whatever its completion later says. */
  if (!atomic_get(&send_completes_with_error) && !atomic_get(&send_swallows_completion)) {
    sim_receive(frame);
  }

  if ((callback == NULL) || atomic_get(&send_swallows_completion)) {
    return 0;
  }

  if (atomic_get(&send_defers_completion)) {
    if (deferred_count < DEFERRED_MAX) {
      deferred[deferred_count] = (struct deferred_completion){
        .callback = callback,
        .dev = dev,
        .user_data = user_data,
      };
    }

    deferred_count++;

    return 0;
  }

  callback(dev, atomic_get(&send_completes_with_error) ? -EIO : 0, user_data);

  return 0;
}

static int test_fake_can_start(const struct device * dev)
{
  ARG_UNUSED(dev);
  return 0;
}

static int robstride_test_init(void)
{
  fake_can_add_rx_filter_fake.custom_fake = test_fake_can_add_rx_filter;
  fake_can_send_fake.custom_fake = test_fake_can_send;
  fake_can_start_fake.custom_fake = test_fake_can_start;

  return 0;
}

SYS_INIT(robstride_test_init, PRE_KERNEL_1, 0);

static uint8_t frame_type(const struct can_frame * frame)
{
  return (uint8_t)(((frame->id & CAN_EXT_ID_MASK) >> 24) & 0x1FU);
}

static uint8_t frame_target(const struct can_frame * frame)
{
  return (uint8_t)(frame->id & 0xFFU);
}

static uint16_t frame_param_index(const struct can_frame * frame)
{
  return sys_get_le16(&frame->data[0]);
}

static float frame_param_float(const struct can_frame * frame)
{
  const uint32_t bits = sys_get_le32(&frame->data[4]);
  float value;

  memcpy(&value, &bits, sizeof(value));

  return value;
}

static void clear_captures(void)
{
  captured_count = 0;
  memset(captured, 0, sizeof(captured));
}

/* Frames for the other motor and the presence probes are noise to most tests. */
static const struct can_frame * find_frame(uint8_t target, uint8_t type, size_t skip)
{
  for (size_t i = 0; i < MIN(captured_count, (size_t)CAPTURE_MAX); ++i) {
    if ((frame_target(&captured[i]) != target) || (frame_type(&captured[i]) != type)) {
      continue;
    }

    if (skip-- == 0U) {
      return &captured[i];
    }
  }

  return NULL;
}

static const struct can_frame * find_param_frame(
  uint8_t target, uint8_t type, uint16_t index, size_t skip)
{
  for (size_t i = 0; i < MIN(captured_count, (size_t)CAPTURE_MAX); ++i) {
    if ((frame_target(&captured[i]) != target) || (frame_type(&captured[i]) != type)) {
      continue;
    }

    if ((frame_param_index(&captured[i]) == index) && (skip-- == 0U)) {
      return &captured[i];
    }
  }

  return NULL;
}

static const struct can_frame * find_set_param(uint8_t target, uint16_t index)
{
  return find_param_frame(target, TYPE_SET_PARAM, index, 0);
}

static size_t count_set_param(uint8_t target, uint16_t index)
{
  size_t count = 0;

  while (find_param_frame(target, TYPE_SET_PARAM, index, count) != NULL) {
    count++;
  }

  return count;
}

static void inject(uint8_t type, uint8_t aux_high, uint8_t motor_id, const uint8_t data[8])
{
  struct can_frame frame = {
    .id = ((uint32_t)type << 24) | ((uint32_t)aux_high << 16) | ((uint32_t)motor_id << 8) |
          TEST_MASTER_ID,
    .dlc = can_bytes_to_dlc(8U),
    .flags = CAN_FRAME_IDE,
  };

  memcpy(frame.data, data, sizeof(frame.data));

  zassert_not_null(rx_callback, "driver never registered an RX filter");
  rx_callback(test_can, &frame, rx_user_data);
}

/* Bits 23..22 of the identifier carry the run state, bits 21..16 the error. */
#define RUN_STATE_RUNNING (ROBSTRIDE_RUN_STATE_RUNNING << 6)

static void inject_feedback_in_state(
  uint8_t motor_id, enum robstride_run_state run_state, uint16_t position, uint16_t velocity,
  uint16_t torque, int16_t temperature)
{
  uint8_t data[8];

  sys_put_be16(position, &data[0]);
  sys_put_be16(velocity, &data[2]);
  sys_put_be16(torque, &data[4]);
  sys_put_be16((uint16_t)temperature, &data[6]);

  inject(TYPE_FEEDBACK, (uint8_t)(run_state << 6), motor_id, data);
}

static void inject_feedback(
  uint8_t motor_id, uint16_t position, uint16_t velocity, uint16_t torque, int16_t temperature)
{
  inject_feedback_in_state(
    motor_id, ROBSTRIDE_RUN_STATE_RUNNING, position, velocity, torque, temperature);
}

static void enable_and_settle(const struct device * dev)
{
  zassert_ok(motor_enable(dev));
  k_msleep(HANDSHAKE_MS);
}

static void before_each(void * fixture)
{
  ARG_UNUSED(fixture);

  /*
   * can_fake registers a ztest rule that resets every fake before each test,
   * which drops the custom implementations installed at boot. Reinstalling
   * them has to be the first thing done here: can_send() with no completion
   * callback waits forever on one supplied by the CAN API, and the default
   * fake never calls it back.
   */
  fake_can_send_fake.custom_fake = test_fake_can_send;
  fake_can_start_fake.custom_fake = test_fake_can_start;
  fake_can_add_rx_filter_fake.custom_fake = test_fake_can_add_rx_filter;

  atomic_set(&send_fails, 0);
  atomic_set(&send_completes_with_error, 0);
  atomic_set(&send_swallows_completion, 0);
  atomic_set(&send_defers_completion, 0);
  atomic_set(&sim_silent, 0);
  atomic_set(&sim_ignores_enable, 0);
  atomic_set(&sim_skewed_index, 0);
  deferred_count = 0;
  (void)motor_disable(motor0);
  (void)motor_disable(motor1);
  (void)motor_disable(motor2);
  k_msleep(CONFIG_MOTOR_ROBSTRIDE_TX_INTERVAL_MS * 2);
  clear_captures();
}

ZTEST_SUITE(robstride_motor, NULL, NULL, before_each, NULL, NULL);

ZTEST(robstride_motor, test_rx_filter_matches_extended_frames_addressed_to_the_master)
{
  zassert_true((rx_filter.flags & CAN_FILTER_IDE) != 0U, "filter must accept extended IDs");
  zassert_equal(rx_filter.mask, 0xFFU, "filter must key on the host byte only");
  zassert_equal(rx_filter.id, TEST_MASTER_ID, "filter must key on the configured master ID");
}

ZTEST(robstride_motor, test_set_output_is_rejected_for_every_mode)
{
  zassert_equal(
    motor_set_output(motor0, MOTOR_OUTPUT_MODE_CURRENT, 100), -ENOTSUP,
    "current has no class-wide unit here");
  zassert_equal(motor_set_output(motor0, MOTOR_OUTPUT_MODE_VELOCITY, 100), -ENOTSUP);
  zassert_equal(motor_set_output(motor0, MOTOR_OUTPUT_MODE_VOLTAGE, 100), -ENOTSUP);
}

ZTEST(robstride_motor, test_enable_stops_the_motor_before_writing_the_run_mode)
{
  zassert_ok(robstride_set_current(motor0, 1.0F));
  enable_and_settle(motor0);

  const struct can_frame * stop = find_frame(TEST_MOTOR0_ID, TYPE_STOP, 0);
  const struct can_frame * run_mode = find_set_param(TEST_MOTOR0_ID, PARAM_RUN_MODE);

  zassert_not_null(stop, "no stop frame");
  zassert_not_null(run_mode, "no run mode write");
  zassert_true(stop < run_mode, "run mode must be written after the motor is stopped");
  zassert_equal(sys_get_le32(&run_mode->data[4]), 3U, "current mode is run mode 3");
}

ZTEST(robstride_motor, test_handshake_reads_back_the_limits_then_sets_the_target_then_enables)
{
  zassert_ok(robstride_set_current(motor0, 2.0F));
  enable_and_settle(motor0);

  const struct can_frame * limit_cur = find_set_param(TEST_MOTOR0_ID, PARAM_LIMIT_CUR);
  const struct can_frame * limit_spd = find_set_param(TEST_MOTOR0_ID, PARAM_LIMIT_SPD);
  const struct can_frame * limit_torque = find_set_param(TEST_MOTOR0_ID, PARAM_LIMIT_TORQUE);
  const struct can_frame * ref = find_set_param(TEST_MOTOR0_ID, PARAM_IQ_REF);
  const struct can_frame * enable = find_frame(TEST_MOTOR0_ID, TYPE_ENABLE, 0);
  const struct can_frame * command = find_param_frame(TEST_MOTOR0_ID, TYPE_SET_PARAM, PARAM_IQ_REF, 1);

  zassert_not_null(limit_cur, "no current limit write");
  zassert_not_null(limit_spd, "no velocity limit write");
  zassert_not_null(limit_torque, "no torque limit write");
  zassert_not_null(ref, "no target before the enable");
  zassert_not_null(enable, "no enable frame");
  zassert_not_null(command, "no target after the enable");

  zassert_true(limit_torque < ref, "limits must be written before the target");
  zassert_true(ref < enable, "the target must be in place before the motor is enabled");
  zassert_true(enable < command, "commands must follow the enable");

  zassert_within(
    frame_param_float(limit_cur), 15.5F, 0.001F, "rs00 defaults to the model current limit");
  zassert_within(frame_param_float(command), 2.0F, 0.001F, "wrong current target");

  if (IS_ENABLED(CONFIG_MOTOR_ROBSTRIDE_VERIFY)) {
    const struct can_frame * read_cur =
      find_param_frame(TEST_MOTOR0_ID, TYPE_GET_PARAM, PARAM_LIMIT_CUR, 0);

    zassert_not_null(read_cur, "the current limit was never read back");
    zassert_true(limit_torque < read_cur, "a read must not share an interval with its write");
    zassert_true(read_cur < ref, "the limits must be confirmed before going on");
  }
}

/*
 * One test because the three phases are one life-cycle on one motor: whether
 * a mode has values of its own is state that outlives a test, so splitting
 * them would make them depend on the execution order.
 */
ZTEST(robstride_motor, test_mode_params_are_written_only_once_the_application_sets_them_and_again_after_a_fault)
{
  zassert_ok(robstride_set_current(motor0, 1.0F));
  enable_and_settle(motor0);

  zassert_is_null(
    find_set_param(TEST_MOTOR0_ID, PARAM_CUR_KP),
    "a motor keeps its stored values until the application supplies some");

  const struct robstride_mode_params params = {
    .position_kp = 30.0F,
    .velocity_kp = 2.0F,
    .velocity_ki = 0.02F,
    .current_kp = 0.05F,
    .current_ki = 0.01F,
  };

  clear_captures();
  zassert_ok(robstride_set_mode_params(motor0, ROBSTRIDE_MODE_CURRENT, &params));
  k_msleep(HANDSHAKE_MS);

  const struct can_frame * cur_kp = find_set_param(TEST_MOTOR0_ID, PARAM_CUR_KP);
  const struct can_frame * cur_ki = find_set_param(TEST_MOTOR0_ID, PARAM_CUR_KI);

  zassert_not_null(cur_kp, "no current loop gain write");
  zassert_not_null(cur_ki, "no current loop integral write");
  zassert_within(frame_param_float(cur_kp), 0.05F, 0.0001F, "wrong current loop gain");
  zassert_within(frame_param_float(cur_ki), 0.01F, 0.0001F, "wrong current loop integral");
  zassert_is_null(
    find_set_param(TEST_MOTOR0_ID, PARAM_LOC_KP), "current mode has no position loop to tune");

  uint8_t data[8] = {0};

  clear_captures();
  sys_put_le32(0x00000004U, &data[0]);
  inject(TYPE_FAULT, 0U, TEST_MOTOR0_ID, data);
  k_msleep(HANDSHAKE_MS);

  const struct can_frame * recovered = find_set_param(TEST_MOTOR0_ID, PARAM_CUR_KP);

  zassert_not_null(recovered, "a tripped motor has dropped the values we supplied");
  zassert_within(frame_param_float(recovered), 0.05F, 0.0001F, "wrong gain on recovery");
}

ZTEST(robstride_motor, test_devicetree_limit_overrides_the_model_limit)
{
  zassert_ok(robstride_set_current(motor1, 1.0F));
  enable_and_settle(motor1);

  const struct can_frame * limit_cur = find_set_param(TEST_MOTOR1_ID, PARAM_LIMIT_CUR);

  zassert_not_null(limit_cur, "no current limit write");
  zassert_within(frame_param_float(limit_cur), 5.0F, 0.001F, "max-current-ma must win");
}

ZTEST(robstride_motor, test_devicetree_limit_above_the_model_limit_is_clamped)
{
  zassert_ok(robstride_set_current(motor2, 1.0F));
  enable_and_settle(motor2);

  const struct can_frame * limit_spd = find_set_param(TEST_MOTOR2_ID, PARAM_LIMIT_SPD);

  zassert_not_null(limit_spd, "no velocity limit write");
  zassert_within(
    frame_param_float(limit_spd), RS05_VELOCITY_MAX, 0.001F,
    "the devicetree must not put a limit on the wire that robstride_set_limits() would reject");
}

ZTEST(robstride_motor, test_a_fault_does_not_write_mode_params_the_application_never_supplied)
{
  uint8_t data[8] = {0};

  zassert_ok(robstride_set_current(motor2, 1.0F));
  enable_and_settle(motor2);
  clear_captures();

  sys_put_le32(0x00000004U, &data[0]);
  inject(TYPE_FAULT, 0U, TEST_MOTOR2_ID, data);
  k_msleep(HANDSHAKE_MS);

  zassert_not_null(find_frame(TEST_MOTOR2_ID, TYPE_ENABLE, 0), "the motor was not recovered");
  zassert_is_null(
    find_set_param(TEST_MOTOR2_ID, PARAM_CUR_KP),
    "recovering must not overwrite the motor's own gains with zeros");
}

ZTEST(robstride_motor, test_target_is_clamped_to_the_model_limit)
{
  zassert_ok(robstride_set_velocity(motor0, 1000.0F));
  enable_and_settle(motor0);

  const struct can_frame * command = find_set_param(TEST_MOTOR0_ID, PARAM_SPD_REF);

  zassert_not_null(command, "no velocity target");
  zassert_within(
    frame_param_float(command), RS00_VELOCITY_MAX, 0.001F, "target must clamp to the model limit");
}

ZTEST(robstride_motor, test_mode_change_stops_the_motor_again)
{
  zassert_ok(robstride_set_current(motor0, 1.0F));
  enable_and_settle(motor0);
  clear_captures();

  zassert_ok(robstride_set_position(motor0, 1.0F));
  k_msleep(HANDSHAKE_MS);

  const struct can_frame * stop = find_frame(TEST_MOTOR0_ID, TYPE_STOP, 0);
  const struct can_frame * run_mode = find_set_param(TEST_MOTOR0_ID, PARAM_RUN_MODE);
  const struct can_frame * enable = find_frame(TEST_MOTOR0_ID, TYPE_ENABLE, 0);
  const struct can_frame * command = find_set_param(TEST_MOTOR0_ID, PARAM_LOC_REF);

  zassert_not_null(stop, "a mode change must stop the motor first");
  zassert_not_null(run_mode, "no run mode write");
  zassert_equal(sys_get_le32(&run_mode->data[4]), 1U, "position mode is run mode 1");
  zassert_not_null(enable, "the motor must be enabled again after the mode change");
  zassert_not_null(command, "no position target");
  zassert_within(frame_param_float(command), 1.0F, 0.001F, "wrong position target");
}

ZTEST(robstride_motor, test_operation_mode_packs_torque_into_the_identifier)
{
  const struct robstride_motion_target target = {
    .position = 0.0F,
    .velocity = 0.0F,
    .kp = 250.0F,
    .kd = 2.5F,
    .torque = 7.0F,
  };

  zassert_ok(robstride_set_motion_target(motor0, &target));
  enable_and_settle(motor0);

  const struct can_frame * command = find_frame(TEST_MOTOR0_ID, TYPE_OP_CONTROL, 0);

  zassert_not_null(command, "no operation control frame");

  const uint16_t torque_raw = (uint16_t)((command->id >> 8) & 0xFFFFU);
  const float torque =
    ((float)torque_raw * (2.0F * RS00_TORQUE_MAX) / (float)UINT16_MAX) - RS00_TORQUE_MAX;

  zassert_within(torque, 7.0F, 0.01F, "torque is carried in bits 23..8");
  zassert_within(
    ((float)sys_get_be16(&command->data[0]) * (2.0F * POSITION_MAX) / (float)UINT16_MAX) -
      POSITION_MAX,
    0.0F, 0.01F, "wrong position field");
  zassert_within(
    (float)sys_get_be16(&command->data[4]) * 500.0F / (float)UINT16_MAX, 250.0F, 0.05F,
    "wrong kp field");
  zassert_within(
    (float)sys_get_be16(&command->data[6]) * 5.0F / (float)UINT16_MAX, 2.5F, 0.001F,
    "wrong kd field");
}

ZTEST(robstride_motor, test_feedback_is_unavailable_until_the_motor_answers)
{
  struct robstride_feedback si;

  zassert_equal(robstride_get_feedback(motor3, &si), -ENODATA, "nothing has answered yet");
  zassert_false(si.online, "a motor that never answered is not online");
}

ZTEST(robstride_motor, test_a_motor_that_only_answered_a_probe_publishes_no_measurement)
{
  struct motor_feedback feedback;
  uint8_t data[8] = {0};

  inject(TYPE_GET_ID, 0U, TEST_MOTOR4_ID, data);

  zassert_equal(
    motor_get_feedback(motor4, &feedback), -EAGAIN,
    "presence alone is not a measurement");
  zassert_true(feedback.online, "a motor that answered the probe is online");
  zassert_equal(feedback.valid_mask, 0U, "nothing has been measured yet");
  zassert_true(feedback.stale, "there is no fresh measurement to report");
}

ZTEST(robstride_motor, test_a_probe_reply_does_not_make_an_old_measurement_look_fresh)
{
  struct motor_feedback feedback;
  uint8_t data[8] = {0};

  inject_feedback(TEST_MOTOR0_ID, 1000U, 0U, 0U, 0);
  zassert_ok(motor_get_feedback(motor0, &feedback));

  k_msleep(60);
  inject(TYPE_GET_ID, 0U, TEST_MOTOR0_ID, data);

  zassert_equal(
    motor_get_feedback(motor0, &feedback), -EAGAIN,
    "a presence reply says the motor is there, not that the position is current");
  zassert_true(feedback.stale, "feedback should still be stale");
}

ZTEST(robstride_motor, test_set_zero_makes_the_next_reading_seed_the_accumulator)
{
  struct motor_feedback feedback;

  /*
   * The accumulator stays congruent to the raw code until a full turn has gone
   * by, so it has to be carried past one before the two behaviours can be told
   * apart at all.
   */
  inject_feedback(TEST_MOTOR0_ID, 1000U, 0U, 0U, 0);
  inject_feedback(TEST_MOTOR0_ID, 30000U, 0U, 0U, 0);
  inject_feedback(TEST_MOTOR0_ID, 60000U, 0U, 0U, 0);
  inject_feedback(TEST_MOTOR0_ID, 20000U, 0U, 0U, 0);
  zassert_ok(motor_get_feedback(motor0, &feedback));
  zassert_not_equal(
    feedback.position, 20000, "the accumulator has not left the raw code behind");

  zassert_ok(robstride_set_zero(motor0));

  /* Accumulating instead would carry the whole turn across the new origin. */
  inject_feedback(TEST_MOTOR0_ID, 5000U, 0U, 0U, 0);
  zassert_ok(motor_get_feedback(motor0, &feedback));
  zassert_equal(
    feedback.position, 5000, "the accumulator must restart from the reading after the zero");
}

ZTEST(robstride_motor, test_a_reseeded_accumulator_reports_no_measurement_until_it_is_filled)
{
  struct motor_feedback feedback;
  struct robstride_feedback si;

  inject_feedback(TEST_MOTOR0_ID, 1000U, 0U, 0U, 0);
  zassert_ok(motor_get_feedback(motor0, &feedback));

  zassert_ok(robstride_set_zero(motor0));

  /* The timestamp is still inside the timeout, but there is nothing behind it
   * any more, so freshness alone must not make this look like a measurement. */
  zassert_equal(
    motor_get_feedback(motor0, &feedback), -EAGAIN, "the accumulator has been emptied");
  zassert_equal(feedback.valid_mask, 0U, "nothing has been measured since the zero");
  zassert_true(feedback.stale, "an emptied accumulator is not fresh");

  zassert_equal(robstride_get_feedback(motor0, &si), -EAGAIN, "the SI view must agree");
  zassert_true(si.stale, "an emptied accumulator is not fresh");
}

ZTEST(robstride_motor, test_feedback_decodes_and_accumulates_across_the_wrap)
{
  struct motor_feedback feedback;
  struct robstride_feedback si;

  /*
   * The accumulator carries over from whichever test ran before, so every
   * expectation below is relative to the reading taken here.
   */
  inject_feedback(TEST_MOTOR0_ID, 1000U, 0U, 0U, 0);
  zassert_ok(motor_get_feedback(motor0, &feedback));
  zassert_equal(
    feedback.valid_mask, MOTOR_FEEDBACK_POSITION | MOTOR_FEEDBACK_TEMPERATURE,
    "only position and temperature have a meaning shared with the other drivers");
  zassert_true(feedback.online, "feedback should be online");

  const int64_t seeded = feedback.position;

  /* A step that stays well inside the wrap window accumulates as-is. */
  inject_feedback(TEST_MOTOR0_ID, 1500U, 0U, 0U, 0);
  zassert_ok(motor_get_feedback(motor0, &feedback));
  zassert_equal(feedback.position, seeded + 500, "wrong accumulated position");

  /* Crossing +4 pi wraps the code back to the low end without moving 65535
   * counts backwards. */
  inject_feedback(TEST_MOTOR0_ID, 65000U, 0U, 0U, 0);
  zassert_ok(motor_get_feedback(motor0, &feedback));
  zassert_equal(feedback.position, seeded + 500 - 2035, "wrong position across the wrap");

  inject_feedback(TEST_MOTOR0_ID, 200U, 0U, 0U, 0);
  zassert_ok(motor_get_feedback(motor0, &feedback));
  zassert_equal(feedback.position, seeded + 500 - 2035 + 735, "wrong position across the wrap");

  /* The SI view reports the same accumulation in rad. */
  zassert_ok(robstride_get_feedback(motor0, &si));
  zassert_within(
    si.position,
    ((float)feedback.position * (2.0F * POSITION_MAX) / (float)UINT16_MAX) - POSITION_MAX, 0.001F,
    "the SI position must match the accumulated counts");
}

ZTEST(robstride_motor, test_feedback_decodes_velocity_torque_and_temperature)
{
  struct motor_feedback feedback;
  struct robstride_feedback si;

  /* Mid-code is the centre of each symmetric range. */
  inject_feedback(TEST_MOTOR0_ID, 32767U, 32767U, 32767U, 255);

  zassert_ok(robstride_get_feedback(motor0, &si));
  zassert_within(si.velocity, 0.0F, 0.01F, "mid-code is zero velocity");
  zassert_within(si.torque, 0.0F, 0.01F, "mid-code is zero torque");
  zassert_within(si.temperature, 25.5F, 0.01F, "temperature is reported in 0.1 degree steps");
  zassert_equal(si.run_state, ROBSTRIDE_RUN_STATE_RUNNING, "wrong run state");

  zassert_ok(motor_get_feedback(motor0, &feedback));
  zassert_equal(feedback.temperature, 25, "the class-wide field is whole degrees");
}

ZTEST(robstride_motor, test_feedback_goes_stale_after_the_timeout)
{
  struct motor_feedback feedback;

  inject_feedback(TEST_MOTOR0_ID, 1000U, 0U, 0U, 0);
  zassert_ok(motor_get_feedback(motor0, &feedback));
  zassert_false(feedback.stale, "feedback should be fresh");

  k_msleep(60);

  zassert_equal(motor_get_feedback(motor0, &feedback), -EAGAIN, "expected stale feedback");
  zassert_true(feedback.stale, "feedback should be stale");
}

ZTEST(robstride_motor, test_fault_report_makes_the_driver_configure_the_motor_again)
{
  uint8_t data[8] = {0};

  zassert_ok(robstride_set_current(motor0, 1.0F));
  enable_and_settle(motor0);
  /* A motor that trips was running, and the fault frame itself carries no
   * measurement, so the feedback below is what keeps the snapshot fresh. */
  inject_feedback(TEST_MOTOR0_ID, 1000U, 0U, 0U, 0);
  clear_captures();

  sys_put_le32(0x00000004U, &data[0]);
  sys_put_le32(0x00000001U, &data[4]);
  inject(TYPE_FAULT, 0U, TEST_MOTOR0_ID, data);

  struct robstride_feedback si;

  zassert_ok(robstride_get_feedback(motor0, &si));
  zassert_equal(si.fault_bits, 0x00000004U, "wrong fault bits");
  zassert_equal(si.warning_bits, 0x00000001U, "wrong warning bits");

  k_msleep(HANDSHAKE_MS);

  zassert_not_null(
    find_frame(TEST_MOTOR0_ID, TYPE_STOP, 0), "a tripped motor must be configured again");
  zassert_not_null(find_set_param(TEST_MOTOR0_ID, PARAM_RUN_MODE), "no run mode write");
  zassert_not_null(find_frame(TEST_MOTOR0_ID, TYPE_ENABLE, 0), "no re-enable");
}

ZTEST(robstride_motor, test_only_the_transition_out_of_running_re_enables_the_motor)
{
  zassert_ok(robstride_set_current(motor0, 1.0F));
  enable_and_settle(motor0);

  /* From here the motor answers no enable, so every one the driver sends is
   * visible and none of them brings the motor back to running on its own. */
  atomic_set(&sim_ignores_enable, 1);

  /* Put the motor in a known non-running state and let the driver settle on
   * it, so the assertions below do not depend on what an earlier test left. */
  inject_feedback_in_state(TEST_MOTOR0_ID, ROBSTRIDE_RUN_STATE_RESET, 100U, 0U, 0U, 0);
  k_msleep(HANDSHAKE_MS);
  clear_captures();

  /* Staying out of running is not a fresh loss of the enable we sent, and a
   * motor still calibrating past the running timeout is still on its way. */
  inject_feedback_in_state(TEST_MOTOR0_ID, ROBSTRIDE_RUN_STATE_CALIBRATION, 110U, 0U, 0U, 0);
  inject_feedback_in_state(TEST_MOTOR0_ID, ROBSTRIDE_RUN_STATE_CALIBRATION, 120U, 0U, 0U, 0);
  k_msleep(HANDSHAKE_MS);
  zassert_is_null(
    find_frame(TEST_MOTOR0_ID, TYPE_ENABLE, 0),
    "a motor that has not reached running must not be enabled again every interval");
  zassert_is_null(
    find_frame(TEST_MOTOR0_ID, TYPE_STOP, 0), "a calibrating motor has not failed to run");

  /* Dropping out of running is. */
  inject_feedback_in_state(TEST_MOTOR0_ID, ROBSTRIDE_RUN_STATE_RUNNING, 130U, 0U, 0U, 0);
  clear_captures();
  inject_feedback_in_state(TEST_MOTOR0_ID, ROBSTRIDE_RUN_STATE_RESET, 140U, 0U, 0U, 0);
  k_msleep(HANDSHAKE_MS);
  zassert_not_null(
    find_frame(TEST_MOTOR0_ID, TYPE_ENABLE, 0),
    "a motor that left running has dropped the enable and needs it again");
  zassert_is_null(find_frame(TEST_MOTOR0_ID, TYPE_ENABLE, 1), "one enable is enough to recover");
  zassert_is_null(
    find_set_param(TEST_MOTOR0_ID, PARAM_RUN_MODE),
    "losing the enable is not losing the parameters");
}

ZTEST(robstride_motor, test_disabled_motor_is_probed_rather_than_commanded)
{
  /* before_each leaves both motors disabled. */
  k_msleep(HANDSHAKE_MS);

  zassert_is_null(
    find_frame(TEST_MOTOR0_ID, TYPE_ENABLE, 0), "a disabled motor must not be enabled");
  zassert_is_null(
    find_set_param(TEST_MOTOR0_ID, PARAM_IQ_REF), "a disabled motor must not be commanded");

  /* The probe is slow, so it may not have come round yet; when it does it must
   * be a presence request and nothing else. */
  for (size_t i = 0; i < MIN(captured_count, (size_t)CAPTURE_MAX); ++i) {
    zassert_true(
      (frame_type(&captured[i]) == TYPE_GET_ID) || (frame_type(&captured[i]) == TYPE_STOP),
      "unexpected frame type %u while disabled", frame_type(&captured[i]));
  }
}

ZTEST(robstride_motor, test_disable_stops_the_motor_without_waiting_for_the_interval)
{
  zassert_ok(robstride_set_current(motor0, 1.0F));
  enable_and_settle(motor0);
  clear_captures();

  zassert_ok(motor_disable(motor0));

  zassert_not_null(
    find_frame(TEST_MOTOR0_ID, TYPE_STOP, 0), "disable must send the stop frame itself");
}

ZTEST(robstride_motor, test_disable_reports_a_stop_it_could_not_queue_and_sends_it_later)
{
  zassert_ok(robstride_set_current(motor0, 1.0F));
  enable_and_settle(motor0);
  clear_captures();

  atomic_set(&send_fails, 1);

  zassert_not_equal(
    motor_disable(motor0), 0,
    "a motor still holding its last target must not be reported as stopped");

  atomic_set(&send_fails, 0);
  k_msleep(HANDSHAKE_MS);

  zassert_not_null(
    find_frame(TEST_MOTOR0_ID, TYPE_STOP, 0), "the stop owed to the motor must be sent again");
  zassert_is_null(
    find_set_param(TEST_MOTOR0_ID, PARAM_IQ_REF), "a disabled motor must not be commanded");
}

static K_THREAD_STACK_DEFINE(sender_stack, READER_STACK_SIZE);
static struct k_thread sender_thread;
static int sender_result;

static void sender_entry(void * a, void * b, void * c)
{
  ARG_UNUSED(a);
  ARG_UNUSED(b);
  ARG_UNUSED(c);

  sender_result = robstride_set_zero(motor0);
}

/*
 * Runs on its own thread so that a driver that waits for the completion fails
 * the join instead of hanging the whole suite.
 */
ZTEST(robstride_motor, test_a_send_does_not_wait_for_the_frame_to_reach_the_wire)
{
  atomic_set(&send_swallows_completion, 1);
  sender_result = -EINPROGRESS;

  k_thread_create(
    &sender_thread, sender_stack, READER_STACK_SIZE, sender_entry, NULL, NULL, NULL,
    READER_PRIORITY, 0, K_NO_WAIT);

  zassert_ok(
    k_thread_join(&sender_thread, K_MSEC(200)),
    "the send is still waiting for a completion the controller never reports");
  zassert_ok(sender_result, "queueing succeeded, so the call must report success");
}

ZTEST(robstride_motor, test_a_late_failure_of_a_write_awaiting_readback_writes_it_again_at_once)
{
  zassert_ok(robstride_set_current(motor0, 1.0F));

  /* Nothing is read back, so only the failure can bring the write back
   * before the readback timeout would. */
  atomic_set(&sim_silent, 1);
  atomic_set(&send_defers_completion, 1);
  zassert_ok(motor_enable(motor0));
  k_msleep((CONFIG_MOTOR_ROBSTRIDE_TX_INTERVAL_MS * 2) + 2);
  atomic_set(&send_defers_completion, 0);
  clear_captures();

  /* Only now does the controller admit the run mode never reached the wire. */
  release_deferred_completions(-EIO);
  k_msleep(CONFIG_MOTOR_ROBSTRIDE_TX_INTERVAL_MS + 2);

  zassert_not_null(
    find_set_param(TEST_MOTOR0_ID, PARAM_RUN_MODE),
    "the failure has to undo the write it belonged to");
  zassert_not_null(
    find_frame(TEST_MOTOR0_ID, TYPE_STOP, 0), "the run mode is only accepted while stopped");
}

ZTEST(robstride_motor, test_a_late_failure_of_a_write_the_readback_confirmed_changes_nothing)
{
  Z_TEST_SKIP_IFNDEF(CONFIG_MOTOR_ROBSTRIDE_VERIFY);

  zassert_ok(robstride_set_current(motor0, 1.0F));

  /* The motor takes the first batch, but its completions are held back. */
  atomic_set(&send_defers_completion, 1);
  zassert_ok(motor_enable(motor0));
  k_msleep(CONFIG_MOTOR_ROBSTRIDE_TX_INTERVAL_MS * 2);
  atomic_set(&send_defers_completion, 0);

  /* Let the handshake finish, so the run mode has long been read back. */
  k_msleep(HANDSHAKE_MS);
  clear_captures();

  release_deferred_completions(-EIO);
  k_msleep(HANDSHAKE_MS);

  zassert_is_null(
    find_frame(TEST_MOTOR0_ID, TYPE_STOP, 0),
    "a motor shown to hold the run mode must not be stopped to write it again");
  zassert_not_null(
    find_set_param(TEST_MOTOR0_ID, PARAM_IQ_REF), "the motor must keep being commanded");
}

ZTEST(robstride_motor, test_a_stage_that_never_reached_the_wire_is_built_again)
{
  zassert_ok(robstride_set_current(motor0, 2.0F));

  /* The controller takes every frame and then reports that none of them was
   * transmitted, so the handshake must not walk forward on the queueing alone. */
  atomic_set(&send_completes_with_error, 1);
  zassert_ok(motor_enable(motor0));
  k_msleep(HANDSHAKE_MS);
  clear_captures();
  k_msleep(HANDSHAKE_MS);

  zassert_not_null(
    find_frame(TEST_MOTOR0_ID, TYPE_STOP, 0), "the first stage must keep being retried");
  zassert_is_null(
    find_frame(TEST_MOTOR0_ID, TYPE_ENABLE, 0),
    "no stage can be counted done while nothing reaches the wire");

  atomic_set(&send_completes_with_error, 0);
  clear_captures();
  k_msleep(HANDSHAKE_MS * 2);

  zassert_not_null(find_frame(TEST_MOTOR0_ID, TYPE_ENABLE, 0), "no enable once the bus recovered");
  zassert_not_null(find_set_param(TEST_MOTOR0_ID, PARAM_IQ_REF), "no target once the bus recovered");
}

ZTEST(robstride_motor, test_a_handshake_stage_the_bus_refused_is_built_again)
{
  zassert_ok(robstride_set_current(motor0, 2.0F));

  /* A controller whose transceiver has no power takes nothing, so the whole
   * handshake has to survive being refused rather than counting itself done. */
  atomic_set(&send_fails, 1);
  zassert_ok(motor_enable(motor0));
  k_msleep(HANDSHAKE_MS);

  zassert_equal(captured_count, 0U, "nothing can have reached a bus that refuses every frame");

  atomic_set(&send_fails, 0);
  k_msleep(HANDSHAKE_MS);

  const struct can_frame * stop = find_frame(TEST_MOTOR0_ID, TYPE_STOP, 0);
  const struct can_frame * run_mode = find_set_param(TEST_MOTOR0_ID, PARAM_RUN_MODE);
  const struct can_frame * enable = find_frame(TEST_MOTOR0_ID, TYPE_ENABLE, 0);

  zassert_not_null(stop, "no stop frame after the bus came back");
  zassert_not_null(run_mode, "no run mode write after the bus came back");
  zassert_not_null(find_set_param(TEST_MOTOR0_ID, PARAM_LIMIT_CUR), "no current limit write");
  zassert_not_null(find_set_param(TEST_MOTOR0_ID, PARAM_LIMIT_SPD), "no velocity limit write");
  zassert_not_null(find_set_param(TEST_MOTOR0_ID, PARAM_LIMIT_TORQUE), "no torque limit write");
  zassert_not_null(enable, "no enable frame after the bus came back");
  zassert_not_null(find_set_param(TEST_MOTOR0_ID, PARAM_IQ_REF), "no target frame");
  zassert_true(stop < run_mode, "the stages must keep their order on the retry");
  zassert_true(run_mode < enable, "the stages must keep their order on the retry");
}

static void param_reply_work_handler(struct k_work * work)
{
  uint8_t data[8] = {0};
  const float value = 4.5F;
  uint32_t bits;

  ARG_UNUSED(work);

  memcpy(&bits, &value, sizeof(bits));
  sys_put_le16(PARAM_LIMIT_TORQUE, &data[0]);
  sys_put_le32(bits, &data[4]);

  inject(TYPE_GET_PARAM, 0U, TEST_MOTOR0_ID, data);
}

static K_WORK_DELAYABLE_DEFINE(param_reply_work, param_reply_work_handler);

ZTEST(robstride_motor, test_parameter_read_completes_on_the_matching_reply)
{
  float value = 0.0F;

  atomic_set(&sim_silent, 1);
  k_work_schedule(&param_reply_work, K_MSEC(5));

  zassert_ok(robstride_get_parameter(motor0, PARAM_LIMIT_TORQUE, &value));
  zassert_within(value, 4.5F, 0.001F, "wrong parameter value");

  const struct can_frame * request = find_frame(TEST_MOTOR0_ID, TYPE_GET_PARAM, 0);

  zassert_not_null(request, "no parameter request went out");
  zassert_equal(frame_param_index(request), PARAM_LIMIT_TORQUE, "wrong parameter index");
}

ZTEST(robstride_motor, test_parameter_read_times_out_without_a_reply)
{
  float value = 0.0F;

  atomic_set(&sim_silent, 1);
  zassert_equal(
    robstride_get_parameter(motor0, PARAM_LIMIT_SPD, &value), -ETIMEDOUT,
    "a read nobody answers must time out");
}

static K_THREAD_STACK_DEFINE(reader_stack, READER_STACK_SIZE);
static struct k_thread reader_thread;
static int reader_result;
static float reader_value;

static void reader_entry(void * a, void * b, void * c)
{
  ARG_UNUSED(a);
  ARG_UNUSED(b);
  ARG_UNUSED(c);

  reader_result = robstride_get_parameter(motor0, PARAM_LIMIT_TORQUE, &reader_value);
}

ZTEST(robstride_motor, test_a_read_holds_the_motor_until_the_caller_that_made_it_returns)
{
  uint8_t data[8] = {0};
  const float answer = 4.5F;
  uint32_t bits;
  float value = 0.0F;

  memcpy(&bits, &answer, sizeof(bits));
  sys_put_le16(PARAM_LIMIT_TORQUE, &data[0]);
  sys_put_le32(bits, &data[4]);

  atomic_set(&sim_silent, 1);
  reader_result = -EINPROGRESS;
  k_thread_create(
    &reader_thread, reader_stack, READER_STACK_SIZE, reader_entry, NULL, NULL, NULL,
    READER_PRIORITY, 0, K_NO_WAIT);

  /* Let the reader claim the slot and block on the reply. */
  k_msleep(5);
  zassert_equal(
    robstride_get_parameter(motor0, PARAM_LIMIT_SPD, &value), -EBUSY,
    "a second read must be refused while one is in flight");

  /*
   * Answering makes the reader runnable but does not run it, because it is the
   * lower priority. The slot has to stay taken across this gap: releasing it
   * here would let the call below overwrite the reply the reader has not read.
   */
  inject(TYPE_GET_PARAM, 0U, TEST_MOTOR0_ID, data);
  zassert_equal(
    robstride_get_parameter(motor0, PARAM_LIMIT_SPD, &value), -EBUSY,
    "the reply belongs to the reader until it has taken it");

  zassert_ok(k_thread_join(&reader_thread, K_MSEC(500)), "the reader never finished");
  zassert_ok(reader_result, "the reader must get its answer");
  zassert_within(reader_value, 4.5F, 0.001F, "the reader got the wrong value");

  /* And the slot is free again once it has. */
  zassert_equal(
    robstride_get_parameter(motor0, PARAM_LIMIT_SPD, &value), -ETIMEDOUT,
    "the slot must be released when the read returns");
}

ZTEST(robstride_motor, test_an_integer_parameter_rejects_a_value_it_cannot_carry)
{
  clear_captures();

  zassert_equal(
    robstride_set_parameter(motor0, PARAM_RUN_MODE, -1.0F), -EINVAL,
    "a negative value has no uint32 representation");
  zassert_equal(
    robstride_set_parameter(motor0, PARAM_RUN_MODE, 5.0e9F), -EINVAL,
    "a value past uint32 has no representation");
  zassert_is_null(
    find_set_param(TEST_MOTOR0_ID, PARAM_RUN_MODE), "a rejected write must not reach the bus");

  /* A float-payload index carries the same values without complaint. */
  zassert_ok(robstride_set_parameter(motor0, PARAM_LIMIT_TORQUE, -1.0F));
}

ZTEST(robstride_motor, test_calls_reject_a_device_that_is_not_a_robstride_motor)
{
  struct robstride_feedback si;

  zassert_equal(robstride_set_current(test_can, 1.0F), -EINVAL);
  zassert_equal(robstride_get_feedback(test_can, &si), -EINVAL);
  zassert_equal(robstride_set_motion_target(motor0, NULL), -EINVAL);
  zassert_equal(robstride_get_feedback(motor0, NULL), -EINVAL);
}

ZTEST(robstride_motor, test_mode_params_reject_what_cannot_be_written)
{
  const struct robstride_mode_params good = {.current_kp = 0.1F};
  struct robstride_mode_params bad = good;

  zassert_equal(robstride_set_mode_params(test_can, ROBSTRIDE_MODE_CURRENT, &good), -EINVAL);
  zassert_equal(robstride_set_mode_params(motor3, ROBSTRIDE_MODE_CURRENT, NULL), -EINVAL);
  zassert_equal(
    robstride_set_mode_params(motor3, (enum robstride_mode)ROBSTRIDE_MODE_COUNT, &good), -EINVAL,
    "not a mode");

  bad.velocity_ki = -0.1F;
  zassert_equal(
    robstride_set_mode_params(motor3, ROBSTRIDE_MODE_CURRENT, &bad), -EINVAL,
    "a negative gain");

  bad.velocity_ki = NAN;
  zassert_equal(
    robstride_set_mode_params(motor3, ROBSTRIDE_MODE_CURRENT, &bad), -EINVAL, "not a number");
}

/* Every SET_PARAM to motor1 since the captures were cleared, as a mask over
 * the indices below, so a test can name what it expects and what it does not
 * in one comparison. */
static const uint16_t mode_param_indices[] = {
  PARAM_VEL_MAX, PARAM_ACC_SET, PARAM_ACC_RAD, PARAM_LOC_KP,
  PARAM_SPD_KP,  PARAM_SPD_KI,  PARAM_CUR_KP,  PARAM_CUR_KI,
};

#define MP_VEL_MAX BIT(0)
#define MP_ACC_SET BIT(1)
#define MP_ACC_RAD BIT(2)
#define MP_LOC_KP BIT(3)
#define MP_SPD_KP BIT(4)
#define MP_SPD_KI BIT(5)
#define MP_CUR_KP BIT(6)
#define MP_CUR_KI BIT(7)
#define MP_CURRENT_LOOP (MP_CUR_KP | MP_CUR_KI)
#define MP_SPEED_LOOP (MP_SPD_KP | MP_SPD_KI | MP_CURRENT_LOOP)

static uint32_t mode_params_written(uint8_t target)
{
  uint32_t mask = 0;

  for (size_t i = 0; i < ARRAY_SIZE(mode_param_indices); ++i) {
    if (find_set_param(target, mode_param_indices[i]) != NULL) {
      mask |= BIT(i);
    }
  }

  return mask;
}

static void switch_and_settle(void (*select)(void))
{
  zassert_ok(motor_disable(motor1));
  select();
  clear_captures();
  enable_and_settle(motor1);
}

static void select_operation(void)
{
  const struct robstride_motion_target target = {0};

  zassert_ok(robstride_set_motion_target(motor1, &target));
}

static void select_position(void)
{
  zassert_ok(robstride_set_position(motor1, 0.5F));
}

static void select_position_csp(void)
{
  zassert_ok(robstride_set_position_csp(motor1, 0.5F));
}

static void select_velocity(void)
{
  zassert_ok(robstride_set_velocity(motor1, 1.0F));
}

static void select_current(void)
{
  zassert_ok(robstride_set_current(motor1, 1.0F));
}

/* The table robstride_actuator_bridge_ros2 configures each mode from. */
ZTEST(robstride_motor, test_each_mode_writes_its_own_parameters)
{
  const struct robstride_mode_params params = {
    .max_velocity = 10.0F,
    .acceleration = 20.0F,
    .position_kp = 30.0F,
    .velocity_kp = 2.0F,
    .velocity_ki = 0.02F,
    .current_kp = 0.125F,
    .current_ki = 0.0158F,
  };

  for (int mode = 0; mode < ROBSTRIDE_MODE_COUNT; ++mode) {
    zassert_ok(robstride_set_mode_params(motor1, (enum robstride_mode)mode, &params));
  }

  switch_and_settle(select_position);
  zassert_equal(
    mode_params_written(TEST_MOTOR1_ID),
    MP_VEL_MAX | MP_ACC_SET | MP_LOC_KP | MP_SPEED_LOOP, "position (PP)");
  zassert_within(
    frame_param_float(find_set_param(TEST_MOTOR1_ID, PARAM_VEL_MAX)), 10.0F, 0.001F,
    "wrong trajectory velocity");
  zassert_within(
    frame_param_float(find_set_param(TEST_MOTOR1_ID, PARAM_ACC_SET)), 20.0F, 0.001F,
    "wrong trajectory acceleration");
  zassert_within(
    frame_param_float(find_set_param(TEST_MOTOR1_ID, PARAM_LIMIT_SPD)), RS05_VELOCITY_MAX,
    0.001F, "PP keeps the velocity limit as it is");

  switch_and_settle(select_position_csp);
  zassert_equal(mode_params_written(TEST_MOTOR1_ID), MP_LOC_KP | MP_SPEED_LOOP, "position (CSP)");
  zassert_within(
    frame_param_float(find_set_param(TEST_MOTOR1_ID, PARAM_LIMIT_SPD)), 10.0F, 0.001F,
    "CSP caps velocity through the limit");

  switch_and_settle(select_velocity);
  zassert_equal(mode_params_written(TEST_MOTOR1_ID), MP_ACC_RAD | MP_SPEED_LOOP, "velocity");
  zassert_within(
    frame_param_float(find_set_param(TEST_MOTOR1_ID, PARAM_ACC_RAD)), 20.0F, 0.001F,
    "wrong velocity ramp");

  switch_and_settle(select_current);
  zassert_equal(mode_params_written(TEST_MOTOR1_ID), MP_CURRENT_LOOP, "current");

  switch_and_settle(select_operation);
  zassert_equal(mode_params_written(TEST_MOTOR1_ID), MP_CURRENT_LOOP, "operation");
  zassert_not_null(
    find_frame(TEST_MOTOR1_ID, TYPE_OP_CONTROL, 0), "operation control must follow");
}

ZTEST(robstride_motor, test_csp_never_raises_the_velocity_limit_above_the_one_set)
{
  struct robstride_limits limits;
  const struct robstride_mode_params params = {.max_velocity = 10.0F};

  zassert_ok(robstride_get_limits(motor2, &limits));
  limits.velocity = 8.0F;
  zassert_ok(robstride_set_limits(motor2, &limits));
  zassert_ok(robstride_set_mode_params(motor2, ROBSTRIDE_MODE_POSITION_CSP, &params));
  zassert_ok(robstride_set_position_csp(motor2, 0.0F));
  enable_and_settle(motor2);

  zassert_within(
    frame_param_float(find_set_param(TEST_MOTOR2_ID, PARAM_LIMIT_SPD)), 8.0F, 0.001F,
    "the CSP velocity cap may narrow the limit, never widen it");

  /* Back to the devicetree's limit, which the other motor2 tests expect. */
  limits.velocity = RS05_VELOCITY_MAX;
  zassert_ok(robstride_set_limits(motor2, &limits));
  zassert_ok(motor_disable(motor2));
  zassert_ok(robstride_set_current(motor2, 0.0F));
}

/* Velocity rather than current, so motor0's current mode is left to the test
 * that needs it never to have been given values. */
ZTEST(robstride_motor, test_changing_the_current_modes_params_rewrites_them_without_stopping)
{
  struct robstride_mode_params params = {.acceleration = 10.0F, .velocity_kp = 1.0F};

  zassert_ok(robstride_set_mode_params(motor0, ROBSTRIDE_MODE_VELOCITY, &params));
  zassert_ok(robstride_set_velocity(motor0, 1.0F));
  enable_and_settle(motor0);
  clear_captures();

  params.velocity_kp = 2.0F;
  zassert_ok(robstride_set_mode_params(motor0, ROBSTRIDE_MODE_VELOCITY, &params));
  k_msleep(HANDSHAKE_MS);

  const struct can_frame * spd_kp = find_set_param(TEST_MOTOR0_ID, PARAM_SPD_KP);

  zassert_not_null(spd_kp, "the new gain was not written");
  zassert_within(frame_param_float(spd_kp), 2.0F, 0.001F, "wrong velocity loop gain");
  zassert_is_null(find_frame(TEST_MOTOR0_ID, TYPE_STOP, 0), "retuning must not stop the motor");
  zassert_is_null(find_frame(TEST_MOTOR0_ID, TYPE_ENABLE, 0), "nor enable it again");

  const struct can_frame * command = find_param_frame(
    TEST_MOTOR0_ID, TYPE_SET_PARAM, PARAM_SPD_REF,
    count_set_param(TEST_MOTOR0_ID, PARAM_SPD_REF) - 1);

  zassert_not_null(command, "commands stopped for good");
  zassert_true(command > spd_kp, "commands must resume after the rewrite");
}

ZTEST(robstride_motor, test_a_value_read_back_wrong_is_written_again_and_holds_the_enable)
{
  Z_TEST_SKIP_IFNDEF(CONFIG_MOTOR_ROBSTRIDE_VERIFY);

  atomic_set(&sim_skewed_index, PARAM_LIMIT_CUR);
  zassert_ok(robstride_set_current(motor0, 1.0F));
  enable_and_settle(motor0);

  zassert_true(
    count_set_param(TEST_MOTOR0_ID, PARAM_LIMIT_CUR) >= 2,
    "a limit the motor reports differently must be written again");
  zassert_is_null(
    find_frame(TEST_MOTOR0_ID, TYPE_ENABLE, 0),
    "a motor not shown to hold its limits must not be enabled");

  atomic_set(&sim_skewed_index, 0);
  k_msleep(CONFIG_MOTOR_ROBSTRIDE_CONFIGURE_BACKOFF_MS + HANDSHAKE_MS);

  zassert_not_null(
    find_frame(TEST_MOTOR0_ID, TYPE_ENABLE, 0), "the motor must come up once it agrees");
}

ZTEST(robstride_motor, test_an_unanswered_batch_is_given_up_after_the_retries_and_started_over)
{
  Z_TEST_SKIP_IFNDEF(CONFIG_MOTOR_ROBSTRIDE_VERIFY);

  /* The timeout and retry options only exist with readback on. */
#ifdef CONFIG_MOTOR_ROBSTRIDE_VERIFY
  struct robstride_feedback si;

  (void)robstride_get_feedback(motor0, &si);

  const uint32_t failures = si.configure_failures;

  atomic_set(&sim_silent, 1);
  zassert_ok(robstride_set_current(motor0, 1.0F));
  zassert_ok(motor_enable(motor0));

  /* Past every retry, inside the pause that follows them. */
  k_msleep(
    (CONFIG_MOTOR_ROBSTRIDE_VERIFY_TIMEOUT_MS + CONFIG_MOTOR_ROBSTRIDE_TX_INTERVAL_MS) *
      (CONFIG_MOTOR_ROBSTRIDE_VERIFY_RETRIES + 1));

  zassert_equal(
    count_set_param(TEST_MOTOR0_ID, PARAM_RUN_MODE), CONFIG_MOTOR_ROBSTRIDE_VERIFY_RETRIES,
    "the run mode must be written once per attempt and no more");
  (void)robstride_get_feedback(motor0, &si);
  zassert_equal(si.configure_failures, failures + 1U, "the give-up must be counted");
  zassert_false(si.configured, "a motor that never confirmed is not configured");

  clear_captures();
  k_msleep(CONFIG_MOTOR_ROBSTRIDE_CONFIGURE_BACKOFF_MS);

  zassert_not_null(
    find_frame(TEST_MOTOR0_ID, TYPE_STOP, 0), "configuring must start over after the pause");
#endif
}

ZTEST(robstride_motor, test_commands_wait_for_the_motor_to_report_running)
{
  struct robstride_feedback si;

  atomic_set(&sim_ignores_enable, 1);
  zassert_ok(robstride_set_current(motor0, 1.0F));
  enable_and_settle(motor0);

  zassert_not_null(find_frame(TEST_MOTOR0_ID, TYPE_ENABLE, 0), "no enable frame");
  zassert_equal(
    count_set_param(TEST_MOTOR0_ID, PARAM_IQ_REF), 1U,
    "only the target ahead of the enable may go out before the motor runs");
  (void)robstride_get_feedback(motor0, &si);
  zassert_false(si.configured, "the motor has not reported running");

  inject_feedback(TEST_MOTOR0_ID, 1000U, 32767U, 32767U, 250);
  k_msleep(CONFIG_MOTOR_ROBSTRIDE_TX_INTERVAL_MS * 2);

  zassert_true(
    count_set_param(TEST_MOTOR0_ID, PARAM_IQ_REF) > 1U, "commands must start once it runs");
  zassert_ok(robstride_get_feedback(motor0, &si));
  zassert_true(si.configured, "a running motor holding everything is configured");
}

ZTEST(robstride_motor, test_a_motor_that_does_not_run_after_the_enable_is_configured_again)
{
  struct robstride_feedback si;

  (void)robstride_get_feedback(motor0, &si);

  const uint32_t failures = si.configure_failures;

  atomic_set(&sim_ignores_enable, 1);
  zassert_ok(robstride_set_current(motor0, 1.0F));
  enable_and_settle(motor0);
  clear_captures();

  k_msleep(CONFIG_MOTOR_ROBSTRIDE_RUNNING_TIMEOUT_MS + CONFIG_MOTOR_ROBSTRIDE_CONFIGURE_BACKOFF_MS);

  (void)robstride_get_feedback(motor0, &si);
  zassert_equal(si.configure_failures, failures + 1U, "the give-up must be counted");
  zassert_not_null(
    find_frame(TEST_MOTOR0_ID, TYPE_STOP, 0), "configuring must start over from the stop");
}

static K_THREAD_STACK_DEFINE(shared_reader_stack, READER_STACK_SIZE);
static struct k_thread shared_reader_thread;
static int shared_reader_result;
static float shared_reader_value;

static void shared_reader_entry(void * a, void * b, void * c)
{
  ARG_UNUSED(a);
  ARG_UNUSED(b);
  ARG_UNUSED(c);

  shared_reader_result = robstride_get_parameter(motor0, PARAM_RUN_MODE, &shared_reader_value);
}

ZTEST(robstride_motor, test_one_reply_answers_both_an_api_read_and_the_readback)
{
  Z_TEST_SKIP_IFNDEF(CONFIG_MOTOR_ROBSTRIDE_VERIFY);

  uint8_t data[8] = {0};

  /* Nothing answers on its own, so the handshake sits on the run mode until
   * the one reply below. */
  atomic_set(&sim_silent, 1);
  zassert_ok(robstride_set_current(motor0, 1.0F));
  zassert_ok(motor_enable(motor0));
  k_msleep((CONFIG_MOTOR_ROBSTRIDE_TX_INTERVAL_MS * 2) + 2);

  shared_reader_result = -EINPROGRESS;
  k_thread_create(
    &shared_reader_thread, shared_reader_stack, READER_STACK_SIZE, shared_reader_entry, NULL,
    NULL, NULL, READER_PRIORITY, 0, K_NO_WAIT);
  k_msleep(1);
  clear_captures();

  sys_put_le16(PARAM_RUN_MODE, &data[0]);
  sys_put_le32(3U, &data[4]);
  inject(TYPE_GET_PARAM, 0U, TEST_MOTOR0_ID, data);

  zassert_ok(k_thread_join(&shared_reader_thread, K_MSEC(500)), "the reader never finished");
  zassert_ok(shared_reader_result, "the API read must get the reply");
  zassert_within(shared_reader_value, 3.0F, 0.001F, "current mode is run mode 3");

  k_msleep(CONFIG_MOTOR_ROBSTRIDE_TX_INTERVAL_MS * 2);
  zassert_not_null(
    find_set_param(TEST_MOTOR0_ID, PARAM_LIMIT_CUR),
    "the same reply must confirm the run mode and let the handshake go on");
}
