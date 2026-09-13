/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can/can_fake.h>
#include <zephyr/fff.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#include <drivers/motor.h>
#include <drivers/motor/robstride.h>

#define TEST_MASTER_ID 0xFDU
#define TEST_MOTOR0_ID 0x7FU
#define TEST_MOTOR1_ID 0x01U

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

#define RS00_VELOCITY_MAX 33.0F
#define RS00_TORQUE_MAX 14.0F
#define POSITION_MAX 12.56637F

/* Long enough for the driver to work through every handshake stage. */
#define HANDSHAKE_MS (CONFIG_MOTOR_ROBSTRIDE_TX_INTERVAL_MS * 8)

static const struct device * const test_can = DEVICE_DT_GET(DT_NODELABEL(test_can0));
static const struct device * const motor0 = DEVICE_DT_GET(DT_NODELABEL(motor0));
static const struct device * const motor1 = DEVICE_DT_GET(DT_NODELABEL(motor1));

#define CAPTURE_MAX 32

static struct can_frame captured[CAPTURE_MAX];
static size_t captured_count;
static can_rx_callback_t rx_callback;
static void * rx_user_data;
static struct can_filter rx_filter;

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

static int test_fake_can_send(
  const struct device * dev, const struct can_frame * frame, k_timeout_t timeout,
  can_tx_callback_t callback, void * user_data)
{
  ARG_UNUSED(timeout);

  if (captured_count < CAPTURE_MAX) {
    captured[captured_count] = *frame;
  }

  captured_count++;

  if (callback != NULL) {
    callback(dev, 0, user_data);
  }

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

static const struct can_frame * find_set_param(uint8_t target, uint16_t index)
{
  for (size_t i = 0; i < MIN(captured_count, (size_t)CAPTURE_MAX); ++i) {
    if ((frame_target(&captured[i]) != target) || (frame_type(&captured[i]) != TYPE_SET_PARAM)) {
      continue;
    }

    if (frame_param_index(&captured[i]) == index) {
      return &captured[i];
    }
  }

  return NULL;
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

  (void)motor_disable(motor0);
  (void)motor_disable(motor1);
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

ZTEST(robstride_motor, test_handshake_writes_limits_then_enables_then_commands)
{
  zassert_ok(robstride_set_current(motor0, 2.0F));
  enable_and_settle(motor0);

  const struct can_frame * limit_cur = find_set_param(TEST_MOTOR0_ID, PARAM_LIMIT_CUR);
  const struct can_frame * limit_spd = find_set_param(TEST_MOTOR0_ID, PARAM_LIMIT_SPD);
  const struct can_frame * limit_torque = find_set_param(TEST_MOTOR0_ID, PARAM_LIMIT_TORQUE);
  const struct can_frame * enable = find_frame(TEST_MOTOR0_ID, TYPE_ENABLE, 0);
  const struct can_frame * command = find_set_param(TEST_MOTOR0_ID, PARAM_IQ_REF);

  zassert_not_null(limit_cur, "no current limit write");
  zassert_not_null(limit_spd, "no velocity limit write");
  zassert_not_null(limit_torque, "no torque limit write");
  zassert_not_null(enable, "no enable frame");
  zassert_not_null(command, "no target frame");

  zassert_true(limit_torque < enable, "limits must be written before the motor is enabled");
  zassert_true(enable < command, "the target must not be sent before the motor is enabled");

  zassert_within(
    frame_param_float(limit_cur), 15.5F, 0.001F, "rs00 defaults to the model current limit");
  zassert_within(frame_param_float(command), 2.0F, 0.001F, "wrong current target");
}

ZTEST(robstride_motor, test_gains_are_written_only_after_the_application_sets_them)
{
  zassert_ok(robstride_set_current(motor0, 1.0F));
  enable_and_settle(motor0);

  zassert_is_null(
    find_set_param(TEST_MOTOR0_ID, PARAM_LOC_KP),
    "a motor keeps its stored gains until the application overrides them");

  const struct robstride_gains gains = {
    .position_kp = 30.0F,
    .velocity_kp = 2.0F,
    .velocity_ki = 0.02F,
    .current_kp = 0.05F,
    .current_ki = 0.01F,
  };

  clear_captures();
  zassert_ok(robstride_set_gains(motor0, &gains));
  k_msleep(HANDSHAKE_MS);

  const struct can_frame * loc_kp = find_set_param(TEST_MOTOR0_ID, PARAM_LOC_KP);
  const struct can_frame * cur_ki = find_set_param(TEST_MOTOR0_ID, PARAM_CUR_KI);

  zassert_not_null(loc_kp, "no position gain write");
  zassert_not_null(find_set_param(TEST_MOTOR0_ID, PARAM_SPD_KP), "no velocity gain write");
  zassert_not_null(find_set_param(TEST_MOTOR0_ID, PARAM_SPD_KI), "no velocity integral write");
  zassert_not_null(find_set_param(TEST_MOTOR0_ID, PARAM_CUR_KP), "no current gain write");
  zassert_not_null(cur_ki, "every gain in the struct must reach the motor");
  zassert_within(frame_param_float(loc_kp), 30.0F, 0.001F, "wrong position gain");
  zassert_within(frame_param_float(cur_ki), 0.01F, 0.001F, "wrong current integral gain");
}

ZTEST(robstride_motor, test_devicetree_limit_overrides_the_model_limit)
{
  zassert_ok(robstride_set_current(motor1, 1.0F));
  enable_and_settle(motor1);

  const struct can_frame * limit_cur = find_set_param(TEST_MOTOR1_ID, PARAM_LIMIT_CUR);

  zassert_not_null(limit_cur, "no current limit write");
  zassert_within(frame_param_float(limit_cur), 5.0F, 0.001F, "max-current-ma must win");
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

  /* motor1 is only addressed by the tests that enable it, and none of them
   * inject feedback for it. */
  zassert_equal(robstride_get_feedback(motor1, &si), -ENODATA, "nothing has answered yet");
  zassert_false(si.online, "a motor that never answered is not online");
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

  /* Put the motor in a known non-running state and let the driver settle on
   * it, so the assertions below do not depend on what an earlier test left. */
  inject_feedback_in_state(TEST_MOTOR0_ID, ROBSTRIDE_RUN_STATE_RESET, 100U, 0U, 0U, 0);
  k_msleep(HANDSHAKE_MS);
  clear_captures();

  /* Staying out of running is not a fresh loss of the enable we sent. */
  inject_feedback_in_state(TEST_MOTOR0_ID, ROBSTRIDE_RUN_STATE_CALIBRATION, 110U, 0U, 0U, 0);
  inject_feedback_in_state(TEST_MOTOR0_ID, ROBSTRIDE_RUN_STATE_CALIBRATION, 120U, 0U, 0U, 0);
  k_msleep(HANDSHAKE_MS);
  zassert_is_null(
    find_frame(TEST_MOTOR0_ID, TYPE_ENABLE, 0),
    "a motor that has not reached running must not be enabled again every interval");

  /* Dropping out of running is. */
  inject_feedback_in_state(TEST_MOTOR0_ID, ROBSTRIDE_RUN_STATE_RUNNING, 130U, 0U, 0U, 0);
  clear_captures();
  inject_feedback_in_state(TEST_MOTOR0_ID, ROBSTRIDE_RUN_STATE_RESET, 140U, 0U, 0U, 0);
  k_msleep(HANDSHAKE_MS);
  zassert_not_null(
    find_frame(TEST_MOTOR0_ID, TYPE_ENABLE, 0),
    "a motor that left running has dropped the enable and needs it again");
  zassert_is_null(find_frame(TEST_MOTOR0_ID, TYPE_ENABLE, 1), "one enable is enough to recover");
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

  zassert_equal(
    robstride_get_parameter(motor0, PARAM_LIMIT_SPD, &value), -ETIMEDOUT,
    "a read nobody answers must time out");
}

ZTEST(robstride_motor, test_calls_reject_a_device_that_is_not_a_robstride_motor)
{
  struct robstride_feedback si;

  zassert_equal(robstride_set_current(test_can, 1.0F), -EINVAL);
  zassert_equal(robstride_get_feedback(test_can, &si), -EINVAL);
  zassert_equal(robstride_set_motion_target(motor0, NULL), -EINVAL);
  zassert_equal(robstride_get_feedback(motor0, NULL), -EINVAL);
}
