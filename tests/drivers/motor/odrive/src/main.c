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
#include <drivers/motor/odrive.h>

#define AXIS0_NODE_ID 0U
#define AXIS1_NODE_ID 5U

#define CMD_HEARTBEAT 0x001U
#define CMD_ESTOP 0x002U
#define CMD_SET_AXIS_STATE 0x007U
#define CMD_GET_ENCODER_ESTIMATES 0x009U
#define CMD_SET_CONTROLLER_MODE 0x00BU
#define CMD_SET_INPUT_POS 0x00CU
#define CMD_SET_INPUT_VEL 0x00DU
#define CMD_SET_INPUT_TORQUE 0x00EU
#define CMD_GET_TEMPERATURE 0x015U
#define CMD_CLEAR_ERRORS 0x018U

#define NODE_SHIFT 5U
#define CMD_MASK 0x1FU

/* Long enough for the driver to work through every arming stage. */
#define ARM_MS (CONFIG_MOTOR_ODRIVE_TX_INTERVAL_MS * 6)
/* One command interval plus slack, for tests that count frames per interval. */
#define TICK_MS (CONFIG_MOTOR_ODRIVE_TX_INTERVAL_MS + 2)
/* The callbacks run on the bus work queue, so give it a chance to be scheduled. */
#define DISPATCH_MS 20

static const struct device * const test_can = DEVICE_DT_GET(DT_NODELABEL(test_can0));
static const struct device * const axis0 = DEVICE_DT_GET(DT_NODELABEL(axis0));
static const struct device * const axis1 = DEVICE_DT_GET(DT_NODELABEL(axis1));

#define CAPTURE_MAX 32

static struct can_frame captured[CAPTURE_MAX];
static size_t captured_count;
static can_rx_callback_t rx_callback;
static void * rx_user_data;
static struct can_filter rx_filters[4];
static size_t rx_filter_count;

DEFINE_FFF_GLOBALS;

static int test_fake_can_add_rx_filter(
  const struct device * dev, can_rx_callback_t callback, void * user_data,
  const struct can_filter * filter)
{
  ARG_UNUSED(dev);

  rx_callback = callback;
  rx_user_data = user_data;

  if (rx_filter_count < ARRAY_SIZE(rx_filters)) {
    rx_filters[rx_filter_count] = *filter;
  }

  rx_filter_count++;

  return (int)rx_filter_count;
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

static int odrive_test_init(void)
{
  fake_can_add_rx_filter_fake.custom_fake = test_fake_can_add_rx_filter;
  fake_can_send_fake.custom_fake = test_fake_can_send;
  fake_can_start_fake.custom_fake = test_fake_can_start;

  return 0;
}

SYS_INIT(odrive_test_init, PRE_KERNEL_1, 0);

static uint8_t frame_node_id(const struct can_frame * frame)
{
  return (uint8_t)((frame->id >> NODE_SHIFT) & 0x3FU);
}

static uint16_t frame_cmd(const struct can_frame * frame)
{
  return (uint16_t)(frame->id & CMD_MASK);
}

static float frame_float(const struct can_frame * frame, size_t offset)
{
  const uint32_t bits = sys_get_le32(&frame->data[offset]);
  float value;

  memcpy(&value, &bits, sizeof(value));

  return value;
}

static void clear_captures(void)
{
  captured_count = 0;
  memset(captured, 0, sizeof(captured));
}

static const struct can_frame * find_frame(uint8_t node_id, uint16_t cmd)
{
  for (size_t i = 0; i < MIN(captured_count, (size_t)CAPTURE_MAX); ++i) {
    if ((frame_node_id(&captured[i]) == node_id) && (frame_cmd(&captured[i]) == cmd)) {
      return &captured[i];
    }
  }

  return NULL;
}

static size_t count_frames(uint8_t node_id, uint16_t cmd)
{
  size_t count = 0;

  for (size_t i = 0; i < MIN(captured_count, (size_t)CAPTURE_MAX); ++i) {
    if ((frame_node_id(&captured[i]) == node_id) && (frame_cmd(&captured[i]) == cmd)) {
      count++;
    }
  }

  return count;
}

/* Index of the first frame matching cmd, or SIZE_MAX. Used to assert ordering. */
static size_t index_of(uint8_t node_id, uint16_t cmd)
{
  for (size_t i = 0; i < MIN(captured_count, (size_t)CAPTURE_MAX); ++i) {
    if ((frame_node_id(&captured[i]) == node_id) && (frame_cmd(&captured[i]) == cmd)) {
      return i;
    }
  }

  return SIZE_MAX;
}

static void inject(uint8_t node_id, uint16_t cmd, const uint8_t data[8], uint8_t len)
{
  struct can_frame frame = {
    .id = ((uint32_t)node_id << NODE_SHIFT) | cmd,
    .dlc = can_bytes_to_dlc(len),
    .flags = 0,
  };

  memcpy(frame.data, data, len);

  zassert_not_null(rx_callback, "driver never registered an RX filter");
  rx_callback(test_can, &frame, rx_user_data);
}

static void inject_heartbeat(uint8_t node_id, enum odrive_axis_state state, uint32_t errors)
{
  uint8_t data[8] = {0};

  sys_put_le32(errors, &data[0]);
  data[4] = (uint8_t)state;
  data[5] = (uint8_t)ODRIVE_PROCEDURE_RESULT_SUCCESS;

  inject(node_id, CMD_HEARTBEAT, data, 8U);
}

static void inject_estimates(uint8_t node_id, float position, float velocity)
{
  uint8_t data[8] = {0};
  uint32_t bits;

  memcpy(&bits, &position, sizeof(bits));
  sys_put_le32(bits, &data[0]);
  memcpy(&bits, &velocity, sizeof(bits));
  sys_put_le32(bits, &data[4]);

  inject(node_id, CMD_GET_ENCODER_ESTIMATES, data, 8U);
}

static void inject_temperature(uint8_t node_id, float fet, float motor)
{
  uint8_t data[8] = {0};
  uint32_t bits;

  memcpy(&bits, &fet, sizeof(bits));
  sys_put_le32(bits, &data[0]);
  memcpy(&bits, &motor, sizeof(bits));
  sys_put_le32(bits, &data[4]);

  inject(node_id, CMD_GET_TEMPERATURE, data, 8U);
}

/*
 * Wait while keeping the axis alive. The driver drops an axis that stops
 * heartbeating, so a test that spans several command intervals has to keep
 * sending them the way a real ODrive does.
 */
static void sleep_armed(uint8_t node_id, int ms)
{
  for (int elapsed = 0; elapsed < ms; elapsed += CONFIG_MOTOR_ODRIVE_TX_INTERVAL_MS) {
    inject_heartbeat(node_id, ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL, 0U);
    k_sleep(K_MSEC(CONFIG_MOTOR_ODRIVE_TX_INTERVAL_MS));
  }
}

/* Bring an axis to the point where the driver is sending setpoints. */
static void arm(const struct device * dev, uint8_t node_id)
{
  zassert_ok(motor_enable(dev));
  k_sleep(K_MSEC(ARM_MS));
  inject_heartbeat(node_id, ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL, 0U);
  k_sleep(K_MSEC(TICK_MS));
}

struct callback_record
{
  size_t count;
  struct odrive_feedback last;
};

static struct callback_record state_record;
static struct callback_record feedback_record;

static void record_state(
  const struct device * dev, const struct odrive_feedback * feedback, void * user_data)
{
  struct callback_record * record = user_data;

  ARG_UNUSED(dev);

  record->count++;
  record->last = *feedback;
}

static void * odrive_suite_setup(void)
{
  zassert_true(device_is_ready(axis0), "axis0 not ready");
  zassert_true(device_is_ready(axis1), "axis1 not ready");

  return NULL;
}

static void odrive_before(void * fixture)
{
  ARG_UNUSED(fixture);

  /*
   * can_fake resets its fakes in a ztest rule before every test. Without
   * reinstalling this one, can_send() without a completion callback waits
   * K_FOREVER on the semaphore the CAN API set up for it.
   */
  fake_can_send_fake.custom_fake = test_fake_can_send;
  fake_can_start_fake.custom_fake = test_fake_can_start;

  (void)motor_disable(axis0);
  (void)motor_disable(axis1);
  (void)odrive_set_state_callback(axis0, NULL, NULL);
  (void)odrive_set_feedback_callback(axis0, NULL, NULL);

  memset(&state_record, 0, sizeof(state_record));
  memset(&feedback_record, 0, sizeof(feedback_record));

  /* Let the axes fall offline so each test starts from the same state. */
  k_sleep(K_MSEC(DISPATCH_MS + 60));
  clear_captures();
}

ZTEST_SUITE(odrive, NULL, odrive_suite_setup, odrive_before, NULL, NULL);

ZTEST(odrive, test_each_axis_gets_its_own_receive_filter)
{
  zassert_equal(rx_filter_count, 2U, "expected one filter per axis, got %u",
                (unsigned int)rx_filter_count);

  for (size_t i = 0; i < 2U; ++i) {
    zassert_equal(rx_filters[i].mask, 0x7E0U, "filter %u masks the wrong bits", (unsigned int)i);
    zassert_equal(rx_filters[i].flags, 0U, "filter %u is not a standard-ID filter",
                  (unsigned int)i);
  }

  const bool axis0_first = rx_filters[0].id == ((uint32_t)AXIS0_NODE_ID << NODE_SHIFT);
  const size_t other = axis0_first ? 1U : 0U;

  zassert_equal(rx_filters[other].id, (uint32_t)AXIS1_NODE_ID << NODE_SHIFT,
                "no filter matches axis1's node ID");
}

ZTEST(odrive, test_enable_clears_errors_then_sets_mode_then_requests_closed_loop)
{
  zassert_ok(motor_enable(axis0));
  k_sleep(K_MSEC(ARM_MS));

  const size_t clear = index_of(AXIS0_NODE_ID, CMD_CLEAR_ERRORS);
  const size_t mode = index_of(AXIS0_NODE_ID, CMD_SET_CONTROLLER_MODE);
  const size_t state = index_of(AXIS0_NODE_ID, CMD_SET_AXIS_STATE);

  zassert_not_equal(clear, SIZE_MAX, "Clear_Errors was never sent");
  zassert_not_equal(mode, SIZE_MAX, "Set_Controller_Mode was never sent");
  zassert_not_equal(state, SIZE_MAX, "Set_Axis_State was never sent");
  zassert_true(clear < mode, "Clear_Errors must come before Set_Controller_Mode");
  zassert_true(mode < state, "Set_Controller_Mode must come before Set_Axis_State");

  const struct can_frame * frame = find_frame(AXIS0_NODE_ID, CMD_SET_AXIS_STATE);

  zassert_equal(sys_get_le32(&frame->data[0]), (uint32_t)ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL,
                "the requested state is not closed loop control");
}

ZTEST(odrive, test_arming_retry_does_not_clear_errors_again)
{
  zassert_ok(motor_enable(axis0));
  k_sleep(K_MSEC(ARM_MS));
  clear_captures();

  /* No heartbeat confirms closed loop, so the driver has to ask again. */
  k_sleep(K_MSEC(CONFIG_MOTOR_ODRIVE_STATE_RETRY_MS + (ARM_MS * 2)));

  zassert_true(count_frames(AXIS0_NODE_ID, CMD_SET_AXIS_STATE) > 0U,
               "the driver gave up asking for closed loop");
  zassert_equal(count_frames(AXIS0_NODE_ID, CMD_CLEAR_ERRORS), 0U,
                "a retry cleared errors, which would cycle a tripped axis through its fault");
}

ZTEST(odrive, test_setpoints_flow_only_once_the_heartbeat_confirms_closed_loop)
{
  zassert_ok(odrive_set_torque(axis0, 1.0F));
  zassert_ok(motor_enable(axis0));
  k_sleep(K_MSEC(ARM_MS));

  zassert_equal(count_frames(AXIS0_NODE_ID, CMD_SET_INPUT_TORQUE), 0U,
                "a setpoint was sent before the axis reported closed loop");

  inject_heartbeat(AXIS0_NODE_ID, ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL, 0U);
  k_sleep(K_MSEC(TICK_MS * 2));

  const struct can_frame * frame = find_frame(AXIS0_NODE_ID, CMD_SET_INPUT_TORQUE);

  zassert_not_null(frame, "no torque setpoint after the axis reported closed loop");
  zassert_within(frame_float(frame, 0), 1.0F, 0.0001F, "wrong torque on the wire");
}

ZTEST(odrive, test_axis_dropping_out_of_closed_loop_stops_commands_and_reports)
{
  zassert_ok(odrive_set_state_callback(axis0, record_state, &state_record));
  zassert_ok(odrive_set_torque(axis0, 1.0F));
  arm(axis0, AXIS0_NODE_ID);

  state_record.count = 0;
  clear_captures();

  inject_heartbeat(AXIS0_NODE_ID, ODRIVE_AXIS_STATE_IDLE, 0x20U);
  k_sleep(K_MSEC(DISPATCH_MS + (TICK_MS * 3)));

  zassert_equal(count_frames(AXIS0_NODE_ID, CMD_SET_INPUT_TORQUE), 0U,
                "the driver kept commanding an axis that disarmed itself");
  zassert_equal(count_frames(AXIS0_NODE_ID, CMD_CLEAR_ERRORS), 0U,
                "the driver cleared the fault by itself");
  zassert_true(state_record.count > 0U, "the state callback did not report the disarm");
  zassert_equal(state_record.last.active_errors, 0x20U, "the reported errors are wrong");
  zassert_false(state_record.last.enabled, "the axis is still reported as enabled");
}

ZTEST(odrive, test_enable_after_a_disarm_starts_the_sequence_over)
{
  arm(axis0, AXIS0_NODE_ID);
  inject_heartbeat(AXIS0_NODE_ID, ODRIVE_AXIS_STATE_IDLE, 0x20U);
  k_sleep(K_MSEC(TICK_MS * 2));
  clear_captures();

  zassert_ok(motor_enable(axis0));
  k_sleep(K_MSEC(ARM_MS));

  zassert_true(count_frames(AXIS0_NODE_ID, CMD_CLEAR_ERRORS) > 0U,
               "re-arming did not clear the errors that caused the disarm");
  zassert_true(count_frames(AXIS0_NODE_ID, CMD_SET_AXIS_STATE) > 0U,
               "re-arming did not request closed loop again");
}

ZTEST(odrive, test_disable_requests_idle_without_waiting_for_the_next_interval)
{
  arm(axis0, AXIS0_NODE_ID);
  clear_captures();

  zassert_ok(motor_disable(axis0));

  const struct can_frame * frame = find_frame(AXIS0_NODE_ID, CMD_SET_AXIS_STATE);

  zassert_not_null(frame, "Set_Axis_State was not sent from motor_disable() itself");
  zassert_equal(sys_get_le32(&frame->data[0]), (uint32_t)ODRIVE_AXIS_STATE_IDLE,
                "motor_disable() did not request idle");
}

ZTEST(odrive, test_estop_sends_the_estop_frame_and_stops_commanding)
{
  zassert_ok(odrive_set_torque(axis0, 1.0F));
  arm(axis0, AXIS0_NODE_ID);
  clear_captures();

  zassert_ok(odrive_estop(axis0));

  zassert_not_null(find_frame(AXIS0_NODE_ID, CMD_ESTOP), "no Estop frame");

  clear_captures();
  k_sleep(K_MSEC(TICK_MS * 3));

  zassert_equal(count_frames(AXIS0_NODE_ID, CMD_SET_INPUT_TORQUE), 0U,
                "the driver kept commanding after an emergency stop");
}

ZTEST(odrive, test_encoder_estimates_become_position_counts)
{
  inject_heartbeat(AXIS0_NODE_ID, ODRIVE_AXIS_STATE_IDLE, 0U);
  inject_estimates(AXIS0_NODE_ID, 2.5F, 1.25F);

  struct motor_feedback feedback;

  zassert_ok(motor_get_feedback(axis0, &feedback));
  zassert_equal(feedback.position, (int64_t)(2.5F * 65536.0F),
                "position is not 65536 counts per revolution");
  zassert_true((feedback.valid_mask & MOTOR_FEEDBACK_POSITION) != 0U,
               "position is not marked valid");

  struct odrive_feedback si;

  zassert_ok(odrive_get_feedback(axis0, &si));
  zassert_within(si.position, 2.5F, 0.0001F, "wrong position in rev");
  zassert_within(si.velocity, 1.25F, 0.0001F, "wrong velocity in rev/s");
  zassert_true((si.valid_mask & ODRIVE_FEEDBACK_ESTIMATES) != 0U,
               "estimates are not marked valid");
}

ZTEST(odrive, test_feedback_callback_coalesces_a_burst_into_one_call)
{
  zassert_ok(odrive_set_feedback_callback(axis0, record_state, &feedback_record));
  inject_heartbeat(AXIS0_NODE_ID, ODRIVE_AXIS_STATE_IDLE, 0U);
  k_sleep(K_MSEC(DISPATCH_MS));
  feedback_record.count = 0;

  inject_estimates(AXIS0_NODE_ID, 1.0F, 0.0F);
  inject_estimates(AXIS0_NODE_ID, 2.0F, 0.0F);
  inject_estimates(AXIS0_NODE_ID, 3.0F, 0.0F);
  k_sleep(K_MSEC(DISPATCH_MS));

  zassert_equal(feedback_record.count, 1U, "expected one coalesced call, got %u",
                (unsigned int)feedback_record.count);
  zassert_within(feedback_record.last.position, 3.0F, 0.0001F,
                 "the coalesced call did not carry the latest snapshot");
}

ZTEST(odrive, test_losing_the_heartbeat_reports_the_axis_offline)
{
  zassert_ok(odrive_set_state_callback(axis0, record_state, &state_record));
  inject_heartbeat(AXIS0_NODE_ID, ODRIVE_AXIS_STATE_IDLE, 0U);
  inject_estimates(AXIS0_NODE_ID, 0.0F, 0.0F);
  k_sleep(K_MSEC(DISPATCH_MS));

  struct odrive_feedback feedback;

  zassert_ok(odrive_get_feedback(axis0, &feedback));
  zassert_true(feedback.online, "the axis is not online after a heartbeat");

  state_record.count = 0;

  /* Nothing is enabled here, so this also covers the timer running regardless. */
  k_sleep(K_MSEC(DT_PROP(DT_NODELABEL(odrive_test), heartbeat_timeout_ms) + DISPATCH_MS + 40));

  zassert_equal(odrive_get_feedback(axis0, &feedback), -ENODATA,
                "a silent axis is still reported as online");
  zassert_true(state_record.count > 0U, "going offline was not reported");
  zassert_false(state_record.last.online, "the callback reported the axis as online");
}

ZTEST(odrive, test_temperature_reaches_the_class_only_with_a_thermistor)
{
  inject_heartbeat(AXIS0_NODE_ID, ODRIVE_AXIS_STATE_IDLE, 0U);
  inject_heartbeat(AXIS1_NODE_ID, ODRIVE_AXIS_STATE_IDLE, 0U);
  inject_estimates(AXIS0_NODE_ID, 0.0F, 0.0F);
  inject_estimates(AXIS1_NODE_ID, 0.0F, 0.0F);
  inject_temperature(AXIS0_NODE_ID, 40.0F, 0.0F);
  inject_temperature(AXIS1_NODE_ID, 40.0F, 31.0F);

  struct motor_feedback feedback;

  zassert_ok(motor_get_feedback(axis0, &feedback));
  zassert_equal(feedback.valid_mask & MOTOR_FEEDBACK_TEMPERATURE, 0U,
                "an axis without a thermistor published a motor temperature");

  zassert_ok(motor_get_feedback(axis1, &feedback));
  zassert_true((feedback.valid_mask & MOTOR_FEEDBACK_TEMPERATURE) != 0U,
               "an axis with a thermistor published no motor temperature");
  zassert_equal(feedback.temperature, 31, "the class field does not carry the motor temperature");

  struct odrive_feedback si;

  zassert_ok(odrive_get_feedback(axis0, &si));
  zassert_within(si.fet_temperature, 40.0F, 0.0001F,
                 "the FET temperature is missing from the driver snapshot");
}

ZTEST(odrive, test_trap_traj_does_not_resend_an_unchanged_position)
{
  zassert_ok(odrive_set_input_mode(axis0, ODRIVE_INPUT_MODE_TRAP_TRAJ));
  zassert_ok(odrive_set_position(axis0, 1.0F, 0.0F, 0.0F));
  arm(axis0, AXIS0_NODE_ID);
  clear_captures();

  sleep_armed(AXIS0_NODE_ID, TICK_MS * 4);

  zassert_equal(count_frames(AXIS0_NODE_ID, CMD_SET_INPUT_POS), 0U,
                "an unchanged trap-traj target was re-sent, which re-plans the trajectory");

  zassert_ok(odrive_set_position(axis0, 2.0F, 0.0F, 0.0F));
  sleep_armed(AXIS0_NODE_ID, TICK_MS * 3);

  zassert_equal(count_frames(AXIS0_NODE_ID, CMD_SET_INPUT_POS), 1U,
                "a changed trap-traj target was not sent exactly once");
}

ZTEST(odrive, test_passthrough_resends_an_unchanged_position_every_interval)
{
  zassert_ok(odrive_set_input_mode(axis0, ODRIVE_INPUT_MODE_PASSTHROUGH));
  zassert_ok(odrive_set_position(axis0, 1.0F, 0.0F, 0.0F));
  arm(axis0, AXIS0_NODE_ID);
  clear_captures();

  sleep_armed(AXIS0_NODE_ID, TICK_MS * 4);

  zassert_true(count_frames(AXIS0_NODE_ID, CMD_SET_INPUT_POS) >= 3U,
               "the watchdog is not being fed in passthrough mode");
}

ZTEST(odrive, test_position_feed_forward_is_int16_scaled_by_a_thousand)
{
  zassert_ok(odrive_set_position(axis0, 0.5F, 1.5F, -0.25F));
  arm(axis0, AXIS0_NODE_ID);

  const struct can_frame * frame = find_frame(AXIS0_NODE_ID, CMD_SET_INPUT_POS);

  zassert_not_null(frame, "no position setpoint");
  zassert_within(frame_float(frame, 0), 0.5F, 0.0001F, "wrong position on the wire");
  zassert_equal((int16_t)sys_get_le16(&frame->data[4]), 1500,
                "velocity feed-forward is not int16 scaled by 1000");
  zassert_equal((int16_t)sys_get_le16(&frame->data[6]), -250,
                "torque feed-forward is not int16 scaled by 1000");
}

ZTEST(odrive, test_switching_target_switches_control_mode)
{
  zassert_ok(odrive_set_torque(axis0, 1.0F));
  arm(axis0, AXIS0_NODE_ID);
  clear_captures();

  zassert_ok(odrive_set_velocity(axis0, 3.0F, 0.0F));
  sleep_armed(AXIS0_NODE_ID, TICK_MS * 3);

  const struct can_frame * mode = find_frame(AXIS0_NODE_ID, CMD_SET_CONTROLLER_MODE);

  zassert_not_null(mode, "the control mode was not re-sent after switching target");
  zassert_equal(sys_get_le32(&mode->data[0]), (uint32_t)ODRIVE_CONTROL_MODE_VELOCITY,
                "the wrong control mode was sent");

  const struct can_frame * setpoint = find_frame(AXIS0_NODE_ID, CMD_SET_INPUT_VEL);

  zassert_not_null(setpoint, "no velocity setpoint after switching mode");
  zassert_within(frame_float(setpoint, 0), 3.0F, 0.0001F, "wrong velocity on the wire");
}

ZTEST(odrive, test_generic_set_output_is_refused)
{
  zassert_equal(motor_set_output(axis0, MOTOR_OUTPUT_MODE_CURRENT, 100), -ENOTSUP);
  zassert_equal(motor_set_output(axis0, MOTOR_OUTPUT_MODE_VELOCITY, 100), -ENOTSUP);
  zassert_equal(motor_set_output(axis0, MOTOR_OUTPUT_MODE_VOLTAGE, 100), -ENOTSUP);
}

ZTEST(odrive, test_axis_state_request_is_refused_while_the_driver_owns_the_axis)
{
  zassert_ok(motor_enable(axis0));

  zassert_equal(odrive_request_axis_state(axis0, ODRIVE_AXIS_STATE_MOTOR_CALIBRATION), -EBUSY,
                "a calibration request fought the driver for the axis state");

  zassert_ok(motor_disable(axis0));
  clear_captures();

  zassert_ok(odrive_request_axis_state(axis0, ODRIVE_AXIS_STATE_MOTOR_CALIBRATION));

  const struct can_frame * frame = find_frame(AXIS0_NODE_ID, CMD_SET_AXIS_STATE);

  zassert_not_null(frame, "no Set_Axis_State for the calibration request");
  zassert_equal(sys_get_le32(&frame->data[0]), (uint32_t)ODRIVE_AXIS_STATE_MOTOR_CALIBRATION,
                "the requested state is wrong");
}
