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

#define TEST_CAN_COUNT 2
#define TEST_CAN0_NODE DT_NODELABEL(test_can0)
#define TEST_CAN1_NODE DT_NODELABEL(test_can1)
#define TEST_MOTOR0_NODE DT_NODELABEL(motor0)
#define TEST_MOTOR1_NODE DT_NODELABEL(motor1)
#define TEST_MOTOR2_NODE DT_NODELABEL(motor2)

static const struct device * const test_can_devs[TEST_CAN_COUNT] = {
	DEVICE_DT_GET(TEST_CAN0_NODE),
	DEVICE_DT_GET(TEST_CAN1_NODE),
};

static const struct device * const motor0 = DEVICE_DT_GET(TEST_MOTOR0_NODE);
static const struct device * const motor1 = DEVICE_DT_GET(TEST_MOTOR1_NODE);
static const struct device * const motor2 = DEVICE_DT_GET(TEST_MOTOR2_NODE);

struct captured_filter {
	can_rx_callback_t callback;
	void *user_data;
	struct can_filter filter;
	bool valid;
};

struct captured_tx {
	struct can_frame frame;
	int count;
};

static struct captured_filter captured_filters[TEST_CAN_COUNT];
static struct captured_tx captured_tx[TEST_CAN_COUNT];

DEFINE_FFF_GLOBALS;

static int test_can_index(const struct device *dev)
{
	for (int i = 0; i < TEST_CAN_COUNT; ++i) {
		if (test_can_devs[i] == dev) {
			return i;
		}
	}

	return -1;
}

static int test_fake_can_add_rx_filter(const struct device *dev, can_rx_callback_t callback,
				       void *user_data, const struct can_filter *filter)
{
	int idx = test_can_index(dev);

	if (idx < 0) {
		return -EINVAL;
	}

	captured_filters[idx].callback = callback;
	captured_filters[idx].user_data = user_data;
	captured_filters[idx].filter = *filter;
	captured_filters[idx].valid = true;

	return idx + 1;
}

static int test_fake_can_send(const struct device *dev, const struct can_frame *frame,
			      k_timeout_t timeout, can_tx_callback_t callback, void *user_data)
{
	int idx = test_can_index(dev);

	ARG_UNUSED(timeout);

	if (idx < 0) {
		return -EINVAL;
	}

	captured_tx[idx].frame = *frame;
	captured_tx[idx].count++;

	if (callback != NULL) {
		callback(dev, 0, user_data);
	}

	return 0;
}

static int test_fake_can_start(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int robomaster_test_init(void)
{
	fake_can_add_rx_filter_fake.custom_fake = test_fake_can_add_rx_filter;
	fake_can_send_fake.custom_fake = test_fake_can_send;
	fake_can_start_fake.custom_fake = test_fake_can_start;

	return 0;
}

SYS_INIT(robomaster_test_init, PRE_KERNEL_1, 0);

static void reset_runtime_state(void)
{
	for (int i = 0; i < TEST_CAN_COUNT; ++i) {
		memset(&captured_tx[i], 0, sizeof(captured_tx[i]));
	}

	fake_can_send_fake.custom_fake = test_fake_can_send;
	fake_can_start_fake.custom_fake = test_fake_can_start;
}

static void *robomaster_setup(void)
{
	zassert_true(device_is_ready(motor0), "motor0 not ready");
	zassert_true(device_is_ready(motor1), "motor1 not ready");
	zassert_true(device_is_ready(motor2), "motor2 not ready");

	for (int i = 0; i < TEST_CAN_COUNT; ++i) {
		zassert_true(captured_filters[i].valid, "missing captured filter %d", i);
		zassert_equal(captured_filters[i].filter.id, 0x200, "wrong filter id");
		zassert_equal(captured_filters[i].filter.mask, 0x7F0, "wrong filter mask");
	}

	return NULL;
}

static void robomaster_before(void *fixture)
{
	ARG_UNUSED(fixture);

	reset_runtime_state();
	zassert_ok(motor_disable(motor0));
	zassert_ok(motor_disable(motor1));
	zassert_ok(motor_disable(motor2));
	reset_runtime_state();
}

static void expect_be16_slot(const struct can_frame *frame, int slot, int16_t value)
{
	zassert_equal((int16_t)sys_get_be16(&frame->data[slot * 2]), value, "slot %d mismatch", slot);
}

static void inject_feedback(const struct device *can_dev, uint16_t can_id, uint16_t orientation,
			    int16_t velocity, int16_t current, uint8_t temperature)
{
	int idx = test_can_index(can_dev);
	struct can_frame frame = {
		.id = can_id,
		.dlc = can_bytes_to_dlc(8),
		.flags = 0,
	};

	zassert_true(idx >= 0, "invalid CAN device");
	zassert_true(captured_filters[idx].callback != NULL, "missing RX callback for CAN %d", idx);

	sys_put_be16(orientation, &frame.data[0]);
	sys_put_be16((uint16_t)velocity, &frame.data[2]);
	sys_put_be16((uint16_t)current, &frame.data[4]);
	frame.data[6] = temperature;
	frame.data[7] = 0;

	captured_filters[idx].callback(can_dev, &frame, captured_filters[idx].user_data);
}

ZTEST_SUITE(robomaster_motor, NULL, robomaster_setup, robomaster_before, NULL, NULL);

ZTEST(robomaster_motor, test_set_output_routes_to_expected_can_group)
{
	zassert_ok(motor_enable(motor0));
	zassert_ok(motor_enable(motor1));
	zassert_ok(motor_enable(motor2));
	reset_runtime_state();

	zassert_ok(motor_set_output(motor0, MOTOR_OUTPUT_MODE_CURRENT, 100));
	zassert_ok(motor_set_output(motor1, MOTOR_OUTPUT_MODE_TORQUE, -200));
	zassert_ok(motor_set_output(motor2, MOTOR_OUTPUT_MODE_CURRENT, 300));

	zassert_equal(captured_tx[0].count, 2, "expected two sends on can0");
	zassert_equal(captured_tx[1].count, 1, "expected one send on can1");

	zassert_equal(captured_tx[0].frame.id, 0x200, "wrong group frame id for can0");
	expect_be16_slot(&captured_tx[0].frame, 0, 100);
	expect_be16_slot(&captured_tx[0].frame, 1, -200);
	expect_be16_slot(&captured_tx[0].frame, 2, 0);
	expect_be16_slot(&captured_tx[0].frame, 3, 0);

	zassert_equal(captured_tx[1].frame.id, 0x1FF, "wrong group frame id for can1");
	expect_be16_slot(&captured_tx[1].frame, 0, 300);
	expect_be16_slot(&captured_tx[1].frame, 1, 0);
	expect_be16_slot(&captured_tx[1].frame, 2, 0);
	expect_be16_slot(&captured_tx[1].frame, 3, 0);
}

ZTEST(robomaster_motor, test_enable_uses_cached_output_and_disable_clears_slot)
{
	zassert_ok(motor_set_output(motor0, MOTOR_OUTPUT_MODE_CURRENT, 512));
	zassert_equal(captured_tx[0].count, 1, "disabled motor should still flush zeroed frame");
	expect_be16_slot(&captured_tx[0].frame, 0, 0);

	zassert_ok(motor_enable(motor0));
	zassert_equal(captured_tx[0].count, 2, "enable should resend cached output");
	expect_be16_slot(&captured_tx[0].frame, 0, 512);

	zassert_ok(motor_disable(motor0));
	zassert_equal(captured_tx[0].count, 3, "disable should send zero output");
	expect_be16_slot(&captured_tx[0].frame, 0, 0);
}

ZTEST(robomaster_motor, test_rejects_unsupported_output_modes)
{
	zassert_equal(motor_set_output(motor0, MOTOR_OUTPUT_MODE_VELOCITY, 1), -ENOTSUP,
		      "velocity mode should be unsupported");
	zassert_equal(motor_set_output(motor0, MOTOR_OUTPUT_MODE_VOLTAGE, 1), -ENOTSUP,
		      "voltage mode should be unsupported");
}

ZTEST(robomaster_motor, test_get_feedback_reports_no_data_and_stale)
{
	struct motor_feedback feedback;

	zassert_equal(motor_get_feedback(motor1, &feedback), -ENODATA, "expected no data");

	inject_feedback(test_can_devs[0], 0x202, 120, 45, 67, 30);
	zassert_ok(motor_get_feedback(motor1, &feedback));
	zassert_false(feedback.stale, "feedback should be fresh");
	zassert_true(feedback.online, "feedback should be online");

	k_msleep(20);

	zassert_equal(motor_get_feedback(motor1, &feedback), -EAGAIN, "expected stale feedback");
	zassert_true(feedback.stale, "feedback should be stale");
}

ZTEST(robomaster_motor, test_feedback_decode_and_wraparound)
{
	struct motor_feedback feedback;

	inject_feedback(test_can_devs[0], 0x201, 8000, 120, 230, 55);
	zassert_ok(motor_get_feedback(motor0, &feedback));
	zassert_equal(feedback.valid_mask,
		      MOTOR_FEEDBACK_CURRENT | MOTOR_FEEDBACK_VELOCITY |
			      MOTOR_FEEDBACK_POSITION | MOTOR_FEEDBACK_ORIENTATION |
			      MOTOR_FEEDBACK_TEMPERATURE,
		      "unexpected valid mask");
	zassert_equal(feedback.current, 230, "wrong current");
	zassert_equal(feedback.velocity, 120, "wrong velocity");
	zassert_equal(feedback.orientation, 8000, "wrong orientation");
	zassert_equal(feedback.position, 8000, "wrong initial position");
	zassert_equal(feedback.temperature, 55, "wrong temperature");

	inject_feedback(test_can_devs[0], 0x201, 10, -10, -20, 56);
	zassert_ok(motor_get_feedback(motor0, &feedback));
	zassert_equal(feedback.orientation, 10, "wrong updated orientation");
	zassert_equal(feedback.position, 8202, "wrong wrapped position");
	zassert_equal(feedback.current, -20, "wrong updated current");
	zassert_equal(feedback.velocity, -10, "wrong updated velocity");
	zassert_equal(feedback.temperature, 56, "wrong updated temperature");
}

ZTEST(robomaster_motor, test_feedback_is_routed_by_can_bus_and_motor_id)
{
	struct motor_feedback feedback1;
	struct motor_feedback feedback2;

	inject_feedback(test_can_devs[1], 0x205, 1024, 77, 88, 40);

	zassert_equal(motor_get_feedback(motor1, &feedback1), -ENODATA,
		      "motor1 should not receive can1 feedback");
	zassert_ok(motor_get_feedback(motor2, &feedback2));
	zassert_equal(feedback2.orientation, 1024, "wrong can1 orientation");
	zassert_equal(feedback2.velocity, 77, "wrong can1 velocity");
	zassert_equal(feedback2.current, 88, "wrong can1 current");
	zassert_equal(feedback2.temperature, 40, "wrong can1 temperature");
}
