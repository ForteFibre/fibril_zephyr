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
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#include <drivers/motor.h>

#define TEST_CAN_COUNT 2
#define TEST_CAN0_NODE DT_NODELABEL(test_can0)
#define TEST_CAN1_NODE DT_NODELABEL(test_can1)
#define TEST_MOTOR0_NODE DT_NODELABEL(motor0)
#define TEST_MOTOR1_NODE DT_NODELABEL(motor1)

/* Must outlast the driver's 500 ms start retry period. */
#define TEST_RETRY_WAIT_MS 700

static const struct device *const test_can_devs[TEST_CAN_COUNT] = {
	DEVICE_DT_GET(TEST_CAN0_NODE),
	DEVICE_DT_GET(TEST_CAN1_NODE),
};

static const struct device *const motor0 = DEVICE_DT_GET(TEST_MOTOR0_NODE);
static const struct device *const motor1 = DEVICE_DT_GET(TEST_MOTOR1_NODE);

struct captured_filter {
	can_rx_callback_t callback;
	void *user_data;
	bool valid;
};

static struct captured_filter captured_filters[TEST_CAN_COUNT];
static atomic_t captured_tx_count[TEST_CAN_COUNT];
static atomic_t captured_start_count[TEST_CAN_COUNT];

/* Bus 0 has no transceiver power until the test grants it. */
static atomic_t can0_startable;

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

	ARG_UNUSED(filter);

	if (idx < 0) {
		return -EINVAL;
	}

	captured_filters[idx].callback = callback;
	captured_filters[idx].user_data = user_data;
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

	if ((frame->id == 0x200) || (frame->id == 0x1FF)) {
		atomic_inc(&captured_tx_count[idx]);
	}

	if (callback != NULL) {
		callback(dev, 0, user_data);
	}

	return 0;
}

static int test_fake_can_start(const struct device *dev)
{
	int idx = test_can_index(dev);

	if (idx < 0) {
		return -EINVAL;
	}

	atomic_inc(&captured_start_count[idx]);

	if ((idx == 0) && !atomic_get(&can0_startable)) {
		/* What a bxCAN/FDCAN controller reports when RX stays dominant. */
		return -EIO;
	}

	return 0;
}

static void install_custom_fakes(void)
{
	fake_can_add_rx_filter_fake.custom_fake = test_fake_can_add_rx_filter;
	fake_can_send_fake.custom_fake = test_fake_can_send;
	fake_can_start_fake.custom_fake = test_fake_can_start;
}

static int robomaster_test_init(void)
{
	install_custom_fakes();

	return 0;
}

SYS_INIT(robomaster_test_init, PRE_KERNEL_1, 0);

static void robomaster_start_retry_before(void *fixture)
{
	ARG_UNUSED(fixture);

	/* can_fake's ztest rule resets every fake before each test. */
	install_custom_fakes();
}

static void wait_for_tx_flush(void)
{
	k_msleep(20);
}

static void inject_feedback(const struct device *can_dev, uint16_t can_id)
{
	int idx = test_can_index(can_dev);
	struct can_frame frame = {
		.id = can_id,
		.dlc = can_bytes_to_dlc(8),
		.flags = 0,
	};

	zassert_true(idx >= 0, "invalid CAN device");
	zassert_true(captured_filters[idx].callback != NULL, "missing RX callback for CAN %d", idx);

	memset(frame.data, 0, sizeof(frame.data));
	sys_put_be16(10, &frame.data[0]);

	captured_filters[idx].callback(can_dev, &frame, captured_filters[idx].user_data);
}

ZTEST_SUITE(robomaster_start_retry, NULL, NULL, robomaster_start_retry_before, NULL, NULL);

/*
 * Everything lives in one test because the driver starts a bus exactly once:
 * splitting the phases would make them depend on the execution order.
 */
ZTEST(robomaster_start_retry, test_unstartable_bus_recovers_without_dropping_motors)
{
	/* A bus that will not start must not take the motor devices down. */
	zassert_true(device_is_ready(motor0), "motor0 not ready");
	zassert_true(device_is_ready(motor1), "motor1 not ready");

	for (int i = 0; i < TEST_CAN_COUNT; ++i) {
		zassert_true(captured_filters[i].valid, "missing captured filter %d", i);
	}

	zassert_ok(motor_set_output(motor0, MOTOR_OUTPUT_MODE_CURRENT, 1000));
	zassert_ok(motor_set_output(motor1, MOTOR_OUTPUT_MODE_CURRENT, 1000));
	zassert_ok(motor_enable(motor0));
	zassert_ok(motor_enable(motor1));

	inject_feedback(test_can_devs[0], 0x201);
	inject_feedback(test_can_devs[1], 0x202);

	atomic_set(&captured_tx_count[0], 0);
	atomic_set(&captured_tx_count[1], 0);
	wait_for_tx_flush();

	zassert_equal(atomic_get(&captured_tx_count[0]), 0, "transmitted on a bus that never started");
	zassert_true(atomic_get(&captured_tx_count[1]) > 0, "started bus stopped transmitting");

	/* Motor power comes up late; the retry has to pick the bus up. */
	atomic_set(&captured_start_count[0], 0);
	atomic_set(&can0_startable, 1);
	k_msleep(TEST_RETRY_WAIT_MS);

	zassert_true(atomic_get(&captured_start_count[0]) > 0, "bus 0 was never retried");

	atomic_set(&captured_tx_count[0], 0);
	wait_for_tx_flush();

	zassert_true(atomic_get(&captured_tx_count[0]) > 0, "bus 0 did not transmit after the retry");

	/* An already started bus is not restarted. */
	atomic_set(&captured_start_count[0], 0);
	k_msleep(TEST_RETRY_WAIT_MS);
	zassert_equal(atomic_get(&captured_start_count[0]), 0, "bus 0 restarted after coming up");
}
