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
#include <drivers/motor/robstride.h>

#define TEST_CAN_COUNT 2
#define TEST_MOTOR_ID 0x7FU

#define TYPE_ENABLE 0x03U
#define TYPE_STOP 0x04U
#define TYPE_SET_PARAM 0x12U

#define PARAM_RUN_MODE 0x7005U
#define PARAM_IQ_REF 0x7006U
#define PARAM_LIMIT_TORQUE 0x700BU
#define PARAM_LIMIT_SPD 0x7017U
#define PARAM_LIMIT_CUR 0x7018U

/* Long enough for the driver to work through every handshake stage. */
#define HANDSHAKE_MS (CONFIG_MOTOR_ROBSTRIDE_TX_INTERVAL_MS * 8)
/* Must outlast the driver's 500 ms start retry period. */
#define RETRY_WAIT_MS 700

#define CAPTURE_MAX 32

static const struct device * const test_can_devs[TEST_CAN_COUNT] = {
  DEVICE_DT_GET(DT_NODELABEL(test_can0)),
  DEVICE_DT_GET(DT_NODELABEL(test_can1)),
};

static const struct device * const motor0 = DEVICE_DT_GET(DT_NODELABEL(motor0));

static struct can_frame captured[TEST_CAN_COUNT][CAPTURE_MAX];
static size_t captured_count[TEST_CAN_COUNT];

/* Neither transceiver has power until the test grants it. */
static atomic_t startable[TEST_CAN_COUNT];

DEFINE_FFF_GLOBALS;

static int test_can_index(const struct device * dev)
{
  for (int i = 0; i < TEST_CAN_COUNT; ++i) {
    if (test_can_devs[i] == dev) {
      return i;
    }
  }

  return -1;
}

static int test_fake_can_add_rx_filter(
  const struct device * dev, can_rx_callback_t callback, void * user_data,
  const struct can_filter * filter)
{
  const int idx = test_can_index(dev);

  ARG_UNUSED(callback);
  ARG_UNUSED(user_data);
  ARG_UNUSED(filter);

  return (idx < 0) ? -EINVAL : (idx + 1);
}

static int test_fake_can_send(
  const struct device * dev, const struct can_frame * frame, k_timeout_t timeout,
  can_tx_callback_t callback, void * user_data)
{
  const int idx = test_can_index(dev);

  ARG_UNUSED(timeout);

  if (idx < 0) {
    return -EINVAL;
  }

  if (captured_count[idx] < CAPTURE_MAX) {
    captured[idx][captured_count[idx]] = *frame;
  }

  captured_count[idx]++;

  if (callback != NULL) {
    callback(dev, 0, user_data);
  }

  return 0;
}

static int test_fake_can_start(const struct device * dev)
{
  const int idx = test_can_index(dev);

  if (idx < 0) {
    return -EINVAL;
  }

  if (!atomic_get(&startable[idx])) {
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

static int robstride_start_retry_test_init(void)
{
  install_custom_fakes();

  return 0;
}

SYS_INIT(robstride_start_retry_test_init, PRE_KERNEL_1, 0);

static void before_each(void * fixture)
{
  ARG_UNUSED(fixture);

  /* can_fake's ztest rule resets every fake before each test. */
  install_custom_fakes();
}

static void clear_captures(void)
{
  memset(captured, 0, sizeof(captured));
  memset(captured_count, 0, sizeof(captured_count));
}

static uint8_t frame_type(const struct can_frame * frame)
{
  return (uint8_t)(((frame->id & CAN_EXT_ID_MASK) >> 24) & 0x1FU);
}

static const struct can_frame * find_frame(int bus, uint8_t type)
{
  for (size_t i = 0; i < MIN(captured_count[bus], (size_t)CAPTURE_MAX); ++i) {
    if (frame_type(&captured[bus][i]) == type) {
      return &captured[bus][i];
    }
  }

  return NULL;
}

static const struct can_frame * find_set_param(int bus, uint16_t index)
{
  for (size_t i = 0; i < MIN(captured_count[bus], (size_t)CAPTURE_MAX); ++i) {
    if (frame_type(&captured[bus][i]) != TYPE_SET_PARAM) {
      continue;
    }

    if (sys_get_le16(&captured[bus][i].data[0]) == index) {
      return &captured[bus][i];
    }
  }

  return NULL;
}

static void assert_full_handshake(int bus)
{
  zassert_not_null(find_frame(bus, TYPE_STOP), "no stop frame on bus %d", bus);
  zassert_not_null(find_set_param(bus, PARAM_RUN_MODE), "no run mode write on bus %d", bus);
  zassert_not_null(find_set_param(bus, PARAM_LIMIT_CUR), "no current limit on bus %d", bus);
  zassert_not_null(find_set_param(bus, PARAM_LIMIT_SPD), "no velocity limit on bus %d", bus);
  zassert_not_null(find_set_param(bus, PARAM_LIMIT_TORQUE), "no torque limit on bus %d", bus);
  zassert_not_null(find_frame(bus, TYPE_ENABLE), "no enable frame on bus %d", bus);
  zassert_not_null(find_set_param(bus, PARAM_IQ_REF), "no target frame on bus %d", bus);
}

ZTEST_SUITE(robstride_start_retry, NULL, NULL, before_each, NULL, NULL);

/*
 * Everything lives in one test because the driver starts a bus exactly once:
 * splitting the phases would make them depend on the execution order.
 */
ZTEST(robstride_start_retry, test_handshake_survives_a_transceiver_that_powers_up_late)
{
  /* A bus that will not start must not take the motor device down. */
  zassert_true(device_is_ready(motor0), "motor0 not ready");

  zassert_ok(robstride_set_current(motor0, 2.0F));
  zassert_ok(motor_enable(motor0));
  k_msleep(HANDSHAKE_MS);

  zassert_equal(captured_count[0], 0U, "transmitted on a bus that never started");
  zassert_equal(captured_count[1], 0U, "transmitted on a bus that never started");

  /*
   * One transceiver comes up. The handshake was never sent, so it has to be
   * still pending rather than counted off against a bus that took nothing.
   */
  atomic_set(&startable[1], 1);
  k_msleep(RETRY_WAIT_MS);

  assert_full_handshake(1);

  /*
   * The other comes up later. The motor has answered nothing, so the driver
   * does not know it is not the one behind this controller and owes it the
   * same handshake.
   */
  clear_captures();
  atomic_set(&startable[0], 1);
  k_msleep(RETRY_WAIT_MS);

  assert_full_handshake(0);
}
