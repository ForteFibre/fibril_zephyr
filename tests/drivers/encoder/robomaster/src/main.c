/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can/can_fake.h>
#include <zephyr/fff.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#include <drivers/encoder.h>
#include <drivers/motor/robomaster.h>

static const struct device *const can0 = DEVICE_DT_GET(DT_NODELABEL(test_can0));
static const struct device *const motor1 = DEVICE_DT_GET(DT_NODELABEL(motor1));
static const struct device *const motor3 = DEVICE_DT_GET(DT_NODELABEL(motor3));
static const struct device *const enc1 = DEVICE_DT_GET(DT_NODELABEL(enc1));
static const struct device *const enc2 = DEVICE_DT_GET(DT_NODELABEL(enc2));

static can_rx_callback_t rx_callback;
static void *rx_user_data;

DEFINE_FFF_GLOBALS;

static int test_fake_can_add_rx_filter(const struct device *dev, can_rx_callback_t callback,
				       void *user_data, const struct can_filter *filter)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(filter);

	rx_callback = callback;
	rx_user_data = user_data;

	return 1;
}

static int test_fake_can_send(const struct device *dev, const struct can_frame *frame,
			      k_timeout_t timeout, can_tx_callback_t callback, void *user_data)
{
	ARG_UNUSED(frame);
	ARG_UNUSED(timeout);

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

static int robomaster_encoder_test_init(void)
{
	fake_can_add_rx_filter_fake.custom_fake = test_fake_can_add_rx_filter;
	fake_can_send_fake.custom_fake = test_fake_can_send;
	fake_can_start_fake.custom_fake = test_fake_can_start;

	return 0;
}

SYS_INIT(robomaster_encoder_test_init, PRE_KERNEL_1, 0);

static void inject_rotor(uint16_t can_id, uint16_t orientation)
{
	struct can_frame frame = {
		.id = can_id,
		.dlc = can_bytes_to_dlc(8),
	};

	zassert_not_null(rx_callback, "the RoboMaster driver installed no RX filter");

	sys_put_be16(orientation, &frame.data[0]);
	rx_callback(can0, &frame, rx_user_data);
}

static uint32_t epoch_of(const struct device *enc)
{
	struct encoder_feedback fb;

	(void)encoder_get_feedback(enc, &fb);

	return fb.position_epoch;
}

static void *robomaster_encoder_setup(void)
{
	zassert_true(device_is_ready(enc1), "enc1 not ready");
	zassert_true(device_is_ready(enc2), "enc2 not ready");

	return NULL;
}

static void robomaster_encoder_before(void *fixture)
{
	ARG_UNUSED(fixture);

	zassert_ok(encoder_reset(enc1));
	zassert_ok(encoder_reset(enc2));
}

ZTEST_SUITE(robomaster_encoder, NULL, robomaster_encoder_setup, robomaster_encoder_before, NULL,
	    NULL);

ZTEST(robomaster_encoder, test_reports_no_data_until_a_frame_arrives)
{
	struct encoder_feedback fb;

	zassert_equal(encoder_get_feedback(enc1, &fb), -ENODATA);
}

ZTEST(robomaster_encoder, test_first_frame_starts_at_the_rotor_angle_and_advances_the_epoch)
{
	struct encoder_feedback fb;
	uint8_t resolution;
	const uint32_t epoch = epoch_of(enc1);

	inject_rotor(0x201, 1000);

	zassert_ok(encoder_get_feedback(enc1, &fb));
	zassert_equal(fb.position, 1000);
	zassert_equal(fb.single_turn, 1000);
	zassert_equal(fb.position_epoch, epoch + 1U);
	zassert_equal(fb.valid_mask, ENCODER_FEEDBACK_POSITION | ENCODER_FEEDBACK_SINGLE_TURN,
		      "a single frame has no interval to derive a velocity over");

	zassert_ok(encoder_get_resolution(enc1, &resolution));
	zassert_equal(resolution, 13);
}

ZTEST(robomaster_encoder, test_follows_the_rotor_across_the_wrap_with_a_measured_velocity)
{
	struct encoder_feedback fb;

	inject_rotor(0x201, 8100);
	const uint32_t epoch = epoch_of(enc1);

	k_msleep(1);
	inject_rotor(0x201, 100);

	zassert_ok(encoder_get_feedback(enc1, &fb));
	zassert_equal(fb.position, 8100 + 192);
	zassert_equal(fb.single_turn, 100);
	zassert_equal(fb.position_epoch, epoch);
	zassert_true((fb.valid_mask & ENCODER_FEEDBACK_VELOCITY) != 0U);
	zassert_true(fb.sample_interval_us > 0U);
	zassert_equal(fb.velocity, (int64_t)192 * USEC_PER_SEC / fb.sample_interval_us);
}

ZTEST(robomaster_encoder, test_invert_direction_flips_the_position_and_velocity)
{
	struct encoder_feedback fb;

	inject_rotor(0x202, 1000);
	k_msleep(1);
	inject_rotor(0x202, 1100);

	zassert_ok(encoder_get_feedback(enc2, &fb));
	zassert_equal(fb.position, -1100);
	zassert_true(fb.velocity < 0);
	zassert_equal(fb.single_turn, 1100, "the single-turn value is what the ESC reported");
}

ZTEST(robomaster_encoder, test_starts_over_when_the_motor_went_silent_past_its_timeout)
{
	struct encoder_feedback fb;

	inject_rotor(0x201, 500);
	inject_rotor(0x201, 600);
	const uint32_t epoch = epoch_of(enc1);

	k_msleep(60);
	zassert_equal(encoder_get_feedback(enc1, &fb), -EAGAIN);
	zassert_true(fb.stale);

	inject_rotor(0x201, 7000);

	zassert_ok(encoder_get_feedback(enc1, &fb));
	zassert_false(fb.stale);
	zassert_equal(fb.position, 7000, "a frame after the gap must not be folded in as motion");
	zassert_equal(fb.position_epoch, epoch + 1U);
}

ZTEST(robomaster_encoder, test_set_position_offsets_the_frames_that_follow)
{
	struct encoder_feedback fb;

	inject_rotor(0x201, 1000);
	zassert_ok(encoder_set_position(enc1, 0));

	inject_rotor(0x201, 1100);

	zassert_ok(encoder_get_feedback(enc1, &fb));
	zassert_equal(fb.position, 100);
	zassert_equal(fb.single_turn, 1100);
}

ZTEST(robomaster_encoder, test_reset_starts_over_from_the_next_frame)
{
	struct encoder_feedback fb;

	inject_rotor(0x201, 1000);
	inject_rotor(0x201, 1500);
	const uint32_t epoch = epoch_of(enc1);

	zassert_ok(encoder_reset(enc1));
	zassert_equal(encoder_get_feedback(enc1, &fb), -ENODATA);

	inject_rotor(0x201, 300);

	zassert_ok(encoder_get_feedback(enc1, &fb));
	zassert_equal(fb.position, 300);
	zassert_equal(fb.position_epoch, epoch + 1U);
}

ZTEST(robomaster_encoder, test_set_zero_is_not_supported)
{
	zassert_equal(encoder_set_zero(enc1), -ENOTSUP);
}

static void unused_rotor_callback(const struct device *motor,
				  const struct robomaster_rotor_sample *sample, void *user_data)
{
	ARG_UNUSED(motor);
	ARG_UNUSED(sample);
	ARG_UNUSED(user_data);
}

ZTEST(robomaster_encoder, test_a_motor_takes_one_listener_and_only_a_robomaster_takes_any)
{
	zassert_equal(robomaster_motor_set_rotor_callback(motor1, unused_rotor_callback, NULL),
		      -EBUSY, "enc1 already listens to motor1");

	zassert_ok(robomaster_motor_set_rotor_callback(motor3, unused_rotor_callback, NULL));
	zassert_ok(robomaster_motor_set_rotor_callback(motor3, NULL, NULL));

	zassert_equal(robomaster_motor_set_rotor_callback(can0, unused_rotor_callback, NULL),
		      -EINVAL);
}
