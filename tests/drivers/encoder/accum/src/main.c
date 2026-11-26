/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for the position accumulator shared by the encoder drivers. No
 * device is involved: the counter values and the instants they were taken at
 * are handed to the helper directly, which is the only way to cover a 32 bit
 * timer counter on a host build.
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "encoder_accum.h"

/** Counter width of a 16-bit STM32 timer, TIM3 and TIM4 among them. */
#define WIDTH16 16
/** Counter width of a 32-bit STM32 timer, TIM2 and TIM5 on the G4 and the F4. */
#define WIDTH32 32

#define SPAN16 0x10000
#define MAX16 0xFFFF
#define MAX32 0xFFFFFFFFU

/** Interval the velocity cases are measured over. */
#define INTERVAL_US 1000U

/**
 * Velocity is derived through a cycle-to-microsecond conversion that rounds, so
 * the interval the helper measures is not exactly INTERVAL_US on every value of
 * CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC. One percent covers that without hiding a
 * wrong sign or a factor-of-two error.
 */
#define VELOCITY_TOLERANCE(expected) (((expected) < 0 ? -(expected) : (expected)) / 100 + 1)

static uint32_t cycles_for_us(uint32_t us)
{
	return k_us_to_cyc_ceil32(us);
}

/** @brief Advance by one reading INTERVAL_US after the previous one. */
static void step(struct encoder_accum *accum, uint64_t raw, uint32_t *cycle,
		 struct encoder_accum_sample *sample)
{
	*cycle += cycles_for_us(INTERVAL_US);
	encoder_accum_update(accum, raw, *cycle, sample);
}

ZTEST(encoder_accum, test_forward_wrap_past_the_top_of_a_16_bit_counter)
{
	struct encoder_accum accum;
	struct encoder_accum_sample sample;
	uint32_t cycle = 0;

	encoder_accum_init(&accum, WIDTH16, false);
	encoder_accum_reset(&accum, 0, MAX16 - 9);

	step(&accum, MAX16, &cycle, &sample);
	zassert_equal(sample.position, 9);

	/* 0xFFFF -> 0x0004 is five counts forward, not 65531 counts backward. */
	step(&accum, 4, &cycle, &sample);
	zassert_equal(sample.position, 14);
}

ZTEST(encoder_accum, test_backward_wrap_past_the_bottom_of_a_16_bit_counter)
{
	struct encoder_accum accum;
	struct encoder_accum_sample sample;
	uint32_t cycle = 0;

	encoder_accum_init(&accum, WIDTH16, false);
	encoder_accum_reset(&accum, 0, 4);

	step(&accum, 0, &cycle, &sample);
	zassert_equal(sample.position, -4);

	step(&accum, MAX16, &cycle, &sample);
	zassert_equal(sample.position, -5);
}

ZTEST(encoder_accum, test_a_difference_of_exactly_half_a_span_counts_as_forward)
{
	struct encoder_accum accum;
	struct encoder_accum_sample sample;
	uint32_t cycle = 0;

	encoder_accum_init(&accum, WIDTH16, false);
	encoder_accum_reset(&accum, 0, 0);

	step(&accum, SPAN16 / 2, &cycle, &sample);
	zassert_equal(sample.position, SPAN16 / 2);

	/* Congruent to the step above, so it resolves the same way round. */
	step(&accum, 0, &cycle, &sample);
	zassert_equal(sample.position, SPAN16);
}

ZTEST(encoder_accum, test_forward_wrap_past_the_top_of_a_32_bit_counter)
{
	struct encoder_accum accum;
	struct encoder_accum_sample sample;
	uint32_t cycle = 0;

	encoder_accum_init(&accum, WIDTH32, false);
	encoder_accum_reset(&accum, 0, MAX32 - 9U);

	step(&accum, MAX32, &cycle, &sample);
	zassert_equal(sample.position, 9);

	step(&accum, 4, &cycle, &sample);
	zassert_equal(sample.position, 14);
}

ZTEST(encoder_accum, test_backward_wrap_past_the_bottom_of_a_32_bit_counter)
{
	struct encoder_accum accum;
	struct encoder_accum_sample sample;
	uint32_t cycle = 0;

	encoder_accum_init(&accum, WIDTH32, false);
	encoder_accum_reset(&accum, 0, 4);

	step(&accum, 0, &cycle, &sample);
	zassert_equal(sample.position, -4);

	step(&accum, MAX32, &cycle, &sample);
	zassert_equal(sample.position, -5);
}

ZTEST(encoder_accum, test_a_difference_across_the_middle_of_a_32_bit_counter_resolves_forward)
{
	struct encoder_accum accum;
	struct encoder_accum_sample sample;
	uint32_t cycle = 0;

	/* Readings either side of the middle of the counter, where the span
	 * itself has to be compared against and a 32 bit span does not fit.
	 */
	encoder_accum_init(&accum, WIDTH32, false);
	encoder_accum_reset(&accum, 0, 0x7FFFFFFFU);

	step(&accum, 0x80000000U, &cycle, &sample);
	zassert_equal(sample.position, 1);

	step(&accum, 0xC0000000U, &cycle, &sample);
	zassert_equal(sample.position, 0x40000001);
}

ZTEST(encoder_accum, test_velocity_is_counts_per_second_over_the_measured_interval)
{
	struct encoder_accum accum;
	struct encoder_accum_sample sample;
	uint32_t cycle = 0;
	const int32_t expected = 100 * (int32_t)USEC_PER_SEC / (int32_t)INTERVAL_US;

	encoder_accum_init(&accum, WIDTH16, false);
	encoder_accum_reset(&accum, 0, 0);

	/* No interval has been measured yet, so the first reading has no
	 * velocity to report and the caller must not treat that as a zero.
	 */
	step(&accum, 100, &cycle, &sample);
	zassert_false(sample.has_velocity);

	step(&accum, 200, &cycle, &sample);
	zassert_true(sample.has_velocity);
	zassert_within(sample.sample_interval_us, INTERVAL_US, INTERVAL_US / 100 + 1);
	zassert_within(sample.velocity, expected, VELOCITY_TOLERANCE(expected));
}

ZTEST(encoder_accum, test_velocity_is_negative_when_the_counter_runs_backwards)
{
	struct encoder_accum accum;
	struct encoder_accum_sample sample;
	uint32_t cycle = 0;
	const int32_t expected = -100 * (int32_t)USEC_PER_SEC / (int32_t)INTERVAL_US;

	encoder_accum_init(&accum, WIDTH16, false);
	encoder_accum_reset(&accum, 0, 500);

	step(&accum, 400, &cycle, &sample);
	step(&accum, 300, &cycle, &sample);

	zassert_true(sample.has_velocity);
	zassert_within(sample.velocity, expected, VELOCITY_TOLERANCE(expected));
}

ZTEST(encoder_accum, test_velocity_saturates_instead_of_wrapping_on_a_32_bit_counter)
{
	struct encoder_accum accum;
	struct encoder_accum_sample sample;
	uint32_t cycle = 0;

	/* Half a span of a 32 bit counter in one interval works out to about
	 * 2.1e12 counts/s, which int32_t cannot hold. Truncating would report a
	 * small, plausible-looking number of the wrong sign.
	 */
	encoder_accum_init(&accum, WIDTH32, false);
	encoder_accum_reset(&accum, 0, 0);

	step(&accum, 1, &cycle, &sample);
	step(&accum, 0x80000000U, &cycle, &sample);

	zassert_true(sample.has_velocity);
	zassert_equal(sample.velocity, INT32_MAX, "velocity %d", sample.velocity);

	/* And the same the other way round. The difference has to clear half a
	 * span, which resolves as forward motion, to come out negative at all.
	 */
	encoder_accum_init(&accum, WIDTH32, false);
	encoder_accum_reset(&accum, 0, 0);

	step(&accum, 1, &cycle, &sample);
	step(&accum, 0x80000002U, &cycle, &sample);

	zassert_true(sample.has_velocity);
	zassert_equal(sample.velocity, INT32_MIN, "velocity %d", sample.velocity);
}

ZTEST(encoder_accum, test_two_readings_in_the_same_cycle_report_no_velocity)
{
	struct encoder_accum accum;
	struct encoder_accum_sample sample;

	encoder_accum_init(&accum, WIDTH16, false);
	encoder_accum_reset(&accum, 0, 0);

	encoder_accum_update(&accum, 100, 4242, &sample);
	encoder_accum_update(&accum, 200, 4242, &sample);

	zassert_false(sample.has_velocity);
	zassert_equal(sample.position, 200);
}

ZTEST(encoder_accum, test_inverting_flips_the_sign_of_both_position_and_velocity)
{
	struct encoder_accum accum;
	struct encoder_accum_sample sample;
	uint32_t cycle = 0;
	const int32_t expected = -100 * (int32_t)USEC_PER_SEC / (int32_t)INTERVAL_US;

	encoder_accum_init(&accum, WIDTH16, true);
	encoder_accum_reset(&accum, 0, 0);

	step(&accum, 100, &cycle, &sample);
	zassert_equal(sample.position, -100);

	step(&accum, 200, &cycle, &sample);
	zassert_equal(sample.position, -200);
	zassert_within(sample.velocity, expected, VELOCITY_TOLERANCE(expected));
}

ZTEST(encoder_accum, test_inverting_still_resolves_a_wrap_the_short_way_round)
{
	struct encoder_accum accum;
	struct encoder_accum_sample sample;
	uint32_t cycle = 0;

	encoder_accum_init(&accum, WIDTH16, true);
	encoder_accum_reset(&accum, 0, MAX16);

	step(&accum, 4, &cycle, &sample);
	zassert_equal(sample.position, -5);
}

ZTEST(encoder_accum, test_set_position_shifts_the_reported_position_without_moving_the_counter)
{
	struct encoder_accum accum;
	struct encoder_accum_sample sample;
	uint32_t cycle = 0;

	encoder_accum_init(&accum, WIDTH16, false);
	encoder_accum_reset(&accum, 0, 0);

	step(&accum, 100, &cycle, &sample);
	zassert_equal(sample.position, 100);

	zassert_ok(encoder_accum_set_position(&accum, 1000));
	zassert_equal(encoder_accum_position(&accum), 1000);

	step(&accum, 150, &cycle, &sample);
	zassert_equal(sample.position, 1050);
}

ZTEST(encoder_accum, test_an_offset_shifts_the_position_without_being_seen_as_motion)
{
	struct encoder_accum accum;
	struct encoder_accum_sample sample;
	uint32_t cycle = 0;
	const int32_t expected = 50 * (int32_t)USEC_PER_SEC / (int32_t)INTERVAL_US;

	encoder_accum_init(&accum, WIDTH16, false);
	encoder_accum_reset(&accum, 0, 0);

	step(&accum, 50, &cycle, &sample);
	step(&accum, 100, &cycle, &sample);

	/* A jump of a million counts in the reported position that came from an
	 * offset rather than from the counter moving.
	 */
	zassert_ok(encoder_accum_set_position(&accum, 1000000));

	step(&accum, 150, &cycle, &sample);
	zassert_equal(sample.position, 1000050);
	zassert_within(sample.velocity, expected, VELOCITY_TOLERANCE(expected));
}

ZTEST(encoder_accum, test_set_position_rejects_a_position_with_no_headroom)
{
	struct encoder_accum accum;
	struct encoder_accum_sample sample;
	uint32_t cycle = 0;

	encoder_accum_init(&accum, WIDTH16, false);
	encoder_accum_reset(&accum, 0, 0);
	step(&accum, 100, &cycle, &sample);

	/* Accepting these would make the offset itself, or a later sum of the raw
	 * count and the offset, overflow int64_t.
	 */
	zassert_equal(encoder_accum_set_position(&accum, INT64_MIN), -EINVAL);
	zassert_equal(encoder_accum_set_position(&accum, INT64_MAX), -EINVAL);
	zassert_equal(encoder_accum_set_position(&accum, ENCODER_ACCUM_POSITION_LIMIT + 1), -EINVAL);

	zassert_equal(encoder_accum_position(&accum), 100, "a rejected request moved the position");

	zassert_ok(encoder_accum_set_position(&accum, ENCODER_ACCUM_POSITION_LIMIT));
}

ZTEST(encoder_accum, test_wrap_delta_resolves_without_touching_an_accumulator)
{
	/* The straddle check in the AMT21 driver compares two readings without
	 * advancing anything, so the modular difference is reachable on its own.
	 */
	zassert_equal(encoder_accum_wrap_delta(14, 4, 16380), 8);
	zassert_equal(encoder_accum_wrap_delta(14, 16380, 4), -8);
	zassert_equal(encoder_accum_wrap_delta(WIDTH16, 4, MAX16), 5);
	zassert_equal(encoder_accum_wrap_delta(WIDTH32, 4, MAX32), 5);
}

ZTEST(encoder_accum, test_set_position_without_a_reading_to_offset_from_fails)
{
	struct encoder_accum accum;

	encoder_accum_init(&accum, WIDTH16, false);

	zassert_false(encoder_accum_is_valid(&accum));
	zassert_equal(encoder_accum_set_position(&accum, 1000), -ENODATA);
}

ZTEST(encoder_accum, test_invalidating_stops_the_accumulator_until_it_is_reset)
{
	struct encoder_accum accum;
	struct encoder_accum_sample sample;
	uint32_t cycle = 0;

	encoder_accum_init(&accum, WIDTH16, false);
	encoder_accum_reset(&accum, 0, 0);
	step(&accum, 100, &cycle, &sample);

	encoder_accum_invalidate(&accum);

	zassert_false(encoder_accum_is_valid(&accum));
	zassert_equal(encoder_accum_set_position(&accum, 1000), -ENODATA);
}

ZTEST(encoder_accum, test_resetting_drops_the_offset_and_the_measured_interval)
{
	struct encoder_accum accum;
	struct encoder_accum_sample sample;
	uint32_t cycle = 0;

	encoder_accum_init(&accum, WIDTH16, false);
	encoder_accum_reset(&accum, 0, 0);

	step(&accum, 100, &cycle, &sample);
	step(&accum, 200, &cycle, &sample);
	zassert_ok(encoder_accum_set_position(&accum, 1000000));
	zassert_true(sample.has_velocity);

	/* What a driver does on recovery from offline: continue from the
	 * absolute reading the device gave, not from what came before.
	 */
	encoder_accum_reset(&accum, 7000, 300);
	zassert_equal(encoder_accum_position(&accum), 7000);

	/* The interval spanning the outage is not a sample interval, so the
	 * first reading after a reset carries no velocity.
	 */
	step(&accum, 350, &cycle, &sample);
	zassert_false(sample.has_velocity);
	zassert_equal(sample.position, 7050);
}

ZTEST(encoder_accum, test_an_absolute_start_position_is_preserved_across_a_wrap)
{
	struct encoder_accum accum;
	struct encoder_accum_sample sample;
	uint32_t cycle = 0;

	/* An AMT21 multi-turn device rebuilds from turns << resolution plus the
	 * single-turn reading, which is well outside one span.
	 */
	encoder_accum_init(&accum, 14, false);
	encoder_accum_reset(&accum, (3 << 14) + 16380, 16380);

	step(&accum, 3, &cycle, &sample);
	zassert_equal(sample.position, (4 << 14) + 3);
}

ZTEST_SUITE(encoder_accum, NULL, NULL, NULL, NULL, NULL);
