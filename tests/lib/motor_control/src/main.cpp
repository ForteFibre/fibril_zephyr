/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

#include <motor_control/motor_control.hpp>

using motor_control::EncoderSample;
using motor_control::Mode;
using motor_control::MotorControl;

/* zassert_within evaluates its first argument twice, which would run
 * update() twice.
 */
#define zassert_near(actual, expected)                                                             \
  do {                                                                                             \
    const float actual_ = (actual);                                                                \
    zassert_within(actual_, (expected), 1e-5f, "%f", static_cast<double>(actual_));                \
  } while (0)

static EncoderSample at(int64_t position, float velocity = 0.0f)
{
  return EncoderSample{true, position, velocity};
}

static EncoderSample offline()
{
  return EncoderSample{false, 0, 0.0f};
}

static float run(MotorControl & mc, const EncoderSample & encoder, uint32_t ticks)
{
  float duty = 0.0f;
  for (uint32_t i = 0; i < ticks; i++) {
    duty = mc.update(encoder);
  }
  return duty;
}

ZTEST(motor_control, test_duty_passes_through_and_clamps)
{
  MotorControl mc;
  mc.set_duty(0.25f);
  zassert_near(mc.update(at(0)), 0.25f);
  mc.set_duty(-3.0f);
  zassert_near(mc.update(at(0)), -1.0f);
  zassert_equal(mc.mode(), Mode::DUTY);
}

ZTEST(motor_control, test_duty_slew_limit_is_per_second)
{
  MotorControl mc;
  mc.duty_slew_limit(10.0f);
  mc.set_duty(1.0f);
  zassert_near(mc.update(at(0)), 0.01f);
  zassert_near(run(mc, at(0), 9), 0.1f);
}

ZTEST(motor_control, test_speed_input_matches_mbed_counts_per_tick_over_dt)
{
  /* Mbed fed the speed PID gain * (counts per tick) / 0.001. Three counts per
   * 1 ms tick at gain 0.002 is 6 units/s there, and 3000 counts/s here.
   */
  MotorControl mc;
  mc.encoder_gain(0.002f);
  mc.speed_filter_coefficient(1.0f);
  mc.speed_controller().kp(0.1f);
  mc.set_speed(8.0f);
  zassert_near(mc.update(at(0, 3000.0f)), 0.1f * (8.0f - 6.0f));
  zassert_near(mc.speed(), 6.0f);
}

ZTEST(motor_control, test_speed_filter_weights_newest_sample_by_coefficient)
{
  MotorControl mc;
  zassert_true(mc.speed_filter_coefficient(0.5f));
  mc.update(at(0, 100.0f));
  zassert_near(mc.speed(), 50.0f);
  mc.update(at(0, 100.0f));
  zassert_near(mc.speed(), 75.0f);
}

ZTEST(motor_control, test_speed_filter_coefficient_outside_unit_interval_is_rejected)
{
  MotorControl mc;
  zassert_false(mc.speed_filter_coefficient(1.5f));
  zassert_false(mc.speed_filter_coefficient(-0.1f));
  zassert_false(mc.speed_filter_coefficient(NAN));
  zassert_near(mc.speed_filter_coefficient(), 0.9f);
}

ZTEST(motor_control, test_position_mode_scales_counts_by_gain)
{
  MotorControl mc;
  mc.encoder_gain(0.5f);
  mc.position_controller().kp(0.01f);
  mc.set_position(100.0f);
  zassert_near(mc.update(at(100)), 0.01f * (100.0f - 50.0f));
  zassert_near(mc.position(), 50.0f);
  zassert_equal(mc.mode(), Mode::POSITION);
}

ZTEST(motor_control, test_speed_position_cascades_position_into_speed_target)
{
  MotorControl mc;
  mc.speed_filter_coefficient(1.0f);
  mc.position_controller().kp(2.0f).max(100.0f);
  mc.speed_controller().kp(0.01f);
  mc.set_position_speed(10.0f);
  /* r_v = 2 * (10 - 4) = 12, duty = 0.01 * (12 - 5) */
  zassert_near(mc.update(at(4, 5.0f)), 0.07f);
}

ZTEST(motor_control, test_speed_position_uses_filtered_speed)
{
  MotorControl mc;
  mc.speed_filter_coefficient(0.5f);
  mc.position_controller().kp(1.0f).max(100.0f);
  mc.speed_controller().kp(0.01f);
  mc.set_position_speed(20.0f);
  /* filtered speed = 0.5 * 10 = 5, not the raw 10 */
  zassert_near(mc.update(at(0, 10.0f)), 0.01f * (20.0f - 5.0f));
}

ZTEST(motor_control, test_invalid_encoder_drops_closed_loop_to_duty_zero)
{
  MotorControl mc;
  mc.speed_controller().kp(1.0f);
  mc.set_speed(0.5f);
  zassert_near(mc.update(at(0)), 0.5f);
  zassert_near(mc.update(offline()), 0.0f);
  zassert_equal(mc.mode(), Mode::DUTY);
  /* stays in DUTY once the encoder is back */
  zassert_near(mc.update(at(0)), 0.0f);
}

ZTEST(motor_control, test_invalid_encoder_stops_a_slew_limited_output_at_once)
{
  MotorControl mc;
  mc.duty_slew_limit(1.0f);
  mc.speed_controller().kp(1.0f);
  mc.set_speed(1.0f);
  zassert_near(run(mc, at(0), 500), 0.5f);
  zassert_near(mc.update(offline()), 0.0f);
  /* the slew limit itself is kept */
  mc.set_duty(1.0f);
  zassert_near(mc.update(at(0)), 0.001f);
}

ZTEST(motor_control, test_invalid_sample_keeps_last_position_and_filtered_speed)
{
  MotorControl mc;
  mc.speed_filter_coefficient(0.5f);
  mc.update(at(40, 100.0f));
  mc.update(offline());
  zassert_near(mc.position(), 40.0f);
  zassert_near(mc.speed(), 50.0f);
}

ZTEST(motor_control, test_invalid_encoder_leaves_open_loop_duty_alone)
{
  MotorControl mc;
  mc.set_duty(0.3f);
  zassert_near(mc.update(offline()), 0.3f);
}

ZTEST(motor_control, test_velocity_ff_adds_to_speed_target_and_lapses_after_timeout)
{
  MotorControl mc;
  mc.speed_controller().kp(0.1f);
  mc.set_position_speed(0.0f);
  mc.set_velocity_ff(2.0f);
  zassert_near(mc.update(at(0)), 0.2f);
  zassert_near(run(mc, at(0), MotorControl::ff_timeout_ticks - 1), 0.2f);
  zassert_near(mc.update(at(0)), 0.0f);
}

ZTEST(motor_control, test_duty_ff_adds_to_output_and_lapses_after_timeout)
{
  MotorControl mc;
  mc.set_position_speed(0.0f);
  mc.set_duty_ff(0.3f);
  zassert_near(run(mc, at(0), MotorControl::ff_timeout_ticks), 0.3f);
  zassert_near(mc.update(at(0)), 0.0f);
}

ZTEST(motor_control, test_feed_forward_is_ignored_outside_speed_position)
{
  MotorControl mc;
  mc.set_speed(0.0f);
  mc.set_duty_ff(0.3f);
  mc.set_velocity_ff(5.0f);
  zassert_near(mc.update(at(0)), 0.0f);
  zassert_equal(mc.mode(), Mode::SPEED);
}

ZTEST(motor_control, test_feed_forward_set_again_revives_a_lapsed_value)
{
  MotorControl mc;
  mc.set_position_speed(0.0f);
  mc.set_duty_ff(0.3f);
  run(mc, at(0), MotorControl::ff_timeout_ticks + 5);
  mc.set_duty_ff(0.2f);
  zassert_near(mc.update(at(0)), 0.2f);
}

ZTEST(motor_control, test_coulomb_friction_follows_target_speed_sign_outside_deadband)
{
  MotorControl mc;
  mc.friction_coulomb(0.05f);
  mc.set_position_speed(0.0f);
  mc.set_velocity_ff(1.0f);
  zassert_near(mc.update(at(0)), 0.05f);
  mc.set_velocity_ff(-1.0f);
  zassert_near(mc.update(at(0)), -0.05f);
  mc.set_velocity_ff(MotorControl::friction_sign_deadband / 2);
  zassert_near(mc.update(at(0)), 0.0f);
}

ZTEST(motor_control, test_viscous_friction_scales_with_target_speed)
{
  MotorControl mc;
  mc.friction_viscous(0.02f);
  mc.set_position_speed(0.0f);
  mc.set_velocity_ff(5.0f);
  zassert_near(mc.update(at(0)), 0.1f);
}

ZTEST(motor_control, test_speed_position_output_is_clamped_after_all_terms)
{
  MotorControl mc;
  mc.friction_coulomb(0.5f);
  mc.set_position_speed(0.0f);
  mc.set_velocity_ff(1.0f);
  mc.set_duty_ff(0.8f);
  zassert_near(mc.update(at(0)), 1.0f);
}

ZTEST(motor_control, test_stall_breaker_trips_after_timeout_and_zeroes_output)
{
  MotorControl mc;
  mc.stall_timeout_ticks(10);
  mc.set_duty(0.5f);
  zassert_near(run(mc, at(0), 11), 0.5f);
  zassert_true(mc.stalled());
  zassert_near(mc.update(at(0)), 0.0f);
}

ZTEST(motor_control, test_stall_breaker_does_not_trip_while_running_backwards)
{
  MotorControl mc;
  mc.stall_timeout_ticks(10);
  mc.set_duty(-0.5f);
  zassert_near(run(mc, at(0, -100.0f), 50), -0.5f);
  zassert_false(mc.stalled());
}

ZTEST(motor_control, test_stall_breaker_ignores_the_sign_of_the_gain)
{
  MotorControl mc;
  mc.encoder_gain(-1.0f);
  mc.stall_timeout_ticks(10);
  mc.set_duty(0.5f);
  run(mc, at(0, 100.0f), 50);
  zassert_false(mc.stalled());
}

ZTEST(motor_control, test_stall_breaker_does_not_count_invalid_samples)
{
  MotorControl mc;
  mc.stall_timeout_ticks(10);
  mc.set_duty(0.5f);
  zassert_near(run(mc, offline(), 50), 0.5f);
  zassert_false(mc.stalled());
}

ZTEST(motor_control, test_stall_breaker_ignores_small_duty)
{
  MotorControl mc;
  mc.stall_timeout_ticks(10);
  mc.set_duty(MotorControl::stall_duty_threshold / 2);
  run(mc, at(0), 50);
  zassert_false(mc.stalled());
}

ZTEST(motor_control, test_stall_breaker_is_off_with_zero_timeout)
{
  MotorControl mc;
  mc.set_duty(1.0f);
  run(mc, at(0), 1000);
  zassert_false(mc.stalled());
}

ZTEST(motor_control, test_reset_safety_clears_a_tripped_breaker)
{
  MotorControl mc;
  mc.stall_timeout_ticks(10);
  mc.set_duty(0.5f);
  run(mc, at(0), 12);
  zassert_true(mc.stalled());
  mc.reset_safety();
  zassert_near(mc.update(at(0)), 0.5f);
}

ZTEST(motor_control, test_changing_stall_timeout_keeps_a_tripped_breaker_tripped)
{
  MotorControl mc;
  mc.stall_timeout_ticks(10);
  mc.set_duty(0.5f);
  run(mc, at(0), 12);
  mc.stall_timeout_ticks(200);
  zassert_true(mc.stalled());
  mc.stall_timeout_ticks(0);
  zassert_false(mc.stalled());
}

ZTEST(motor_control, test_entering_speed_mode_clears_the_integrator)
{
  MotorControl mc;
  mc.speed_controller().ki(0.1f).i_saturation(10.0f).max(10.0f);
  mc.set_speed(1.0f);
  run(mc, at(0), 5);
  mc.set_duty(0.0f);
  mc.update(at(0));
  mc.set_speed(1.0f);
  zassert_near(mc.update(at(0)), 0.1f);
}

ZTEST(motor_control, test_reset_all_restores_power_on_state)
{
  MotorControl mc;
  mc.encoder_gain(3.0f);
  mc.speed_filter_coefficient(0.2f);
  mc.stall_timeout_ticks(5);
  mc.friction_coulomb(0.1f);
  mc.speed_controller().kp(1.0f);
  mc.set_speed(1.0f);
  mc.update(at(0));

  mc.reset_all();
  zassert_equal(mc.mode(), Mode::DUTY);
  zassert_near(mc.encoder_gain(), 1.0f);
  zassert_near(mc.speed_filter_coefficient(), 0.9f);
  zassert_equal(mc.stall_timeout_ticks(), 0u);
  zassert_near(mc.update(at(0)), 0.0f);
  mc.set_speed(1.0f);
  zassert_near(mc.update(at(0)), 0.0f);
}

ZTEST(motor_control, test_reset_all_drops_a_slew_limited_output_at_once)
{
  MotorControl mc;
  mc.duty_slew_limit(1.0f);
  mc.set_duty(1.0f);
  run(mc, at(0), 100);
  mc.reset_all();
  zassert_near(mc.update(at(0)), 0.0f);
}

ZTEST_SUITE(motor_control, NULL, NULL, NULL, NULL, NULL);
