/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include <algorithm>
#include <cmath>

#include <motor_control/motor_control.hpp>

namespace motor_control
{

MotorControl::MotorControl()
{
  reset_all();
}

void MotorControl::reset_all()
{
  _mode = Mode::DUTY;
  _encoder_gain = 1.0f;
  _speed_filter_coefficient = 0.9f;
  _speed_filtered = 0.0f;
  _position = 0.0f;
  _duty = 0.0f;

  for (auto * pid : {&_speed_controller, &_position_controller}) {
    pid->reset();
    pid->kp(0.0f).ki(0.0f).kd(0.0f).max(1.0f).i_saturation(1.0f).target(0.0f);
  }
  _duty_limiter.acc_max(0.0f);
  drop_duty_limiter();

  _velocity_ff = 0.0f;
  _velocity_ff_ticks = 0;
  _duty_ff = 0.0f;
  _duty_ff_ticks = 0;
  _friction_coulomb = 0.0f;
  _friction_viscous = 0.0f;

  _stall_timeout_ticks = 0;
  _stall_ticks = 0;
  _stall_tripped = false;
}

float MotorControl::update(const EncoderSample & encoder)
{
  /* An invalid sample carries no measurement: keep the last position and
   * leave the filter where it was, so a closed loop resumed later does not
   * start from a speed pulled towards zero.
   */
  if (encoder.valid) {
    const float speed = _encoder_gain * encoder.velocity;
    _position = _encoder_gain * static_cast<float>(encoder.position);
    _speed_filtered =
      _speed_filter_coefficient * speed + (1.0f - _speed_filter_coefficient) * _speed_filtered;
  }

  /* Leaves the mode at DUTY, as CanMotorMbed does: the host has to send its
   * setpoint again once the encoder is back. Unlike CanMotorMbed the output
   * stops at once rather than ramping down at the slew limit, since nothing
   * closes the loop during the ramp.
   */
  if (!encoder.valid && _mode != Mode::DUTY) {
    set_duty(0.0f);
    drop_duty_limiter();
  }

  switch (_mode) {
    case Mode::SPEED:
      _speed_controller.update(_speed_filtered);
      _duty_limiter.target(_speed_controller.output());
      break;

    case Mode::POSITION:
      _position_controller.update(_position);
      _duty_limiter.target(_position_controller.output());
      break;

    case Mode::SPEED_POSITION:
      _duty_limiter.target(speed_position_duty(_speed_filtered));
      break;

    case Mode::DUTY:
      break;
  }

  /* Lapse after the tick that read them, so a value set just before a tick
   * is applied for exactly ff_timeout_ticks ticks.
   */
  if (_velocity_ff_ticks > 0) {
    _velocity_ff_ticks--;
  }
  if (_duty_ff_ticks > 0) {
    _duty_ff_ticks--;
  }

  _duty_limiter.update(tick_s);
  const float limited = std::clamp(_duty_limiter.output(), -1.0f, 1.0f);

  /* The breaker acts on the next tick, as the Mbed one did: this tick's
   * output is decided before the stall is counted.
   */
  _duty = _stall_tripped ? 0.0f : limited;
  watch_stall(limited, encoder);
  return _duty;
}

void MotorControl::drop_duty_limiter()
{
  /* AccelerationLimit has no reset: with acc_max 0 an update() passes the
   * target straight through, which discards the old output.
   */
  const float limit = _duty_limiter.acc_max();
  _duty_limiter.acc_max(0.0f);
  _duty_limiter.target(0.0f);
  _duty_limiter.update(tick_s);
  _duty_limiter.acc_max(limit);
}

float MotorControl::speed_position_duty(float speed)
{
  _position_controller.update(_position);
  const float velocity_ff = _velocity_ff_ticks > 0 ? _velocity_ff : 0.0f;
  const float target_speed = _position_controller.output() + velocity_ff;
  _speed_controller.target(target_speed);
  _speed_controller.update(speed);

  float duty = _speed_controller.output();
  if (_duty_ff_ticks > 0) {
    duty += _duty_ff;
  }
  /* Friction follows the target speed rather than the measured one, which
   * would chatter around standstill.
   */
  if (target_speed > friction_sign_deadband) {
    duty += _friction_coulomb;
  } else if (target_speed < -friction_sign_deadband) {
    duty -= _friction_coulomb;
  }
  duty += _friction_viscous * target_speed;
  return std::clamp(duty, -1.0f, 1.0f);
}

void MotorControl::watch_stall(float duty, const EncoderSample & encoder)
{
  /* Without a valid sample there is no telling whether it moves. */
  if (_stall_timeout_ticks == 0 || !encoder.valid) {
    return;
  }
  const bool driving = std::fabs(duty) > stall_duty_threshold;
  /* Raw counts: the sign of the gain must not decide whether it is moving. */
  const bool moving = std::fabs(encoder.velocity) > 0.0f;
  if (!driving || moving) {
    _stall_ticks = 0;
    return;
  }
  if (_stall_ticks < _stall_timeout_ticks) {
    _stall_ticks++;
  } else {
    _stall_tripped = true;
  }
}

void MotorControl::set_duty(float duty)
{
  _mode = Mode::DUTY;
  _duty_limiter.target(std::clamp(duty, -1.0f, 1.0f));
}

void MotorControl::set_speed(float speed)
{
  if (_mode != Mode::SPEED) {
    _speed_controller.reset();
    _mode = Mode::SPEED;
  }
  _speed_controller.target(speed);
}

void MotorControl::set_position(float position)
{
  if (_mode != Mode::POSITION) {
    _position_controller.reset();
    _mode = Mode::POSITION;
  }
  _position_controller.target(position);
}

void MotorControl::set_position_speed(float position)
{
  if (_mode != Mode::SPEED_POSITION) {
    _position_controller.reset();
    _speed_controller.reset();
    _mode = Mode::SPEED_POSITION;
  }
  _position_controller.target(position);
}

void MotorControl::set_velocity_ff(float speed)
{
  _velocity_ff = speed;
  _velocity_ff_ticks = ff_timeout_ticks;
}

void MotorControl::set_duty_ff(float duty)
{
  _duty_ff = duty;
  _duty_ff_ticks = ff_timeout_ticks;
}

void MotorControl::reset_safety()
{
  _stall_ticks = 0;
  _stall_tripped = false;
}

bool MotorControl::speed_filter_coefficient(float coefficient)
{
  if (!(coefficient >= 0.0f && coefficient <= 1.0f)) {
    return false;
  }
  _speed_filter_coefficient = coefficient;
  return true;
}

void MotorControl::stall_timeout_ticks(uint32_t ticks)
{
  /* Only switching the breaker on or off clears it, as in CanMotorMbed.
   * Retuning the timeout leaves a tripped breaker tripped.
   */
  if ((ticks == 0) != (_stall_timeout_ticks == 0)) {
    reset_safety();
  }
  _stall_timeout_ticks = ticks;
}

}  // namespace motor_control
