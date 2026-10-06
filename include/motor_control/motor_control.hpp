/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Per-motor control core ported from CanMotorMbed's MotorWorker.
 *
 * Knows nothing about Zephyr, CAN or the devicetree. The caller samples the
 * encoder, calls update() once per control tick, and writes the returned duty
 * to the motor. Time is counted in ticks, so the core can run on a host.
 */

#ifndef APP_INCLUDE_MOTOR_CONTROL_MOTOR_CONTROL_HPP_
#define APP_INCLUDE_MOTOR_CONTROL_MOTOR_CONTROL_HPP_

#include <cstdint>

#include <fibril/controller/acceleration_limit.hpp>
#include <fibril/controller/pid.hpp>

/**
 * @addtogroup motor_control
 * @{
 */

namespace motor_control
{

enum class Mode : uint8_t { DUTY, SPEED, POSITION, SPEED_POSITION };

/**
 * @brief One encoder reading, in the units of struct encoder_feedback.
 * @ingroup motor_control
 */
struct EncoderSample
{
  /** False while the encoder is offline or stale. */
  bool valid;
  /** Accumulated position in counts. */
  int64_t position;
  /** Velocity in counts per second. */
  float velocity;
};

/**
 * @brief Control state of one motor, advanced once per tick by update().
 * @ingroup motor_control
 */
class MotorControl
{
public:
  /** update() must be called at this period. The PID gains are per tick. */
  static constexpr uint32_t tick_us = 1000;
  static constexpr float tick_s = 0.001f;
  /** A feed-forward value lapses this many ticks after it was last set. */
  static constexpr uint32_t ff_timeout_ticks = 100;
  /** |r_v| below this, in user units per second, adds no Coulomb friction. */
  static constexpr float friction_sign_deadband = 0.01f;
  /** The stall breaker only watches outputs above this duty. */
  static constexpr float stall_duty_threshold = 0.1f;

  MotorControl();

  MotorControl(const MotorControl &) = delete;
  MotorControl & operator=(const MotorControl &) = delete;

  /** Run one control tick and return the duty to apply, in [-1, 1]. */
  float update(const EncoderSample & encoder);

  /** Back to the power-on state: DUTY at 0, every setting at its default. */
  void reset_all();

  void set_duty(float duty);
  void set_speed(float speed);
  void set_position(float position);
  void set_position_speed(float position);

  /** Only SPEED_POSITION reads these. Neither changes the mode. */
  void set_velocity_ff(float speed);
  void set_duty_ff(float duty);

  void reset_safety();

  /** User units per encoder count. Position and speed are scaled by it. */
  void encoder_gain(float gain) { _encoder_gain = gain; }
  float encoder_gain() const { return _encoder_gain; }

  /** Weight of the newest sample, in [0, 1]. 1 disables the filter. */
  bool speed_filter_coefficient(float coefficient);
  float speed_filter_coefficient() const { return _speed_filter_coefficient; }

  /** Duty slew limit in duty per second. 0 disables it. */
  void duty_slew_limit(float limit) { _duty_limiter.acc_max(limit); }

  /** Ticks of stall before the breaker trips. 0 disables the breaker. */
  void stall_timeout_ticks(uint32_t ticks);
  uint32_t stall_timeout_ticks() const { return _stall_timeout_ticks; }

  void friction_coulomb(float duty) { _friction_coulomb = duty; }
  void friction_viscous(float duty_per_speed) { _friction_viscous = duty_per_speed; }

  fibril::PIDController<float> & speed_controller() { return _speed_controller; }
  fibril::PIDController<float> & position_controller() { return _position_controller; }

  Mode mode() const { return _mode; }
  bool stalled() const { return _stall_tripped; }
  /** Filtered speed in user units per second. */
  float speed() const { return _speed_filtered; }
  /** Position in user units, as of the last update(). */
  float position() const { return _position; }
  /** The duty update() last returned. */
  float duty() const { return _duty; }

private:
  void drop_duty_limiter();
  float speed_position_duty(float speed);
  void watch_stall(float duty, const EncoderSample & encoder);

  fibril::PIDController<float> _speed_controller;
  fibril::PIDController<float> _position_controller;
  fibril::AccelerationLimit _duty_limiter;

  Mode _mode;
  float _encoder_gain;
  float _speed_filter_coefficient;
  float _speed_filtered;
  float _position;
  float _duty;

  float _velocity_ff;
  uint32_t _velocity_ff_ticks;
  float _duty_ff;
  uint32_t _duty_ff_ticks;
  float _friction_coulomb;
  float _friction_viscous;

  uint32_t _stall_timeout_ticks;
  uint32_t _stall_ticks;
  bool _stall_tripped;
};

}  // namespace motor_control

/** @} */

#endif /* APP_INCLUDE_MOTOR_CONTROL_MOTOR_CONTROL_HPP_ */
