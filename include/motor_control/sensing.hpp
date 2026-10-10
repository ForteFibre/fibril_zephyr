/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief The calibration input watch and the current baseline, ported from
 * CanMotorMbed's GenericSensorWorker and the current baseline of its
 * tmp/saramander/robomaster branch.
 *
 * Like the control core, these know nothing about Zephyr or CAN, and count
 * time in the caller's ticks.
 */

#ifndef APP_INCLUDE_MOTOR_CONTROL_SENSING_HPP_
#define APP_INCLUDE_MOTOR_CONTROL_SENSING_HPP_

#include <cmath>
#include <cstdint>

/**
 * @addtogroup motor_control
 * @{
 */

namespace motor_control
{

/**
 * @brief Reports when an input crosses a threshold, one watcher per motor.
 *
 * The level is |value| >= threshold. update() answers the first level it sees
 * after start() and every change after that. The previous level belongs to
 * the watcher, not to the input, so two motors watching the same input both
 * see every crossing.
 */
class LevelWatch
{
public:
  enum class Event : uint8_t { NONE, LOW, HIGH };

  void start()
  {
    _active = true;
    _known = false;
  }

  void stop() { _active = false; }

  bool active() const { return _active; }

  /** Feed one sample. A non-finite value or threshold is skipped and keeps
   * the previous level. */
  Event update(float value, float threshold)
  {
    if (!_active || !std::isfinite(value) || !std::isfinite(threshold)) {
      return Event::NONE;
    }

    const bool high = std::fabs(value) >= threshold;

    if (_known && (high == _high)) {
      return Event::NONE;
    }

    _known = true;
    _high = high;

    return high ? Event::HIGH : Event::LOW;
  }

private:
  bool _active = false;
  bool _known = false;
  bool _high = false;
};

/**
 * @brief Averages the motor current while the motor holds still.
 *
 * Any tick that is not still, or has no current reading, ends the run as
 * ABORTED, as CanMotorMbed did; the previous offset stays.
 */
class CurrentBaseline
{
public:
  enum class State : uint8_t { IDLE, RUNNING, DONE, ABORTED };

  /** Begin a run of `ticks` samples. 0 is taken as 1. */
  void start(uint32_t ticks)
  {
    _state = State::RUNNING;
    _remaining = (ticks == 0U) ? 1U : ticks;
    _sum = 0.0F;
    _count = 0U;
  }

  /** Feed one tick. Returns the state after it; DONE and ABORTED are
   * reported once, after which the state is IDLE again. */
  State update(bool still, bool valid, float current)
  {
    if (_state != State::RUNNING) {
      return State::IDLE;
    }

    if (!still || !valid || !std::isfinite(current)) {
      _state = State::IDLE;
      return State::ABORTED;
    }

    _sum += current;
    _count++;

    if (--_remaining > 0U) {
      return State::RUNNING;
    }

    _offset = _sum / (float)_count;
    _state = State::IDLE;

    return State::DONE;
  }

  bool running() const { return _state == State::RUNNING; }

  /** The mean of the last run that completed. 0 before any. */
  float offset() const { return _offset; }

private:
  State _state = State::IDLE;
  uint32_t _remaining = 0U;
  uint32_t _count = 0U;
  /* float, not double: the Cortex-M4F has no double-precision FPU, and a
   * run is at most a few thousand samples. */
  float _sum = 0.0F;
  float _offset = 0.0F;
};

}  // namespace motor_control

/** @} */

#endif /* APP_INCLUDE_MOTOR_CONTROL_SENSING_HPP_ */
