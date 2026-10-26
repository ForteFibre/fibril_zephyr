/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>

#include <odrive_sequencer/odrive_sequencer.hpp>

namespace odrive_sequencer
{

namespace
{

/* States in which the driver has been asked to arm the axis, so leaving them
 * any other way than into READY has to disarm it. */
bool arming_or_armed(State state)
{
  return (state == State::READY_WAIT) || (state == State::READY) ||
         (state == State::SWITCH_TO_READY_WAIT);
}

}  // namespace

uint32_t error_bits(const struct odrive_feedback & feedback)
{
  uint32_t errors = feedback.active_errors;

  if ((feedback.valid_mask & ODRIVE_FEEDBACK_ERROR) != 0U) {
    errors |= feedback.disarm_reason;
  }

  return errors;
}

Sequencer::Sequencer()
: _settings{false, 0.05f},
  _state(State::UNKNOWN),
  _calibrated(false),
  _request_ms(0),
  _last_clear_ms(-1),
  _pending{},
  _has_pending(false),
  _active_type(0),
  _has_active(false)
{
}

void Sequencer::enter(State next)
{
  if (next == State::ERROR) {
    _last_clear_ms = -1;
  }

  _state = next;
}

/* The driver keeps the control mode across motor_enable() but drops the
 * value. Selecting the mode first arms the axis in the mode it is about to be
 * commanded in, so no later mode change sends the driver back to arming; the
 * value goes in again straight after, so the driver has a setpoint to send,
 * and to feed the ODrive's watchdog with, the moment the axis is armed. */
void Sequencer::arm(int64_t now_ms, Port & port)
{
  if (_has_pending) {
    (void)port.apply(_pending);
  }

  (void)port.enable();

  if (_has_pending) {
    (void)port.apply(_pending);
  }

  _request_ms = now_ms;
}

void Sequencer::flush_pending(Port & port)
{
  if (!_has_pending) {
    return;
  }

  _has_pending = false;
  (void)port.apply(_pending);
  _active_type = _pending.type;
  _has_active = true;
}

bool Sequencer::answered(const struct odrive_feedback & feedback, int64_t now_ms) const
{
  const bool closed_loop_requested =
    (_state == State::READY_WAIT) || (_state == State::SWITCH_TO_READY_WAIT);

  if (closed_loop_requested && (feedback.axis_state == ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) &&
      (feedback.timestamp_ms > _request_ms)) {
    return true;
  }

  const int64_t due = _request_ms + response_delay_ms;

  if ((now_ms < due) || (feedback.timestamp_ms < due)) {
    return false;
  }

  /* ros_odrive waits out BUSY except for closed loop, which never finishes by
   * itself. */
  return closed_loop_requested || (feedback.procedure_result != ODRIVE_PROCEDURE_RESULT_BUSY);
}

bool Sequencer::switch_stop_complete(const struct odrive_feedback & feedback) const
{
  return (feedback.axis_state == ODRIVE_AXIS_STATE_IDLE) &&
         (fabsf(feedback.velocity) <= _settings.switch_stop_velocity);
}

/* The adapter's request_state_response(), judging the request now waited on by
 * the axis state and procedure result it reports. */
void Sequencer::evaluate(const struct odrive_feedback & feedback, int64_t now_ms, Port & port)
{
  const uint8_t axis_state = (uint8_t)feedback.axis_state;
  const uint8_t result = (uint8_t)feedback.procedure_result;

  /* Leaving closed loop for a mode switch reports the closed-loop procedure as
   * CANCELLED while the heartbeat already shows IDLE. */
  const bool idle_cancelled = (result == procedure_result_cancelled) &&
                              (axis_state == ODRIVE_AXIS_STATE_IDLE) &&
                              (_state == State::SWITCH_TO_IDLE_WAIT);

  if ((result == procedure_result_cancelled) && !idle_cancelled) {
    if (arming_or_armed(_state)) {
      (void)port.disable();
    }

    enter(State::IDLE);
    return;
  }

  /* Closed loop keeps reporting BUSY once reached (firmware >= 0.6.11). */
  const bool closed_loop_busy =
    (result == ODRIVE_PROCEDURE_RESULT_BUSY) &&
    (axis_state == ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) &&
    ((_state == State::READY_WAIT) || (_state == State::SWITCH_TO_READY_WAIT));

  if ((result != ODRIVE_PROCEDURE_RESULT_SUCCESS) && !idle_cancelled && !closed_loop_busy) {
    if (arming_or_armed(_state)) {
      (void)port.disable();
    }

    enter(State::ERROR);
    return;
  }

  switch (_state) {
    case State::CALIBRATE_WAIT:
      if (axis_state == ODRIVE_AXIS_STATE_IDLE) {
        _calibrated = true;
        enter(State::CALIBRATED);
      } else if (axis_state == ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) {
        /* An ODrive configured to start in closed loop skips calibration. The
         * adapter took that as ready; here arming waits for a command, so the
         * axis is idled until then. */
        (void)port.disable();
        _calibrated = true;
        enter(State::CALIBRATED);
      } else {
        enter(State::IDLE);
      }
      break;
    case State::READY_WAIT:
    case State::SWITCH_TO_READY_WAIT:
      if (axis_state == ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) {
        enter(State::READY);
        flush_pending(port);
      } else {
        (void)port.disable();
        _calibrated = false;
        enter(State::ERROR);
      }
      break;
    case State::SWITCH_TO_IDLE_WAIT:
      if (axis_state != ODRIVE_AXIS_STATE_IDLE) {
        enter(State::ERROR);
      } else if (switch_stop_complete(feedback)) {
        arm(now_ms, port);
        enter(State::SWITCH_TO_READY_WAIT);
      } else {
        enter(State::SWITCH_STOP_WAIT);
      }
      break;
    default:
      break;
  }
}

void Sequencer::step(
  const struct odrive_feedback & feedback, bool running, int64_t now_ms, Port & port)
{
  const uint32_t errors = error_bits(feedback);
  const bool has_error = errors != 0U;
  const bool powered = feedback.online && ((errors & error_dc_bus_under_voltage) == 0U);

  /* The adapter saw a silent ODrive only from IDLE and ERROR. An armed axis
   * that goes silent is dropped too, so the driver stops commanding it. */
  if (!feedback.online && (_state != State::UNKNOWN) && (_state != State::NOT_POWERED)) {
    if (arming_or_armed(_state)) {
      (void)port.disable();
    }

    enter(State::NOT_POWERED);
    return;
  }

  switch (_state) {
    case State::UNKNOWN:
      if (feedback.online) {
        enter(powered ? State::IDLE : State::NOT_POWERED);
      }
      break;
    case State::NOT_POWERED:
      if (powered) {
        enter(State::IDLE);
      }
      break;
    case State::IDLE:
      if (!powered) {
        enter(State::NOT_POWERED);
      } else if (_calibrated) {
        enter(State::CALIBRATED);
      } else if (running) {
        const uint8_t calibration = _settings.full_calibration
                                      ? axis_state_full_calibration_sequence
                                      : (uint8_t)ODRIVE_AXIS_STATE_MOTOR_CALIBRATION;

        if (port.request_state(calibration) == 0) {
          _request_ms = now_ms;
          enter(State::CALIBRATE_WAIT);
        }
      }
      break;
    case State::CALIBRATED:
      if (!powered) {
        enter(State::NOT_POWERED);
      } else if (_has_pending && running) {
        arm(now_ms, port);
        enter(State::READY_WAIT);
      }
      break;
    case State::CALIBRATE_WAIT:
    case State::READY_WAIT:
    case State::SWITCH_TO_IDLE_WAIT:
    case State::SWITCH_TO_READY_WAIT:
      if (answered(feedback, now_ms)) {
        evaluate(feedback, now_ms, port);
      }
      break;
    case State::READY: {
      const uint8_t axis_state = (uint8_t)feedback.axis_state;

      /* Leaving closed loop without being asked is an error too, so commands
       * are not silently dropped. UNDEFINED means no heartbeat state yet. */
      if (has_error || ((axis_state != axis_state_undefined) &&
                        (axis_state != ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL))) {
        (void)port.disable();
        enter(State::ERROR);
      }
      break;
    }
    case State::SWITCH_STOP_WAIT:
      if (switch_stop_complete(feedback)) {
        arm(now_ms, port);
        enter(State::SWITCH_TO_READY_WAIT);
      }
      break;
    case State::ERROR:
      if (!has_error) {
        enter(State::IDLE);
      } else if ((_last_clear_ms < 0) || ((now_ms - _last_clear_ms) >= clear_errors_interval_ms)) {
        _last_clear_ms = now_ms;

        if (!powered) {
          enter(State::NOT_POWERED);
        } else {
          (void)port.clear_errors();
        }
      }
      break;
  }
}

void Sequencer::command(const Setpoint & setpoint, int64_t now_ms, Port & port)
{
  switch (_state) {
    case State::SWITCH_TO_IDLE_WAIT:
    case State::SWITCH_STOP_WAIT:
    case State::SWITCH_TO_READY_WAIT:
      _pending = setpoint;
      _has_pending = true;
      break;
    case State::READY:
      if (_has_active && (_active_type != setpoint.type)) {
        _pending = setpoint;
        _has_pending = true;
        (void)port.disable();
        _request_ms = now_ms;
        enter(State::SWITCH_TO_IDLE_WAIT);
      } else {
        (void)port.apply(setpoint);
        _active_type = setpoint.type;
        _has_active = true;
      }
      break;
    default:
      /* The adapter dropped commands until READY and relied on the master
       * repeating them. Holding the latest one is what arms the axis here. */
      _pending = setpoint;
      _has_pending = true;
      break;
  }
}

void Sequencer::release(Port & port)
{
  _has_pending = false;
  _has_active = false;

  switch (_state) {
    case State::READY_WAIT:
    case State::READY:
    case State::SWITCH_TO_IDLE_WAIT:
    case State::SWITCH_STOP_WAIT:
    case State::SWITCH_TO_READY_WAIT:
      (void)port.disable();
      enter(State::IDLE);
      break;
    default:
      break;
  }
}

}  // namespace odrive_sequencer
