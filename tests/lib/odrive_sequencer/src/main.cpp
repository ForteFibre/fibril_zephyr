/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/ztest.h>

#include <odrive_sequencer/odrive_sequencer.hpp>

using odrive_sequencer::Port;
using odrive_sequencer::Sequencer;
using odrive_sequencer::Setpoint;
using odrive_sequencer::State;

namespace
{

constexpr uint8_t TARGET_VELOCITY = 1;
constexpr uint8_t TARGET_POSITION = 3;

constexpr uint8_t AXIS_IDLE = ODRIVE_AXIS_STATE_IDLE;
constexpr uint8_t AXIS_MOTOR_CALIBRATION = ODRIVE_AXIS_STATE_MOTOR_CALIBRATION;
constexpr uint8_t AXIS_CLOSED_LOOP = ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL;

constexpr uint8_t RESULT_SUCCESS = ODRIVE_PROCEDURE_RESULT_SUCCESS;
constexpr uint8_t RESULT_BUSY = ODRIVE_PROCEDURE_RESULT_BUSY;
constexpr uint8_t RESULT_CANCELLED = odrive_sequencer::procedure_result_cancelled;
constexpr uint8_t RESULT_POLE_PAIR_CPR_MISMATCH = 5;

constexpr uint32_t ERROR_MOTOR = 0x00000100U;

class Recorder : public Port
{
public:
  int enable() override
  {
    enables++;
    order[calls++ % sizeof(order)] = 'e';
    return 0;
  }

  int disable() override
  {
    disables++;
    order[calls++ % sizeof(order)] = 'd';
    return 0;
  }

  int request_state(uint8_t state) override
  {
    requests++;
    requested = state;
    return request_result;
  }

  int clear_errors() override
  {
    clears++;
    return 0;
  }

  int apply(const Setpoint & setpoint) override
  {
    applies++;
    applied = setpoint;
    order[calls++ % sizeof(order)] = 'a';
    return 0;
  }

  int enables = 0;
  int disables = 0;
  int requests = 0;
  int clears = 0;
  int applies = 0;
  uint8_t requested = 0;
  int request_result = 0;
  Setpoint applied{};
  char order[16] = {};
  size_t calls = 0;
};

struct odrive_feedback heartbeat(
  uint8_t axis_state, uint8_t result, int64_t stamp_ms, uint32_t errors = 0U,
  float velocity = 0.0f)
{
  struct odrive_feedback fb;

  memset(&fb, 0, sizeof(fb));
  fb.online = true;
  fb.axis_state = (enum odrive_axis_state)axis_state;
  fb.procedure_result = (enum odrive_procedure_result)result;
  fb.active_errors = errors;
  fb.velocity = velocity;
  fb.timestamp_ms = stamp_ms;
  return fb;
}

struct odrive_feedback silent()
{
  struct odrive_feedback fb;

  memset(&fb, 0, sizeof(fb));
  return fb;
}

Setpoint velocity(float value)
{
  return Setpoint{TARGET_VELOCITY, ODRIVE_CONTROL_MODE_VELOCITY, ODRIVE_INPUT_MODE_PASSTHROUGH,
                  value};
}

Setpoint position(float value)
{
  return Setpoint{TARGET_POSITION, ODRIVE_CONTROL_MODE_POSITION, ODRIVE_INPUT_MODE_PASSTHROUGH,
                  value};
}

/* The adapter's calibration on boot, ending in CALIBRATED at now_ms. */
int64_t calibrate(Sequencer & seq, Recorder & port)
{
  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, 0), true, 0, port);
  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, 0), true, 0, port);
  zassert_equal(seq.state(), State::CALIBRATE_WAIT);
  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, 1000), true, 1000, port);
  zassert_equal(seq.state(), State::CALIBRATED);
  return 1000;
}

/* Calibrated, commanded and in closed loop, ending in READY at the returned
 * time. */
int64_t bring_up(Sequencer & seq, Recorder & port, const Setpoint & first)
{
  int64_t now = calibrate(seq, port);

  seq.command(first, now, port);
  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, now), true, now, port);
  zassert_equal(seq.state(), State::READY_WAIT);
  now += 50;
  seq.step(heartbeat(AXIS_CLOSED_LOOP, RESULT_BUSY, now), true, now, port);
  zassert_equal(seq.state(), State::READY);
  return now;
}

}  // namespace

ZTEST(odrive_sequencer, test_does_not_calibrate_without_the_master)
{
  Sequencer seq;
  Recorder port;

  for (int64_t now = 0; now < 3000; now += 100) {
    seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, now), false, now, port);
  }

  zassert_equal(seq.state(), State::IDLE);
  zassert_equal(port.requests, 0);
}

ZTEST(odrive_sequencer, test_calibrates_with_the_configured_procedure)
{
  Sequencer seq;
  Recorder port;

  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, 0), true, 0, port);
  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, 0), true, 0, port);
  zassert_equal(port.requested, AXIS_MOTOR_CALIBRATION);

  Sequencer full;
  Recorder full_port;

  full.settings().full_calibration = true;
  full.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, 0), true, 0, full_port);
  full.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, 0), true, 0, full_port);
  zassert_equal(full_port.requested, odrive_sequencer::axis_state_full_calibration_sequence);
}

ZTEST(odrive_sequencer, test_calibration_is_judged_after_a_second_and_once_not_busy)
{
  Sequencer seq;
  Recorder port;

  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, 0), true, 0, port);
  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, 0), true, 0, port);

  /* The heartbeat before the ODrive took the request still says IDLE. */
  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, 500), true, 500, port);
  zassert_equal(seq.state(), State::CALIBRATE_WAIT);

  seq.step(heartbeat(AXIS_MOTOR_CALIBRATION, RESULT_BUSY, 1500), true, 1500, port);
  zassert_equal(seq.state(), State::CALIBRATE_WAIT);

  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, 4000), true, 4000, port);
  zassert_equal(seq.state(), State::CALIBRATED);
  zassert_true(seq.calibrated());
}

ZTEST(odrive_sequencer, test_failed_calibration_is_an_error)
{
  Sequencer seq;
  Recorder port;

  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, 0), true, 0, port);
  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, 0), true, 0, port);
  seq.step(heartbeat(AXIS_IDLE, RESULT_POLE_PAIR_CPR_MISMATCH, 1000, ERROR_MOTOR), true, 1000,
           port);

  zassert_equal(seq.state(), State::ERROR);
  zassert_false(seq.calibrated());
}

ZTEST(odrive_sequencer, test_calibrated_axis_waits_for_a_command_to_arm)
{
  Sequencer seq;
  Recorder port;
  const int64_t now = calibrate(seq, port);

  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, now + 2000), true, now + 2000, port);
  zassert_equal(seq.state(), State::CALIBRATED);
  zassert_equal(port.enables, 0);
}

ZTEST(odrive_sequencer, test_command_selects_the_mode_before_arming_and_is_sent_right_after)
{
  Sequencer seq;
  Recorder port;
  const int64_t now = calibrate(seq, port);

  seq.command(velocity(2.0f), now, port);
  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, now), true, now, port);

  /* The driver drops the value on enable but keeps the mode. */
  zassert_mem_equal(port.order, "aea", 3);
  zassert_equal(port.applied.control_mode, ODRIVE_CONTROL_MODE_VELOCITY);
  zassert_within(port.applied.value, 2.0f, 1e-6f);
}

ZTEST(odrive_sequencer, test_command_is_sent_again_once_ready)
{
  Sequencer seq;
  Recorder port;

  bring_up(seq, port, velocity(2.0f));

  zassert_mem_equal(port.order, "aeaa", 4);
  zassert_equal(port.applied.control_mode, ODRIVE_CONTROL_MODE_VELOCITY);
  zassert_within(port.applied.value, 2.0f, 1e-6f);
}

ZTEST(odrive_sequencer, test_arming_waits_for_the_master)
{
  Sequencer seq;
  Recorder port;
  const int64_t now = calibrate(seq, port);

  seq.command(velocity(1.0f), now, port);
  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, now), false, now, port);

  zassert_equal(seq.state(), State::CALIBRATED);
  zassert_equal(port.enables, 0);
}

ZTEST(odrive_sequencer, test_closed_loop_not_reached_in_a_second_is_an_error)
{
  Sequencer seq;
  Recorder port;
  int64_t now = calibrate(seq, port);

  seq.command(velocity(1.0f), now, port);
  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, now), true, now, port);
  now += 1000;
  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, now), true, now, port);

  zassert_equal(seq.state(), State::ERROR);
  zassert_false(seq.calibrated());
  zassert_equal(port.disables, 1);
}

ZTEST(odrive_sequencer, test_same_type_command_is_applied_directly_in_ready)
{
  Sequencer seq;
  Recorder port;
  const int64_t now = bring_up(seq, port, velocity(1.0f));
  const int applies = port.applies;

  seq.command(velocity(3.0f), now, port);

  zassert_equal(seq.state(), State::READY);
  zassert_equal(port.applies, applies + 1);
  zassert_within(port.applied.value, 3.0f, 1e-6f);
}

ZTEST(odrive_sequencer, test_type_change_goes_through_idle_and_waits_for_the_axis_to_stop)
{
  Sequencer seq;
  Recorder port;
  int64_t now = bring_up(seq, port, velocity(1.0f));

  seq.command(position(0.5f), now, port);
  zassert_equal(seq.state(), State::SWITCH_TO_IDLE_WAIT);
  zassert_equal(port.disables, 1);

  /* The ODrive reports the closed-loop procedure as cancelled. */
  now += 1000;
  seq.step(heartbeat(AXIS_IDLE, RESULT_CANCELLED, now, 0U, 0.5f), true, now, port);
  zassert_equal(seq.state(), State::SWITCH_STOP_WAIT);

  now += 100;
  seq.step(heartbeat(AXIS_IDLE, RESULT_CANCELLED, now, 0U, 0.01f), true, now, port);
  zassert_equal(seq.state(), State::SWITCH_TO_READY_WAIT);
  zassert_equal(port.enables, 2);
  zassert_equal(port.applied.control_mode, ODRIVE_CONTROL_MODE_POSITION);

  now += 50;
  seq.step(heartbeat(AXIS_CLOSED_LOOP, RESULT_BUSY, now), true, now, port);
  zassert_equal(seq.state(), State::READY);
  zassert_within(port.applied.value, 0.5f, 1e-6f);
}

ZTEST(odrive_sequencer, test_commands_during_a_switch_replace_the_pending_one)
{
  Sequencer seq;
  Recorder port;
  int64_t now = bring_up(seq, port, velocity(1.0f));

  seq.command(position(0.5f), now, port);
  seq.command(position(0.7f), now, port);
  now += 1000;
  seq.step(heartbeat(AXIS_IDLE, RESULT_CANCELLED, now), true, now, port);
  now += 50;
  seq.step(heartbeat(AXIS_CLOSED_LOOP, RESULT_BUSY, now), true, now, port);

  zassert_equal(seq.state(), State::READY);
  zassert_within(port.applied.value, 0.7f, 1e-6f);
}

ZTEST(odrive_sequencer, test_error_in_ready_disarms_and_clears_every_half_second)
{
  Sequencer seq;
  Recorder port;
  int64_t now = bring_up(seq, port, velocity(1.0f));

  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, now, ERROR_MOTOR), true, now, port);
  zassert_equal(seq.state(), State::ERROR);
  zassert_equal(port.disables, 1);

  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, now, ERROR_MOTOR), true, now, port);
  zassert_equal(port.clears, 1);
  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, now + 400, ERROR_MOTOR), true, now + 400, port);
  zassert_equal(port.clears, 1);
  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, now + 500, ERROR_MOTOR), true, now + 500, port);
  zassert_equal(port.clears, 2);
}

ZTEST(odrive_sequencer, test_recovered_axis_rearms_on_the_next_command)
{
  Sequencer seq;
  Recorder port;
  int64_t now = bring_up(seq, port, velocity(1.0f));

  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, now, ERROR_MOTOR), true, now, port);
  now += 500;
  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, now), true, now, port);
  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, now), true, now, port);
  zassert_equal(seq.state(), State::CALIBRATED);

  seq.command(velocity(1.0f), now, port);
  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, now), true, now, port);
  zassert_equal(seq.state(), State::READY_WAIT);
  zassert_equal(port.enables, 2);
}

ZTEST(odrive_sequencer, test_leaving_closed_loop_unasked_is_an_error)
{
  Sequencer seq;
  Recorder port;
  const int64_t now = bring_up(seq, port, velocity(1.0f));

  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, now), true, now, port);

  zassert_equal(seq.state(), State::ERROR);
}

ZTEST(odrive_sequencer, test_release_disarms_and_waits_for_a_new_command)
{
  Sequencer seq;
  Recorder port;
  int64_t now = bring_up(seq, port, velocity(1.0f));

  seq.release(port);
  zassert_equal(seq.state(), State::IDLE);
  zassert_equal(port.disables, 1);

  now += 100;
  seq.step(heartbeat(AXIS_IDLE, RESULT_CANCELLED, now), true, now, port);
  seq.step(heartbeat(AXIS_IDLE, RESULT_CANCELLED, now), true, now, port);
  zassert_equal(seq.state(), State::CALIBRATED);
  zassert_equal(port.enables, 1);
}

ZTEST(odrive_sequencer, test_silent_axis_is_disarmed)
{
  Sequencer seq;
  Recorder port;

  bring_up(seq, port, velocity(1.0f));
  seq.step(silent(), true, 5000, port);

  zassert_equal(seq.state(), State::NOT_POWERED);
  zassert_equal(port.disables, 1);
}

ZTEST(odrive_sequencer, test_under_voltage_is_not_powered)
{
  Sequencer seq;
  Recorder port;

  seq.step(heartbeat(AXIS_IDLE, RESULT_SUCCESS, 0, odrive_sequencer::error_dc_bus_under_voltage),
           true, 0, port);

  zassert_equal(seq.state(), State::NOT_POWERED);
  zassert_equal(port.requests, 0);
}

ZTEST(odrive_sequencer, test_disarm_reason_counts_only_when_get_error_is_fresh)
{
  struct odrive_feedback fb = heartbeat(AXIS_IDLE, RESULT_SUCCESS, 0);

  fb.disarm_reason = ERROR_MOTOR;
  zassert_equal(odrive_sequencer::error_bits(fb), 0U);

  fb.valid_mask = ODRIVE_FEEDBACK_ERROR;
  zassert_equal(odrive_sequencer::error_bits(fb), ERROR_MOTOR);
}

ZTEST_SUITE(odrive_sequencer, NULL, NULL, NULL, NULL, NULL);
