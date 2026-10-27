/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Bring-up and recovery of one ODrive axis, ported from
 *        fibril_ros's fibril_odrive_adapter.
 *
 * Calibrates the axis, arms it when a command arrives, passes the commands
 * on, switches control modes through idle, and clears errors until the axis
 * can be armed again. It knows nothing about fibril_can: the caller reads the
 * driver's snapshot, hands it to step() once per tick, and implements Port
 * on top of include/drivers/motor/odrive.h. Time is the caller's uptime in
 * milliseconds, so the sequencer can run on a host.
 */

#ifndef APP_INCLUDE_ODRIVE_SEQUENCER_ODRIVE_SEQUENCER_HPP_
#define APP_INCLUDE_ODRIVE_SEQUENCER_ODRIVE_SEQUENCER_HPP_

#include <cstdint>

#include <drivers/motor/odrive.h>

/**
 * @addtogroup odrive_sequencer
 * @{
 */

namespace odrive_sequencer
{

/**
 * @brief Where the sequencer is with its axis.
 *
 * The order and the names are fibril_odrive_adapter's, so the number the
 * diagnostics carry means the same thing on both sides.
 */
enum class State : uint8_t {
  /** No heartbeat yet. */
  UNKNOWN,
  /** Silent, or reporting DC bus under-voltage. */
  NOT_POWERED,
  /** Powered and idle; decides between calibrating and waiting. */
  IDLE,
  /** Calibration requested. */
  CALIBRATE_WAIT,
  /** Calibrated and idle, waiting for a command to arm on. */
  CALIBRATED,
  /** Arming requested. */
  READY_WAIT,
  /** In closed loop; commands are passed on. */
  READY,
  /** Idle requested to change the control mode. */
  SWITCH_TO_IDLE_WAIT,
  /** Idle, waiting for the axis to slow down before re-arming. */
  SWITCH_STOP_WAIT,
  /** Re-arming after a control mode change. */
  SWITCH_TO_READY_WAIT,
  /** The axis reported an error, or left closed loop on its own. */
  ERROR,
};

/**
 * @brief One command, already in the ODrive's units.
 * @ingroup odrive_sequencer
 */
struct Setpoint
{
  /**
   * What the master asked for, as fibril_control_msgs/msg/Target's type. Two
   * types that map to the same control mode, DUTY and TORQUE, still count as
   * a change, as they did in the adapter.
   */
  uint8_t type;
  enum odrive_control_mode control_mode;
  enum odrive_input_mode input_mode;
  /** rev, rev/s or Nm, depending on the control mode. */
  float value;
};

/**
 * @brief What the sequencer does to the axis.
 * @ingroup odrive_sequencer
 *
 * The production implementation forwards to the ODrive driver; a test
 * records the calls. Return values are the driver's.
 */
class Port
{
public:
  virtual int enable() = 0;
  virtual int disable() = 0;
  virtual int request_state(uint8_t state) = 0;
  virtual int clear_errors() = 0;
  /** Select the setpoint's input and control mode, and its value. */
  virtual int apply(const Setpoint & setpoint) = 0;

protected:
  ~Port() = default;
};

/**
 * @brief Values the master may retune at any time.
 * @ingroup odrive_sequencer
 */
struct Settings
{
  /** Calibrate with FULL_CALIBRATION_SEQUENCE rather than MOTOR_CALIBRATION. */
  bool full_calibration;
  /** |velocity| in rev/s below which a mode switch may re-arm the axis. */
  float switch_stop_velocity;
};

/**
 * @brief The sequence of one axis, advanced by step().
 * @ingroup odrive_sequencer
 */
class Sequencer
{
public:
  /**
   * ros_odrive's request_axis_state answers with the first heartbeat at least
   * this long after the request, and the adapter judged every request by that
   * answer. The wait is kept for the states that report a procedure result;
   * closed loop is also accepted as soon as it is seen.
   */
  static constexpr int64_t response_delay_ms = 1000;
  /** The adapter cleared errors from its 500 ms update timer. */
  static constexpr int64_t clear_errors_interval_ms = 500;

  Sequencer();

  Sequencer(const Sequencer &) = delete;
  Sequencer & operator=(const Sequencer &) = delete;

  /**
   * Advance with the driver's latest snapshot.
   *
   * @param feedback What odrive_get_feedback() filled in, whatever it
   *        returned. Only @c online decides whether the axis is there.
   * @param running Whether the master is present. Calibration and arming wait
   *        for it, so a board powered on its own does not move the motor.
   * @param now_ms Uptime in milliseconds.
   * @param port What to do to the axis.
   */
  void step(const struct odrive_feedback & feedback, bool running, int64_t now_ms, Port & port);

  /**
   * A new command from the master. Applied at once in READY, held until the
   * axis is armed otherwise; a type change goes through idle first.
   */
  void command(const Setpoint & setpoint, int64_t now_ms, Port & port);

  /**
   * The master's commands stopped. Disarms an armed axis and forgets the held
   * command, so only a new command arms it again.
   */
  void release(Port & port);

  Settings & settings() { return _settings; }

  State state() const { return _state; }
  bool ready() const { return _state == State::READY; }
  bool calibrated() const { return _calibrated; }

private:
  void enter(State next);
  void arm(int64_t now_ms, Port & port);
  void flush_pending(Port & port);
  bool answered(const struct odrive_feedback & feedback, int64_t now_ms) const;
  void evaluate(const struct odrive_feedback & feedback, int64_t now_ms, Port & port);
  bool switch_stop_complete(const struct odrive_feedback & feedback) const;

  Settings _settings;
  State _state;
  bool _calibrated;

  /** When the request now being waited on was made. */
  int64_t _request_ms;
  int64_t _last_clear_ms;

  /** The latest command not yet applied. Arming waits for one. */
  Setpoint _pending;
  bool _has_pending;

  /** The type of the command last applied in READY. */
  uint8_t _active_type;
  bool _has_active;
};

/**
 * @brief Errors the axis reports, as the adapter combined them.
 * @ingroup odrive_sequencer
 *
 * The heartbeat carries active errors and disarm reason ORed together, and
 * Get_Error carries them apart. Whichever arrived last, the result is the
 * same.
 */
uint32_t error_bits(const struct odrive_feedback & feedback);

/** ODrive's ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE. */
constexpr uint32_t error_dc_bus_under_voltage = 0x00000200U;

/** ODrive AxisState values the driver's enum leaves out. */
constexpr uint8_t axis_state_undefined = 0U;
constexpr uint8_t axis_state_full_calibration_sequence = 3U;

/** ODrive ProcedureResult values the driver's enum leaves out. */
constexpr uint8_t procedure_result_cancelled = 2U;

}  // namespace odrive_sequencer

/** @} */

#endif /* APP_INCLUDE_ODRIVE_SEQUENCER_ODRIVE_SEQUENCER_HPP_ */
