#ifndef FIBRIL_ZEPHYR_INCLUDE_DRIVERS_MOTOR_ODRIVE_H_
#define FIBRIL_ZEPHYR_INCLUDE_DRIVERS_MOTOR_ODRIVE_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file
 * @brief Control interface specific to the ODrive CANSimple driver.
 *
 * The generic motor class in @ref drivers/motor.h carries a single 16-bit
 * command and gives no unit to it, so it cannot express what an ODrive
 * accepts: a position target with velocity and torque feed-forward in one
 * frame, and setpoints whose range is bounded by the ODrive's own
 * configuration rather than by the protocol. The command side therefore lives
 * here, in @c float. @ref motor_set_output returns @c -ENOTSUP for every mode.
 *
 * What the generic class does carry is @ref motor_enable, @ref motor_disable
 * and @ref motor_get_feedback. The feedback snapshot reports the scaled
 * position and, when the axis has a motor thermistor, the temperature; the
 * remaining measurements are read through @ref odrive_get_feedback.
 *
 * **Units are the ones on the wire: rev, rev/s, Nm and A.** The ODrive's own
 * limits and gains are in the same units, so a value passed here reads the
 * same in odrivetool.
 *
 * The driver never writes the ODrive's saved configuration on its own. It
 * depends on that configuration being right, in particular on the cyclic
 * message rates: only heartbeat and encoder estimates are enabled by default,
 * and a measurement whose message is disabled never appears in
 * @ref odrive_feedback.valid_mask.
 *
 * @defgroup motor_odrive ODrive driver
 * @ingroup motor_interface
 * @{
 */

/** @brief Control mode the ODrive is running in. */
enum odrive_control_mode {
  /** Voltage. Not supported by this driver. */
  ODRIVE_CONTROL_MODE_VOLTAGE = 0,
  /** Torque, in Nm. */
  ODRIVE_CONTROL_MODE_TORQUE = 1,
  /** Velocity, in rev/s. */
  ODRIVE_CONTROL_MODE_VELOCITY = 2,
  /** Position, in rev. */
  ODRIVE_CONTROL_MODE_POSITION = 3,
};

/**
 * @brief How the ODrive shapes the setpoint before its controller sees it.
 *
 * @ref ODRIVE_INPUT_MODE_TRAP_TRAJ re-plans its trajectory every time it
 * receives a position frame, so the driver stops re-sending an unchanged
 * target in that mode. That also means the ODrive's watchdog is not fed
 * between target changes: do not combine it with @c enable_watchdog.
 */
enum odrive_input_mode {
  /** Setpoint is applied as given. */
  ODRIVE_INPUT_MODE_PASSTHROUGH = 1,
  /** Velocity is ramped towards the setpoint. */
  ODRIVE_INPUT_MODE_VEL_RAMP = 2,
  /** Position is filtered towards the setpoint. */
  ODRIVE_INPUT_MODE_POS_FILTER = 3,
  /** Position follows a trapezoidal trajectory. */
  ODRIVE_INPUT_MODE_TRAP_TRAJ = 5,
  /** Torque is ramped towards the setpoint. */
  ODRIVE_INPUT_MODE_TORQUE_RAMP = 6,
};

/**
 * @brief Axis states used by this driver.
 *
 * The protocol defines more; these are the ones the driver commands or tests
 * for. Others may still appear in @ref odrive_feedback.axis_state while the
 * ODrive runs a procedure of its own.
 */
enum odrive_axis_state {
  /** Output stage is off. */
  ODRIVE_AXIS_STATE_IDLE = 1,
  /** Motor calibration. */
  ODRIVE_AXIS_STATE_MOTOR_CALIBRATION = 4,
  /** Encoder offset calibration. */
  ODRIVE_AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7,
  /** Axis is driving the output. */
  ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL = 8,
  /** Homing. */
  ODRIVE_AXIS_STATE_HOMING = 11,
};

/** @brief Result of the procedure the axis last ran. */
enum odrive_procedure_result {
  /** Finished without error. */
  ODRIVE_PROCEDURE_RESULT_SUCCESS = 0,
  /** Still running. */
  ODRIVE_PROCEDURE_RESULT_BUSY = 1,
};

/**
 * @brief Measurements that may be present in an @ref odrive_feedback snapshot.
 *
 * Each value corresponds to one cyclic message. A bit stays clear while its
 * message has never arrived, and is cleared again once the value is older than
 * the bus's @c estimate-timeout-ms. **Most of these messages are disabled in
 * the ODrive's default configuration**, so a clear bit usually means the rate
 * was never enabled rather than that the bus is down.
 */
enum odrive_feedback_field {
  /** Position and velocity, from @c Get_Encoder_Estimates. */
  ODRIVE_FEEDBACK_ESTIMATES = 1,
  /** Iq setpoint and measurement, from @c Get_Iq. */
  ODRIVE_FEEDBACK_IQ = 1 << 1,
  /** FET and motor temperature, from @c Get_Temperature. */
  ODRIVE_FEEDBACK_TEMPERATURE = 1 << 2,
  /** Torque target and estimate, from @c Get_Torques. */
  ODRIVE_FEEDBACK_TORQUES = 1 << 3,
  /** Bus voltage and current, from @c Get_Bus_Voltage_Current. */
  ODRIVE_FEEDBACK_BUS = 1 << 4,
  /** Active errors and disarm reason, from @c Get_Error. */
  ODRIVE_FEEDBACK_ERROR = 1 << 5,
};

/**
 * @brief Latest measurements and axis state, in wire units.
 *
 * @ref valid_mask says which measurements are present. The state fields below
 * it come from the heartbeat, which is enabled by default, so they are valid
 * whenever @ref online is true.
 */
struct odrive_feedback
{
  /**
   * @brief Bit-mask of valid measurements in this snapshot.
   *
   * This uses values from @ref odrive_feedback_field.
   */
  uint32_t valid_mask;

  /** Position in rev, continuous across turns. */
  float position;
  /** Velocity in rev/s. */
  float velocity;
  /** Quadrature current setpoint in A. */
  float iq_setpoint;
  /** Measured quadrature current in A. */
  float iq_measured;
  /** Inverter temperature in degrees Celsius. */
  float fet_temperature;
  /** Motor temperature in degrees Celsius. Meaningless without a thermistor. */
  float motor_temperature;
  /** Commanded torque in Nm. */
  float torque_target;
  /** Estimated torque in Nm. */
  float torque_estimate;
  /** DC bus voltage in V. */
  float bus_voltage;
  /** DC bus current in A. */
  float bus_current;

  /** State the axis reported in its most recent heartbeat. */
  enum odrive_axis_state axis_state;
  /** Result of the procedure the axis last ran. */
  enum odrive_procedure_result procedure_result;
  /** Errors active on the axis. Non-zero means the axis has tripped. */
  uint32_t active_errors;
  /** Why the axis last disarmed. Only valid with @ref ODRIVE_FEEDBACK_ERROR. */
  uint32_t disarm_reason;
  /** True when the current trajectory has finished. */
  bool trajectory_done;

  /** Mode the driver is currently commanding. */
  enum odrive_control_mode control_mode;
  /** Input shaping the driver is currently commanding. */
  enum odrive_input_mode input_mode;
  /** True when the driver considers the axis armed and is sending setpoints. */
  bool enabled;

  /** True when a heartbeat has arrived within @c heartbeat-timeout-ms. */
  bool online;
  /** True when the estimates are present but older than @c estimate-timeout-ms. */
  bool stale;
  /** Timestamp of the most recent frame from this axis, in milliseconds. */
  int64_t timestamp_ms;
};

/**
 * @brief Callback invoked when an axis has something new to report.
 *
 * Invoked from a work queue owned by the bus, never from the CAN receive
 * interrupt, so the ODrive API may be called from here. In particular
 * @ref motor_enable is how an application re-arms an axis that disarmed on its
 * own; the driver never does that by itself.
 *
 * Calls may coalesce. If several frames arrive before the work runs, the
 * callback is invoked once with the latest snapshot; no queue of events is
 * kept. Nothing is lost by this: the ODrive holds @c active_errors until they
 * are cleared.
 *
 * @param dev ODrive axis device instance.
 * @param feedback Snapshot as of the moment the callback was prepared.
 * @param user_data Value passed when the callback was registered.
 */
typedef void (*odrive_callback_t)(
  const struct device * dev, const struct odrive_feedback * feedback, void * user_data);

/**
 * @brief Register a callback for changes in the axis state.
 *
 * Invoked when the heartbeat reports a different @c axis_state,
 * @c procedure_result or @c active_errors, and when the axis goes online or
 * offline. This is the cheap callback: the state of a healthy axis does not
 * change, so it does not fire in steady state.
 *
 * @param dev ODrive axis device instance.
 * @param callback Callback to invoke, or NULL to remove the current one.
 * @param user_data Opaque value passed back to @p callback.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not an ODrive axis.
 */
int odrive_set_state_callback(
  const struct device * dev, odrive_callback_t callback, void * user_data);

/**
 * @brief Register a callback for new measurements.
 *
 * Invoked when @c Get_Encoder_Estimates arrives, which is every
 * @c encoder_msg_rate_ms on the ODrive and defaults to 10 ms. The other cyclic
 * messages update the snapshot without invoking this.
 *
 * @param dev ODrive axis device instance.
 * @param callback Callback to invoke, or NULL to remove the current one.
 * @param user_data Opaque value passed back to @p callback.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not an ODrive axis.
 */
int odrive_set_feedback_callback(
  const struct device * dev, odrive_callback_t callback, void * user_data);

/**
 * @brief Read the latest measurements and axis state.
 *
 * This never blocks on the bus. It reports what the most recent frames
 * carried.
 *
 * @param dev ODrive axis device instance.
 * @param feedback Destination for the snapshot.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not an ODrive axis, or @p feedback is NULL.
 * @retval -ENODATA The axis has not sent a heartbeat yet.
 * @retval -EAGAIN The estimates are older than @c estimate-timeout-ms.
 */
int odrive_get_feedback(const struct device * dev, struct odrive_feedback * feedback);

/**
 * @brief Command a position, switching to @ref ODRIVE_CONTROL_MODE_POSITION.
 *
 * @param dev ODrive axis device instance.
 * @param position Position in rev.
 * @param velocity_ff Velocity feed-forward in rev/s, quantised to 0.001.
 * @param torque_ff Torque feed-forward in Nm, quantised to 0.001.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not an ODrive axis.
 */
int odrive_set_position(
  const struct device * dev, float position, float velocity_ff, float torque_ff);

/**
 * @brief Command a velocity, switching to @ref ODRIVE_CONTROL_MODE_VELOCITY.
 *
 * @param dev ODrive axis device instance.
 * @param velocity Velocity in rev/s.
 * @param torque_ff Torque feed-forward in Nm.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not an ODrive axis.
 */
int odrive_set_velocity(const struct device * dev, float velocity, float torque_ff);

/**
 * @brief Command a torque, switching to @ref ODRIVE_CONTROL_MODE_TORQUE.
 *
 * @param dev ODrive axis device instance.
 * @param torque Torque in Nm.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not an ODrive axis.
 */
int odrive_set_torque(const struct device * dev, float torque);

/**
 * @brief Select how the ODrive shapes the setpoint.
 *
 * Takes effect on the next command interval. An armed axis is not stopped to
 * apply it.
 *
 * @param dev ODrive axis device instance.
 * @param mode Input shaping to apply.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not an ODrive axis.
 */
int odrive_set_input_mode(const struct device * dev, enum odrive_input_mode mode);

/**
 * @brief Override the velocity and current limits the ODrive enforces.
 *
 * The driver does not write limits on its own: an ODrive runs with the limits
 * saved in its own configuration unless this is called. The values are not
 * persisted and are lost when the ODrive reboots.
 *
 * @param dev ODrive axis device instance.
 * @param velocity_limit Velocity limit in rev/s.
 * @param current_limit Current limit in A.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not an ODrive axis.
 * @retval negative_errno The frame could not be queued.
 */
int odrive_set_limits(const struct device * dev, float velocity_limit, float current_limit);

/**
 * @brief Set the limits of the trapezoidal trajectory planner.
 *
 * Only meaningful with @ref ODRIVE_INPUT_MODE_TRAP_TRAJ.
 *
 * @param dev ODrive axis device instance.
 * @param velocity_limit Cruise velocity in rev/s.
 * @param accel_limit Acceleration limit in rev/s^2.
 * @param decel_limit Deceleration limit in rev/s^2.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not an ODrive axis.
 * @retval negative_errno A frame could not be queued.
 */
int odrive_set_traj_limits(
  const struct device * dev, float velocity_limit, float accel_limit, float decel_limit);

/**
 * @brief Override the gains of the ODrive's position and velocity loops.
 *
 * As with @ref odrive_set_limits, the driver writes nothing until this is
 * called, and the values are not persisted.
 *
 * @param dev ODrive axis device instance.
 * @param pos_gain Position loop gain, in (rev/s)/rev.
 * @param vel_gain Velocity loop gain, in Nm/(rev/s).
 * @param vel_integrator_gain Velocity loop integrator gain, in Nm/rev.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not an ODrive axis.
 * @retval negative_errno A frame could not be queued.
 */
int odrive_set_gains(
  const struct device * dev, float pos_gain, float vel_gain, float vel_integrator_gain);

/**
 * @brief Redefine the current position as @p position.
 *
 * @param dev ODrive axis device instance.
 * @param position Position in rev to assign to where the axis is now.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not an ODrive axis.
 * @retval negative_errno The frame could not be queued.
 */
int odrive_set_absolute_position(const struct device * dev, float position);

/**
 * @brief Clear the errors active on the axis without arming it.
 *
 * @ref motor_enable already clears errors once as part of arming, and the
 * driver deliberately does not clear them again while it retries, so that a
 * fault cannot be cycled through automatically. Use this when an application
 * wants the errors gone without arming.
 *
 * @param dev ODrive axis device instance.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not an ODrive axis.
 * @retval negative_errno The frame could not be queued.
 */
int odrive_clear_errors(const struct device * dev);

/**
 * @brief Disarm the axis through the protocol's emergency stop.
 *
 * Unlike @ref motor_disable, which requests the idle state and leaves the axis
 * ready to be armed again, this makes the ODrive disarm with
 * @c ESTOP_REQUESTED, which stays as an active error. The next
 * @ref motor_enable clears it as part of arming.
 *
 * The frame is sent immediately rather than at the next command interval.
 *
 * @param dev ODrive axis device instance.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not an ODrive axis.
 * @retval negative_errno The frame could not be queued.
 */
int odrive_estop(const struct device * dev);

/**
 * @brief Request an arbitrary axis state.
 *
 * This is the escape hatch for the states the driver does not command itself,
 * such as the calibration procedures. It does not clear errors first and does
 * not wait: watch @ref odrive_feedback.axis_state and
 * @ref odrive_feedback.procedure_result for the outcome.
 *
 * @param dev ODrive axis device instance.
 * @param state State to request.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not an ODrive axis.
 * @retval -EBUSY The driver is arming or has armed this axis.
 * @retval negative_errno The frame could not be queued.
 */
int odrive_request_axis_state(const struct device * dev, enum odrive_axis_state state);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
