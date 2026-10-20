#ifndef FIBRIL_ZEPHYR_INCLUDE_DRIVERS_MOTOR_ROBSTRIDE_H_
#define FIBRIL_ZEPHYR_INCLUDE_DRIVERS_MOTOR_ROBSTRIDE_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file
 * @brief Control interface specific to the RobStride actuator driver.
 *
 * The generic motor class in @ref drivers/motor.h carries a single 16-bit
 * command and gives no unit to it, so it cannot express what a RobStride
 * accepts: SI-valued position, velocity and current targets, and an operation
 * mode that takes five values in one frame. The command side therefore lives
 * here, in @c float and in SI units, and @ref motor_set_output returns
 * @c -ENOTSUP for every mode.
 *
 * What the generic class does carry is @ref motor_enable, @ref motor_disable
 * and @ref motor_get_feedback. The feedback snapshot reports the accumulated
 * position and the temperature; the remaining measurements are SI values with
 * no counterpart in the class, and are read through
 * @ref robstride_get_feedback.
 *
 * @ref motor_disable sends the stop frame itself rather than waiting for the
 * next command interval, and returns the error when the controller will not
 * take it: the motor holds its last target until the stop reaches it. Getting
 * the frame onto the wire is reported separately, so a stop that is accepted
 * and then fails is re-sent by the driver rather than surfacing here.
 *
 * Selecting a target also selects the mode it belongs to. Switching mode makes
 * the driver stop the motor, write the new mode and its parameters, read them
 * back and enable it again, which takes several command intervals.
 *
 * @defgroup motor_robstride RobStride driver
 * @ingroup motor_interface
 * @{
 */

/** @brief Actuator model, which fixes the velocity, torque and current limits. */
enum robstride_model {
  /** RS00. */
  ROBSTRIDE_MODEL_RS00 = 0,
  /** RS05. */
  ROBSTRIDE_MODEL_RS05 = 1,
};

/** @brief Control mode the motor is running in. */
enum robstride_mode {
  /** Position, velocity, gains and feed-forward torque in a single frame. */
  ROBSTRIDE_MODE_OPERATION = 0,
  /** Position, reached through the motor's own trajectory generator. */
  ROBSTRIDE_MODE_POSITION = 1,
  /** Position, applied directly on every cycle. */
  ROBSTRIDE_MODE_POSITION_CSP = 2,
  /** Velocity. */
  ROBSTRIDE_MODE_VELOCITY = 3,
  /** Quadrature current. */
  ROBSTRIDE_MODE_CURRENT = 4,
};

/** @brief Number of values in @ref robstride_mode. */
#define ROBSTRIDE_MODE_COUNT 5

/** @brief Run state reported by the motor alongside every feedback frame. */
enum robstride_run_state {
  /** Output stage is off. */
  ROBSTRIDE_RUN_STATE_RESET = 0,
  /** Motor is calibrating and ignores targets. */
  ROBSTRIDE_RUN_STATE_CALIBRATION = 1,
  /** Motor is driving the output. */
  ROBSTRIDE_RUN_STATE_RUNNING = 2,
};

/**
 * @brief Target for @ref ROBSTRIDE_MODE_OPERATION.
 *
 * The motor applies @c kp against the position error and @c kd against the
 * velocity error, then adds @c torque. Leaving both gains at zero makes this a
 * pure torque command.
 */
struct robstride_motion_target
{
  /** Position target in rad. */
  float position;
  /** Velocity target in rad/s. */
  float velocity;
  /** Proportional gain on position, 0 to 500. */
  float kp;
  /** Derivative gain on velocity, 0 to 5. */
  float kd;
  /** Feed-forward torque in Nm. */
  float torque;
};

/** @brief Limits the motor enforces on its own, in SI units. */
struct robstride_limits
{
  /** Current limit in A. */
  float current;
  /** Velocity limit in rad/s. */
  float velocity;
  /** Torque limit in Nm. */
  float torque;
};

/**
 * @brief What the driver writes into the motor for one control mode.
 *
 * Each mode uses only some of the fields: @ref ROBSTRIDE_MODE_POSITION all of
 * them, @ref ROBSTRIDE_MODE_POSITION_CSP everything but @c acceleration,
 * @ref ROBSTRIDE_MODE_VELOCITY everything but @c max_velocity and
 * @c position_kp, and @ref ROBSTRIDE_MODE_CURRENT and
 * @ref ROBSTRIDE_MODE_OPERATION the current loop alone. The others are ignored.
 *
 * Until @ref robstride_set_mode_params has been called for a mode, the driver
 * writes none of these for it, and the motor runs that mode on the values in
 * its own memory.
 */
struct robstride_mode_params
{
  /**
   * Velocity cap in rad/s, motor side. The trajectory generator's cruise
   * velocity in @ref ROBSTRIDE_MODE_POSITION, and the velocity limit in
   * @ref ROBSTRIDE_MODE_POSITION_CSP, where it never exceeds the limit from
   * @ref robstride_set_limits.
   */
  float max_velocity;
  /**
   * Acceleration in rad/s^2, motor side. The trajectory generator's in
   * @ref ROBSTRIDE_MODE_POSITION, the velocity ramp's in
   * @ref ROBSTRIDE_MODE_VELOCITY.
   */
  float acceleration;
  /** Proportional gain of the position loop. */
  float position_kp;
  /** Proportional gain of the velocity loop. */
  float velocity_kp;
  /** Integral gain of the velocity loop. */
  float velocity_ki;
  /** Proportional gain of the current loop. */
  float current_kp;
  /** Integral gain of the current loop. */
  float current_ki;
};

/** @brief Latest measurements in SI units. */
struct robstride_feedback
{
  /** Accumulated position in rad, with the wrap at +-4 pi resolved. */
  float position;
  /** Velocity in rad/s. */
  float velocity;
  /** Torque in Nm. */
  float torque;
  /** Temperature in degrees Celsius. */
  float temperature;
  /** Mode the driver is currently commanding. */
  enum robstride_mode mode;
  /** Run state from the most recent feedback frame. */
  enum robstride_run_state run_state;
  /** Error code from the most recent feedback frame. */
  uint8_t error_code;
  /** Faults from the most recent fault frame. Non-zero means the motor tripped. */
  uint32_t fault_bits;
  /** Warnings from the most recent fault frame. */
  uint32_t warning_bits;
  /** True when the motor has answered at least once, whatever it answered. */
  bool online;
  /** True when no feedback frame has arrived within the configured timeout. */
  bool stale;
  /**
   * @brief Timestamp of the most recent feedback frame, in milliseconds.
   *
   * Only feedback frames carry measurements, so a presence or parameter reply
   * marks the motor online without moving this on.
   */
  int64_t timestamp_ms;
  /**
   * @brief True once the motor holds everything the driver writes for the
   *        current mode and has reported running since it was enabled.
   *
   * Each value is read back from the motor before it counts as written (unless
   * @c CONFIG_MOTOR_ROBSTRIDE_VERIFY is off). Commands are only sent
   * while this is true.
   */
  bool configured;
  /**
   * @brief Times configuring the motor has been given up on and started over.
   *
   * Counts from boot and is never reset, so a motor that keeps needing retries
   * stays visible after it has come up.
   */
  uint32_t configure_failures;
};

/**
 * @brief Command a position, switching to @ref ROBSTRIDE_MODE_POSITION.
 *
 * @param dev RobStride motor device instance.
 * @param position Position in rad, clamped to +-4 pi.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not a RobStride motor.
 */
int robstride_set_position(const struct device * dev, float position);

/**
 * @brief Command a position, switching to @ref ROBSTRIDE_MODE_POSITION_CSP.
 *
 * Unlike @ref robstride_set_position, the motor does not shape a trajectory,
 * so the caller is responsible for feeding a continuous sequence.
 *
 * @param dev RobStride motor device instance.
 * @param position Position in rad, clamped to +-4 pi.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not a RobStride motor.
 */
int robstride_set_position_csp(const struct device * dev, float position);

/**
 * @brief Command a velocity, switching to @ref ROBSTRIDE_MODE_VELOCITY.
 *
 * @param dev RobStride motor device instance.
 * @param velocity Velocity in rad/s, clamped to the model limit.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not a RobStride motor.
 */
int robstride_set_velocity(const struct device * dev, float velocity);

/**
 * @brief Command a current, switching to @ref ROBSTRIDE_MODE_CURRENT.
 *
 * @param dev RobStride motor device instance.
 * @param current Quadrature current in A, clamped to the model limit.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not a RobStride motor.
 */
int robstride_set_current(const struct device * dev, float current);

/**
 * @brief Command a full motion target, switching to @ref ROBSTRIDE_MODE_OPERATION.
 *
 * @param dev RobStride motor device instance.
 * @param target Target to apply. Each field is clamped to its own range.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not a RobStride motor, or @p target is NULL.
 */
int robstride_set_motion_target(
  const struct device * dev, const struct robstride_motion_target * target);

/**
 * @brief Read the latest measurements in SI units.
 *
 * This never blocks on the bus. It reports what the most recent frame carried.
 *
 * @param dev RobStride motor device instance.
 * @param feedback Destination for the measurements.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not a RobStride motor, or @p feedback is NULL.
 * @retval -ENODATA The motor has not answered yet.
 * @retval -EAGAIN The motor is online but holds no measurement to report: no
 *                 feedback frame has arrived within the configured timeout, or
 *                 none has arrived at all because the motor has only answered
 *                 a presence probe or has just been zeroed.
 */
int robstride_get_feedback(const struct device * dev, struct robstride_feedback * feedback);

/**
 * @brief Set the limits the motor enforces on its own.
 *
 * The values are written on the next command interval, and again whenever the
 * motor is re-enabled.
 *
 * @param dev RobStride motor device instance.
 * @param limits Limits to apply. Each field is clamped to the model limit.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not a RobStride motor, or @p limits is NULL.
 */
int robstride_set_limits(const struct device * dev, const struct robstride_limits * limits);

/**
 * @brief Read back the limits currently in force.
 *
 * These are what the devicetree asked for, or the model limit where it asked
 * for nothing, narrowed by any later @ref robstride_set_limits. A control
 * layer that wants to express a command as a fraction of full scale has no
 * other way to learn the denominator: the model is not exposed, and the
 * devicetree property is optional.
 *
 * @param dev RobStride motor device instance.
 * @param limits Destination for the limits.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not a RobStride motor, or @p limits is NULL.
 */
int robstride_get_limits(const struct device * dev, struct robstride_limits * limits);

/**
 * @brief Set what the driver writes into the motor for one mode.
 *
 * Every time the motor is configured for @p mode, which is on enable, on a
 * switch into the mode and after a fault, these values are written and read
 * back before the motor is enabled. Changing them for the mode the motor is in
 * rewrites them without stopping it; commands pause for the few intervals the
 * readback takes.
 *
 * @param dev RobStride motor device instance.
 * @param mode Mode these values belong to.
 * @param params Values to write. @c max_velocity is clamped to the model limit.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not a RobStride motor, @p mode is not a mode,
 *                 @p params is NULL, or a field is negative or not a number.
 */
int robstride_set_mode_params(
  const struct device * dev, enum robstride_mode mode, const struct robstride_mode_params * params);

/**
 * @brief Store the current position as the mechanical zero.
 *
 * The motor must be stopped for this to take effect, so disable it first. The
 * accumulated position restarts from the new zero on the next feedback frame.
 *
 * @param dev RobStride motor device instance.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not a RobStride motor.
 * @retval negative_errno The frame could not be queued.
 */
int robstride_set_zero(const struct device * dev);

/**
 * @brief Save the motor's current parameters to its non-volatile memory.
 *
 * @param dev RobStride motor device instance.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not a RobStride motor.
 * @retval negative_errno The frame could not be queued.
 */
int robstride_save_parameters(const struct device * dev);

/**
 * @brief Clear the motor's own fault latch.
 *
 * A motor that has tripped refuses to run until the latch is cleared, and
 * nothing the driver sends in the course of a handshake clears it. This is
 * the way back. It is deliberately separate from @ref motor_disable, because
 * the frame stops the output as a side effect, so it is not a way to
 * acknowledge a fault while the motor keeps driving.
 *
 * Clearing does not repair the condition. A motor that is still too hot, or
 * still stalled, trips again on the next attempt to run.
 *
 * @param dev RobStride motor device instance.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not a RobStride motor.
 * @retval negative_errno The frame could not be queued.
 */
int robstride_clear_faults(const struct device * dev);

/**
 * @brief Write one motor parameter by index.
 *
 * This is an escape hatch for parameters the typed calls above do not cover.
 * The driver knows which indices carry an integer payload and converts @p value
 * accordingly.
 *
 * @param dev RobStride motor device instance.
 * @param index Parameter index, as defined by the RobStride protocol.
 * @param value Value to write. For an index whose payload is an integer it has
 *              to be a number in the range of @c uint32_t; anything else has no
 *              representation on the wire and is rejected rather than converted.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not a RobStride motor, or @p value cannot be
 *                 carried by this index.
 * @retval negative_errno The frame could not be queued.
 */
int robstride_set_parameter(const struct device * dev, uint16_t index, float value);

/**
 * @brief Read one motor parameter by index.
 *
 * This blocks until the motor answers or the read times out, so it must not be
 * called from a control loop or from an interrupt.
 *
 * @param dev RobStride motor device instance.
 * @param index Parameter index, as defined by the RobStride protocol.
 * @param value Destination for the value.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not a RobStride motor, or @p value is NULL.
 * @retval -ETIMEDOUT The motor did not answer in time.
 * @retval -EBUSY Another read of this motor is already in flight. A read holds
 *                the motor from the call until it returns, so this is reported
 *                for the whole of that window and not only until the reply
 *                arrives.
 *
 * @note A reply carries only the parameter index, with nothing tying it to the
 *       request it answers. After a read of an index times out, the next read
 *       of that same index can therefore be completed by the earlier reply and
 *       return the value it carried. Reading a parameter that changes is worth
 *       repeating once after a timeout.
 * @retval negative_errno The request could not be queued.
 */
int robstride_get_parameter(const struct device * dev, uint16_t index, float * value);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
