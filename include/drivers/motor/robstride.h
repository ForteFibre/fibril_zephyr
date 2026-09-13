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
 * Selecting a target also selects the mode it belongs to. Switching mode makes
 * the driver stop the motor, write the new mode and enable it again, which
 * takes a few command intervals.
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
 * @brief Gains of the motor's internal loops.
 *
 * The driver does not write these until @ref robstride_set_gains is called, so
 * a motor keeps the gains stored in its own memory unless the application
 * overrides them.
 */
struct robstride_gains
{
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
  /** True when the motor has answered at least once and has not timed out. */
  bool online;
  /** True when the measurements are present but no longer fresh. */
  bool stale;
  /** Timestamp of the most recent frame from this motor, in milliseconds. */
  int64_t timestamp_ms;
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
 * @retval -EAGAIN The measurements are older than the configured timeout.
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
 * @brief Override the gains of the motor's internal loops.
 *
 * @param dev RobStride motor device instance.
 * @param gains Gains to apply.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not a RobStride motor, or @p gains is NULL.
 */
int robstride_set_gains(const struct device * dev, const struct robstride_gains * gains);

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
 * @brief Write one motor parameter by index.
 *
 * This is an escape hatch for parameters the typed calls above do not cover.
 * The driver knows which indices carry an integer payload and converts @p value
 * accordingly.
 *
 * @param dev RobStride motor device instance.
 * @param index Parameter index, as defined by the RobStride protocol.
 * @param value Value to write.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not a RobStride motor.
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
 * @retval -EBUSY Another read of this motor is already in flight.
 * @retval negative_errno The request could not be queued.
 */
int robstride_get_parameter(const struct device * dev, uint16_t index, float * value);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
