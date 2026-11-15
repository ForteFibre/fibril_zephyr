#ifndef FIBRIL_ZEPHYR_INCLUDE_DRIVERS_MOTOR_H_
#define FIBRIL_ZEPHYR_INCLUDE_DRIVERS_MOTOR_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file
 * @brief Generic motor driver interface.
 *
 * This class covers motor controllers that accept a command in one output
 * domain and report the most recent measurements as a snapshot. Reading a
 * snapshot never blocks on the underlying transport, which makes it usable
 * from a control loop.
 *
 * Commands and measurements are in SI units, the same for every driver:
 * current in A, velocity in rad/s, voltage in V, torque in Nm, temperature in
 * degrees Celsius, and duty as a ratio from -1 to 1. Velocity and position are
 * those of the shaft the driver measures (the rotor, for a geared motor); a
 * gear ratio belongs to the control layer. Only @ref motor_feedback.position
 * stays in driver-specific counts, which @ref motor_info.counts_per_rev
 * converts. Raw device codes never cross this interface. The reasons are in
 * doc/adr/0024-motor-class-si-units.md.
 *
 * The functions are plain inline calls rather than system calls: a system
 * call cannot take a float argument, and no image here uses user mode.
 *
 * @defgroup motor_interface Motor driver class
 * @ingroup drivers
 * @{
 */

/**
 * @brief Command modes, each with its unit.
 *
 * The values are bits, so @ref motor_info.output_modes can list the modes a
 * driver accepts. Positive values turn the shaft in the driver's positive
 * direction, the one in which @ref motor_feedback.position counts up.
 */
enum motor_output_mode {
  /** Motor current in A. */
  MOTOR_OUTPUT_MODE_CURRENT = 1,
  /** Shaft velocity in rad/s. */
  MOTOR_OUTPUT_MODE_VELOCITY = 1 << 1,
  /**
   * @brief Voltage across the motor in V.
   *
   * A controller that takes only duty may implement this by dividing by its
   * measured supply voltage, and returns -EAGAIN while it has none.
   */
  MOTOR_OUTPUT_MODE_VOLTAGE = 1 << 2,
  /** Shaft torque in Nm. Not an alias of current: converting needs the torque constant. */
  MOTOR_OUTPUT_MODE_TORQUE = 1 << 3,
  /** Duty as a ratio of the supply voltage, from -1 to 1. */
  MOTOR_OUTPUT_MODE_DUTY = 1 << 4,
};

/**
 * @brief Feedback fields that may be present in a @ref motor_feedback snapshot.
 *
 * Drivers set @ref motor_feedback.valid_mask to indicate which values are valid.
 */
enum motor_feedback_type {
  /** Current feedback is valid. */
  MOTOR_FEEDBACK_CURRENT = 1,
  /** Velocity feedback is valid. */
  MOTOR_FEEDBACK_VELOCITY = 1 << 1,
  /** Position feedback is valid. */
  MOTOR_FEEDBACK_POSITION = 1 << 2,
  /**
   * @brief Orientation feedback is valid.
   *
   * For RoboMaster motors, this corresponds to the mechanical angle.
   */
  MOTOR_FEEDBACK_ORIENTATION = 1 << 3,
  /** Temperature feedback is valid. */
  MOTOR_FEEDBACK_TEMPERATURE = 1 << 4,
  /** Torque feedback is valid. */
  MOTOR_FEEDBACK_TORQUE = 1 << 5,
};

/**
 * @brief Snapshot of the latest motor feedback.
 */
struct motor_feedback
{
  /**
   * @brief Bit-mask of valid fields in this snapshot.
   *
   * This uses values from @ref motor_feedback_type.
   */
  uint32_t valid_mask;
  /** Latest measured motor current, in A. */
  float current;
  /** Latest measured shaft velocity, in rad/s. */
  float velocity;
  /** Latest measured shaft torque, in Nm. */
  float torque;
  /** Latest measured motor temperature, in degrees Celsius. Not the inverter's. */
  float temperature;
  /**
   * @brief Position accumulated by the driver, in driver-specific counts.
   *
   * The driver resolves the wrap of the underlying device and accumulates
   * here, so a control loop sees a continuous value. What one count is worth
   * depends on the device; @ref motor_info.counts_per_rev gives the counts
   * per turn of the measured shaft.
   */
  int64_t position;
  /** Instantaneous orientation reported by the motor. */
  int32_t orientation;
  /** True when the motor is currently considered online. */
  bool online;
  /** True when the feedback is present but no longer fresh. */
  bool stale;
  /** Timestamp of the latest feedback update in milliseconds. */
  int64_t timestamp_ms;
};

/**
 * @brief What a motor accepts and how to read its position.
 */
struct motor_info
{
  /** The @ref motor_output_mode bits motor_set_output() accepts. */
  uint32_t output_modes;
  /** @ref motor_feedback.position counts per turn of the measured shaft. */
  float counts_per_rev;
};

/** @brief Driver API for enabling motor output. */
typedef int (*motor_enable_t)(const struct device * dev);

/** @brief Driver API for disabling motor output. */
typedef int (*motor_disable_t)(const struct device * dev);

/** @brief Driver API for updating the commanded motor output. */
typedef int (*motor_set_output_t)(
  const struct device * dev, enum motor_output_mode mode, float value);

/** @brief Driver API for retrieving the latest feedback snapshot. */
typedef int (*motor_get_feedback_t)(const struct device * dev, void * feedback);

/** @brief Driver API for describing the motor. */
typedef int (*motor_get_info_t)(const struct device * dev, struct motor_info * info);

/**
 * @brief Motor driver API vtable.
 */
struct motor_driver_api
{
  motor_enable_t enable;
  motor_disable_t disable;
  motor_set_output_t set_output;
  motor_get_feedback_t get_feedback;
  motor_get_info_t get_info;
};

/**
 * @brief Enable a motor device.
 *
 * @param dev Motor device instance.
 * @retval 0 Success.
 * @retval negative_errno Failed to enable the motor.
 */
static inline int motor_enable(const struct device * dev)
{
  const struct motor_driver_api * api = (const struct motor_driver_api *)dev->api;

  return api->enable(dev);
}

/**
 * @brief Disable a motor device.
 *
 * @param dev Motor device instance.
 * @retval 0 Success.
 * @retval negative_errno Failed to disable the motor.
 */
static inline int motor_disable(const struct device * dev)
{
  const struct motor_driver_api * api = (const struct motor_driver_api *)dev->api;

  return api->disable(dev);
}

/**
 * @brief Set the commanded motor output.
 *
 * @param dev Motor device instance.
 * @param mode Output mode to apply.
 * @param value Command in the unit of @p mode (see @ref motor_output_mode).
 *        The driver clamps it to the motor's limits.
 * @retval 0 Success.
 * @retval -ENOTSUP The driver does not accept @p mode.
 * @retval -EINVAL @p value is not finite.
 * @retval negative_errno Failed to update the motor command.
 */
static inline int motor_set_output(
  const struct device * dev, enum motor_output_mode mode, float value)
{
  const struct motor_driver_api * api = (const struct motor_driver_api *)dev->api;

  return api->set_output(dev, mode, value);
}

/**
 * @brief Read the latest motor feedback.
 *
 * @param dev Motor device instance.
 * @param feedback Pointer to a driver-defined feedback buffer.
 *
 * Current in-tree drivers expect @p feedback to point to
 * a @ref motor_feedback structure.
 *
 * @retval 0 Success.
 * @retval negative_errno Failed to retrieve feedback.
 */
static inline int motor_get_feedback(const struct device * dev, void * feedback)
{
  const struct motor_driver_api * api = (const struct motor_driver_api *)dev->api;

  return api->get_feedback(dev, feedback);
}

/**
 * @brief Describe the motor: the modes it accepts and its position counts.
 *
 * @param dev Motor device instance.
 * @param info Filled on success.
 * @retval 0 Success.
 * @retval negative_errno Failed to describe the motor.
 */
static inline int motor_get_info(const struct device * dev, struct motor_info * info)
{
  const struct motor_driver_api * api = (const struct motor_driver_api *)dev->api;

  return api->get_info(dev, info);
}

/** @} */

#ifdef __cplusplus
}
#endif

#endif
