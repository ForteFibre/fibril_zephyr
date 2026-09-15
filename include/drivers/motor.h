#ifndef FIBRIL_ZEPHYR_INCLUDE_DRIVERS_MOTOR_H_
#define FIBRIL_ZEPHYR_INCLUDE_DRIVERS_MOTOR_H_

#include <stdbool.h>

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
 * @defgroup motor_interface Motor driver class
 * @ingroup drivers
 * @{
 */

/**
 * @brief Command modes supported by motor drivers.
 *
 * These values are used as a bit mask to describe the requested output domain.
 * Actual support depends on the concrete driver implementation.
 */
enum motor_output_mode {
  /** Command output as motor current. */
  MOTOR_OUTPUT_MODE_CURRENT = 1,
  /** Alias for current-based control. */
  MOTOR_OUTPUT_MODE_TORQUE = MOTOR_OUTPUT_MODE_CURRENT,
  /** Command output as motor velocity. */
  MOTOR_OUTPUT_MODE_VELOCITY = 1 << 1,
  /** Command output as motor voltage. */
  MOTOR_OUTPUT_MODE_VOLTAGE = 1 << 2,
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
  /** Latest measured current. */
  int16_t current;
  /** Latest measured velocity. */
  int16_t velocity;
  /**
   * @brief Position accumulated by the driver, in driver-specific counts.
   *
   * The driver resolves the wrap of the underlying device and accumulates
   * here, so a control loop sees a continuous value. What one count is worth
   * depends on the device, and is documented by each driver; converting to a
   * physical unit belongs to the control layer.
   */
  int64_t position;
  /** Instantaneous orientation reported by the motor. */
  int32_t orientation;
  /** Latest measured temperature. */
  int16_t temperature;
  /** True when the motor is currently considered online. */
  bool online;
  /** True when the feedback is present but no longer fresh. */
  bool stale;
  /** Timestamp of the latest feedback update in milliseconds. */
  int64_t timestamp_ms;
};

/** @brief Driver API for enabling motor output. */
typedef int (*motor_enable_t)(const struct device * dev);

/** @brief Driver API for disabling motor output. */
typedef int (*motor_disable_t)(const struct device * dev);

/** @brief Driver API for updating the commanded motor output. */
typedef int (*motor_set_output_t)(
  const struct device * dev, const enum motor_output_mode mode, const int16_t output);

/** @brief Driver API for retrieving the latest feedback snapshot. */
typedef int (*motor_get_feedback_t)(const struct device * dev, void * feedback);

/**
 * @brief Motor driver API vtable.
 */
struct motor_driver_api
{
  motor_enable_t enable;
  motor_disable_t disable;
  motor_set_output_t set_output;
  motor_get_feedback_t get_feedback;
};

/**
 * @brief Enable a motor device.
 *
 * @param dev Motor device instance.
 * @retval 0 Success.
 * @retval negative_errno Failed to enable the motor.
 */
__syscall int motor_enable(const struct device * dev);

/**
 * @brief Disable a motor device.
 *
 * @param dev Motor device instance.
 * @retval 0 Success.
 * @retval negative_errno Failed to disable the motor.
 */
__syscall int motor_disable(const struct device * dev);

/**
 * @brief Set the commanded motor output.
 *
 * @param dev Motor device instance.
 * @param mode Output mode to apply.
 * @param output Command value in the unit implied by @p mode.
 * @retval 0 Success.
 * @retval negative_errno Failed to update the motor command.
 */
__syscall int motor_set_output(
  const struct device * dev, const enum motor_output_mode mode, const int16_t output);

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
__syscall int motor_get_feedback(const struct device * dev, void * feedback);

static inline int z_impl_motor_enable(const struct device * dev)
{
  const struct motor_driver_api * api = (const struct motor_driver_api *)dev->api;

  return api->enable(dev);
}

static inline int z_impl_motor_disable(const struct device * dev)
{
  const struct motor_driver_api * api = (const struct motor_driver_api *)dev->api;

  return api->disable(dev);
}

static inline int z_impl_motor_set_output(
  const struct device * dev, const enum motor_output_mode mode, const int16_t output)
{
  const struct motor_driver_api * api = (const struct motor_driver_api *)dev->api;

  return api->set_output(dev, mode, output);
}

static inline int z_impl_motor_get_feedback(const struct device * dev, void * feedback)
{
  const struct motor_driver_api * api = (const struct motor_driver_api *)dev->api;

  return api->get_feedback(dev, feedback);
}

/** @} */

#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/motor.h>

#endif
