#ifndef FIBRIL_ZEPHYR_INCLUDE_DRIVERS_MOTOR_H_
#define FIBRIL_ZEPHYR_INCLUDE_DRIVERS_MOTOR_H_

#include <stdbool.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

enum motor_output_mode {
  MOTOR_OUTPUT_MODE_CURRENT = 1,
  MOTOR_OUTPUT_MODE_TORQUE = MOTOR_OUTPUT_MODE_CURRENT,
  MOTOR_OUTPUT_MODE_VELOCITY = 1 << 1,
  MOTOR_OUTPUT_MODE_VOLTAGE = 1 << 2,
};

enum motor_feedback_type {
  MOTOR_FEEDBACK_CURRENT = 1,
  MOTOR_FEEDBACK_VELOCITY = 1 << 1,
  MOTOR_FEEDBACK_POSITION = 1 << 2,
  /**
   * @brief Orientation feedback (e.g., mechanical angle)
   */
  MOTOR_FEEDBACK_ORIENTATION = 1 << 3,
  MOTOR_FEEDBACK_TEMPERATURE = 1 << 4,
};

struct motor_feedback
{
  /**
   * @brief Bit-mask of valid fields in this snapshot.
   *
   * This uses values from @ref motor_feedback_type.
   */
  uint32_t valid_mask;
  int16_t current;
  int16_t velocity;
  int32_t position;
  int32_t orientation;
  int16_t temperature;
  bool online;
  bool stale;
  int64_t timestamp_ms;
};

typedef int (*motor_enable_t)(const struct device * dev);

typedef int (*motor_disable_t)(const struct device * dev);

typedef int (*motor_set_output_t)(
  const struct device * dev, const enum motor_output_mode mode, const int16_t output);

typedef int (*motor_get_feedback_t)(const struct device * dev, void * feedback);

struct motor_driver_api
{
  motor_enable_t enable;
  motor_disable_t disable;
  motor_set_output_t set_output;
  motor_get_feedback_t get_feedback;
};

__syscall int motor_enable(const struct device * dev);

__syscall int motor_disable(const struct device * dev);

__syscall int motor_set_output(
  const struct device * dev, const enum motor_output_mode mode, const int16_t output);

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

#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/motor.h>

#endif
