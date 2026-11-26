#ifndef FIBRIL_ZEPHYR_INCLUDE_DRIVERS_IMU_H_
#define FIBRIL_ZEPHYR_INCLUDE_DRIVERS_IMU_H_

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file
 * @brief Inertial measurement unit driver interface.
 *
 * This class covers IMUs and attitude sensors that stream or are polled by
 * their driver and expose the most recent reading as a snapshot. Reading a
 * snapshot never blocks on the underlying transport, so a tick or a control
 * loop can read it every period.
 *
 * Values are in SI units in the sensor's own axes. Unlike the encoder class,
 * the conversion from raw counts belongs to the driver here: the scale is a
 * property of the device, and there is no gear ratio or wheel radius for an
 * application to fold in. Mapping the sensor's axes onto a robot's is left to
 * the consumer, because only the mounting says how they relate.
 *
 * The yaw is continuous. Devices report it wrapped, and following the wrap
 * needs every sample in order, which only the driver sees. Breaks in that
 * continuity are reported through @ref imu_feedback.yaw_epoch.
 *
 * @defgroup imu_interface IMU driver class
 * @ingroup drivers
 * @{
 */

/**
 * @brief Feedback fields that may be present in an @ref imu_feedback snapshot.
 *
 * Drivers set @ref imu_feedback.valid_mask to indicate which values are valid.
 */
enum imu_feedback_type {
  /** Linear acceleration is valid. */
  IMU_FEEDBACK_ACCEL = 1,
  /** Angular velocity is valid. */
  IMU_FEEDBACK_GYRO = 1 << 1,
  /** Roll, pitch and yaw are valid. */
  IMU_FEEDBACK_ATTITUDE = 1 << 2,
};

/**
 * @brief Snapshot of the latest IMU reading.
 */
struct imu_feedback
{
  /**
   * @brief Bit-mask of valid fields in this snapshot.
   *
   * This uses values from @ref imu_feedback_type.
   */
  uint32_t valid_mask;
  /** Linear acceleration in m/s^2, x, y and z of the sensor, gravity included. */
  float accel[3];
  /** Angular velocity in rad/s about x, y and z of the sensor. */
  float gyro[3];
  /** Roll in rad, as the device reports it. */
  float roll;
  /** Pitch in rad, as the device reports it. */
  float pitch;
  /**
   * @brief Yaw in rad, continuous across the device's wrap.
   *
   * The driver follows the wrap of the device's yaw sample by sample, so the
   * value keeps growing past one turn. Its sign is the device's; on a sensor
   * whose yaw turns the other way from the robot's, the consumer negates it.
   */
  float yaw;
  /**
   * @brief Number of times the yaw lost continuity.
   *
   * The driver advances this whenever it restarts following the yaw instead
   * of continuing, which happens on the first sample and after
   * @ref imu_reset. @ref imu_feedback.yaw jumps across such an event, so a
   * consumer that integrates or differentiates it discards that state when
   * this value changes. Compare successive readings; the value wraps around.
   */
  uint32_t yaw_epoch;
  /** True when the IMU is currently considered online. */
  bool online;
  /** True when the reading is present but no longer fresh. */
  bool stale;
  /** Timestamp of the latest successful update in milliseconds. */
  int64_t timestamp_ms;
  /**
   * @brief Total number of samples or transfers that failed.
   *
   * An aggregate health signal, alongside @ref imu_feedback.online and
   * @ref imu_feedback.stale. It wraps around, so consumers compare successive
   * readings rather than treat it as an absolute count.
   */
  uint32_t error_count;
};

/** @brief Driver API for retrieving the latest feedback snapshot. */
typedef int (*imu_get_feedback_t)(const struct device * dev, struct imu_feedback * feedback);

/** @brief Driver API for resetting the IMU. */
typedef int (*imu_reset_t)(const struct device * dev);

/**
 * @brief IMU driver API vtable.
 */
struct imu_driver_api
{
  imu_get_feedback_t get_feedback;
  imu_reset_t reset;
};

/**
 * @brief Read the latest IMU feedback.
 *
 * @param dev IMU device instance.
 * @param feedback Destination for the snapshot. It is filled in whenever a
 *        reading has ever been obtained, including when this returns
 *        -EAGAIN or -EIO, so a caller can still see the last values.
 *
 * @retval 0 Success.
 * @retval -ENODATA No reading has been obtained yet.
 * @retval -EAGAIN A reading is available but no longer fresh.
 * @retval -EIO The IMU is currently offline.
 * @retval negative_errno Failed to retrieve feedback.
 */
__syscall int imu_get_feedback(const struct device * dev, struct imu_feedback * feedback);

/**
 * @brief Reset an IMU.
 *
 * What is reset is device-defined; an attitude sensor typically zeroes its
 * yaw. The driver restarts following the yaw and advances
 * @ref imu_feedback.yaw_epoch. Readings may be unavailable for a
 * driver-defined period afterwards.
 *
 * @param dev IMU device instance.
 * @retval 0 Success.
 * @retval -ENOTSUP The device cannot be reset.
 * @retval negative_errno Failed to reset the IMU.
 */
__syscall int imu_reset(const struct device * dev);

static inline int z_impl_imu_get_feedback(const struct device * dev, struct imu_feedback * feedback)
{
  const struct imu_driver_api * api = (const struct imu_driver_api *)dev->api;

  if (api->get_feedback == NULL) {
    return -ENOSYS;
  }

  return api->get_feedback(dev, feedback);
}

static inline int z_impl_imu_reset(const struct device * dev)
{
  const struct imu_driver_api * api = (const struct imu_driver_api *)dev->api;

  if (api->reset == NULL) {
    return -ENOSYS;
  }

  return api->reset(dev);
}

/** @} */

#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/imu.h>

#endif
