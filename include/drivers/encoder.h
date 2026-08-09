#ifndef FIBRIL_ZEPHYR_INCLUDE_DRIVERS_ENCODER_H_
#define FIBRIL_ZEPHYR_INCLUDE_DRIVERS_ENCODER_H_

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file
 * @brief Generic rotary encoder driver interface.
 *
 * This class covers absolute and incremental rotary encoders that are polled by
 * their driver and expose the most recent reading as a snapshot. Reading a
 * snapshot never blocks on the underlying transport, which makes it usable from
 * a control loop.
 *
 * Diagnostics that depend on the concrete device are intentionally absent from
 * this API. A driver that tracks per-transport error causes exposes them
 * through its own header, for example drivers/encoder/amt21.h.
 *
 * @defgroup encoder_interface Encoder driver class
 * @ingroup drivers
 * @{
 */

/**
 * @brief Feedback fields that may be present in a @ref encoder_feedback snapshot.
 *
 * Drivers set @ref encoder_feedback.valid_mask to indicate which values are
 * valid.
 */
enum encoder_feedback_type {
  /** Single-turn position in raw counts is valid. */
  ENCODER_FEEDBACK_POSITION = 1,
  /** Turns counter is valid. */
  ENCODER_FEEDBACK_TURNS = 1 << 1,
  /** Derived angle in millidegrees is valid. */
  ENCODER_FEEDBACK_ANGLE = 1 << 2,
};

/**
 * @brief Snapshot of the latest encoder reading.
 */
struct encoder_feedback
{
  /**
   * @brief Bit-mask of valid fields in this snapshot.
   *
   * This uses values from @ref encoder_feedback_type.
   */
  uint32_t valid_mask;
  /**
   * @brief Single-turn position in raw counts.
   *
   * The value is in the range 0 to (1 << resolution) - 1, where the resolution
   * is the one reported by @ref encoder_get_resolution.
   */
  uint32_t position;
  /** Signed turns counter, for encoders that track multiple turns. */
  int32_t turns;
  /** Position converted to millidegrees within a single turn. */
  int32_t angle_mdeg;
  /** True when the encoder is currently considered online. */
  bool online;
  /** True when the reading is present but no longer fresh. */
  bool stale;
  /** Timestamp of the latest successful update in milliseconds. */
  int64_t timestamp_ms;
  /**
   * @brief Total number of failed transactions for this encoder.
   *
   * This is an aggregate health signal that is always available, alongside
   * @ref encoder_feedback.online and @ref encoder_feedback.stale. It wraps
   * around, so consumers should compare successive readings rather than treat
   * it as an absolute count. Drivers may offer a per-cause breakdown through
   * their own API.
   */
  uint32_t error_count;
};

/** @brief Driver API for retrieving the latest feedback snapshot. */
typedef int (*encoder_get_feedback_t)(const struct device * dev, void * feedback);

/** @brief Driver API for querying the position resolution in bits. */
typedef int (*encoder_get_resolution_t)(const struct device * dev, uint8_t * resolution);

/** @brief Driver API for storing the current position as the zero point. */
typedef int (*encoder_set_zero_t)(const struct device * dev);

/** @brief Driver API for resetting the encoder. */
typedef int (*encoder_reset_t)(const struct device * dev);

/**
 * @brief Encoder driver API vtable.
 */
struct encoder_driver_api
{
  encoder_get_feedback_t get_feedback;
  encoder_get_resolution_t get_resolution;
  encoder_set_zero_t set_zero;
  encoder_reset_t reset;
};

/**
 * @brief Read the latest encoder feedback.
 *
 * @param dev Encoder device instance.
 * @param feedback Pointer to a driver-defined feedback buffer.
 *
 * Current in-tree drivers expect @p feedback to point to
 * a @ref encoder_feedback structure.
 *
 * @retval 0 Success.
 * @retval -ENODATA No reading has been obtained yet.
 * @retval -EAGAIN A reading is available but no longer fresh.
 * @retval -EIO The encoder is currently offline.
 * @retval negative_errno Failed to retrieve feedback.
 */
__syscall int encoder_get_feedback(const struct device * dev, void * feedback);

/**
 * @brief Query the position resolution of an encoder.
 *
 * @param dev Encoder device instance.
 * @param resolution Destination for the resolution in bits.
 * @retval 0 Success.
 * @retval negative_errno Failed to query the resolution.
 */
__syscall int encoder_get_resolution(const struct device * dev, uint8_t * resolution);

/**
 * @brief Store the current position as the zero point.
 *
 * The change is persistent on devices that support it. Devices commonly reset
 * themselves as part of this operation, so readings may be unavailable for a
 * driver-defined period afterwards.
 *
 * @param dev Encoder device instance.
 * @retval 0 Success.
 * @retval -ENOTSUP The device cannot store a zero point.
 * @retval negative_errno Failed to set the zero point.
 */
__syscall int encoder_set_zero(const struct device * dev);

/**
 * @brief Reset an encoder.
 *
 * Readings may be unavailable for a driver-defined period afterwards while the
 * device restarts.
 *
 * @param dev Encoder device instance.
 * @retval 0 Success.
 * @retval -ENOTSUP The device cannot be reset.
 * @retval negative_errno Failed to reset the encoder.
 */
__syscall int encoder_reset(const struct device * dev);

static inline int z_impl_encoder_get_feedback(const struct device * dev, void * feedback)
{
  const struct encoder_driver_api * api = (const struct encoder_driver_api *)dev->api;

  if (api->get_feedback == NULL) {
    return -ENOSYS;
  }

  return api->get_feedback(dev, feedback);
}

static inline int z_impl_encoder_get_resolution(const struct device * dev, uint8_t * resolution)
{
  const struct encoder_driver_api * api = (const struct encoder_driver_api *)dev->api;

  if (api->get_resolution == NULL) {
    return -ENOSYS;
  }

  return api->get_resolution(dev, resolution);
}

static inline int z_impl_encoder_set_zero(const struct device * dev)
{
  const struct encoder_driver_api * api = (const struct encoder_driver_api *)dev->api;

  if (api->set_zero == NULL) {
    return -ENOSYS;
  }

  return api->set_zero(dev);
}

static inline int z_impl_encoder_reset(const struct device * dev)
{
  const struct encoder_driver_api * api = (const struct encoder_driver_api *)dev->api;

  if (api->reset == NULL) {
    return -ENOSYS;
  }

  return api->reset(dev);
}

/** @} */

#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/encoder.h>

#endif
