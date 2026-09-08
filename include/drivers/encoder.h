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
 * The primary value is @ref encoder_feedback.position, a signed accumulated
 * count that the driver keeps continuous across the wrap of whatever the device
 * actually reports. Drivers own the accumulator because the events that break
 * its continuity, such as a device reset or a turns counter cleared by a power
 * cycle, are only visible inside the driver. Those events are reported through
 * @ref encoder_feedback.position_epoch.
 *
 * Values are in raw counts. Converting them to physical units needs a gear
 * ratio and a wheel radius, which belong to the application rather than to the
 * device, so this API does not carry a scale factor.
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
  /** Accumulated position is valid. */
  ENCODER_FEEDBACK_POSITION = 1,
  /** Velocity is valid. */
  ENCODER_FEEDBACK_VELOCITY = 1 << 1,
  /** Single-turn absolute position is valid. */
  ENCODER_FEEDBACK_SINGLE_TURN = 1 << 2,
  /** Turns counter is valid. */
  ENCODER_FEEDBACK_TURNS = 1 << 3,
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
   * @brief Accumulated position in raw counts.
   *
   * The driver keeps this continuous across the wrap of the underlying counter
   * and applies whatever offset was requested through
   * @ref encoder_set_position. It is the value a control loop should use.
   *
   * On an absolute encoder the accumulator starts at the first absolute reading
   * the driver obtained, so the position at power-on is preserved. On an
   * incremental encoder there is no absolute reference and it starts at zero.
   */
  int64_t position;
  /**
   * @brief Velocity in counts per second.
   *
   * This is a single-interval estimate: the driver divides the change in
   * @ref encoder_feedback.position by @ref encoder_feedback.sample_interval_us.
   * Its quantisation noise therefore grows as the interval shrinks and
   * dominates at low speed, so a control loop should filter it rather than use
   * it directly. How much to filter depends on the application, which is why
   * the driver does not do it.
   */
  int32_t velocity;
  /**
   * @brief Measured interval the latest position and velocity were derived over.
   *
   * This is the elapsed time between the two most recent successful readings,
   * not the interval the driver was configured with. A driver that skips a
   * sampling period reports the longer interval it actually observed.
   */
  uint32_t sample_interval_us;
  /**
   * @brief Number of times the accumulator lost continuity.
   *
   * The driver advances this whenever it rebuilds the accumulator instead of
   * continuing it, which happens on the first successful reading, on recovery
   * from being offline, and after a reset or a stored zero point. Any offset
   * previously set through @ref encoder_set_position is dropped at the same
   * time.
   *
   * @ref encoder_feedback.position jumps across such an event, so a consumer
   * that integrates or differentiates it must discard that state when this
   * value changes rather than carry it over. Compare successive readings; the
   * value wraps around.
   */
  uint32_t position_epoch;
  /**
   * @brief Single-turn absolute position in raw counts.
   *
   * The value is in the range 0 to (1 << resolution) - 1, where the resolution
   * is the one reported by @ref encoder_get_resolution. This is what the device
   * reported, without any offset applied. Incremental encoders have no
   * single-turn absolute position and leave this field invalid.
   */
  uint32_t single_turn;
  /**
   * @brief Signed turns counter, for encoders that track multiple turns.
   *
   * Like @ref encoder_feedback.single_turn this is the value the device
   * reported, without any offset applied.
   */
  int32_t turns;
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

/** @brief Driver API for redefining the current accumulated position. */
typedef int (*encoder_set_position_t)(const struct device * dev, int64_t position);

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
  encoder_set_position_t set_position;
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
 * @brief Redefine the current accumulated position.
 *
 * This shifts @ref encoder_feedback.position so that the present reading
 * becomes @p position. Nothing is written to the device, and
 * @ref encoder_feedback.single_turn and @ref encoder_feedback.turns keep
 * reporting what the device said.
 *
 * The offset is dropped whenever the driver rebuilds its accumulator, which it
 * reports by advancing @ref encoder_feedback.position_epoch. A caller that
 * needs the offset to survive such an event has to set it again.
 *
 * @param dev Encoder device instance.
 * @param position Value the current position should read as.
 * @retval 0 Success.
 * @retval -ENODATA There is no accumulator to offset from, either because no
 *         reading has been obtained yet or because the encoder is offline and
 *         its accumulator is waiting to be rebuilt.
 * @retval -ENOSYS The driver does not implement this operation.
 * @retval negative_errno Failed to set the position.
 */
__syscall int encoder_set_position(const struct device * dev, int64_t position);

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

static inline int z_impl_encoder_set_position(const struct device * dev, int64_t position)
{
  const struct encoder_driver_api * api = (const struct encoder_driver_api *)dev->api;

  if (api->set_position == NULL) {
    return -ENOSYS;
  }

  return api->set_position(dev, position);
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
