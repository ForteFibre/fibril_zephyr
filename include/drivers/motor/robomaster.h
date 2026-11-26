#ifndef FIBRIL_ZEPHYR_INCLUDE_DRIVERS_MOTOR_ROBOMASTER_H_
#define FIBRIL_ZEPHYR_INCLUDE_DRIVERS_MOTOR_ROBOMASTER_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file
 * @brief Interface specific to the DJI RoboMaster driver.
 *
 * The generic motor class in @ref drivers/motor.h covers commanding a
 * RoboMaster and reading its latest feedback. What it cannot carry is each
 * rotor angle as it arrives, with the instant it arrived, which is what an
 * encoder built on the rotor needs to measure its own interval and to notice
 * that a gap has made the next difference meaningless. This header exposes
 * that stream to one listener per motor.
 *
 * @defgroup motor_robomaster RoboMaster driver
 * @ingroup motor_interface
 * @{
 */

/** @brief One rotor angle, as the motor reported it. */
struct robomaster_rotor_sample
{
  /** Rotor angle in the range 0 to 8191. */
  uint16_t orientation;
  /** Instant the frame was received, from k_cycle_get_32(). */
  uint32_t cycle;
  /**
   * @brief Whether this angle continues from the previous one.
   *
   * False for the first frame from the motor, and for the first frame after
   * the motor read as stale (no frame for longer than the transport's
   * feedback-timeout-ms). Across such a gap the rotor may have turned any
   * number of times, so the difference to the previous angle says nothing.
   */
  bool continuous;
};

/**
 * @brief Called for every feedback frame of one motor.
 *
 * Runs in the CAN controller's receive callback, which is interrupt context on
 * most controllers, so it must not block.
 */
typedef void (*robomaster_rotor_callback_t)(
  const struct device * motor, const struct robomaster_rotor_sample * sample, void * user_data);

/**
 * @brief Receive every rotor angle of a motor as it arrives.
 *
 * @param motor A @c dji,robomaster-motor device.
 * @param callback Function to call, or NULL to stop.
 * @param user_data Passed to @p callback unchanged.
 * @retval 0 Success.
 * @retval -EINVAL @p motor is not a RoboMaster motor.
 * @retval -EBUSY The motor already has a listener.
 */
int robomaster_motor_set_rotor_callback(
  const struct device * motor, robomaster_rotor_callback_t callback, void * user_data);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
