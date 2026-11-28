#ifndef FIBRIL_ZEPHYR_INCLUDE_DRIVERS_ENCODER_AS5047P_H_
#define FIBRIL_ZEPHYR_INCLUDE_DRIVERS_ENCODER_AS5047P_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file
 * @brief Diagnostics and zero point specific to the ams AS5047P driver.
 *
 * The generic encoder class in @ref drivers/encoder.h reports only an aggregate
 * error count. The AS5047P tells a failed SPI transfer, a corrupted frame and an
 * error it raised itself apart, and it reports the state of the magnetic field
 * separately from any transfer, so those live here rather than in the class API.
 *
 * Every function here performs SPI transfers or takes the lock the polling
 * thread holds during one, so none of them may be called from an interrupt.
 *
 * @defgroup encoder_as5047p AS5047P driver
 * @ingroup encoder_interface
 * @{
 */

/** @brief Width of the angle and of the zero position, in bits. */
#define AS5047P_ANGLE_BITS 14U

/**
 * @brief Reason a read of the angle failed.
 */
enum as5047p_error_cause {
  /** The SPI controller reported an error for the transfer. */
  AS5047P_ERROR_SPI,
  /** The response failed its even parity check, so its content is unreliable. */
  AS5047P_ERROR_PARITY,
  /**
   * The response arrived intact but had its error flag set. The cause the
   * device latched is in @ref as5047p_stats.last_errfl.
   */
  AS5047P_ERROR_FLAG,
  /** Number of causes, for sizing arrays. */
  AS5047P_ERROR_CAUSE_COUNT,
};

/** @brief ERRFL bit: the device saw a frame with the wrong number of clocks. */
#define AS5047P_ERRFL_FRERR  0x0001U
/** @brief ERRFL bit: the device was asked to access an invalid register. */
#define AS5047P_ERRFL_INVCOMM 0x0002U
/** @brief ERRFL bit: a frame the device received failed its parity check. */
#define AS5047P_ERRFL_PARERR 0x0004U

/**
 * @brief Failed reads of the angle, by cause.
 *
 * Counters are 32-bit and wrap around. Compare successive readings rather than
 * treating them as absolute totals. Their sum over all causes equals
 * @ref encoder_feedback.error_count.
 */
struct as5047p_stats
{
  /** Failed reads of the angle, indexed by @ref as5047p_error_cause. */
  uint32_t errors[AS5047P_ERROR_CAUSE_COUNT];
  /**
   * @brief ERRFL as read after the most recent @ref AS5047P_ERROR_FLAG.
   *
   * A combination of the AS5047P_ERRFL_ bits, or 0 when no error flag has been
   * seen yet or ERRFL could not be read back. Reading ERRFL clears it on the
   * device, which the driver does as soon as it sees the flag.
   */
  uint16_t last_errfl;
};

/**
 * @brief State of the magnetic field, from the DIAAGC register.
 *
 * These describe the magnet rather than the SPI link: a reading can arrive
 * without error while the magnet is too far away to be trusted.
 */
struct as5047p_diag
{
  /** Automatic gain control value, 0 to 255. High means a weak field. */
  uint8_t agc;
  /** The field is too strong: the gain control is at its minimum. */
  bool magh;
  /** The field is too weak: the gain control is at its maximum. */
  bool magl;
  /** The CORDIC overflowed, so the angle is invalid. */
  bool cof;
  /** Offset compensation has finished. False shortly after power-on. */
  bool lf;
};

/**
 * @brief Read the per-cause failure counters.
 *
 * @param dev AS5047P encoder device.
 * @param stats Destination for the counters.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not an AS5047P encoder, or @p stats is NULL.
 */
int as5047p_get_stats(const struct device * dev, struct as5047p_stats * stats);

/**
 * @brief Read the state of the magnetic field.
 *
 * This performs a transfer of its own, interleaved with the polling thread.
 *
 * @param dev AS5047P encoder device.
 * @param diag Destination for the field state.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not an AS5047P encoder, or @p diag is NULL.
 * @retval -EIO The response was corrupted or had its error flag set.
 * @retval negative_errno The SPI transfer failed.
 */
int as5047p_read_diag(const struct device * dev, struct as5047p_diag * diag);

/**
 * @brief Read the zero position the device currently subtracts.
 *
 * @param dev AS5047P encoder device.
 * @param zero_position Destination for the zero position, in counts of
 *        the 14-bit angle (0 to 16383).
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not an AS5047P encoder, or @p zero_position is NULL.
 * @retval -EIO A response was corrupted or had its error flag set.
 * @retval negative_errno An SPI transfer failed.
 */
int as5047p_get_zero_position(const struct device * dev, uint16_t * zero_position);

/**
 * @brief Set the zero position the device subtracts from the angle.
 *
 * The value is held in volatile registers and is lost when the device loses
 * power, so an application that keeps a zero point across power cycles stores
 * it itself and sets it again after start-up. Programming it into the OTP is
 * irreversible and is deliberately not offered.
 *
 * The reported angle jumps by the change, so the driver rebuilds its
 * accumulator and advances @ref encoder_feedback.position_epoch on the next
 * reading.
 *
 * @param dev AS5047P encoder device.
 * @param zero_position Zero position in counts of the 14-bit angle (0 to 16383).
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not an AS5047P encoder, or @p zero_position is
 *         out of range.
 * @retval -EIO A response was corrupted or had its error flag set.
 * @retval negative_errno An SPI transfer failed.
 */
int as5047p_set_zero_position(const struct device * dev, uint16_t zero_position);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
