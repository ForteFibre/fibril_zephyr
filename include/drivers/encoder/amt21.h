#ifndef FIBRIL_ZEPHYR_INCLUDE_DRIVERS_ENCODER_AMT21_H_
#define FIBRIL_ZEPHYR_INCLUDE_DRIVERS_ENCODER_AMT21_H_

#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file
 * @brief Diagnostics specific to the Same Sky (CUI Devices) AMT21x driver.
 *
 * The generic encoder class in @ref drivers/encoder.h reports only an aggregate
 * error count, because the ways a transaction can fail depend entirely on the
 * transport. The counters here name the failure modes of the AMT21 RS485
 * protocol, so they live in this driver-specific header rather than in the class
 * API.
 *
 * Statistics are only recorded when @kconfig{CONFIG_ENCODER_AMT21_STATS} is
 * enabled. The types and functions below are always declared, so callers do not
 * need to guard their code; the accessors return @c -ENOTSUP instead.
 *
 * All counters are 32-bit and wrap around. Compare successive readings rather
 * than treating them as absolute totals.
 *
 * @defgroup encoder_amt21 AMT21x driver
 * @ingroup encoder_interface
 * @{
 */

/**
 * @brief Reason a transaction failed.
 */
enum amt21_error_cause {
  /** No response, or a response that stopped part way through. */
  AMT21_ERROR_TIMEOUT,
  /** The two check bits of the response did not match its data. */
  AMT21_ERROR_CHECKSUM,
  /** More bytes arrived than the transaction expected. */
  AMT21_ERROR_DESYNC,
  /** The echoed command byte was missing or did not match what was sent. */
  AMT21_ERROR_ECHO,
  /** The UART rejected a transmit or receive request. */
  AMT21_ERROR_BUS,
  /** Number of causes, for sizing arrays. */
  AMT21_ERROR_CAUSE_COUNT,
};

/**
 * @brief Per-encoder transaction statistics.
 *
 * These count events that belong to a single encoder, so a failing encoder can
 * be told apart from a failing bus.
 */
struct amt21_stats
{
  /** Transactions attempted, including retries. */
  uint32_t transactions;
  /** Transactions that produced a valid reading. */
  uint32_t successes;
  /** Failed transactions, indexed by @ref amt21_error_cause. */
  uint32_t errors[AMT21_ERROR_CAUSE_COUNT];
  /** Transactions that were retried. */
  uint32_t retries;
  /** Consecutive failures as of the most recent transaction. */
  uint32_t consecutive_errors;
  /** Largest number of consecutive failures observed. */
  uint32_t max_consecutive_errors;
  /** Cause of the most recent failure. */
  enum amt21_error_cause last_error;
  /** Timestamp of the most recent failure in milliseconds. */
  int64_t last_error_timestamp_ms;
};

/**
 * @brief Per-bus statistics.
 *
 * These count events that belong to the UART rather than to any one encoder.
 * Attributing them to an encoder would be misleading, since a single overrun
 * disturbs every transaction on the bus.
 */
struct amt21_bus_stats
{
  /** Scans of every encoder on the bus that ran to completion. */
  uint32_t scans;
  /** Scans skipped because the previous one overran the poll interval. */
  uint32_t scans_skipped;
  /** Times reception was restarted after being stopped or resynchronised. */
  uint32_t rx_restarts;
  /** UART framing and parity errors reported by the driver. */
  uint32_t framing_errors;
  /** UART overrun errors reported by the driver. */
  uint32_t overrun_errors;
  /** Received bytes that did not belong to any transaction. */
  uint32_t stray_bytes;
};

/**
 * @brief One recorded transaction failure.
 *
 * The raw bytes are the useful part: they are what distinguishes a framing
 * desync from a checksum failure or a stray echo byte.
 */
struct amt21_error_record
{
  /** Timestamp of the failure in milliseconds. */
  int64_t timestamp_ms;
  /** RS485 node address of the encoder. */
  uint8_t node_addr;
  /** Command byte that was sent. */
  uint8_t command;
  /** Number of valid bytes in @ref amt21_error_record.rx_bytes. */
  uint8_t rx_len;
  /** Bytes that were received, truncated to the size of this field. */
  uint8_t rx_bytes[4];
  /** Reason the transaction failed. */
  enum amt21_error_cause cause;
};

/**
 * @brief Read the transaction statistics of an AMT21 encoder.
 *
 * @param dev AMT21 encoder device instance.
 * @param stats Destination for the statistics.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not an AMT21 encoder, or @p stats is NULL.
 * @retval -ENOTSUP @kconfig{CONFIG_ENCODER_AMT21_STATS} is disabled.
 */
int amt21_get_stats(const struct device * dev, struct amt21_stats * stats);

/**
 * @brief Reset the transaction statistics of an AMT21 encoder.
 *
 * @param dev AMT21 encoder device instance.
 * @retval 0 Success.
 * @retval -EINVAL @p dev is not an AMT21 encoder.
 * @retval -ENOTSUP @kconfig{CONFIG_ENCODER_AMT21_STATS} is disabled.
 */
int amt21_clear_stats(const struct device * dev);

/**
 * @brief Read back recorded transaction failures for an AMT21 encoder.
 *
 * Records are returned newest first. The log holds
 * @kconfig{CONFIG_ENCODER_AMT21_ERROR_LOG_SIZE} entries and overwrites the
 * oldest one when full.
 *
 * @param dev AMT21 encoder device instance.
 * @param buf Destination array.
 * @param count Number of entries @p buf can hold.
 * @retval non-negative Number of records written to @p buf.
 * @retval -EINVAL @p dev is not an AMT21 encoder, or @p buf is NULL.
 * @retval -ENOTSUP The error log is disabled.
 */
int amt21_get_error_log(
  const struct device * dev, struct amt21_error_record * buf, size_t count);

/**
 * @brief Read the statistics of an AMT21 bus.
 *
 * @param bus_dev AMT21 bus device instance, the parent of the encoder nodes.
 * @param stats Destination for the statistics.
 * @retval 0 Success.
 * @retval -EINVAL @p bus_dev is not an AMT21 bus, or @p stats is NULL.
 * @retval -ENOTSUP @kconfig{CONFIG_ENCODER_AMT21_STATS} is disabled.
 */
int amt21_bus_get_stats(const struct device * bus_dev, struct amt21_bus_stats * stats);

/**
 * @brief Reset the statistics of an AMT21 bus.
 *
 * @param bus_dev AMT21 bus device instance.
 * @retval 0 Success.
 * @retval -EINVAL @p bus_dev is not an AMT21 bus.
 * @retval -ENOTSUP @kconfig{CONFIG_ENCODER_AMT21_STATS} is disabled.
 */
int amt21_bus_clear_stats(const struct device * bus_dev);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
