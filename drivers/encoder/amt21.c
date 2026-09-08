/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Driver for Same Sky (CUI Devices) AMT21x absolute encoders on a half-duplex
 * RS485 bus.
 *
 * The protocol is a single command byte followed by a two byte response, low
 * byte first, whose top two bits are odd-parity check bits over the 14 data
 * bits. At 2 Mbps the encoder starts replying about 3 us after the command
 * byte, which is far too soon to arm a receiver in software. The transport
 * therefore enables reception once at init and never disables it during normal
 * operation; every received byte lands in a ring buffer that the poll thread
 * consumes. Re-arming per transaction is what loses the first byte of the
 * response.
 */

#include <errno.h>
#include <string.h>

#include <drivers/encoder.h>
#include <drivers/encoder/amt21.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(encoder_amt21, CONFIG_ENCODER_LOG_LEVEL);

/** Command appended to the node address to read the single-turn position. */
#define AMT21_CMD_POSITION 0x00U
/** Command appended to the node address to read the turns counter. */
#define AMT21_CMD_TURNS 0x01U
/** Command appended to the node address to begin a two byte extended command. */
#define AMT21_CMD_EXTENDED 0x02U

/** Extended command that stores the current position as the zero point. */
#define AMT21_EXT_SET_ZERO 0x5EU
/** Extended command that resets the encoder. */
#define AMT21_EXT_RESET 0x75U

/** Data bits carried by a response, below the two check bits. */
#define AMT21_DATA_BITS 14
#define AMT21_DATA_MASK BIT_MASK(AMT21_DATA_BITS)
#define AMT21_TURNS_SIGN_BIT BIT(AMT21_DATA_BITS - 1)

/** Response length in bytes, excluding any echoed command byte. */
#define AMT21_RESP_LEN 2
/** Longest chunk the receiver is configured with, an echo plus a response. */
#define AMT21_MAX_CHUNK (AMT21_RESP_LEN + 1)

/** Ring buffer depth for received bytes. Deliberately larger than a chunk so
 *  that stray bytes are captured rather than silently dropped.
 */
#define AMT21_RX_RING_SIZE 16

/** Startup time of the encoder after a reset, from the datasheet, plus margin. */
#define AMT21_RESET_BLACKOUT_MS 250

/** Bound on how long the transport waits for the receive line to fall quiet. */
#define AMT21_QUIET_WAIT_ATTEMPTS 3

/**
 * Multiplier applied to response-timeout-us when waiting for bytes to arrive.
 *
 * A response that fills the receive chunk exactly is delivered as soon as the
 * last byte lands, so this has no effect in the steady state. It matters for a
 * short chunk, which the UART only reports after its own inactivity timeout of
 * response-timeout-us has elapsed: waiting for exactly that long would expire
 * just before the data is handed over. Both the echo probe and a truncated
 * response take that path.
 */
#define AMT21_COLLECT_TIMEOUT_FACTOR 2

/** Spin granularity used while waiting for a response below tick resolution. */
#define AMT21_SPIN_STEP_US 5U

/** Timeout for handing bytes to the UART, in microseconds as uart_tx() expects. */
#define AMT21_TX_TIMEOUT_US 10000
/** Timeout for the transmission to complete. */
#define AMT21_TX_DONE_TIMEOUT K_USEC(AMT21_TX_TIMEOUT_US)

#if defined(CONFIG_ENCODER_AMT21_STATS)
#define AMT21_STATS_ENABLED 1
#else
#define AMT21_STATS_ENABLED 0
#endif

#if defined(CONFIG_ENCODER_AMT21_STATS) && (CONFIG_ENCODER_AMT21_ERROR_LOG_SIZE > 0)
#define AMT21_ERROR_LOG_ENABLED 1
#define AMT21_ERROR_LOG_SIZE CONFIG_ENCODER_AMT21_ERROR_LOG_SIZE
#else
#define AMT21_ERROR_LOG_ENABLED 0
#endif

/** Whether the transmitted byte is echoed back by the transceiver. */
enum amt21_echo_state {
  AMT21_ECHO_UNKNOWN = 0,
  AMT21_ECHO_ABSENT,
  AMT21_ECHO_PRESENT,
};

struct amt21_bus_config
{
  const struct device * uart;
  struct gpio_dt_spec de;
  bool has_de;
  bool tx_echo_forced;
  uint32_t poll_interval_us;
  uint32_t response_timeout_us;
  uint32_t inter_command_delay_us;
  uint32_t rx_quiet_period_us;
  uint8_t max_retries;
  uint8_t offline_threshold;
};

struct amt21_bus_data
{
  const struct device * dev;

  /* Serialises transactions between the poll thread and set_zero/reset. */
  struct k_mutex lock;

  struct k_sem tx_sem;
  struct k_sem rx_sem;

  struct ring_buf rx_rb;
  uint8_t rx_rb_buf[AMT21_RX_RING_SIZE];

  /* Two chunk sized buffers alternated through UART_RX_BUF_REQUEST. Failing to
   * supply the next buffer stops reception, which would lose every subsequent
   * response, so the callback always answers the request.
   */
  uint8_t rx_buf[2][AMT21_MAX_CHUNK];
  uint8_t rx_buf_idx;
  uint8_t rx_chunk;

  uint8_t tx_buf[2];

  enum amt21_echo_state echo;

  /* Set from the UART callback when reception stopped and has to be restarted. */
  atomic_t rx_restart_needed;
  /* Suppresses the restart request caused by our own uart_rx_disable(). */
  atomic_t rx_stopping;

  uint32_t last_rx_cycle;
  /* Instant the previous transaction finished, used to honour the gap. */
  uint32_t last_tx_end_cycle;
  bool last_tx_end_valid;

  const struct device * encoders[CONFIG_ENCODER_AMT21_MAX_ENCODERS];
  uint8_t encoder_count;

#if AMT21_STATS_ENABLED
  struct amt21_bus_stats stats;
#endif

  struct k_thread thread;
};

struct amt21_encoder_config
{
  const struct device * bus;
  uint8_t node_addr;
  uint8_t resolution;
  bool multiturn;
};

struct amt21_encoder_data
{
  struct k_spinlock lock;
  struct encoder_feedback feedback;
  bool has_reading;
  uint32_t consecutive_errors;
  int64_t blackout_until_ms;

  /* Accumulator state. raw_count is what the readings add up to and bias is the
   * offset requested through encoder_set_position(); the reported position is
   * their sum. has_accum being false means the next successful reading rebuilds
   * both instead of continuing them.
   */
  bool has_accum;
  int64_t raw_count;
  int64_t bias;
  uint32_t last_single_turn;
  /* Instant of the previous successful reading, for the velocity interval. */
  uint32_t last_commit_cycle;
  bool has_commit_cycle;

#if AMT21_STATS_ENABLED
  struct amt21_stats stats;
#endif
#if AMT21_ERROR_LOG_ENABLED
  struct amt21_error_record log[AMT21_ERROR_LOG_SIZE];
  uint8_t log_head;
  uint8_t log_count;
#endif
};

/**
 * @brief Marker placed in the API slot of a bus device.
 *
 * A bus is not an encoder and has no class API, but the driver specific
 * accessors need a way to reject a device that is not an AMT21 bus.
 */
struct amt21_bus_api
{
  uint8_t reserved;
};

static const struct encoder_driver_api amt21_encoder_api;
static const struct amt21_bus_api amt21_bus_api;

/* -------------------------------------------------------------------------- */
/* Statistics                                                                 */
/* -------------------------------------------------------------------------- */

#if AMT21_STATS_ENABLED
#define AMT21_BUS_STAT_INC(bus, field) ((bus)->stats.field++)
#else
#define AMT21_BUS_STAT_INC(bus, field) ((void)(bus))
#endif

static void amt21_stats_record_success(struct amt21_encoder_data * data)
{
#if AMT21_STATS_ENABLED
  data->stats.transactions++;
  data->stats.successes++;
  data->stats.consecutive_errors = 0;
#else
  ARG_UNUSED(data);
#endif
}

static void amt21_stats_record_attempt(struct amt21_encoder_data * data, bool is_retry)
{
#if AMT21_STATS_ENABLED
  data->stats.transactions++;
  if (is_retry) {
    data->stats.retries++;
  }
#else
  ARG_UNUSED(data);
  ARG_UNUSED(is_retry);
#endif
}

static void amt21_stats_record_error(
  struct amt21_encoder_data * data, enum amt21_error_cause cause, int64_t now_ms)
{
#if AMT21_STATS_ENABLED
  data->stats.errors[cause]++;
  data->stats.consecutive_errors++;
  data->stats.max_consecutive_errors =
    MAX(data->stats.max_consecutive_errors, data->stats.consecutive_errors);
  data->stats.last_error = cause;
  data->stats.last_error_timestamp_ms = now_ms;
#else
  ARG_UNUSED(data);
  ARG_UNUSED(cause);
  ARG_UNUSED(now_ms);
#endif
}

static void amt21_stats_log_error(
  struct amt21_encoder_data * data, uint8_t node_addr, uint8_t command, const uint8_t * rx,
  size_t rx_len, enum amt21_error_cause cause, int64_t now_ms)
{
#if AMT21_ERROR_LOG_ENABLED
  struct amt21_error_record * rec = &data->log[data->log_head];

  memset(rec, 0, sizeof(*rec));
  rec->timestamp_ms = now_ms;
  rec->node_addr = node_addr;
  rec->command = command;
  rec->rx_len = (uint8_t)MIN(rx_len, sizeof(rec->rx_bytes));
  if (rec->rx_len > 0U) {
    memcpy(rec->rx_bytes, rx, rec->rx_len);
  }
  rec->cause = cause;

  data->log_head = (uint8_t)((data->log_head + 1U) % AMT21_ERROR_LOG_SIZE);
  if (data->log_count < AMT21_ERROR_LOG_SIZE) {
    data->log_count++;
  }
#else
  ARG_UNUSED(data);
  ARG_UNUSED(node_addr);
  ARG_UNUSED(command);
  ARG_UNUSED(rx);
  ARG_UNUSED(rx_len);
  ARG_UNUSED(cause);
  ARG_UNUSED(now_ms);
#endif
}

/* -------------------------------------------------------------------------- */
/* Protocol helpers                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Verify the two odd-parity check bits of a response.
 *
 * K1 covers the odd numbered data bits and K0 the even numbered ones.
 */
static bool amt21_checksum_ok(uint16_t msg)
{
  uint16_t checksum = 0x3U;

  for (int i = 0; i < AMT21_DATA_BITS; i += 2) {
    checksum ^= (uint16_t)((msg >> i) & 0x3U);
  }

  return checksum == (uint16_t)(msg >> AMT21_DATA_BITS);
}

static uint32_t amt21_position_from_msg(uint16_t msg, uint8_t resolution)
{
  uint32_t raw = msg & AMT21_DATA_MASK;

  /* A 12-bit encoder reports its position in the upper 12 bits of the field. */
  return (resolution == 12U) ? (raw >> 2) : raw;
}

static int32_t amt21_turns_from_msg(uint16_t msg)
{
  uint32_t raw = msg & AMT21_DATA_MASK;

  /* Sign extend the 14-bit two's complement turns counter. */
  return (int32_t)(raw ^ AMT21_TURNS_SIGN_BIT) - (int32_t)AMT21_TURNS_SIGN_BIT;
}

/**
 * @brief Signed change between two single-turn readings.
 *
 * The shorter of the two ways round is taken as the real motion, which is
 * correct as long as the encoder turns by less than half a revolution between
 * two readings. At 14 bits and the default 1000 us poll interval that limit is
 * about 30000 rpm, well above the motors this driver is used with.
 *
 * Only the single-turn reading feeds the accumulator. The turns counter is a
 * separate transaction taken after a gap, so a revolution boundary crossed
 * inside that gap would make the combined value jump by exactly one
 * revolution, and nothing downstream could tell that apart from real motion.
 */
static int32_t amt21_single_turn_delta(uint32_t current, uint32_t previous, uint8_t resolution)
{
  const int32_t span = (int32_t)BIT(resolution);
  int32_t delta = (int32_t)current - (int32_t)previous;

  if (delta > (span / 2)) {
    delta -= span;
  } else if (delta < -(span / 2)) {
    delta += span;
  }

  return delta;
}

/** @brief Absolute count an accumulator is rebuilt from. */
static int64_t amt21_absolute_count(
  uint32_t single_turn, int32_t turns, bool have_turns, uint8_t resolution)
{
  if (!have_turns) {
    return (int64_t)single_turn;
  }

  return ((int64_t)turns << resolution) + (int64_t)single_turn;
}

/* -------------------------------------------------------------------------- */
/* UART plumbing                                                              */
/* -------------------------------------------------------------------------- */

static void amt21_de_set(const struct device * bus_dev, bool active)
{
  const struct amt21_bus_config * config = bus_dev->config;

  if (config->has_de) {
    (void)gpio_pin_set_dt(&config->de, active ? 1 : 0);
  }
}

static void amt21_uart_callback(
  const struct device * uart, struct uart_event * evt, void * user_data)
{
  const struct device * bus_dev = user_data;
  struct amt21_bus_data * bus = bus_dev->data;

  ARG_UNUSED(uart);

  switch (evt->type) {
    case UART_TX_DONE:
    case UART_TX_ABORTED:
      amt21_de_set(bus_dev, false);
      k_sem_give(&bus->tx_sem);
      break;

    case UART_RX_RDY:
      bus->last_rx_cycle = k_cycle_get_32();
      (void)ring_buf_put(&bus->rx_rb, evt->data.rx.buf + evt->data.rx.offset, evt->data.rx.len);
      k_sem_give(&bus->rx_sem);
      break;

    case UART_RX_BUF_REQUEST:
      /* Always answer, otherwise reception stops for good. */
      (void)uart_rx_buf_rsp(uart, bus->rx_buf[bus->rx_buf_idx], bus->rx_chunk);
      bus->rx_buf_idx ^= 1U;
      break;

    case UART_RX_BUF_RELEASED:
      break;

    case UART_RX_STOPPED:
#if AMT21_STATS_ENABLED
      if ((evt->data.rx_stop.reason & UART_ERROR_OVERRUN) != 0) {
        bus->stats.overrun_errors++;
      }
      if ((evt->data.rx_stop.reason & (UART_ERROR_FRAMING | UART_ERROR_PARITY)) != 0) {
        bus->stats.framing_errors++;
      }
#endif
      atomic_set(&bus->rx_restart_needed, 1);
      k_sem_give(&bus->rx_sem);
      break;

    case UART_RX_DISABLED:
      if (atomic_get(&bus->rx_stopping) == 0) {
        atomic_set(&bus->rx_restart_needed, 1);
        k_sem_give(&bus->rx_sem);
      }
      break;

    default:
      break;
  }
}

static int amt21_rx_start(const struct device * bus_dev, uint8_t chunk)
{
  const struct amt21_bus_config * config = bus_dev->config;
  struct amt21_bus_data * bus = bus_dev->data;

  bus->rx_chunk = chunk;
  bus->rx_buf_idx = 1U;
  ring_buf_reset(&bus->rx_rb);
  k_sem_reset(&bus->rx_sem);
  atomic_set(&bus->rx_restart_needed, 0);

  return uart_rx_enable(config->uart, bus->rx_buf[0], chunk, config->response_timeout_us);
}

static void amt21_rx_stop(const struct device * bus_dev)
{
  const struct amt21_bus_config * config = bus_dev->config;
  struct amt21_bus_data * bus = bus_dev->data;

  atomic_set(&bus->rx_stopping, 1);
  (void)uart_rx_disable(config->uart);
  atomic_set(&bus->rx_stopping, 0);
}

/**
 * @brief Restart reception, resynchronising the chunk boundary.
 *
 * Called on the error path only. The 3 us turnaround does not apply here
 * because a gap is always observed before the next command byte.
 */
static int amt21_rx_restart(const struct device * bus_dev, uint8_t chunk)
{
  struct amt21_bus_data * bus = bus_dev->data;
  int ret;

  amt21_rx_stop(bus_dev);
  ret = amt21_rx_start(bus_dev, chunk);
  if (ret < 0) {
    LOG_ERR("%s: failed to restart reception (%d)", bus_dev->name, ret);
    return ret;
  }

  AMT21_BUS_STAT_INC(bus, rx_restarts);

  return 0;
}

static uint8_t amt21_expected_chunk(const struct amt21_bus_data * bus)
{
  /* While the echo state is unknown the transport listens for the longer of the
   * two possibilities and lets the receive timeout deliver a short response.
   */
  return (bus->echo == AMT21_ECHO_ABSENT) ? AMT21_RESP_LEN : AMT21_MAX_CHUNK;
}

/* -------------------------------------------------------------------------- */
/* Timing helpers                                                             */
/* -------------------------------------------------------------------------- */

static uint32_t amt21_cycles_to_us(uint32_t cycles)
{
  return (uint32_t)k_cyc_to_us_floor64(cycles);
}

/** @brief Wait until @p us have elapsed, sleeping when that is worth doing. */
static void amt21_wait_us(uint32_t us)
{
  uint32_t tick_us = k_ticks_to_us_floor32(1U);

  if (us == 0U) {
    return;
  }

  if ((tick_us > 0U) && (us >= tick_us)) {
    /* Give the CPU up for the bulk of the gap, then spin for the remainder. */
    uint32_t sleep_ticks = us / MAX(tick_us, 1U);

    (void)k_sleep(K_TICKS(sleep_ticks));

    uint32_t spent = sleep_ticks * tick_us;
    if (us > spent) {
      k_busy_wait(us - spent);
    }
    return;
  }

  k_busy_wait(us);
}

/**
 * @brief Honour the gap the encoder needs between two commands.
 *
 * Sending commands back to back without this gap is the most common way to make
 * an AMT21 drop a response.
 */
static void amt21_gap_wait(const struct device * bus_dev)
{
  const struct amt21_bus_config * config = bus_dev->config;
  struct amt21_bus_data * bus = bus_dev->data;

  if (!bus->last_tx_end_valid) {
    return;
  }

  uint32_t elapsed_us = amt21_cycles_to_us(k_cycle_get_32() - bus->last_tx_end_cycle);

  if (elapsed_us < config->inter_command_delay_us) {
    amt21_wait_us(config->inter_command_delay_us - elapsed_us);
  }
}

/**
 * @brief Make sure the line is quiet and the ring is empty before transmitting.
 *
 * A byte still in flight means a previous response was late. Transmitting on top
 * of it would corrupt both frames, so the leftovers are counted and discarded.
 */
static void amt21_wait_for_quiet(const struct device * bus_dev)
{
  const struct amt21_bus_config * config = bus_dev->config;
  struct amt21_bus_data * bus = bus_dev->data;

  for (int attempt = 0; attempt < AMT21_QUIET_WAIT_ATTEMPTS; ++attempt) {
    uint32_t stray = ring_buf_size_get(&bus->rx_rb);

    if (stray > 0U) {
#if AMT21_STATS_ENABLED
      bus->stats.stray_bytes += stray;
#endif
      ring_buf_reset(&bus->rx_rb);
    }

    uint32_t idle_us = amt21_cycles_to_us(k_cycle_get_32() - bus->last_rx_cycle);

    if ((idle_us >= config->rx_quiet_period_us) && (stray == 0U)) {
      break;
    }

    amt21_wait_us(config->rx_quiet_period_us);
  }

  k_sem_reset(&bus->rx_sem);
  ring_buf_reset(&bus->rx_rb);
}

/* -------------------------------------------------------------------------- */
/* Transactions                                                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief Collect up to @p want bytes, giving up after @p timeout_us.
 *
 * The deadline is tracked in cycles rather than left to the kernel, because a
 * response window of a few hundred microseconds is finer than the tick on many
 * configurations. Handing such a timeout to k_sem_take() rounds it up to a whole
 * tick, which on a 100 Hz tick turns a 200 us wait into 10 ms and collapses the
 * achievable poll rate.
 */
static size_t amt21_collect(
  struct amt21_bus_data * bus, uint8_t * buf, size_t want, uint32_t timeout_us)
{
  uint32_t start = k_cycle_get_32();
  uint32_t budget = (uint32_t)k_us_to_cyc_ceil32(timeout_us);
  uint32_t tick_us = k_ticks_to_us_floor32(1U);
  size_t got = 0;

  while (true) {
    got += ring_buf_get(&bus->rx_rb, &buf[got], want - got);
    if (got >= want) {
      break;
    }

    uint32_t elapsed = k_cycle_get_32() - start;

    if (elapsed >= budget) {
      break;
    }

    uint32_t remaining_us = amt21_cycles_to_us(budget - elapsed);

    if ((tick_us > 0U) && (remaining_us >= tick_us)) {
      (void)k_sem_take(&bus->rx_sem, K_USEC(remaining_us));
    } else {
      /* Below the tick resolution, so spin in short steps instead. */
      (void)k_sem_take(&bus->rx_sem, K_NO_WAIT);
      k_busy_wait(MIN(remaining_us, AMT21_SPIN_STEP_US));
    }
  }

  return got;
}

/** @brief Send @p len command bytes without expecting a response. */
static int amt21_send(const struct device * bus_dev, const uint8_t * bytes, size_t len)
{
  const struct amt21_bus_config * config = bus_dev->config;
  struct amt21_bus_data * bus = bus_dev->data;
  int ret;

  if ((len == 0U) || (len > sizeof(bus->tx_buf))) {
    return -EINVAL;
  }

  memcpy(bus->tx_buf, bytes, len);

  k_sem_reset(&bus->tx_sem);
  amt21_de_set(bus_dev, true);

  ret = uart_tx(config->uart, bus->tx_buf, len, AMT21_TX_TIMEOUT_US);
  if (ret < 0) {
    amt21_de_set(bus_dev, false);
    return ret;
  }

  if (k_sem_take(&bus->tx_sem, AMT21_TX_DONE_TIMEOUT) != 0) {
    (void)uart_tx_abort(config->uart);
    amt21_de_set(bus_dev, false);
    return -ETIMEDOUT;
  }

  return 0;
}

/**
 * @brief Run one command and response exchange.
 *
 * @param raw Receives the bytes that arrived, for the error log.
 * @param raw_len Receives the number of bytes in @p raw.
 * @param cause Receives the failure reason when the return value is negative.
 */
static int amt21_transact(
  const struct device * bus_dev, uint8_t command, uint16_t * msg, uint8_t * raw, size_t * raw_len,
  enum amt21_error_cause * cause)
{
  const struct amt21_bus_config * config = bus_dev->config;
  struct amt21_bus_data * bus = bus_dev->data;
  uint8_t chunk = amt21_expected_chunk(bus);
  size_t got;
  int ret;

  *raw_len = 0;
  *cause = AMT21_ERROR_BUS;

  /* Restart when reception stopped, or when the chunk length must change. */
  if ((atomic_get(&bus->rx_restart_needed) != 0) || (bus->rx_chunk != chunk)) {
    (void)amt21_rx_restart(bus_dev, chunk);
  }

  amt21_gap_wait(bus_dev);
  amt21_wait_for_quiet(bus_dev);

  ret = amt21_send(bus_dev, &command, 1U);
  if (ret < 0) {
    bus->last_tx_end_cycle = k_cycle_get_32();
    bus->last_tx_end_valid = true;
    *cause = AMT21_ERROR_BUS;
    return ret;
  }

  got = amt21_collect(
    bus, raw, chunk, config->response_timeout_us * AMT21_COLLECT_TIMEOUT_FACTOR);
  *raw_len = got;

  bus->last_tx_end_cycle = k_cycle_get_32();
  bus->last_tx_end_valid = true;

  /* Latch whether the transceiver echoes our command byte. Until this is known
   * the transport asks for one extra byte, so a short read here means no echo.
   */
  if (bus->echo == AMT21_ECHO_UNKNOWN) {
    if ((got == AMT21_MAX_CHUNK) && (raw[0] == command)) {
      bus->echo = AMT21_ECHO_PRESENT;
    } else if (got == AMT21_RESP_LEN) {
      bus->echo = AMT21_ECHO_ABSENT;
    } else if (got == AMT21_MAX_CHUNK) {
      /* Full chunk arrived but the first byte is not our echo. The
       * transport is out of sync with the frame boundaries; treat as
       * DESYNC and rearm before returning so the next transaction starts
       * from a clean state.
       */
      *cause = AMT21_ERROR_DESYNC;
      (void)amt21_rx_restart(bus_dev, chunk);
      return -EIO;
    }
  }

  size_t offset = (bus->echo == AMT21_ECHO_PRESENT) ? 1U : 0U;

  if (got < (offset + AMT21_RESP_LEN)) {
    *cause = AMT21_ERROR_TIMEOUT;
    (void)amt21_rx_restart(bus_dev, amt21_expected_chunk(bus));
    return -ETIMEDOUT;
  }

  if ((offset == 1U) && (raw[0] != command)) {
    *cause = AMT21_ERROR_ECHO;
    (void)amt21_rx_restart(bus_dev, amt21_expected_chunk(bus));
    return -EIO;
  }

  if (ring_buf_size_get(&bus->rx_rb) > 0U) {
    *cause = AMT21_ERROR_DESYNC;
    (void)amt21_rx_restart(bus_dev, amt21_expected_chunk(bus));
    return -EIO;
  }

  uint16_t value = (uint16_t)raw[offset] | ((uint16_t)raw[offset + 1U] << 8);

  if (!amt21_checksum_ok(value)) {
    *cause = AMT21_ERROR_CHECKSUM;
    return -EIO;
  }

  *msg = value;

  return 0;
}

/** @brief Run a transaction, retrying up to the configured limit. */
static int amt21_transact_retry(
  const struct device * bus_dev, const struct device * enc_dev, uint8_t command, uint16_t * msg)
{
  const struct amt21_bus_config * config = bus_dev->config;
  const struct amt21_encoder_config * enc_config = enc_dev->config;
  struct amt21_encoder_data * enc = enc_dev->data;
  enum amt21_error_cause cause = AMT21_ERROR_BUS;
  uint8_t raw[AMT21_MAX_CHUNK];
  size_t raw_len = 0;
  int ret = -EIO;

  for (uint8_t attempt = 0; attempt <= config->max_retries; ++attempt) {
    amt21_stats_record_attempt(enc, attempt > 0U);

    ret = amt21_transact(bus_dev, command, msg, raw, &raw_len, &cause);
    if (ret == 0) {
      return 0;
    }

    int64_t now_ms = k_uptime_get();

    amt21_stats_record_error(enc, cause, now_ms);
    amt21_stats_log_error(
      enc, enc_config->node_addr, command, raw, raw_len, cause, now_ms);
  }

  LOG_DBG("%s: command 0x%02x failed, cause %d", enc_dev->name, command, (int)cause);

  return ret;
}

/* -------------------------------------------------------------------------- */
/* Feedback bookkeeping                                                       */
/* -------------------------------------------------------------------------- */

static void amt21_commit_success(
  const struct device * bus_dev, const struct device * enc_dev, uint32_t single_turn,
  bool have_turns, int32_t turns)
{
  const struct amt21_encoder_config * config = enc_dev->config;
  struct amt21_encoder_data * data = enc_dev->data;
  uint32_t now_cycle = k_cycle_get_32();
  k_spinlock_key_t key;

  ARG_UNUSED(bus_dev);

  key = k_spin_lock(&data->lock);

  data->feedback.valid_mask = ENCODER_FEEDBACK_POSITION | ENCODER_FEEDBACK_SINGLE_TURN;

  if (data->has_accum) {
    data->raw_count +=
      amt21_single_turn_delta(single_turn, data->last_single_turn, config->resolution);
  } else {
    /* Nothing to continue from, so start over at the absolute reading and tell
     * consumers that the previous position and any offset no longer apply.
     */
    data->raw_count = amt21_absolute_count(single_turn, turns, have_turns, config->resolution);
    data->bias = 0;
    data->feedback.position_epoch++;
    data->has_accum = true;
    data->has_commit_cycle = false;
  }

  data->last_single_turn = single_turn;

  int64_t previous_position = data->feedback.position;

  data->feedback.position = data->raw_count + data->bias;

  /* The interval is measured rather than taken from poll-interval-us, because a
   * scan that overruns is skipped and a transaction may have been retried.
   */
  if (data->has_commit_cycle) {
    uint32_t interval_us = amt21_cycles_to_us(now_cycle - data->last_commit_cycle);

    if (interval_us > 0U) {
      int64_t delta = data->feedback.position - previous_position;

      data->feedback.sample_interval_us = interval_us;
      data->feedback.velocity =
        (int32_t)((delta * (int64_t)USEC_PER_SEC) / (int64_t)interval_us);
      data->feedback.valid_mask |= ENCODER_FEEDBACK_VELOCITY;
    }
  }

  data->last_commit_cycle = now_cycle;
  data->has_commit_cycle = true;

  if (have_turns) {
    data->feedback.valid_mask |= ENCODER_FEEDBACK_TURNS;
    data->feedback.turns = turns;
  } else if (!config->multiturn) {
    data->feedback.turns = 0;
  }
  data->feedback.single_turn = single_turn;
  data->feedback.online = true;
  data->feedback.stale = false;
  data->feedback.timestamp_ms = k_uptime_get();
  data->has_reading = true;
  data->consecutive_errors = 0U;

  amt21_stats_record_success(data);

  k_spin_unlock(&data->lock, key);
}

static void amt21_commit_failure(const struct device * bus_dev, const struct device * enc_dev)
{
  const struct amt21_bus_config * config = bus_dev->config;
  struct amt21_encoder_data * data = enc_dev->data;
  k_spinlock_key_t key;

  key = k_spin_lock(&data->lock);

  data->feedback.error_count++;
  data->consecutive_errors++;
  data->feedback.stale = true;

  /* A single dropped response must not disturb a control loop, so the last good
   * reading is kept until the failures pile up.
   */
  if (data->consecutive_errors >= config->offline_threshold) {
    data->feedback.online = false;
    /* Long enough offline that the encoder may have turned past the half
     * revolution the unwrap can resolve, so the accumulator is rebuilt on
     * recovery rather than continued.
     */
    data->has_accum = false;
  }

  k_spin_unlock(&data->lock, key);
}

/* -------------------------------------------------------------------------- */
/* Poll thread                                                                */
/* -------------------------------------------------------------------------- */

static void amt21_poll_encoder(const struct device * bus_dev, const struct device * enc_dev)
{
  const struct amt21_encoder_config * config = enc_dev->config;
  struct amt21_encoder_data * data = enc_dev->data;
  struct amt21_bus_data * bus = bus_dev->data;
  uint16_t msg = 0U;
  uint32_t single_turn = 0U;
  int32_t turns = 0;
  bool have_turns = false;

  if (data->blackout_until_ms > k_uptime_get()) {
    return;
  }

  k_mutex_lock(&bus->lock, K_FOREVER);

  int ret = amt21_transact_retry(
    bus_dev, enc_dev, (uint8_t)(config->node_addr | AMT21_CMD_POSITION), &msg);

  if (ret == 0) {
    single_turn = amt21_position_from_msg(msg, config->resolution);

    if (config->multiturn) {
      ret = amt21_transact_retry(
        bus_dev, enc_dev, (uint8_t)(config->node_addr | AMT21_CMD_TURNS), &msg);
      if (ret == 0) {
        turns = amt21_turns_from_msg(msg);
        have_turns = true;
      }
    }
  }

  k_mutex_unlock(&bus->lock);

  if (ret == 0) {
    amt21_commit_success(bus_dev, enc_dev, single_turn, have_turns, turns);
  } else {
    amt21_commit_failure(bus_dev, enc_dev);
  }
}

static void amt21_poll_thread(void * p1, void * p2, void * p3)
{
  const struct device * bus_dev = p1;
  const struct amt21_bus_config * config = bus_dev->config;
  struct amt21_bus_data * bus = bus_dev->data;

  ARG_UNUSED(p2);
  ARG_UNUSED(p3);

  while (true) {
    int64_t start = k_uptime_ticks();
    int64_t next = start + (int64_t)k_us_to_ticks_ceil64(config->poll_interval_us);

    /* The encoder list is filled in by the child devices as they initialise, so
     * a scan that starts early simply sees fewer encoders and picks up the rest
     * on the next pass.
     */
    for (uint8_t i = 0; i < bus->encoder_count; ++i) {
      const struct device * enc_dev = bus->encoders[i];

      if (enc_dev != NULL) {
        amt21_poll_encoder(bus_dev, enc_dev);
      }
    }

    AMT21_BUS_STAT_INC(bus, scans);

    if (k_uptime_ticks() >= next) {
      /* Overran the interval. Skip rather than queue, so the bus never builds
       * up a backlog of scans it can never catch up on.
       */
      AMT21_BUS_STAT_INC(bus, scans_skipped);
      k_yield();
      continue;
    }

    (void)k_sleep(K_TIMEOUT_ABS_TICKS(next));
  }
}

/* -------------------------------------------------------------------------- */
/* Encoder API                                                                */
/* -------------------------------------------------------------------------- */

static int amt21_get_feedback(const struct device * dev, void * feedback)
{
  struct amt21_encoder_data * data = dev->data;
  struct encoder_feedback * out = feedback;
  k_spinlock_key_t key;
  int ret = 0;

  if (out == NULL) {
    return -EINVAL;
  }

  key = k_spin_lock(&data->lock);
  *out = data->feedback;

  if (!data->has_reading) {
    ret = -ENODATA;
  } else if (!data->feedback.online) {
    ret = -EIO;
  } else if (data->feedback.stale) {
    ret = -EAGAIN;
  }

  k_spin_unlock(&data->lock, key);

  return ret;
}

static int amt21_get_resolution(const struct device * dev, uint8_t * resolution)
{
  const struct amt21_encoder_config * config = dev->config;

  if (resolution == NULL) {
    return -EINVAL;
  }

  *resolution = config->resolution;

  return 0;
}

/**
 * @brief Send a two byte extended command.
 *
 * The encoder does not answer these and resets itself immediately, so the
 * encoder is parked for its startup time instead of being polled.
 */
static int amt21_extended_command(const struct device * dev, uint8_t ext)
{
  const struct amt21_encoder_config * config = dev->config;
  struct amt21_encoder_data * data = dev->data;
  const struct device * bus_dev = config->bus;
  struct amt21_bus_data * bus = bus_dev->data;
  uint8_t bytes[2] = {(uint8_t)(config->node_addr | AMT21_CMD_EXTENDED), ext};
  int ret;

  k_mutex_lock(&bus->lock, K_FOREVER);

  amt21_gap_wait(bus_dev);
  amt21_wait_for_quiet(bus_dev);

  ret = amt21_send(bus_dev, bytes, sizeof(bytes));

  bus->last_tx_end_cycle = k_cycle_get_32();
  bus->last_tx_end_valid = true;

  k_mutex_unlock(&bus->lock);

  if (ret < 0) {
    return ret;
  }

  k_spinlock_key_t key = k_spin_lock(&data->lock);

  data->blackout_until_ms = k_uptime_get() + AMT21_RESET_BLACKOUT_MS;
  data->feedback.online = false;
  data->feedback.stale = true;
  data->has_reading = false;
  data->consecutive_errors = 0U;
  /* The device restarts, and a stored zero point moves what it reports, so
   * neither the accumulator nor the offset carries over.
   */
  data->has_accum = false;

  k_spin_unlock(&data->lock, key);

  return 0;
}

static int amt21_set_zero(const struct device * dev)
{
  const struct amt21_encoder_config * config = dev->config;

  if (config->multiturn) {
    /* Only single-turn devices can store a zero point. */
    return -ENOTSUP;
  }

  return amt21_extended_command(dev, AMT21_EXT_SET_ZERO);
}

static int amt21_set_position(const struct device * dev, int64_t position)
{
  struct amt21_encoder_data * data = dev->data;
  k_spinlock_key_t key;
  int ret = 0;

  key = k_spin_lock(&data->lock);

  if (!data->has_accum) {
    ret = -ENODATA;
  } else {
    data->bias = position - data->raw_count;
    data->feedback.position = position;
  }

  k_spin_unlock(&data->lock, key);

  return ret;
}

static int amt21_reset(const struct device * dev)
{
  return amt21_extended_command(dev, AMT21_EXT_RESET);
}

static const struct encoder_driver_api amt21_encoder_api = {
  .get_feedback = amt21_get_feedback,
  .get_resolution = amt21_get_resolution,
  .set_zero = amt21_set_zero,
  .set_position = amt21_set_position,
  .reset = amt21_reset,
};

/* -------------------------------------------------------------------------- */
/* Driver specific diagnostics                                                */
/* -------------------------------------------------------------------------- */

static bool amt21_is_encoder(const struct device * dev)
{
  return (dev != NULL) && (dev->api == &amt21_encoder_api);
}

static bool amt21_is_bus(const struct device * dev)
{
  return (dev != NULL) && (dev->api == &amt21_bus_api);
}

int amt21_get_stats(const struct device * dev, struct amt21_stats * stats)
{
  if (!amt21_is_encoder(dev) || (stats == NULL)) {
    return -EINVAL;
  }

#if AMT21_STATS_ENABLED
  struct amt21_encoder_data * data = dev->data;
  k_spinlock_key_t key = k_spin_lock(&data->lock);

  *stats = data->stats;

  k_spin_unlock(&data->lock, key);

  return 0;
#else
  return -ENOTSUP;
#endif
}

int amt21_clear_stats(const struct device * dev)
{
  if (!amt21_is_encoder(dev)) {
    return -EINVAL;
  }

#if AMT21_STATS_ENABLED
  struct amt21_encoder_data * data = dev->data;
  k_spinlock_key_t key = k_spin_lock(&data->lock);

  memset(&data->stats, 0, sizeof(data->stats));
#if AMT21_ERROR_LOG_ENABLED
  data->log_head = 0U;
  data->log_count = 0U;
#endif

  k_spin_unlock(&data->lock, key);

  return 0;
#else
  return -ENOTSUP;
#endif
}

int amt21_get_error_log(const struct device * dev, struct amt21_error_record * buf, size_t count)
{
  if (!amt21_is_encoder(dev) || (buf == NULL)) {
    return -EINVAL;
  }

#if AMT21_ERROR_LOG_ENABLED
  struct amt21_encoder_data * data = dev->data;
  k_spinlock_key_t key = k_spin_lock(&data->lock);
  size_t n = MIN(count, (size_t)data->log_count);

  /* Newest first. log_head points one past the newest entry. */
  for (size_t i = 0; i < n; ++i) {
    size_t idx = (size_t)((data->log_head + AMT21_ERROR_LOG_SIZE - 1U - i) % AMT21_ERROR_LOG_SIZE);

    buf[i] = data->log[idx];
  }

  k_spin_unlock(&data->lock, key);

  return (int)n;
#else
  ARG_UNUSED(count);
  return -ENOTSUP;
#endif
}

int amt21_bus_get_stats(const struct device * bus_dev, struct amt21_bus_stats * stats)
{
  if (!amt21_is_bus(bus_dev) || (stats == NULL)) {
    return -EINVAL;
  }

#if AMT21_STATS_ENABLED
  struct amt21_bus_data * bus = bus_dev->data;

  k_mutex_lock(&bus->lock, K_FOREVER);
  *stats = bus->stats;
  k_mutex_unlock(&bus->lock);

  return 0;
#else
  return -ENOTSUP;
#endif
}

int amt21_bus_clear_stats(const struct device * bus_dev)
{
  if (!amt21_is_bus(bus_dev)) {
    return -EINVAL;
  }

#if AMT21_STATS_ENABLED
  struct amt21_bus_data * bus = bus_dev->data;

  k_mutex_lock(&bus->lock, K_FOREVER);
  memset(&bus->stats, 0, sizeof(bus->stats));
  k_mutex_unlock(&bus->lock);

  return 0;
#else
  return -ENOTSUP;
#endif
}

/* -------------------------------------------------------------------------- */
/* Initialisation                                                             */
/* -------------------------------------------------------------------------- */

static int amt21_bus_register_encoder(
  const struct device * bus_dev, const struct device * enc_dev)
{
  struct amt21_bus_data * bus = bus_dev->data;
  int ret = 0;

  k_mutex_lock(&bus->lock, K_FOREVER);

  if (bus->encoder_count >= ARRAY_SIZE(bus->encoders)) {
    ret = -ENOMEM;
  } else {
    bus->encoders[bus->encoder_count++] = enc_dev;
  }

  k_mutex_unlock(&bus->lock);

  if (ret < 0) {
    LOG_ERR(
      "%s: cannot register %s, raise CONFIG_ENCODER_AMT21_MAX_ENCODERS above %u", bus_dev->name,
      enc_dev->name, (unsigned int)ARRAY_SIZE(bus->encoders));
  }

  return ret;
}

static int amt21_bus_init(const struct device * dev)
{
  const struct amt21_bus_config * config = dev->config;
  struct amt21_bus_data * bus = dev->data;
  int ret;

  bus->dev = dev;
  bus->echo = config->tx_echo_forced ? AMT21_ECHO_PRESENT : AMT21_ECHO_UNKNOWN;

  k_mutex_init(&bus->lock);
  k_sem_init(&bus->tx_sem, 0, 1);
  k_sem_init(&bus->rx_sem, 0, K_SEM_MAX_LIMIT);
  ring_buf_init(&bus->rx_rb, sizeof(bus->rx_rb_buf), bus->rx_rb_buf);

  if (!device_is_ready(config->uart)) {
    LOG_ERR("%s: UART not ready", dev->name);
    return -ENODEV;
  }

  if (config->has_de) {
    if (!gpio_is_ready_dt(&config->de)) {
      LOG_ERR("%s: driver enable GPIO not ready", dev->name);
      return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&config->de, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
      LOG_ERR("%s: failed to configure driver enable GPIO (%d)", dev->name, ret);
      return ret;
    }
  }

  ret = uart_callback_set(config->uart, amt21_uart_callback, (void *)dev);
  if (ret < 0) {
    LOG_ERR("%s: failed to install UART callback (%d)", dev->name, ret);
    return ret;
  }

  ret = amt21_rx_start(dev, amt21_expected_chunk(bus));
  if (ret < 0) {
    LOG_ERR("%s: failed to enable reception (%d)", dev->name, ret);
    return ret;
  }

  return 0;
}

static int amt21_encoder_init(const struct device * dev)
{
  const struct amt21_encoder_config * config = dev->config;

  if (!device_is_ready(config->bus)) {
    LOG_ERR("%s: bus not ready", dev->name);
    return -ENODEV;
  }

  return amt21_bus_register_encoder(config->bus, dev);
}

#define DT_DRV_COMPAT cui_amt21

#define AMT21_BUS_DE_INIT(inst)                                                   \
  COND_CODE_1(                                                                    \
    DT_INST_NODE_HAS_PROP(inst, de_gpios),                                        \
    (.de = GPIO_DT_SPEC_INST_GET(inst, de_gpios), .has_de = true), (.has_de = false))

#define AMT21_BUS_DEFINE(inst)                                                              \
  BUILD_ASSERT(                                                                             \
    DT_INST_PROP(inst, poll_interval_us) > 0, "poll-interval-us must be positive");         \
  BUILD_ASSERT(                                                                             \
    DT_INST_PROP(inst, response_timeout_us) > 0, "response-timeout-us must be positive");   \
  BUILD_ASSERT(                                                                             \
    DT_INST_CHILD_NUM_STATUS_OKAY(inst) > 0, "an AMT21 bus needs at least one encoder");    \
  BUILD_ASSERT(                                                                             \
    DT_INST_CHILD_NUM_STATUS_OKAY(inst) <= CONFIG_ENCODER_AMT21_MAX_ENCODERS,               \
    "more encoders than CONFIG_ENCODER_AMT21_MAX_ENCODERS allows");                         \
  static K_THREAD_STACK_DEFINE(                                                             \
    amt21_stack_##inst, CONFIG_ENCODER_AMT21_THREAD_STACK_SIZE);                            \
  static struct amt21_bus_data amt21_bus_data_##inst;                                       \
  static const struct amt21_bus_config amt21_bus_config_##inst = {                           \
    .uart = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                            \
    AMT21_BUS_DE_INIT(inst),                                                                \
    .tx_echo_forced = DT_INST_PROP(inst, tx_echo),                                          \
    .poll_interval_us = DT_INST_PROP(inst, poll_interval_us),                               \
    .response_timeout_us = DT_INST_PROP(inst, response_timeout_us),                          \
    .inter_command_delay_us = DT_INST_PROP(inst, inter_command_delay_us),                   \
    .rx_quiet_period_us = DT_INST_PROP(inst, rx_quiet_period_us),                           \
    .max_retries = DT_INST_PROP(inst, max_retries),                                         \
    .offline_threshold = DT_INST_PROP(inst, offline_threshold),                             \
  };                                                                                        \
  static int amt21_bus_init_##inst(const struct device * dev)                                \
  {                                                                                          \
    int ret = amt21_bus_init(dev);                                                           \
                                                                                             \
    if (ret < 0) {                                                                           \
      return ret;                                                                            \
    }                                                                                        \
                                                                                             \
    k_thread_create(                                                                         \
      &amt21_bus_data_##inst.thread, amt21_stack_##inst,                                     \
      K_THREAD_STACK_SIZEOF(amt21_stack_##inst), amt21_poll_thread, (void *)dev, NULL, NULL, \
      CONFIG_ENCODER_AMT21_THREAD_PRIORITY, 0, K_NO_WAIT);                                   \
    k_thread_name_set(&amt21_bus_data_##inst.thread, "amt21_poll");                           \
                                                                                             \
    return 0;                                                                                \
  }                                                                                          \
  DEVICE_DT_INST_DEFINE(                                                                     \
    inst, amt21_bus_init_##inst, NULL, &amt21_bus_data_##inst, &amt21_bus_config_##inst,      \
    POST_KERNEL, CONFIG_ENCODER_INIT_PRIORITY, &amt21_bus_api);

DT_INST_FOREACH_STATUS_OKAY(AMT21_BUS_DEFINE)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT cui_amt21_encoder

#define AMT21_ENCODER_DEFINE(inst)                                                        \
  BUILD_ASSERT(                                                                            \
    (DT_INST_REG_ADDR(inst) & 0x3U) == 0U,                                                \
    "AMT21 node address must be a multiple of four, the low two bits encode the command"); \
  BUILD_ASSERT(DT_INST_REG_ADDR(inst) <= 0xFCU, "AMT21 node address must fit in one byte"); \
  static struct amt21_encoder_data amt21_encoder_data_##inst;                              \
  static const struct amt21_encoder_config amt21_encoder_config_##inst = {                 \
    .bus = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                            \
    .node_addr = DT_INST_REG_ADDR(inst),                                                   \
    .resolution = DT_INST_PROP(inst, resolution),                                          \
    .multiturn = DT_INST_PROP(inst, multiturn),                                            \
  };                                                                                       \
  DEVICE_DT_INST_DEFINE(                                                                   \
    inst, amt21_encoder_init, NULL, &amt21_encoder_data_##inst,                            \
    &amt21_encoder_config_##inst, POST_KERNEL, CONFIG_ENCODER_INIT_PRIORITY,               \
    &amt21_encoder_api);

DT_INST_FOREACH_STATUS_OKAY(AMT21_ENCODER_DEFINE)
