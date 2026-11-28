/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * ams AS5047P 14-bit magnetic angle sensor on SPI, seen as an encoder.
 *
 * The angle is absolute within a turn and wraps once per revolution, so the
 * driver has to observe it at least twice per turn for the accumulated position
 * to stay continuous. A thread per device polls it at a fixed interval for that
 * reason; an SPI transfer blocks, so the k_timer expiry the quadrature driver
 * samples from cannot do it.
 */

#define DT_DRV_COMPAT ams_as5047p

#include <errno.h>

#include <drivers/encoder.h>
#include <drivers/encoder/as5047p.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "encoder_accum.h"

LOG_MODULE_REGISTER(encoder_as5047p, CONFIG_ENCODER_LOG_LEVEL);

#define AS5047P_REG_NOP      0x0000U
#define AS5047P_REG_ERRFL    0x0001U
#define AS5047P_REG_DIAAGC   0x3FFCU
#define AS5047P_REG_ANGLECOM 0x3FFFU
#define AS5047P_REG_ZPOSM    0x0016U
#define AS5047P_REG_ZPOSL    0x0017U

#define AS5047P_FRAME_PARITY BIT(15)
/* Bit 14 is the read flag in a command and the error flag in a response. */
#define AS5047P_FRAME_RW     BIT(14)
#define AS5047P_FRAME_EF     BIT(14)
#define AS5047P_DATA_MASK    GENMASK(13, 0)

#define AS5047P_ANGLE_MASK   GENMASK(AS5047P_ANGLE_BITS - 1U, 0)

/* ZPOSM holds the upper 8 bits of the zero position, ZPOSL the lower 6. The
 * upper bits of ZPOSL enable the error flags for the magnetic field and must
 * survive a change of the zero position. */
#define AS5047P_ZPOSM_MASK   GENMASK(7, 0)
#define AS5047P_ZPOSL_MASK   GENMASK(5, 0)
#define AS5047P_ZPOSM_SHIFT  6U

#define AS5047P_DIAAGC_AGC   GENMASK(7, 0)
#define AS5047P_DIAAGC_LF    BIT(8)
#define AS5047P_DIAAGC_COF   BIT(9)
#define AS5047P_DIAAGC_MAGH  BIT(10)
#define AS5047P_DIAAGC_MAGL  BIT(11)

struct as5047p_config
{
  struct spi_dt_spec spi;
  k_thread_stack_t * stack;
  size_t stack_size;
  uint32_t poll_interval_us;
  uint8_t offline_threshold;
  bool invert;
};

struct as5047p_data
{
  /* Serialises transfers between the polling thread and the API. */
  struct k_mutex bus_lock;
  /* Guards everything below. Readers of the snapshot never wait on a transfer. */
  struct k_spinlock lock;
  struct encoder_feedback feedback;
  struct encoder_accum accum;
  struct as5047p_stats stats;
  uint8_t consecutive_errors;

  struct k_thread thread;
};

static const struct encoder_driver_api as5047p_api;

static bool as5047p_is_device(const struct device * dev)
{
  return (dev != NULL) && (dev->api == &as5047p_api);
}

static uint16_t as5047p_with_parity(uint16_t frame)
{
  frame &= (uint16_t)~AS5047P_FRAME_PARITY;

  if ((__builtin_popcount(frame) & 1) != 0) {
    frame |= AS5047P_FRAME_PARITY;
  }

  return frame;
}

static bool as5047p_parity_ok(uint16_t frame)
{
  return (__builtin_popcount(frame) & 1) == 0;
}

/**
 * @brief Exchange one 16-bit frame.
 *
 * Each frame is its own transaction because the device latches a command on the
 * rising edge of chip select, so two frames sent under one assertion would be
 * read as one malformed frame.
 */
static int as5047p_transfer(const struct device * dev, uint16_t tx_frame, uint16_t * rx_frame)
{
  const struct as5047p_config * config = dev->config;
  uint8_t tx_buf[2] = {(uint8_t)(tx_frame >> 8), (uint8_t)tx_frame};
  uint8_t rx_buf[2] = {0};
  const struct spi_buf tx = {.buf = tx_buf, .len = sizeof(tx_buf)};
  const struct spi_buf rx = {.buf = rx_buf, .len = sizeof(rx_buf)};
  const struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};
  const struct spi_buf_set rx_set = {.buffers = &rx, .count = 1};
  int ret;

  ret = spi_transceive_dt(&config->spi, &tx_set, &rx_set);
  if (ret < 0) {
    return ret;
  }

  *rx_frame = (uint16_t)((rx_buf[0] << 8) | rx_buf[1]);

  return 0;
}

/**
 * @brief Read one register, classifying a failure.
 *
 * The response to a command arrives in the frame that follows it, so a read
 * takes two frames and the second one carries a NOP.
 *
 * @param cause Set to the reason when the read fails; untouched on success.
 */
static int as5047p_read_reg(
  const struct device * dev, uint16_t reg, uint16_t * value, enum as5047p_error_cause * cause)
{
  uint16_t response;
  int ret;

  ret = as5047p_transfer(dev, as5047p_with_parity(AS5047P_FRAME_RW | reg), &response);
  if (ret == 0) {
    ret = as5047p_transfer(dev, as5047p_with_parity(AS5047P_FRAME_RW | AS5047P_REG_NOP), &response);
  }

  if (ret < 0) {
    *cause = AS5047P_ERROR_SPI;
    return ret;
  }

  if (!as5047p_parity_ok(response)) {
    *cause = AS5047P_ERROR_PARITY;
    return -EIO;
  }

  if ((response & AS5047P_FRAME_EF) != 0U) {
    *cause = AS5047P_ERROR_FLAG;
    return -EIO;
  }

  *value = response & AS5047P_DATA_MASK;

  return 0;
}

static int as5047p_write_reg(const struct device * dev, uint16_t reg, uint16_t value)
{
  enum as5047p_error_cause cause;
  uint16_t response;
  uint16_t readback;
  int ret;

  ret = as5047p_transfer(dev, as5047p_with_parity(reg), &response);
  if (ret == 0) {
    ret = as5047p_transfer(dev, as5047p_with_parity(value & AS5047P_DATA_MASK), &response);
  }

  if (ret < 0) {
    return ret;
  }

  /* The device answers a write with the register's new content, so reading the
   * register back both collects that answer and confirms the write landed. */
  ret = as5047p_read_reg(dev, reg, &readback, &cause);
  if (ret < 0) {
    return ret;
  }

  return (readback == (value & AS5047P_DATA_MASK)) ? 0 : -EIO;
}

static void as5047p_commit_success(const struct device * dev, uint16_t angle, uint32_t cycle)
{
  const struct as5047p_config * config = dev->config;
  struct as5047p_data * data = dev->data;
  struct encoder_accum_sample sample;
  k_spinlock_key_t key = k_spin_lock(&data->lock);

  data->consecutive_errors = 0U;

  if (!encoder_accum_is_valid(&data->accum)) {
    const int64_t start = config->invert ? -(int64_t)angle : (int64_t)angle;

    encoder_accum_reset(&data->accum, start, angle);
    data->feedback.position_epoch++;
  }

  encoder_accum_update(&data->accum, angle, cycle, &sample);

  data->feedback.valid_mask = ENCODER_FEEDBACK_POSITION | ENCODER_FEEDBACK_SINGLE_TURN;
  data->feedback.position = sample.position;
  data->feedback.single_turn = angle;

  if (sample.has_velocity) {
    data->feedback.valid_mask |= ENCODER_FEEDBACK_VELOCITY;
    data->feedback.velocity = sample.velocity;
    data->feedback.sample_interval_us = sample.sample_interval_us;
  }

  data->feedback.online = true;
  data->feedback.stale = false;
  data->feedback.timestamp_ms = k_uptime_get();

  k_spin_unlock(&data->lock, key);
}

static void as5047p_commit_failure(
  const struct device * dev, enum as5047p_error_cause cause, uint16_t errfl)
{
  const struct as5047p_config * config = dev->config;
  struct as5047p_data * data = dev->data;
  k_spinlock_key_t key = k_spin_lock(&data->lock);

  data->feedback.error_count++;
  data->stats.errors[cause]++;

  if (cause == AS5047P_ERROR_FLAG) {
    data->stats.last_errfl = errfl;
  }

  data->feedback.stale = true;

  if (data->consecutive_errors < UINT8_MAX) {
    data->consecutive_errors++;
  }

  if (data->consecutive_errors >= config->offline_threshold) {
    /* The shaft may turn by more than half a revolution while nothing is
     * read, after which the next difference cannot be resolved. */
    data->feedback.online = false;
    encoder_accum_invalidate(&data->accum);
  }

  k_spin_unlock(&data->lock, key);
}

static void as5047p_poll_once(const struct device * dev)
{
  struct as5047p_data * data = dev->data;
  enum as5047p_error_cause cause = AS5047P_ERROR_SPI;
  uint16_t angle = 0U;
  uint16_t errfl = 0U;
  uint32_t cycle;
  int ret;

  k_mutex_lock(&data->bus_lock, K_FOREVER);

  ret = as5047p_read_reg(dev, AS5047P_REG_ANGLECOM, &angle, &cause);
  cycle = k_cycle_get_32();

  if ((ret < 0) && (cause == AS5047P_ERROR_FLAG)) {
    enum as5047p_error_cause errfl_cause;

    /* Reading ERRFL clears it, which the next read relies on: the flag stays
     * set on every response until then. */
    if (as5047p_read_reg(dev, AS5047P_REG_ERRFL, &errfl, &errfl_cause) < 0) {
      errfl = 0U;
    }
  }

  k_mutex_unlock(&data->bus_lock);

  if (ret == 0) {
    as5047p_commit_success(dev, angle & AS5047P_ANGLE_MASK, cycle);
  } else {
    as5047p_commit_failure(dev, cause, errfl);
  }
}

static void as5047p_poll_thread(void * p1, void * p2, void * p3)
{
  const struct device * dev = p1;
  const struct as5047p_config * config = dev->config;
  const int64_t period = (int64_t)k_us_to_ticks_ceil64(config->poll_interval_us);
  int64_t next = k_uptime_ticks();

  ARG_UNUSED(p2);
  ARG_UNUSED(p3);

  while (true) {
    as5047p_poll_once(dev);

    next += period;

    /* After an overrun, skip the boundaries already passed instead of polling
     * back to back to catch up. */
    int64_t now = k_uptime_ticks();

    if (next <= now) {
      next = now + period - ((now - next) % period);
    }

    k_sleep(K_TIMEOUT_ABS_TICKS(next));
  }
}

static int as5047p_get_feedback(const struct device * dev, void * feedback)
{
  struct as5047p_data * data = dev->data;
  struct encoder_feedback * out = feedback;
  k_spinlock_key_t key;
  int ret = 0;

  if (out == NULL) {
    return -EINVAL;
  }

  key = k_spin_lock(&data->lock);

  *out = data->feedback;

  if (data->feedback.valid_mask == 0U) {
    ret = -ENODATA;
  } else if (!data->feedback.online) {
    ret = -EIO;
  } else if (data->feedback.stale) {
    ret = -EAGAIN;
  }

  k_spin_unlock(&data->lock, key);

  return ret;
}

static int as5047p_get_resolution(const struct device * dev, uint8_t * resolution)
{
  ARG_UNUSED(dev);

  if (resolution == NULL) {
    return -EINVAL;
  }

  *resolution = AS5047P_ANGLE_BITS;

  return 0;
}

static int as5047p_read_zero_position_locked(const struct device * dev, uint16_t * zero_position)
{
  enum as5047p_error_cause cause;
  uint16_t zposm;
  uint16_t zposl;
  int ret;

  ret = as5047p_read_reg(dev, AS5047P_REG_ZPOSM, &zposm, &cause);
  if (ret == 0) {
    ret = as5047p_read_reg(dev, AS5047P_REG_ZPOSL, &zposl, &cause);
  }

  if (ret < 0) {
    return ret;
  }

  *zero_position =
    (uint16_t)(((zposm & AS5047P_ZPOSM_MASK) << AS5047P_ZPOSM_SHIFT) | (zposl & AS5047P_ZPOSL_MASK));

  return 0;
}

static int as5047p_write_zero_position_locked(const struct device * dev, uint16_t zero_position)
{
  struct as5047p_data * data = dev->data;
  enum as5047p_error_cause cause;
  uint16_t zposl;
  k_spinlock_key_t key;
  int ret;

  ret = as5047p_read_reg(dev, AS5047P_REG_ZPOSL, &zposl, &cause);
  if (ret < 0) {
    return ret;
  }

  ret = as5047p_write_reg(
    dev, AS5047P_REG_ZPOSM, (uint16_t)((zero_position >> AS5047P_ZPOSM_SHIFT) & AS5047P_ZPOSM_MASK));
  if (ret == 0) {
    ret = as5047p_write_reg(
      dev, AS5047P_REG_ZPOSL,
      (uint16_t)((zposl & ~AS5047P_ZPOSL_MASK) | (zero_position & AS5047P_ZPOSL_MASK)));
  }

  /* Invalidate even after a partial write: the reported angle may have moved
   * either way, and the jump must not be read as motion. */
  key = k_spin_lock(&data->lock);
  encoder_accum_invalidate(&data->accum);
  k_spin_unlock(&data->lock, key);

  return ret;
}

static int as5047p_set_zero(const struct device * dev)
{
  struct as5047p_data * data = dev->data;
  enum as5047p_error_cause cause;
  uint16_t zero_position;
  uint16_t angle;
  int ret;

  k_mutex_lock(&data->bus_lock, K_FOREVER);

  ret = as5047p_read_zero_position_locked(dev, &zero_position);
  if (ret == 0) {
    ret = as5047p_read_reg(dev, AS5047P_REG_ANGLECOM, &angle, &cause);
  }

  if (ret == 0) {
    /* ANGLECOM is already measured from the current zero position, so the new
     * one is the sum of the two. */
    ret = as5047p_write_zero_position_locked(
      dev, (uint16_t)((zero_position + angle) & AS5047P_ANGLE_MASK));
  }

  k_mutex_unlock(&data->bus_lock);

  return ret;
}

static int as5047p_set_position(const struct device * dev, int64_t position)
{
  struct as5047p_data * data = dev->data;
  k_spinlock_key_t key;
  int ret;

  key = k_spin_lock(&data->lock);

  ret = encoder_accum_set_position(&data->accum, position);

  if (ret == 0) {
    data->feedback.position = position;
  }

  k_spin_unlock(&data->lock, key);

  return ret;
}

static int as5047p_reset(const struct device * dev)
{
  struct as5047p_data * data = dev->data;
  k_spinlock_key_t key;

  /* The device has no reset command. Starting over from the next reading is
   * what a reset means here: the position returns to the absolute angle and
   * the epoch advances. */
  key = k_spin_lock(&data->lock);

  encoder_accum_invalidate(&data->accum);
  data->feedback.valid_mask = 0U;

  k_spin_unlock(&data->lock, key);

  return 0;
}

static const struct encoder_driver_api as5047p_api = {
  .get_feedback = as5047p_get_feedback,
  .get_resolution = as5047p_get_resolution,
  .set_zero = as5047p_set_zero,
  .set_position = as5047p_set_position,
  .reset = as5047p_reset,
};

int as5047p_get_stats(const struct device * dev, struct as5047p_stats * stats)
{
  struct as5047p_data * data;
  k_spinlock_key_t key;

  if (!as5047p_is_device(dev) || (stats == NULL)) {
    return -EINVAL;
  }

  data = dev->data;
  key = k_spin_lock(&data->lock);
  *stats = data->stats;
  k_spin_unlock(&data->lock, key);

  return 0;
}

int as5047p_read_diag(const struct device * dev, struct as5047p_diag * diag)
{
  struct as5047p_data * data;
  enum as5047p_error_cause cause;
  uint16_t diaagc;
  int ret;

  if (!as5047p_is_device(dev) || (diag == NULL)) {
    return -EINVAL;
  }

  data = dev->data;

  k_mutex_lock(&data->bus_lock, K_FOREVER);
  ret = as5047p_read_reg(dev, AS5047P_REG_DIAAGC, &diaagc, &cause);
  k_mutex_unlock(&data->bus_lock);

  if (ret < 0) {
    return ret;
  }

  diag->agc = (uint8_t)(diaagc & AS5047P_DIAAGC_AGC);
  diag->lf = (diaagc & AS5047P_DIAAGC_LF) != 0U;
  diag->cof = (diaagc & AS5047P_DIAAGC_COF) != 0U;
  diag->magh = (diaagc & AS5047P_DIAAGC_MAGH) != 0U;
  diag->magl = (diaagc & AS5047P_DIAAGC_MAGL) != 0U;

  return 0;
}

int as5047p_get_zero_position(const struct device * dev, uint16_t * zero_position)
{
  struct as5047p_data * data;
  int ret;

  if (!as5047p_is_device(dev) || (zero_position == NULL)) {
    return -EINVAL;
  }

  data = dev->data;

  k_mutex_lock(&data->bus_lock, K_FOREVER);
  ret = as5047p_read_zero_position_locked(dev, zero_position);
  k_mutex_unlock(&data->bus_lock);

  return ret;
}

int as5047p_set_zero_position(const struct device * dev, uint16_t zero_position)
{
  struct as5047p_data * data;
  int ret;

  if (!as5047p_is_device(dev) || (zero_position > AS5047P_ANGLE_MASK)) {
    return -EINVAL;
  }

  data = dev->data;

  k_mutex_lock(&data->bus_lock, K_FOREVER);
  ret = as5047p_write_zero_position_locked(dev, zero_position);
  k_mutex_unlock(&data->bus_lock);

  return ret;
}

static int as5047p_init(const struct device * dev)
{
  const struct as5047p_config * config = dev->config;
  struct as5047p_data * data = dev->data;

  if (!spi_is_ready_dt(&config->spi)) {
    LOG_ERR("%s: SPI bus %s not ready", dev->name, config->spi.bus->name);
    return -ENODEV;
  }

  k_mutex_init(&data->bus_lock);
  encoder_accum_init(&data->accum, AS5047P_ANGLE_BITS, config->invert);

  k_thread_create(
    &data->thread, config->stack, config->stack_size, as5047p_poll_thread, (void *)dev, NULL, NULL,
    CONFIG_ENCODER_AS5047P_THREAD_PRIORITY, 0, K_NO_WAIT);
  k_thread_name_set(&data->thread, dev->name);

  return 0;
}

/* The device samples MOSI on the falling edge of SCLK and idles low (mode 1),
 * and sends the most significant bit first. Frames are 16 bits; they are
 * carried as two bytes so that any controller can drive them. */
#define AS5047P_SPI_OPERATION \
  (SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_TRANSFER_MSB | SPI_WORD_SET(8U))

#define AS5047P_INIT(inst)                                                                   \
  BUILD_ASSERT(                                                                              \
    DT_INST_PROP(inst, poll_interval_us) > 0, "poll-interval-us must be positive");          \
  BUILD_ASSERT(                                                                              \
    DT_INST_PROP(inst, offline_threshold) > 0 && DT_INST_PROP(inst, offline_threshold) <= 255, \
    "offline-threshold must be 1 to 255");                                                   \
                                                                                             \
  static K_THREAD_STACK_DEFINE(as5047p_stack_##inst, CONFIG_ENCODER_AS5047P_THREAD_STACK_SIZE); \
                                                                                             \
  static const struct as5047p_config as5047p_config_##inst = {                               \
    .spi = SPI_DT_SPEC_INST_GET(inst, AS5047P_SPI_OPERATION),                                \
    .stack = as5047p_stack_##inst,                                                           \
    .stack_size = K_THREAD_STACK_SIZEOF(as5047p_stack_##inst),                               \
    .poll_interval_us = DT_INST_PROP(inst, poll_interval_us),                                \
    .offline_threshold = DT_INST_PROP(inst, offline_threshold),                              \
    .invert = DT_INST_PROP(inst, invert_direction),                                          \
  };                                                                                         \
                                                                                             \
  static struct as5047p_data as5047p_data_##inst;                                            \
                                                                                             \
  DEVICE_DT_INST_DEFINE(                                                                     \
    inst, as5047p_init, NULL, &as5047p_data_##inst, &as5047p_config_##inst, POST_KERNEL,     \
    CONFIG_ENCODER_INIT_PRIORITY, &as5047p_api);

DT_INST_FOREACH_STATUS_OKAY(AS5047P_INIT)
