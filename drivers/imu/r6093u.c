/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * CruizCore R6093U, a gyro that streams its attitude, angular velocity and
 * acceleration over a UART at about 100 Hz without being asked.
 *
 * The packet layout, the units and the reset command are those of PCMainFW's
 * lib/R6093U, the driver this one replaces. doc/drivers/r6093u.md has the
 * layout and what is known only from that driver.
 */

#define DT_DRV_COMPAT cruizcore_r6093u

#include <errno.h>
#include <string.h>

#include <drivers/imu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(imu_r6093u, CONFIG_IMU_LOG_LEVEL);

#define R6093U_PACKET_LEN 26U
#define R6093U_HEADER 0xA6U
/* The checksum is the low byte of the sum of bytes 2..24, stored in byte 25. */
#define R6093U_CHECKSUM_FIRST 2U
#define R6093U_CHECKSUM_INDEX 25U

#define R6093U_ROLL 3U
#define R6093U_PITCH 5U
#define R6093U_YAW 7U
#define R6093U_GYRO_X 9U
#define R6093U_ACCEL_X 15U

/* Angles and rates are in 0.01 deg and 0.01 deg/s, acceleration in mg. */
#define R6093U_CENTIDEG_TO_RAD (0.01F * 3.14159265358979F / 180.0F)
#define R6093U_MG_TO_MPS2 (9.80665F / 1000.0F)
#define R6093U_CENTIDEG_HALF_TURN 18000
#define R6093U_CENTIDEG_TURN 36000

/* PCMainFW sends the string with its terminating NUL, and that is the form
 * known to work; without the datasheet there is no telling whether the
 * device needs it. */
static const char r6093u_reset_command[] = "$MIB,RESET*87";

struct r6093u_config
{
  const struct device * uart;
  uint32_t stale_timeout_ms;
  uint32_t offline_timeout_ms;
};

struct r6093u_data
{
  const struct device * dev;
  struct k_spinlock lock;

  uint8_t packet[R6093U_PACKET_LEN];
  uint8_t packet_len;

  struct imu_feedback feedback;
  bool yaw_following;
  int16_t yaw_prev;
  int64_t yaw_centideg;

  struct k_work_delayable watchdog;
};

static bool r6093u_prefix_ok(const uint8_t * packet, uint8_t len)
{
  return ((len < 1U) || (packet[0] == R6093U_HEADER)) &&
         ((len < 2U) || (packet[1] == R6093U_HEADER));
}

/* Drops the first byte and then everything up to the next place a packet
 * could start, so a corrupted packet costs the bytes before the next header
 * pair rather than a whole packet length. */
static void r6093u_resync(struct r6093u_data * data)
{
  do {
    data->packet_len--;
    memmove(data->packet, data->packet + 1, data->packet_len);
  } while ((data->packet_len > 0U) && !r6093u_prefix_ok(data->packet, data->packet_len));
}

static bool r6093u_checksum_ok(const uint8_t * packet)
{
  uint8_t sum = 0U;

  for (size_t i = R6093U_CHECKSUM_FIRST; i < R6093U_CHECKSUM_INDEX; i++) {
    sum += packet[i];
  }

  return sum == packet[R6093U_CHECKSUM_INDEX];
}

static float r6093u_field(const uint8_t * packet, size_t offset, float scale)
{
  return (float)(int16_t)sys_get_le16(&packet[offset]) * scale;
}

static void r6093u_follow_yaw(struct r6093u_data * data, int16_t raw)
{
  if (!data->yaw_following) {
    data->yaw_centideg = raw;
    data->yaw_following = true;
    data->feedback.yaw_epoch++;
  } else {
    /* Samples arrive every 10 ms and the device saturates at 250 deg/s, so
     * the shorter way round between two samples is always the true one. */
    int32_t diff = (int32_t)raw - data->yaw_prev;

    if (diff > R6093U_CENTIDEG_HALF_TURN) {
      diff -= R6093U_CENTIDEG_TURN;
    } else if (diff < -R6093U_CENTIDEG_HALF_TURN) {
      diff += R6093U_CENTIDEG_TURN;
    }

    data->yaw_centideg += diff;
  }

  data->yaw_prev = raw;
}

static void r6093u_decode(struct r6093u_data * data)
{
  const uint8_t * packet = data->packet;
  struct imu_feedback * fb = &data->feedback;

  for (size_t axis = 0; axis < 3; axis++) {
    fb->gyro[axis] = r6093u_field(packet, R6093U_GYRO_X + 2U * axis, R6093U_CENTIDEG_TO_RAD);
    fb->accel[axis] = r6093u_field(packet, R6093U_ACCEL_X + 2U * axis, R6093U_MG_TO_MPS2);
  }

  fb->roll = r6093u_field(packet, R6093U_ROLL, R6093U_CENTIDEG_TO_RAD);
  fb->pitch = r6093u_field(packet, R6093U_PITCH, R6093U_CENTIDEG_TO_RAD);

  r6093u_follow_yaw(data, (int16_t)sys_get_le16(&packet[R6093U_YAW]));
  fb->yaw = (float)data->yaw_centideg * R6093U_CENTIDEG_TO_RAD;

  fb->valid_mask = IMU_FEEDBACK_ACCEL | IMU_FEEDBACK_GYRO | IMU_FEEDBACK_ATTITUDE;
  fb->timestamp_ms = k_uptime_get();
}

/* Called with the lock held. */
static void r6093u_feed(struct r6093u_data * data, uint8_t byte)
{
  data->packet[data->packet_len++] = byte;

  if (!r6093u_prefix_ok(data->packet, data->packet_len)) {
    r6093u_resync(data);
    return;
  }

  if (data->packet_len < R6093U_PACKET_LEN) {
    return;
  }

  if (!r6093u_checksum_ok(data->packet)) {
    data->feedback.error_count++;
    r6093u_resync(data);
    return;
  }

  r6093u_decode(data);
  data->packet_len = 0U;
}

static void r6093u_uart_cb(const struct device * uart, void * user_data)
{
  const struct device * dev = user_data;
  struct r6093u_data * data = dev->data;
  uint8_t chunk[16];
  k_spinlock_key_t key;

  if (uart_irq_update(uart) < 0) {
    return;
  }

  key = k_spin_lock(&data->lock);

  while (uart_irq_rx_ready(uart) > 0) {
    const int len = uart_fifo_read(uart, chunk, sizeof(chunk));

    if (len <= 0) {
      break;
    }

    for (int i = 0; i < len; i++) {
      r6093u_feed(data, chunk[i]);
    }
  }

  /* Cleared on every pass, not only when reported: an overrun left standing
   * is what stopped the receiver of an earlier driver for good. */
  if (uart_err_check(uart) > 0) {
    data->feedback.error_count++;
    /* The bytes around an error belong to a damaged packet. */
    data->packet_len = 0U;
  }

  k_spin_unlock(&data->lock, key);
}

static void r6093u_send_reset(const struct device * dev)
{
  const struct r6093u_config * config = dev->config;

  for (size_t i = 0; i < sizeof(r6093u_reset_command); i++) {
    uart_poll_out(config->uart, (unsigned char)r6093u_reset_command[i]);
  }
}

/*
 * Re-arms the receiver whenever nothing has arrived for offline-timeout-ms.
 * A device that is merely unplugged comes back without it, but a receiver
 * wedged by an error the callback never saw does not.
 */
static void r6093u_watchdog(struct k_work * work)
{
  struct k_work_delayable * dwork = k_work_delayable_from_work(work);
  struct r6093u_data * data = CONTAINER_OF(dwork, struct r6093u_data, watchdog);
  const struct device * dev = data->dev;
  const struct r6093u_config * config = dev->config;
  const int64_t now = k_uptime_get();
  k_spinlock_key_t key;
  bool silent;

  key = k_spin_lock(&data->lock);
  silent = (data->feedback.valid_mask == 0U) ||
           ((now - data->feedback.timestamp_ms) > (int64_t)config->offline_timeout_ms);
  if (silent) {
    data->packet_len = 0U;
  }
  k_spin_unlock(&data->lock, key);

  if (silent) {
    LOG_DBG("%s: silent, re-arming the receiver", dev->name);
    uart_irq_rx_disable(config->uart);
    (void)uart_err_check(config->uart);
    uart_irq_rx_enable(config->uart);
  }

  k_work_schedule(dwork, K_MSEC(config->offline_timeout_ms));
}

static int r6093u_get_feedback(const struct device * dev, struct imu_feedback * feedback)
{
  const struct r6093u_config * config = dev->config;
  struct r6093u_data * data = dev->data;
  k_spinlock_key_t key;
  int64_t age;
  int ret;

  if (feedback == NULL) {
    return -EINVAL;
  }

  key = k_spin_lock(&data->lock);

  if (data->feedback.valid_mask == 0U) {
    k_spin_unlock(&data->lock, key);
    return -ENODATA;
  }

  *feedback = data->feedback;

  k_spin_unlock(&data->lock, key);

  /* Judged at read time so that a device going silent shows on the next read
   * rather than on the next watchdog pass. */
  age = k_uptime_get() - feedback->timestamp_ms;
  feedback->online = age <= (int64_t)config->offline_timeout_ms;
  feedback->stale = age > (int64_t)config->stale_timeout_ms;

  if (!feedback->online) {
    ret = -EIO;
  } else if (feedback->stale) {
    ret = -EAGAIN;
  } else {
    ret = 0;
  }

  return ret;
}

static int r6093u_reset(const struct device * dev)
{
  struct r6093u_data * data = dev->data;
  k_spinlock_key_t key;

  r6093u_send_reset(dev);

  /* The device restarts its yaw from zero, so following it on from the old
   * value would offset every later reading. */
  key = k_spin_lock(&data->lock);
  data->yaw_following = false;
  data->packet_len = 0U;
  k_spin_unlock(&data->lock, key);

  return 0;
}

static const struct imu_driver_api r6093u_api = {
  .get_feedback = r6093u_get_feedback,
  .reset = r6093u_reset,
};

static int r6093u_init(const struct device * dev)
{
  const struct r6093u_config * config = dev->config;
  struct r6093u_data * data = dev->data;
  int ret;

  if (!device_is_ready(config->uart)) {
    LOG_ERR("%s: UART %s not ready", dev->name, config->uart->name);
    return -ENODEV;
  }

  data->dev = dev;
  k_work_init_delayable(&data->watchdog, r6093u_watchdog);

  ret = uart_irq_callback_user_data_set(config->uart, r6093u_uart_cb, (void *)dev);
  if (ret < 0) {
    LOG_ERR("%s: no interrupt-driven API on %s (%d)", dev->name, config->uart->name, ret);
    return ret;
  }

  /* Zeroes the yaw, so a fresh image starts facing 0 as PCMainFW did. The
   * accumulator starts from the first sample whatever the device does. */
  r6093u_send_reset(dev);

  uart_irq_rx_enable(config->uart);
  k_work_schedule(&data->watchdog, K_MSEC(config->offline_timeout_ms));

  return 0;
}

#define R6093U_INIT(inst)                                                                   \
  BUILD_ASSERT(                                                                             \
    DT_INST_PROP(inst, stale_timeout_ms) <= DT_INST_PROP(inst, offline_timeout_ms),        \
    "stale-timeout-ms must not exceed offline-timeout-ms");                                 \
                                                                                            \
  static const struct r6093u_config r6093u_config_##inst = {                                \
    .uart = DEVICE_DT_GET(DT_INST_BUS(inst)),                                               \
    .stale_timeout_ms = DT_INST_PROP(inst, stale_timeout_ms),                               \
    .offline_timeout_ms = DT_INST_PROP(inst, offline_timeout_ms),                           \
  };                                                                                        \
                                                                                            \
  static struct r6093u_data r6093u_data_##inst;                                             \
                                                                                            \
  DEVICE_DT_INST_DEFINE(                                                                    \
    inst, r6093u_init, NULL, &r6093u_data_##inst, &r6093u_config_##inst, POST_KERNEL,       \
    CONFIG_IMU_INIT_PRIORITY, &r6093u_api);

DT_INST_FOREACH_STATUS_OKAY(R6093U_INIT)
