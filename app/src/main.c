/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>

#include <app/drivers/blink.h>
#include <app_version.h>
#include <drivers/encoder.h>
#include <drivers/encoder/amt21.h>
#include <drivers/motor.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#define BLINK_PERIOD_MS_STEP 100U
#define BLINK_PERIOD_MS_MAX 1000U

#define ENCODER_REPORT_PERIOD K_MSEC(1000)

#if DT_NODE_EXISTS(DT_NODELABEL(encoder_left))

static void report_encoder(const struct device * dev)
{
  struct encoder_feedback fb;
  int ret = encoder_get_feedback(dev, &fb);

  /* A stale reading still carries the last good value, so print it along with
   * the reason it is not fresh rather than discarding it.
   */
  LOG_INF(
    "%s: ret=%d pos=%u turns=%d angle=%d.%03d deg online=%d stale=%d errors=%u", dev->name, ret,
    fb.position, fb.turns, fb.angle_mdeg / 1000, abs(fb.angle_mdeg % 1000), (int)fb.online,
    (int)fb.stale, fb.error_count);

#if defined(CONFIG_ENCODER_AMT21_STATS)
  struct amt21_stats stats;

  if (amt21_get_stats(dev, &stats) == 0) {
    LOG_INF(
      "  transactions=%u ok=%u timeout=%u checksum=%u desync=%u echo=%u bus=%u retries=%u",
      stats.transactions, stats.successes, stats.errors[AMT21_ERROR_TIMEOUT],
      stats.errors[AMT21_ERROR_CHECKSUM], stats.errors[AMT21_ERROR_DESYNC],
      stats.errors[AMT21_ERROR_ECHO], stats.errors[AMT21_ERROR_BUS], stats.retries);
  }
#endif
}

static void report_bus(void)
{
#if defined(CONFIG_ENCODER_AMT21_STATS)
  const struct device * bus = DEVICE_DT_GET(DT_NODELABEL(amt21_bus));
  struct amt21_bus_stats stats;

  if (amt21_bus_get_stats(bus, &stats) == 0) {
    LOG_INF(
      "%s: scans=%u skipped=%u rx_restarts=%u framing=%u overrun=%u stray=%u", bus->name,
      stats.scans, stats.scans_skipped, stats.rx_restarts, stats.framing_errors,
      stats.overrun_errors, stats.stray_bytes);
  }
#endif
}

#endif /* DT_NODE_EXISTS(DT_NODELABEL(encoder_left)) */

int main(void)
{
  const struct device * robomaster = DEVICE_DT_GET(DT_NODELABEL(robomaster_controller));
  ARG_UNUSED(robomaster);

#if DT_NODE_EXISTS(DT_NODELABEL(encoder_left))
  const struct device * encoder_left = DEVICE_DT_GET(DT_NODELABEL(encoder_left));
  const struct device * encoder_right = DEVICE_DT_GET(DT_NODELABEL(encoder_right));

  if (!device_is_ready(encoder_left)) {
    LOG_ERR("%s is not ready", encoder_left->name);
    return -ENODEV;
  }

  while (true) {
    report_encoder(encoder_left);
    report_encoder(encoder_right);
    report_bus();

    k_sleep(ENCODER_REPORT_PERIOD);
  }
#endif

  return 0;
}
