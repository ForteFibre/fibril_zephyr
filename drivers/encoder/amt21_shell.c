/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shell commands for inspecting AMT21x encoders on a running target.
 *
 * The devices are enumerated from the devicetree rather than looked up by name
 * through the generic device registry, which keeps the commands from being
 * pointed at something that is not an AMT21 encoder.
 */

#include <stdlib.h>
#include <string.h>

#include <drivers/encoder.h>
#include <drivers/encoder/amt21.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#define AMT21_DEV_ELEM(node) DEVICE_DT_GET(node),

static const struct device * const amt21_encoders[] = {
  DT_FOREACH_STATUS_OKAY(cui_amt21_encoder, AMT21_DEV_ELEM)};

static const struct device * const amt21_buses[] = {
  DT_FOREACH_STATUS_OKAY(cui_amt21, AMT21_DEV_ELEM)};

static const char * const amt21_cause_names[AMT21_ERROR_CAUSE_COUNT] = {
  [AMT21_ERROR_TIMEOUT] = "timeout",
  [AMT21_ERROR_CHECKSUM] = "checksum",
  [AMT21_ERROR_DESYNC] = "desync",
  [AMT21_ERROR_ECHO] = "echo",
  [AMT21_ERROR_BUS] = "bus",
};

/* Guard against a new enum value being added without a matching label: without
 * this the missing slot would be NULL and later shell_fprintf() calls that
 * pass it as %s would either print garbage or crash.
 */
BUILD_ASSERT(
  (AMT21_ERROR_BUS + 1) == AMT21_ERROR_CAUSE_COUNT,
  "amt21_cause_names does not cover every AMT21_ERROR_CAUSE_COUNT entry");

static const struct device * amt21_find(
  const struct shell * sh, const struct device * const * list, size_t count, const char * name)
{
  for (size_t i = 0; i < count; ++i) {
    if (strcmp(list[i]->name, name) == 0) {
      return list[i];
    }
  }

  shell_error(sh, "%s is not an AMT21 device on this target", name);

  return NULL;
}

static void amt21_dev_name_get(size_t idx, struct shell_static_entry * entry)
{
  entry->syntax = (idx < ARRAY_SIZE(amt21_encoders)) ? amt21_encoders[idx]->name : NULL;
  entry->handler = NULL;
  entry->help = NULL;
  entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(amt21_dev_names, amt21_dev_name_get);

static void amt21_bus_name_get(size_t idx, struct shell_static_entry * entry)
{
  entry->syntax = (idx < ARRAY_SIZE(amt21_buses)) ? amt21_buses[idx]->name : NULL;
  entry->handler = NULL;
  entry->help = NULL;
  entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(amt21_bus_names, amt21_bus_name_get);

static int cmd_list(const struct shell * sh, size_t argc, char ** argv)
{
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);

  for (size_t i = 0; i < ARRAY_SIZE(amt21_buses); ++i) {
    shell_print(sh, "bus %s", amt21_buses[i]->name);
  }

  for (size_t i = 0; i < ARRAY_SIZE(amt21_encoders); ++i) {
    const struct device * dev = amt21_encoders[i];
    uint8_t resolution = 0U;

    (void)encoder_get_resolution(dev, &resolution);
    shell_print(
      sh, "encoder %s: %u-bit, ready=%d", dev->name, resolution, (int)device_is_ready(dev));
  }

  return 0;
}

static int cmd_read(const struct shell * sh, size_t argc, char ** argv)
{
  ARG_UNUSED(argc);

  const struct device * dev =
    amt21_find(sh, amt21_encoders, ARRAY_SIZE(amt21_encoders), argv[1]);

  if (dev == NULL) {
    return -EINVAL;
  }

  struct encoder_feedback fb;
  int ret = encoder_get_feedback(dev, &fb);

  const char * state;

  switch (ret) {
    case 0:
      state = "fresh";
      break;
    case -EAGAIN:
      state = "stale";
      break;
    case -EIO:
      state = "offline";
      break;
    case -ENODATA:
      state = "no reading yet";
      break;
    default:
      state = "error";
      break;
  }

  shell_print(sh, "%s: %s (%d)", dev->name, state, ret);
  shell_print(sh, "  position     %u", fb.position);
  if ((fb.valid_mask & ENCODER_FEEDBACK_TURNS) != 0U) {
    shell_print(sh, "  turns        %d", fb.turns);
  }
  shell_print(
    sh, "  angle        %d.%03d deg", fb.angle_mdeg / 1000, abs(fb.angle_mdeg % 1000));
  shell_print(sh, "  online       %d", (int)fb.online);
  shell_print(sh, "  stale        %d", (int)fb.stale);
  shell_print(sh, "  errors       %u", fb.error_count);
  shell_print(sh, "  updated at   %lld ms", fb.timestamp_ms);

  return 0;
}

static int cmd_stats(const struct shell * sh, size_t argc, char ** argv)
{
  ARG_UNUSED(argc);

  const struct device * dev =
    amt21_find(sh, amt21_encoders, ARRAY_SIZE(amt21_encoders), argv[1]);

  if (dev == NULL) {
    return -EINVAL;
  }

  struct amt21_stats stats;
  int ret = amt21_get_stats(dev, &stats);

  if (ret == -ENOTSUP) {
    shell_warn(sh, "statistics are not built in, enable CONFIG_ENCODER_AMT21_STATS");
    return 0;
  }
  if (ret < 0) {
    shell_error(sh, "failed to read statistics (%d)", ret);
    return ret;
  }

  shell_print(sh, "%s:", dev->name);
  shell_print(sh, "  transactions %u", stats.transactions);
  shell_print(sh, "  successes    %u", stats.successes);
  shell_print(sh, "  retries      %u", stats.retries);

  for (size_t i = 0; i < AMT21_ERROR_CAUSE_COUNT; ++i) {
    shell_print(sh, "  %-12s %u", amt21_cause_names[i], stats.errors[i]);
  }

  shell_print(sh, "  consecutive  %u (max %u)", stats.consecutive_errors,
              stats.max_consecutive_errors);

  if (stats.transactions > 0U) {
    /* Integer tenths of a percent, which is enough to spot a marginal bus. */
    uint32_t failed = stats.transactions - stats.successes;
    uint32_t rate = (uint32_t)(((uint64_t)failed * 1000U) / stats.transactions);

    shell_print(sh, "  error rate   %u.%u%%", rate / 10U, rate % 10U);
  }

  if (stats.last_error_timestamp_ms > 0) {
    shell_print(
      sh, "  last error   %s at %lld ms", amt21_cause_names[stats.last_error],
      stats.last_error_timestamp_ms);
  }

  return 0;
}

static int cmd_stats_clear(const struct shell * sh, size_t argc, char ** argv)
{
  ARG_UNUSED(argc);

  const struct device * dev =
    amt21_find(sh, amt21_encoders, ARRAY_SIZE(amt21_encoders), argv[1]);

  if (dev == NULL) {
    return -EINVAL;
  }

  int ret = amt21_clear_stats(dev);

  if (ret == -ENOTSUP) {
    shell_warn(sh, "statistics are not built in, enable CONFIG_ENCODER_AMT21_STATS");
    return 0;
  }

  return ret;
}

static int cmd_errlog(const struct shell * sh, size_t argc, char ** argv)
{
  ARG_UNUSED(argc);

  const struct device * dev =
    amt21_find(sh, amt21_encoders, ARRAY_SIZE(amt21_encoders), argv[1]);

  if (dev == NULL) {
    return -EINVAL;
  }

  struct amt21_error_record records[8];
  int n = amt21_get_error_log(dev, records, ARRAY_SIZE(records));

  if (n == -ENOTSUP) {
    shell_warn(sh, "the error log is not built in, set CONFIG_ENCODER_AMT21_ERROR_LOG_SIZE");
    return 0;
  }
  if (n < 0) {
    shell_error(sh, "failed to read the error log (%d)", n);
    return n;
  }
  if (n == 0) {
    shell_print(sh, "%s: no failures recorded", dev->name);
    return 0;
  }

  shell_print(sh, "%s: %d most recent failures, newest first", dev->name, n);

  for (int i = 0; i < n; ++i) {
    const struct amt21_error_record * rec = &records[i];
    char bytes[3 * sizeof(rec->rx_bytes) + 1];
    size_t pos = 0;

    for (uint8_t b = 0; b < rec->rx_len; ++b) {
      pos += (size_t)snprintk(&bytes[pos], sizeof(bytes) - pos, "%02x ", rec->rx_bytes[b]);
    }
    bytes[pos] = '\0';

    shell_print(
      sh, "  %lld ms addr=0x%02x cmd=0x%02x %-9s rx=[%s]", rec->timestamp_ms, rec->node_addr,
      rec->command, amt21_cause_names[rec->cause], bytes);
  }

  return 0;
}

static int cmd_bus_stats(const struct shell * sh, size_t argc, char ** argv)
{
  ARG_UNUSED(argc);

  const struct device * dev = amt21_find(sh, amt21_buses, ARRAY_SIZE(amt21_buses), argv[1]);

  if (dev == NULL) {
    return -EINVAL;
  }

  struct amt21_bus_stats stats;
  int ret = amt21_bus_get_stats(dev, &stats);

  if (ret == -ENOTSUP) {
    shell_warn(sh, "statistics are not built in, enable CONFIG_ENCODER_AMT21_STATS");
    return 0;
  }
  if (ret < 0) {
    shell_error(sh, "failed to read bus statistics (%d)", ret);
    return ret;
  }

  shell_print(sh, "%s:", dev->name);
  shell_print(sh, "  scans        %u", stats.scans);
  shell_print(sh, "  skipped      %u", stats.scans_skipped);
  shell_print(sh, "  rx restarts  %u", stats.rx_restarts);
  shell_print(sh, "  framing      %u", stats.framing_errors);
  shell_print(sh, "  overrun      %u", stats.overrun_errors);
  shell_print(sh, "  stray bytes  %u", stats.stray_bytes);

  return 0;
}

static int cmd_bus_stats_clear(const struct shell * sh, size_t argc, char ** argv)
{
  ARG_UNUSED(argc);

  const struct device * dev = amt21_find(sh, amt21_buses, ARRAY_SIZE(amt21_buses), argv[1]);

  if (dev == NULL) {
    return -EINVAL;
  }

  int ret = amt21_bus_clear_stats(dev);

  if (ret == -ENOTSUP) {
    shell_warn(sh, "statistics are not built in, enable CONFIG_ENCODER_AMT21_STATS");
    return 0;
  }

  return ret;
}

static int cmd_zero(const struct shell * sh, size_t argc, char ** argv)
{
  ARG_UNUSED(argc);

  const struct device * dev =
    amt21_find(sh, amt21_encoders, ARRAY_SIZE(amt21_encoders), argv[1]);

  if (dev == NULL) {
    return -EINVAL;
  }

  int ret = encoder_set_zero(dev);

  if (ret == -ENOTSUP) {
    shell_error(sh, "%s is a multi-turn device and cannot store a zero point", dev->name);
    return 0;
  }
  if (ret < 0) {
    shell_error(sh, "failed to set the zero point (%d)", ret);
    return ret;
  }

  shell_warn(sh, "%s resets itself; readings resume in about 200 ms", dev->name);

  return 0;
}

static int cmd_reset(const struct shell * sh, size_t argc, char ** argv)
{
  ARG_UNUSED(argc);

  const struct device * dev =
    amt21_find(sh, amt21_encoders, ARRAY_SIZE(amt21_encoders), argv[1]);

  if (dev == NULL) {
    return -EINVAL;
  }

  int ret = encoder_reset(dev);

  if (ret < 0) {
    shell_error(sh, "failed to reset (%d)", ret);
    return ret;
  }

  shell_warn(sh, "%s is restarting; readings resume in about 200 ms", dev->name);

  return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
  amt21_cmds, SHELL_CMD(list, NULL, "List the AMT21 buses and encoders", cmd_list),
  SHELL_CMD_ARG(read, &amt21_dev_names, "<dev>: read the latest snapshot", cmd_read, 2, 0),
  SHELL_CMD_ARG(stats, &amt21_dev_names, "<dev>: per-encoder statistics", cmd_stats, 2, 0),
  SHELL_CMD_ARG(
    stats-clear, &amt21_dev_names, "<dev>: reset the per-encoder statistics", cmd_stats_clear, 2,
    0),
  SHELL_CMD_ARG(
    errlog, &amt21_dev_names, "<dev>: recent failures with the raw bytes received", cmd_errlog, 2,
    0),
  SHELL_CMD_ARG(bus-stats, &amt21_bus_names, "<bus>: per-bus statistics", cmd_bus_stats, 2, 0),
  SHELL_CMD_ARG(
    bus-stats-clear, &amt21_bus_names, "<bus>: reset the per-bus statistics",
    cmd_bus_stats_clear, 2, 0),
  SHELL_CMD_ARG(
    zero, &amt21_dev_names, "<dev>: store the current position as zero", cmd_zero, 2, 0),
  SHELL_CMD_ARG(reset, &amt21_dev_names, "<dev>: reset the encoder", cmd_reset, 2, 0),
  SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(amt21, &amt21_cmds, "AMT21x absolute encoder diagnostics", NULL);
