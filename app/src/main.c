/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <app/drivers/blink.h>
#include <app_version.h>
#include <drivers/motor.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#define BLINK_PERIOD_MS_STEP 100U
#define BLINK_PERIOD_MS_MAX 1000U

int main(void)
{
#if DT_NODE_HAS_STATUS(DT_NODELABEL(robomaster_controller), okay)
  const struct device * robomaster = DEVICE_DT_GET(DT_NODELABEL(robomaster_controller));

  if (!device_is_ready(robomaster)) {
    LOG_ERR("RoboMaster transport is not ready");
    return -ENODEV;
  }
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(fdcan1), okay)
  if (!device_is_ready(DEVICE_DT_GET(DT_NODELABEL(fdcan1)))) {
    LOG_ERR("Control CAN is not ready");
    return -ENODEV;
  }
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(fdcan2), okay)
  if (!device_is_ready(DEVICE_DT_GET(DT_NODELABEL(fdcan2)))) {
    LOG_ERR("Motor CAN0 is not ready");
    return -ENODEV;
  }
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(fdcan3), okay)
  if (!device_is_ready(DEVICE_DT_GET(DT_NODELABEL(fdcan3)))) {
    LOG_ERR("Motor CAN1 is not ready");
    return -ENODEV;
  }
#endif

  LOG_INF("Baseline bring-up complete");

  return 0;
}
