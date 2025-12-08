/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <app/drivers/blink.h>
#include <app_version.h>
#include <drivers/motor.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#define BLINK_PERIOD_MS_STEP 100U
#define BLINK_PERIOD_MS_MAX 1000U

int main(void)
{
  const struct device * robomaster = DEVICE_DT_GET(DT_NODELABEL(robomaster_controller));

  return 0;
}
