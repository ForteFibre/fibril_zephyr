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

#if defined(CONFIG_USB_DEVICE_GS_USB)
#include <zephyr/usb/bos.h>
#include <zephyr/usb/usb_device.h>

#include <cannectivity/usb/class/gs_usb.h>
#endif

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#define BLINK_PERIOD_MS_STEP 100U
#define BLINK_PERIOD_MS_MAX 1000U

#if defined(CONFIG_USB_DEVICE_GS_USB)
USB_DEVICE_BOS_DESC_DEFINE_CAP const struct usb_bos_capability_lpm bos_cap_lpm = {
  .bLength = sizeof(struct usb_bos_capability_lpm),
  .bDescriptorType = USB_DESC_DEVICE_CAPABILITY,
  .bDevCapabilityType = USB_BOS_CAPABILITY_EXTENSION,
  .bmAttributes = 0UL,
};

static int app_usb_init(void)
{
  usb_bos_register_cap((void *)&bos_cap_lpm);

  return usb_enable(NULL);
}
#endif

int main(void)
{
  int ret;

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

#if defined(CONFIG_USB_DEVICE_GS_USB) && DT_HAS_CHOSEN(zephyr_canbus) && \
  DT_NODE_HAS_STATUS(DT_NODELABEL(gs_usb0), okay)
  const struct device * gs_usb = DEVICE_DT_GET(DT_NODELABEL(gs_usb0));
  const struct device * channels[] = {
    DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus)),
  };

  if (!device_is_ready(gs_usb)) {
    LOG_ERR("gs_usb device is not ready");
    return -ENODEV;
  }

  ret = gs_usb_register(gs_usb, channels, ARRAY_SIZE(channels), NULL, NULL);
  if (ret != 0) {
    LOG_ERR("Failed to register gs_usb: %d", ret);
    return ret;
  }

  ret = app_usb_init();
  if (ret != 0) {
    LOG_ERR("Failed to initialize USB: %d", ret);
    return ret;
  }
#endif

  LOG_INF("Baseline bring-up complete");

  return 0;
}
