/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <array>
#include <cerrno>

#include <app_version.h>
#include <drivers/motor.h>
#include <fibril/motor_control.hpp>
#include <fibril/robomaster_protocol.hpp>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#if defined(CONFIG_USB_DEVICE_GS_USB)
#include <zephyr/usb/bos.h>
#include <zephyr/usb/usb_device.h>

#include <cannectivity/usb/class/gs_usb.h>
#endif

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

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

namespace
{
constexpr std::array<const struct device *, fibril::MotorControlService::MotorCount> motor_devices = {
  DEVICE_DT_GET(DT_NODELABEL(motor_1)),
  DEVICE_DT_GET(DT_NODELABEL(motor_2)),
  DEVICE_DT_GET(DT_NODELABEL(motor_3)),
  DEVICE_DT_GET(DT_NODELABEL(motor_4)),
  DEVICE_DT_GET(DT_NODELABEL(motor_5)),
  DEVICE_DT_GET(DT_NODELABEL(motor_6)),
  DEVICE_DT_GET(DT_NODELABEL(motor_7)),
  DEVICE_DT_GET(DT_NODELABEL(motor_8)),
};

constexpr std::array<fibril::MotorControlConfig, fibril::MotorControlService::MotorCount>
  motor_configs = {{
    {DEVICE_DT_GET(DT_NODELABEL(motor_1)), DT_PROP(DT_NODELABEL(motor_1), max_current)},
    {DEVICE_DT_GET(DT_NODELABEL(motor_2)), DT_PROP(DT_NODELABEL(motor_2), max_current)},
    {DEVICE_DT_GET(DT_NODELABEL(motor_3)), DT_PROP(DT_NODELABEL(motor_3), max_current)},
    {DEVICE_DT_GET(DT_NODELABEL(motor_4)), DT_PROP(DT_NODELABEL(motor_4), max_current)},
    {DEVICE_DT_GET(DT_NODELABEL(motor_5)), DT_PROP(DT_NODELABEL(motor_5), max_current)},
    {DEVICE_DT_GET(DT_NODELABEL(motor_6)), DT_PROP(DT_NODELABEL(motor_6), max_current)},
    {DEVICE_DT_GET(DT_NODELABEL(motor_7)), DT_PROP(DT_NODELABEL(motor_7), max_current)},
    {DEVICE_DT_GET(DT_NODELABEL(motor_8)), DT_PROP(DT_NODELABEL(motor_8), max_current)},
  }};
}  // namespace

extern "C" int main(void)
{
  int ret;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(robomaster_controller), okay)
  const struct device * robomaster = DEVICE_DT_GET(DT_NODELABEL(robomaster_controller));

  if (!device_is_ready(robomaster)) {
    LOG_ERR("RoboMaster transport is not ready");
    return -ENODEV;
  }
#endif

  for (const struct device * motor : motor_devices) {
    if (!device_is_ready(motor)) {
      LOG_ERR("A motor device is not ready");
      return -ENODEV;
    }
  }

#if DT_NODE_HAS_STATUS(DT_NODELABEL(fdcan1), okay)
  const struct device * control_can = DEVICE_DT_GET(DT_NODELABEL(fdcan1));
  if (!device_is_ready(control_can)) {
    LOG_ERR("Control CAN is not ready");
    return -ENODEV;
  }
#else
  LOG_ERR("Control CAN is missing");
  return -ENODEV;
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

  static fibril::MotorControlService motor_control;
  ret = motor_control.init(motor_configs);
  if (ret != 0) {
    LOG_ERR("Failed to initialize motor control: %d", ret);
    return ret;
  }

  ret = motor_control.start();
  if (ret != 0) {
    LOG_ERR("Failed to start motor control: %d", ret);
    return ret;
  }

  static fibril::RoboMasterProtocolService protocol(control_can, motor_control, motor_devices);
  ret = protocol.start();
  if (ret != 0) {
    LOG_ERR("Failed to start RoboMaster protocol: %d", ret);
    return ret;
  }

  LOG_INF("RoboMaster Mini V1 control stack ready");
  return 0;
}
