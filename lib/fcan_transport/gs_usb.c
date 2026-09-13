/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Present the transport's external port to a USB host as a CANnectivity
 * gs_usb channel, so a PC on SocketCAN sees the node's bus through a single
 * canX interface.
 *
 * Full speed only, no DFU and no MSOSV2 descriptor set — a board that needs
 * WinUSB auto-binding or firmware update over USB should run CANnectivity's
 * own application instead of this.
 */

#include "gs_usb.h"

#include <zephyr/logging/log.h>
#include <zephyr/usb/usbd.h>

#include <cannectivity/usb/class/gs_usb.h>

LOG_MODULE_DECLARE(fcan_transport, CONFIG_FCAN_TRANSPORT_LOG_LEVEL);

/* gs_usb.c's USBD_DEFINE_CLASS naming: the single "gs_usb" devicetree node
 * lands at index 0.
 */
#define GS_USB_CLASS_INSTANCE_NAME "gs_usb_0"

/* Copied from CANnectivity's defaults rather than taken from its Kconfig,
 * which only exists when its application is on the build. Matching them keeps
 * host-side udev rules and gs_usb pid filters that already work with a
 * CANnectivity dongle working here too. 0x1209 is pid.codes; 0xca01 is the
 * block CANnectivity reserved for gs_usb devices.
 */
#define FCAN_GS_USB_VID          0x1209U
#define FCAN_GS_USB_PID          0xca01U
#define FCAN_GS_USB_MANUFACTURER "CANnectivity"
#define FCAN_GS_USB_PRODUCT      "CANnectivity USB to CAN adapter"
#define FCAN_GS_USB_MAX_POWER    125U /* 250 mA; bMaxPower counts 2 mA units */

USBD_DEVICE_DEFINE(
  usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)), FCAN_GS_USB_VID, FCAN_GS_USB_PID);

USBD_DESC_LANG_DEFINE(lang);
USBD_DESC_MANUFACTURER_DEFINE(mfr, FCAN_GS_USB_MANUFACTURER);
USBD_DESC_PRODUCT_DEFINE(product, FCAN_GS_USB_PRODUCT);
USBD_DESC_SERIAL_NUMBER_DEFINE(sn);
USBD_DESC_CONFIG_DEFINE(fs_config_desc, "Full-Speed Configuration");

USBD_CONFIGURATION_DEFINE(
  fs_config, USB_SCD_SELF_POWERED, FCAN_GS_USB_MAX_POWER, &fs_config_desc);

static int usbd_setup(void)
{
  int err = usbd_add_descriptor(&usbd, &lang);

  if (err != 0) {
    LOG_ERR("usbd_add_descriptor(lang) failed (%d)", err);
    return err;
  }

  err = usbd_add_descriptor(&usbd, &mfr);
  if (err != 0) {
    LOG_ERR("usbd_add_descriptor(mfr) failed (%d)", err);
    return err;
  }

  err = usbd_add_descriptor(&usbd, &product);
  if (err != 0) {
    LOG_ERR("usbd_add_descriptor(product) failed (%d)", err);
    return err;
  }

  err = usbd_add_descriptor(&usbd, &sn);
  if (err != 0) {
    LOG_ERR("usbd_add_descriptor(sn) failed (%d)", err);
    return err;
  }

  err = usbd_add_configuration(&usbd, USBD_SPEED_FS, &fs_config);
  if (err != 0) {
    LOG_ERR("usbd_add_configuration(fs) failed (%d)", err);
    return err;
  }

  err = usbd_register_class(&usbd, GS_USB_CLASS_INSTANCE_NAME, USBD_SPEED_FS, 1);
  if (err != 0) {
    LOG_ERR("usbd_register_class(%s) failed (%d)", GS_USB_CLASS_INSTANCE_NAME, err);
    return err;
  }

  err = usbd_device_set_code_triple(&usbd, USBD_SPEED_FS, 0, 0, 0);
  if (err != 0) {
    LOG_ERR("usbd_device_set_code_triple(fs) failed (%d)", err);
    return err;
  }

  err = usbd_device_set_bcd_usb(&usbd, USBD_SPEED_FS, USB_SRN_2_0_1);
  if (err != 0) {
    LOG_ERR("usbd_device_set_bcd_usb(fs) failed (%d)", err);
    return err;
  }

  return usbd_init(&usbd);
}

int fcan_transport_gs_usb_start(const struct device * channel)
{
  const struct device * gs_usb = DEVICE_DT_GET_ONE(gs_usb);

  if (!device_is_ready(gs_usb)) {
    LOG_ERR("gs_usb device %s not ready", gs_usb->name);
    return -ENODEV;
  }

  int err = usbd_setup();

  if (err != 0) {
    return err;
  }

  /* One channel: the transport's external port. Binding the peers instead
   * would show the host several interfaces that the hub has already joined
   * into one logical bus.
   */
  const struct device * channels[] = {channel};

  err = gs_usb_register(gs_usb, channels, ARRAY_SIZE(channels), NULL, NULL);
  if (err != 0) {
    LOG_ERR("gs_usb_register failed (%d)", err);
    return err;
  }

  /* Enabling is the last step: from here the host can enumerate, open the
   * channel and start the external port.
   */
  err = usbd_enable(&usbd);
  if (err != 0) {
    LOG_ERR("usbd_enable failed (%d)", err);
    return err;
  }

  LOG_INF("gs_usb up on %s", channel->name);

  return 0;
}
