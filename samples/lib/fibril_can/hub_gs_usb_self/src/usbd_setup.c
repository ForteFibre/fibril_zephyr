/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal usbd_next boilerplate for the hub+self+gs_usb sample.
 *
 * Compared to CANnectivity's app/src/usb.c this drops:
 *   - HS configuration      (Mini V1 is USB FS only)
 *   - DFU descriptors + switch-to-DFU flow
 *   - BOS + MSOSV2 platform capability (Windows WinUSB auto-bind)
 *
 * The registered class instance name matches gs_usb.c's USBD_DEFINE_CLASS
 * naming: the single "gs_usb" DT node lands at index 0 -> "gs_usb_0".
 */

#include "usbd_setup.h"

#include <zephyr/logging/log.h>
#include <zephyr/usb/usbd.h>

LOG_MODULE_REGISTER(usbd_setup, LOG_LEVEL_INF);

#define GS_USB_CLASS_INSTANCE_NAME "gs_usb_0"

/* Values match the CANnectivity firmware defaults so host-side udev rules and
 * gs_usb pid filters that already work with CANnectivity dongles work here
 * too. 0x1209 is pid.codes, 0xca01 is the block CANnectivity reserved for
 * gs_usb devices. Copy these values, don't include cannectivity's Kconfig --
 * that Kconfig only exists when its app is on the build. */
#define ROUTER_GS_USB_SELF_VID          0x1209U
#define ROUTER_GS_USB_SELF_PID          0xca01U
#define ROUTER_GS_USB_SELF_MANUFACTURER "CANnectivity"
#define ROUTER_GS_USB_SELF_PRODUCT      "CANnectivity USB to CAN adapter"
#define ROUTER_GS_USB_SELF_MAX_POWER    125U /* 250 mA (bMaxPower is in 2 mA units) */

USBD_DEVICE_DEFINE(usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
		   ROUTER_GS_USB_SELF_VID, ROUTER_GS_USB_SELF_PID);

USBD_DESC_LANG_DEFINE(lang);
USBD_DESC_MANUFACTURER_DEFINE(mfr, ROUTER_GS_USB_SELF_MANUFACTURER);
USBD_DESC_PRODUCT_DEFINE(product, ROUTER_GS_USB_SELF_PRODUCT);
USBD_DESC_SERIAL_NUMBER_DEFINE(sn);
USBD_DESC_CONFIG_DEFINE(fs_config_desc, "Full-Speed Configuration");

USBD_CONFIGURATION_DEFINE(fs_config, USB_SCD_SELF_POWERED,
			  ROUTER_GS_USB_SELF_MAX_POWER, &fs_config_desc);

int usbd_setup_init(void)
{
	int err;

	err = usbd_add_descriptor(&usbd, &lang);
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

	err = usbd_register_class(&usbd, GS_USB_CLASS_INSTANCE_NAME,
				  USBD_SPEED_FS, 1);
	if (err != 0) {
		LOG_ERR("usbd_register_class(gs_usb_0) failed (%d)", err);
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

	err = usbd_init(&usbd);
	if (err != 0) {
		LOG_ERR("usbd_init failed (%d)", err);
		return err;
	}

	return 0;
}

int usbd_setup_enable(void)
{
	int err = usbd_enable(&usbd);

	if (err != 0) {
		LOG_ERR("usbd_enable failed (%d)", err);
	}
	return err;
}
