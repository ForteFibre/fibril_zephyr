/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * usbd_next boilerplate for the router+latency-probe+gs_usb sample. Copy of
 * samples/lib/fibril_can/router_gs_usb_self/src/usbd_setup.c with a
 * differentiating product string; keeping the VID/PID identical means the
 * same host-side udev rules and gs_usb pid filters catch either firmware.
 */

#include "usbd_setup.h"

#include <zephyr/logging/log.h>
#include <zephyr/usb/usbd.h>

LOG_MODULE_REGISTER(usbd_setup, LOG_LEVEL_INF);

#define GS_USB_CLASS_INSTANCE_NAME "gs_usb_0"

/* VID/PID/manufacturer match CANnectivity's Kconfig defaults so pre-existing
 * host-side rules apply. The product string is the only differentiator. */
#define ROUTER_GS_USB_PROBE_VID          0x1209U
#define ROUTER_GS_USB_PROBE_PID          0xca01U
#define ROUTER_GS_USB_PROBE_MANUFACTURER "CANnectivity"
#define ROUTER_GS_USB_PROBE_PRODUCT      "CANnectivity USB to CAN adapter (latency probe)"
#define ROUTER_GS_USB_PROBE_MAX_POWER    125U /* 250 mA (bMaxPower is in 2 mA units) */

USBD_DEVICE_DEFINE(usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
		   ROUTER_GS_USB_PROBE_VID, ROUTER_GS_USB_PROBE_PID);

USBD_DESC_LANG_DEFINE(lang);
USBD_DESC_MANUFACTURER_DEFINE(mfr, ROUTER_GS_USB_PROBE_MANUFACTURER);
USBD_DESC_PRODUCT_DEFINE(product, ROUTER_GS_USB_PROBE_PRODUCT);
USBD_DESC_SERIAL_NUMBER_DEFINE(sn);
USBD_DESC_CONFIG_DEFINE(fs_config_desc, "Full-Speed Configuration");

USBD_CONFIGURATION_DEFINE(fs_config, USB_SCD_SELF_POWERED,
			  ROUTER_GS_USB_PROBE_MAX_POWER, &fs_config_desc);

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
