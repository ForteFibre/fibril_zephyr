/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal usbd_next boilerplate for the hub+latency-probe+gs_usb sample.
 * A cut-down of CANnectivity's app/src/usb.c: full-speed only, no DFU, no
 * MSOSV2 descriptor set. The registered gs_usb class instance
 * ("gs_usb_0") is the same one CANnectivity registers, so host-side udev
 * rules that already work with CANnectivity dongles work here too.
 *
 * The VID/PID pair matches CANnectivity's defaults (0x1209:0xca01); only
 * the product string differs so `lsusb -v` distinguishes this firmware
 * from the hub_gs_usb_self sample when both are on the bench.
 */

#ifndef HUB_GS_USB_LATENCY_PROBE_USBD_SETUP_H_
#define HUB_GS_USB_LATENCY_PROBE_USBD_SETUP_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Register descriptors + full-speed configuration + the "gs_usb_0" class
 * instance, then call usbd_init(). Does NOT enable the device — the caller
 * must first bind CAN channels via gs_usb_register() and only then call
 * usbd_setup_enable(). Returns 0 on success, a negative errno on failure. */
int usbd_setup_init(void);

/* Enable the previously-initialised USB device. Splitting this from init lets
 * the sample attach the hub self node BEFORE the host can enumerate. */
int usbd_setup_enable(void);

#ifdef __cplusplus
}
#endif

#endif /* HUB_GS_USB_LATENCY_PROBE_USBD_SETUP_H_ */
