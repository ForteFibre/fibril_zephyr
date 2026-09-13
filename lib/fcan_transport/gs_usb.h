/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Internal to lib/fcan_transport/. A backend whose external port can face a
 * USB host calls this after the node is attached; the no-op keeps the call
 * site free of preprocessor conditionals when no gs_usb node exists.
 */

#ifndef FCAN_TRANSPORT_GS_USB_H_
#define FCAN_TRANSPORT_GS_USB_H_

#include <zephyr/device.h>

#ifdef CONFIG_FCAN_TRANSPORT_GS_USB

/**
 * Bind @p channel as the gs_usb channel and enable the USB device.
 *
 * Must run after the node has been attached to the transport: enabling the
 * device is what lets the host enumerate, and the host opening the channel
 * is what starts the external port.
 */
int fcan_transport_gs_usb_start(const struct device * channel);

#else

static inline int fcan_transport_gs_usb_start(const struct device * channel)
{
  ARG_UNUSED(channel);
  return 0;
}

#endif /* CONFIG_FCAN_TRANSPORT_GS_USB */

#endif /* FCAN_TRANSPORT_GS_USB_H_ */
