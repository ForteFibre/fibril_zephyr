/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * The node is the self port of a CAN hub. The hub driver owns the thread that
 * routes frames and drives fcan_poll for the attached node, so this backend
 * hands over the node and gets out of the way.
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <fibril_can_zephyr/can_hub.h>

#include <fcan_transport/transport.h>

#include "gs_usb.h"

LOG_MODULE_REGISTER(fcan_transport, CONFIG_FCAN_TRANSPORT_LOG_LEVEL);

/* One hub per image: the driver owns every peer bus exclusively, so a second
 * instance would have nothing left to own.
 */
#define HUB_DEV DEVICE_DT_GET_ONE(fibril_can_hub)

static fcan_hub_t * hub;

int fcan_transport_init(void)
{
  const struct device * dev = HUB_DEV;

  if (!device_is_ready(dev)) {
    LOG_ERR("hub device %s not ready", dev->name);
    return -ENODEV;
  }

  hub = fcan_hub_zephyr_get(dev);
  if (hub == NULL) {
    LOG_ERR("fcan_hub_zephyr_get returned NULL (hub init failed)");
    return -ENODEV;
  }

  /* can_start() is deliberately not called. It opens the external port only,
   * which is the gateway's business, and the peers are already up from the
   * driver's own init. A board with no host attached still heartbeats onto
   * its peer segments.
   */
  LOG_INF("hub self port on %s", dev->name);

  return 0;
}

fcan_hal_t fcan_transport_hal(void)
{
  /* Not a wrapper around a Zephyr CAN device: the hub broadcasts the self
   * node's transmissions onto every live port, because a hub port does not
   * know which direction the master lies in.
   */
  return fcan_hub_get_self_hal(hub);
}

int fcan_transport_attach(fcan_node_t * node)
{
  /* Mutates state the hub thread reads, so it has to happen before the device
   * is started — that is, before a host can open the external port. Boot-time
   * attachment satisfies this with room to spare.
   */
  if (fcan_hub_attach_self(hub, node) != FCAN_OK) {
    LOG_ERR("fcan_hub_attach_self failed");
    return -EIO;
  }

  /* Only now may a host enumerate. A no-op unless the devicetree describes a
   * gs_usb node, which is what makes the external port face USB instead of
   * staying unused.
   */
  return fcan_transport_gs_usb_start(HUB_DEV);
}

void fcan_transport_run(fcan_node_t * node, void (*tick)(void))
{
  ARG_UNUSED(node);

  if (tick == NULL) {
    /* Nothing periodic to run, and the hub thread is already driving the
     * node. Returning lets main() finish; the node keeps running.
     */
    return;
  }

  /* Periodic work runs on this thread, not on the hub thread, because the
   * driver's loop offers no application hook. That is safe for publishing:
   * fcan_topic_commit is built for a committer and a scheduler in different
   * contexts. It does mean a tick must not assume it shares a thread with
   * fcan_poll.
   */
  while (true) {
    tick();
    k_sleep(K_MSEC(1));
  }
}
