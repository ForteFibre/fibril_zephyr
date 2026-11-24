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
static struct k_timer tick_timer;

/* The application thread waits on this for the tick timer and, when a
 * function acts on arrivals, for an M→S frame for the node.
 */
static K_SEM_DEFINE(app_wake, 0, 1);
static atomic_t rx_arrived;
static atomic_t wake_on_rx;

/* Runs on the hub thread right after it writes an M→S frame for the node. */
static void self_rx(void * arg)
{
  ARG_UNUSED(arg);
  atomic_set(&rx_arrived, 1);
  if (atomic_get(&wake_on_rx) != 0) {
    k_sem_give(&app_wake);
  }
}

static void tick_expiry(struct k_timer * timer)
{
  ARG_UNUSED(timer);
  k_sem_give(&app_wake);
}

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
   *
   * This is also the moment the node becomes reachable: the peers are already
   * up from the driver's init, so from here the hub thread polls the node and
   * a peer's request can be served. The application runs every start hook
   * before calling this, which is what keeps that request from racing a
   * function's initial publish.
   */
  /* Installed before the node is attached, so the hub thread never sees it
   * change. Whether an arrival wakes anyone is decided in run.
   */
  fcan_hub_zephyr_set_self_rx_callback(HUB_DEV, self_rx, NULL);

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

void fcan_transport_run(fcan_node_t * node, void (*tick)(void), void (*on_rx)(void))
{
  ARG_UNUSED(node);

  if (tick == NULL && on_rx == NULL) {
    /* Nothing for this thread to run, and the hub thread is already driving
     * the node. Returning lets main() finish; the node keeps running.
     */
    return;
  }

  /* Application work runs on this thread, not on the hub thread, because the
   * driver's loop offers no application hook. That is safe for publishing and
   * reading: the runtime guards its topics with a critical section. It does
   * mean neither hook may assume it shares a thread with fcan_poll.
   *
   * A periodic timer rather than a sleep after each tick, so the tick's own
   * run time does not stretch the period.
   */
  if (tick != NULL) {
    k_timer_init(&tick_timer, tick_expiry, NULL);
    k_timer_start(&tick_timer, K_MSEC(1), K_MSEC(1));
  }
  atomic_set(&wake_on_rx, on_rx != NULL);

  uint32_t missed = 0;

  while (true) {
    (void)k_sem_take(&app_wake, K_FOREVER);

    if (on_rx != NULL && atomic_set(&rx_arrived, 0) != 0) {
      on_rx();
    }

    if (tick == NULL) {
      continue;
    }

    const uint32_t expired = k_timer_status_get(&tick_timer);

    if (expired == 0) {
      continue; /* woken by an arrival */
    }

    /* Missed periods are dropped, not replayed: a burst of ticks would send
     * the motors several commands back to back without a fresh feedback in
     * between.
     */
    if (expired > 1) {
      missed += expired - 1;
      LOG_WRN_RATELIMIT_RATE(1000, "tick overran; %u period(s) missed so far", missed);
    }

    tick();
  }
}
