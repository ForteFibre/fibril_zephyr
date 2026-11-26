/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * The node sits directly on a CAN controller and owns the poll loop.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <fibril_can_zephyr/can_hal.h>

#include <fcan_transport/transport.h>

LOG_MODULE_REGISTER(fcan_transport, CONFIG_FCAN_TRANSPORT_LOG_LEVEL);

#define CAN_BUS_DEV DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus))

#define RX_QUEUE_DEPTH 32
K_MSGQ_DEFINE(rx_msgq, sizeof(struct fcan_zephyr_can_rx_item), RX_QUEUE_DEPTH, 4);

static struct fcan_zephyr_can_hal hal;
static struct k_timer tick_timer;

/* Bounds scheduler_wait so the application's periodic work keeps its cadence
 * even when fcan has no closer deadline of its own. Only armed when there is
 * periodic work to do.
 */
static void tick_expiry(struct k_timer * timer)
{
  fcan_notify_tx_ready(k_timer_user_data_get(timer));
}

static const char * state_str(fcan_node_state_t s)
{
  switch (s) {
  case FCAN_STATE_UNPROVISIONED: return "UNPROVISIONED";
  case FCAN_STATE_PROVISIONED: return "PROVISIONED";
  case FCAN_STATE_RUNNING: return "RUNNING";
  case FCAN_STATE_FAULT: return "FAULT";
  case FCAN_STATE_SUSPENDED: return "SUSPENDED";
  default: return "?";
  }
}

int fcan_transport_init(void)
{
  const struct device * can_dev = CAN_BUS_DEV;

  if (!device_is_ready(can_dev)) {
    LOG_ERR("CAN device %s not ready", can_dev->name);
    return -ENODEV;
  }

  int ret = can_set_mode(can_dev, CAN_MODE_FD);

  if (ret != 0) {
    LOG_ERR("can_set_mode(FD) failed (%d)", ret);
    return ret;
  }

  ret = fcan_zephyr_can_hal_init(&hal, can_dev, &rx_msgq);
  if (ret != 0) {
    return ret;
  }

  /* Started here rather than in attach: the HAL has to be able to receive
   * before fcan_init runs, or the T_listen window opens on a deaf bus and the
   * node cannot see an id conflict.
   */
  ret = can_start(can_dev);
  if (ret != 0) {
    LOG_ERR("can_start failed (%d)", ret);
    return ret;
  }

  LOG_INF("direct on %s", can_dev->name);

  return 0;
}

fcan_hal_t fcan_transport_hal(void)
{
  return fcan_zephyr_can_hal_get(&hal);
}

int fcan_transport_attach(fcan_node_t * node)
{
  fcan_zephyr_can_hal_attach_node(&hal, node);
  return 0;
}

void fcan_transport_run(fcan_node_t * node, void (*tick)(void))
{
  if (tick != NULL) {
    k_timer_init(&tick_timer, tick_expiry, NULL);
    k_timer_user_data_set(&tick_timer, node);
    k_timer_start(&tick_timer, K_MSEC(1), K_MSEC(1));
  }

  fcan_node_state_t last_state = fcan_state(node);
  uint32_t last_tick_ms = k_uptime_get_32();

  while (true) {
    /* Block until the HAL posts a wake (RX enqueue, TX complete, topic
     * commit, or the timer above) or fcan's next deadline falls due, drain RX
     * onto this thread so fcan_on_can_rx() never races fcan_poll(), then
     * advance the state machine and the TX scheduler.
     */
    fcan_zephyr_can_hal_scheduler_wait(&hal, node);
    fcan_zephyr_can_hal_drain_rx(&hal);
    fcan_poll(node);

    const uint32_t now_ms = k_uptime_get_32();

    /* scheduler_wait returns more often than 1 kHz during bursts; gate on the
     * millisecond boundary so a burst does not run the application's periodic
     * work faster than it was written for.
     */
    if (tick != NULL && (int32_t)(now_ms - last_tick_ms) > 0) {
      tick();
      last_tick_ms = now_ms;
    }

    const fcan_node_state_t s = fcan_state(node);

    if (s != last_state) {
      LOG_INF(
        "state: %s -> %s (fault=%d)", state_str(last_state), state_str(s),
        (int)fcan_fault(node));
      last_state = s;
    }
  }
}
