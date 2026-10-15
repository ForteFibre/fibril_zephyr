/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * This board's CAN health on the bus: one CanPortDiag per CAN controller the
 * transport drives, and a CanHubDiag behind a CAN hub. The values come from
 * fibril_can's Zephyr helpers (can_diag.h, can_hub.h); this file only maps
 * them onto the generated topics.
 */

#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <fibril_can_zephyr/can_diag.h>
#if defined(CONFIG_FCAN_TRANSPORT_HUB)
#include <fibril_can_zephyr/can_hub.h>
#endif

#include <fibril_can_node/func.h>

#include "schema_gen.hpp"

LOG_MODULE_REGISTER(fcan_can_diag, CONFIG_FIBRIL_CAN_NODE_LOG_LEVEL);

namespace
{

/* The topics go out once a second; refreshing faster only costs the hub's
 * run lock and a can_get_state() per controller.
 */
constexpr uint32_t REFRESH_MS = 1000U;

#if defined(CONFIG_FCAN_TRANSPORT_HUB)

#define HUB_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(fibril_can_hub)

const struct device * const hub = DEVICE_DT_GET(HUB_NODE);

/* CanPortDiag instance i is peer i in the "peers" property, which is hub
 * port i + 1; port 0 is the external face and has no controller.
 */
constexpr uint8_t port_count = DT_PROP_LEN(HUB_NODE, peers);

BUILD_ASSERT(
  CONFIG_FCAN_HUB_MAX_PORTS <= FCAN_ZEPHYR_CAN_HUB_DIAG_PORTS,
  "CanHubDiag carries fwd_count for at most FCAN_ZEPHYR_CAN_HUB_DIAG_PORTS ports");

const struct device * port_device(uint8_t i)
{
  return fcan_hub_zephyr_peer(hub, i);
}

uint32_t port_bus_off_count(uint8_t i)
{
  return fcan_hub_zephyr_peer_bus_off_count(hub, i);
}

#else

const struct device * const can_bus = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

constexpr uint8_t port_count = 1U;

/* Nothing else installs a state change callback on the direct transport's
 * controller, so the bus-off count is kept here.
 */
fcan_zephyr_bus_off_counter bus_off;

const struct device * port_device(uint8_t i)
{
  ARG_UNUSED(i);
  return can_bus;
}

uint32_t port_bus_off_count(uint8_t i)
{
  ARG_UNUSED(i);
  return fcan_zephyr_bus_off_count(&bus_off);
}

#endif

BUILD_ASSERT(
  port_count <= fcan_gen::canportdiag::max_count,
  "more CAN controllers than the CanPortDiag instances registered in CMake");

/* Drivers reset CAN_STATS in can_start(); these carry the counts across so
 * the topic stays monotonic since boot.
 */
fcan_zephyr_can_stats_totals stats_totals[port_count];

uint32_t last_refresh_ms;
bool refreshed_once;

void publish_port(uint8_t i)
{
  fcan_zephyr_can_port_diag d;

  /* A failed can_get_state() leaves the state and error counters zero; the
   * rest of the topic is still worth sending.
   */
  (void)fcan_zephyr_can_port_diag_read(port_device(i), &stats_totals[i], &d);

  if (auto pub = fcan_gen::canportdiag(i).publish_diag()) {
    pub->state = d.state;
    pub->tec = d.tec;
    pub->rec = d.rec;
    pub->bus_off_count = port_bus_off_count(i);
    pub->stats_valid = d.stats_valid;
    pub->bit_error = d.bit_error;
    pub->bit0_error = d.bit0_error;
    pub->bit1_error = d.bit1_error;
    pub->stuff_error = d.stuff_error;
    pub->crc_error = d.crc_error;
    pub->form_error = d.form_error;
    pub->ack_error = d.ack_error;
    pub->rx_overrun = d.rx_overrun;
  }
}

#if defined(CONFIG_FCAN_TRANSPORT_HUB)
void publish_hub(void)
{
  fcan_hub_diag_t diag;

  if (fcan_hub_zephyr_get_diag(hub, &diag) != 0) {
    return;
  }

  if (auto pub = fcan_gen::canhubdiag(0).publish_diag()) {
    pub->port_count = port_count + 1U;
    for (uint8_t k = 0; k < port_count + 1U; k++) {
      pub->fwd_count[k] = diag.fwd_count[k];
    }
    pub->delivered_to_self = diag.delivered_to_self;
    pub->drop_send_failed = diag.drop_send_failed;
    pub->ingress_drops = fcan_hub_zephyr_ingress_drops(hub);
  }
}
#endif

int can_diag_init(void)
{
#if !defined(CONFIG_FCAN_TRANSPORT_HUB)
  /* Before fcan_transport_init() starts the controller, so a bus-off during
   * bring-up is counted too.
   */
  fcan_zephyr_bus_off_counter_attach(can_bus, &bus_off);
#endif
  return 0;
}

void can_diag_tick(void)
{
  const uint32_t now_ms = k_uptime_get_32();

  if (refreshed_once && (now_ms - last_refresh_ms) < REFRESH_MS) {
    return;
  }
  last_refresh_ms = now_ms;
  refreshed_once = true;

  for (uint8_t i = 0; i < port_count; i++) {
    publish_port(i);
  }
#if defined(CONFIG_FCAN_TRANSPORT_HUB)
  publish_hub();
#endif
}

}  // namespace

FIBRIL_FCAN_FUNC_DEFINE(
  can_port_diag,
  .array = fcan_gen::canportdiag::block_array_index,
  .count = port_count,
  .init = can_diag_init,
  .tick = can_diag_tick);

#if defined(CONFIG_FCAN_TRANSPORT_HUB)
FIBRIL_FCAN_FUNC_DEFINE(
  can_hub_diag,
  .array = fcan_gen::canhubdiag::block_array_index,
  .count = 1U);
#endif
