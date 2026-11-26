/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr entry point for the fibril_can hub + latency-probe self node +
 * gs_usb sample. Cross-references:
 *
 *   fibril_can_benchmark/docs/firmware-requirements.md — wire contract
 *   fibril_can/docs/09-hub.md                          — hub lifecycle
 *   samples/lib/fibril_can/hub_gs_usb_self              — non-benchmark twin
 *   samples/lib/fibril_can/latency_probe_node           — single-bus twin
 *
 * Threading model
 * ---------------
 * The hub driver thread owns fcan_hub_on_rx() and calls
 * fcan_poll(self) at CONFIG_CAN_FCAN_HUB_POLL_INTERVAL_US granularity.
 * This main thread only calls probe_logic_tick() (topic begin/commit/read,
 * all seqlock-protected). Do NOT add fcan_svc_* calls here — they would
 * race the hub thread's fcan_service_poll() slots.
 *
 * Latency vs the single-bus sample
 * --------------------------------
 * The single-bus sample (samples/lib/fibril_can/latency_probe_node) can pin
 * commit → emit into a single tick via the §3.4.1 "on_tick before fcan_poll"
 * order. Here the two are on different threads, so a commit dwell of ~one
 * CONFIG_CAN_FCAN_HUB_POLL_INTERVAL_US is intrinsic. Lower that Kconfig
 * to trade CPU for dwell.
 *
 * What this sample does NOT verify
 * --------------------------------
 * - End-to-end against fibril_can_bridge or fibril_can_web; that setup is
 *   the same as the hub_gs_usb_self README.
 * - Peer-side physical CAN latency budgets. This sample terminates the probe
 *   at the external port; a probe running on a node attached to fdcan1
 *   would add the peer bus time.
 */

#include <cannectivity/usb/class/gs_usb.h>
#include <fibril_can/fcan.h>
#include <fibril_can/fcan_protocol.h>
#include <fibril_can/hub/fcan_hub.h>
#include <fibril_can_zephyr/can_hub.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/time_units.h>

#include "probe_logic.h"
#include "schema_gen.h"
#include "usbd_setup.h"

extern const uint8_t fcan_schema_blob[];
extern const size_t fcan_schema_blob_len;
extern const uint64_t fcan_schema_hash;

LOG_MODULE_REGISTER(fcan_hub_latency_probe, LOG_LEVEL_INF);

#define HUB_NODE DT_NODELABEL(fcan_hub)
#define HUB_DEV DEVICE_DT_GET(HUB_NODE)

/* §6 default; probe launch expects --node-id 0x20. Override via
 * -DCONFIG_EXTRA_CFLAGS=-DNODE_ID=0x?? when running two boards side by side. */
#ifndef NODE_ID
#define NODE_ID 0x20
#endif

#define MASTER_LOST_US 0U /* §7 — disabled so echo never stalls. */

#ifndef TICK_HZ
#define TICK_HZ 10000U
#endif

K_HEAP_DEFINE(fcan_heap, 16 * 1024);

static void * heap_alloc(size_t size, size_t align, void * ctx)
{
  ARG_UNUSED(ctx);
  return k_heap_aligned_alloc(&fcan_heap, align, size, K_NO_WAIT);
}

static const char * state_str(fcan_node_state_t s)
{
  switch (s) {
    case FCAN_STATE_UNPROVISIONED:
      return "UNPROVISIONED";
    case FCAN_STATE_PROVISIONED:
      return "PROVISIONED";
    case FCAN_STATE_RUNNING:
      return "RUNNING";
    case FCAN_STATE_FAULT:
      return "FAULT";
    default:
      return "?";
  }
}

/* Handed to the status thread after bring-up. Reads happen from a lower
 * priority thread; the values themselves are word-sized so a torn read is
 * at worst a one-cycle stale counter in the log line. */
static fcan_node_t * g_self;
static fcan_hub_t * g_hub;
static const struct device * g_hub_dev;

static void status_thread_entry(void * a, void * b, void * c)
{
  ARG_UNUSED(a);
  ARG_UNUSED(b);
  ARG_UNUSED(c);

  fcan_node_state_t last = FCAN_STATE_UNPROVISIONED;
  while (true) {
    if (g_self != NULL && g_hub != NULL && g_hub_dev != NULL) {
      const fcan_node_state_t s = fcan_state(g_self);
      if (s != last) {
        LOG_INF(
          "state: %s -> %s (fault=%d)", state_str(last), state_str(s), (int)fcan_fault(g_self));
        last = s;
      }
      fcan_hub_diag_t d;
      fcan_hub_get_diag(g_hub, &d);
      uint32_t ing_drops = fcan_hub_zephyr_ingress_drops(g_hub_dev);
      LOG_INF(
        "alive: state=%s fwd[0]=%u fwd[1]=%u "
        "to_self=%u drop_send=%u ingress=%u",
        state_str(s), d.fwd_count[0], d.fwd_count[1], d.delivered_to_self, d.drop_send_failed,
        ing_drops);
    }
    k_sleep(K_SECONDS(1));
  }
}
K_THREAD_DEFINE(status_tid, 1024, status_thread_entry, NULL, NULL, NULL, 7, 0, 0);

int main(void)
{
  const struct device * hub = HUB_DEV;
  const struct device * gs_usb = DEVICE_DT_GET(DT_NODELABEL(gs_usb0));

  if (!device_is_ready(hub)) {
    LOG_ERR("hub device %s not ready", hub->name);
    return -ENODEV;
  }

  fcan_hub_t * h = fcan_hub_zephyr_get(hub);
  if (h == NULL) {
    LOG_ERR("fcan_hub_zephyr_get returned NULL (init failed)");
    return -ENODEV;
  }

  fcan_node_t * self = k_heap_aligned_alloc(&fcan_heap, 8, fcan_node_storage_size(), K_NO_WAIT);
  if (self == NULL) {
    LOG_ERR("fcan_heap OOM allocating node (%u bytes)", (unsigned)fcan_node_storage_size());
    return -ENOMEM;
  }

  /* Indexed by FCAN_ARRAY_*, never positionally: the block array order is
   * derived from the schema, so a positional initialiser keeps compiling
   * while handing a count to the wrong block.
   */
  static const uint8_t counts[FCAN_NUM_BLOCK_ARRAYS] = {
    [FCAN_ARRAY_LATENCYPROBE] = 1U,
  };

  fcan_config_t cfg = {
    .node_id = NODE_ID,
    .boot_id = sys_rand32_get(),
    .schema_blob = fcan_schema_blob,
    .schema_blob_len = (uint16_t)fcan_schema_blob_len,
    .schema_hash = fcan_schema_hash,
    .protocol_version = FCAN_PROTOCOL_VERSION,
    .block_count = FCAN_NUM_BLOCK_ARRAYS,
    .instance_counts = counts,
    .limits =
      {
        .max_frames = FCAN_MAX_FRAMES,
        .max_copy_entries = FCAN_MAX_COPY_ENTRIES,
        .service_buffer = FCAN_SERVICE_BUFFER,
        .service_reassembly = FCAN_SERVICE_REASSEMBLY,
      },
    /* HAL comes from the hub: fcan_poll(self) TX broadcasts onto
		 * every live port (external gs_usb + fdcan1 peer), and
		 * fcan_hub_on_rx() delivers RX from either side back into
		 * self. */
    .hal = fcan_hub_get_self_hal(h),
    .allocator = {.alloc = heap_alloc, .ctx = NULL},
    .master_lost_us = MASTER_LOST_US,
  };
  fcan_apply_schema_capacities(&cfg.limits, counts);

  if (fcan_init(self, &cfg) != FCAN_OK) {
    LOG_ERR("fcan_init failed (fault=%d)", (int)fcan_fault(self));
    return -EIO;
  }
  LOG_INF(
    "fcan_init OK: node_id=0x%02x schema_hash=0x%016llx blob_len=%u tick=%u Hz", cfg.node_id,
    (unsigned long long)fcan_schema_hash, (unsigned)fcan_schema_blob_len, (unsigned)TICK_HZ);

  /* Attach BEFORE the host can open the gs_usb channel — see
	 * docs/09-hub.md for why the hub thread must not see frames
	 * before the self node is wired in. */
  if (fcan_hub_attach_self(h, self) != FCAN_OK) {
    LOG_ERR("fcan_hub_attach_self failed (fault=%d)", (int)fcan_fault(self));
    return -EIO;
  }
  LOG_INF("fcan_hub_attach_self OK");

  if (fcan_register_all(self) != FCAN_OK) {
    LOG_ERR("fcan_register_all failed (fault=%d)", (int)fcan_fault(self));
    return -EIO;
  }
  LOG_INF("fcan_register_all OK");

  probe_logic_init();

  if (!device_is_ready(gs_usb)) {
    LOG_ERR("gs_usb device %s not ready", gs_usb->name);
    return -ENODEV;
  }

  int rc = usbd_setup_init();
  if (rc != 0) {
    LOG_ERR("usbd_setup_init failed (%d)", rc);
    return rc;
  }

  const struct device * channels[] = {hub};
  rc = gs_usb_register(gs_usb, channels, ARRAY_SIZE(channels), NULL, NULL);
  if (rc != 0) {
    LOG_ERR("gs_usb_register failed (%d)", rc);
    return rc;
  }
  LOG_INF("gs_usb_register OK (1 channel: %s)", hub->name);

  rc = usbd_setup_enable();
  if (rc != 0) {
    return rc;
  }
  LOG_INF("usbd_enable OK");

  g_self = self;
  g_hub = h;
  g_hub_dev = hub;

  LOG_INF("entering main loop");

  const int64_t period_ticks = (int64_t)k_us_to_ticks_ceil64(1000000ULL / TICK_HZ);
  int64_t next = k_uptime_ticks();
  while (true) {
    probe_logic_tick();

    next += period_ticks;
    k_sleep(K_TIMEOUT_ABS_TICKS(next));
  }
  /* unreachable */
}
