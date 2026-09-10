/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * fibril_can slave entry point.
 *
 * Names no block type. The functions this image carries register themselves
 * in the fibril_fcan_func section (see include/fibril_can_node/func.h), and
 * the schema decides which of them survive the build, so adding a valve or a
 * motor to the bus does not touch this file.
 */

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>

#include <fibril_can/fcan.h>
#include <fibril_can/fcan_protocol.h>

#include <fibril_can_zephyr/can_hal.h>

#include <fibril_can_node/func.h>

#include "schema_gen.h"

/* The C emitter puts these in schema_blob.c but does not declare them. */
extern const uint8_t fcan_schema_blob[];
extern const size_t fcan_schema_blob_len;
extern const uint64_t fcan_schema_hash;

LOG_MODULE_REGISTER(fibril_node, CONFIG_APP_LOG_LEVEL);

#define CAN_BUS_DEV DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus))

#define MASTER_LOST_US 300000U /* SPEC §5.11 */

/* fcan_init carves every variable-length buffer from this once. Sized with
 * headroom rather than tuned: the node schema can gain a block type without
 * a matching edit here, and running out is a boot failure, not a slow path.
 */
K_HEAP_DEFINE(fcan_heap, 16 * 1024);

#define FCAN_RX_QUEUE_DEPTH 32
K_MSGQ_DEFINE(fcan_rx_msgq, sizeof(struct fcan_zephyr_can_rx_item), FCAN_RX_QUEUE_DEPTH, 4);

static struct k_timer tick_timer;

static void * heap_alloc(size_t size, size_t align, void * ctx)
{
  ARG_UNUSED(ctx);
  return k_heap_aligned_alloc(&fcan_heap, align, size, K_NO_WAIT);
}

/* Bounds scheduler_wait so the function ticks keep a ~1 kHz cadence even when
 * fcan has no closer deadline of its own.
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

int main(void)
{
  const struct device * can_dev = CAN_BUS_DEV;
  static struct fcan_zephyr_can_hal hal;

  if (!device_is_ready(can_dev)) {
    LOG_ERR("CAN device %s not ready", can_dev->name);
    return -ENODEV;
  }

  /* Indexed by the codegen's FCAN_ARRAY_*, so a function fills its own slot
   * whatever order the schema puts the block arrays in.
   */
  uint8_t counts[FCAN_NUM_BLOCK_ARRAYS] = {0};

  STRUCT_SECTION_FOREACH(fibril_fcan_func, f) {
    int ret = (f->init != NULL) ? f->init() : 0;

    if (ret != 0) {
      LOG_ERR("%s: init failed (%d)", f->name, ret);
      return ret;
    }

    counts[f->array] = f->count;
    LOG_INF("%s: %u instance(s) on block array %u", f->name, f->count, f->array);
  }

  int rc = can_set_mode(can_dev, CAN_MODE_FD);

  if (rc != 0) {
    LOG_ERR("can_set_mode(FD) rc=%d", rc);
    return rc;
  }

  rc = fcan_zephyr_can_hal_init(&hal, can_dev, &fcan_rx_msgq);
  if (rc != 0) {
    return rc;
  }

  rc = can_start(can_dev);
  if (rc != 0) {
    LOG_ERR("can_start rc=%d", rc);
    return rc;
  }

  /* fcan_node_storage_size() is a runtime call because fcan_node_t is opaque,
   * so the node cannot be a static array. 8-byte alignment satisfies the
   * seqlock and u64 hash fields the runtime keeps inside.
   */
  fcan_node_t * node =
    k_heap_aligned_alloc(&fcan_heap, 8, fcan_node_storage_size(), K_NO_WAIT);

  if (node == NULL) {
    LOG_ERR("fcan_heap OOM allocating node (%u bytes)", (unsigned)fcan_node_storage_size());
    return -ENOMEM;
  }

  fcan_config_t cfg = {
    /* Build-time for now. On a board with a DIP switch or a config ROM this
     * is read at boot so that identical boards share one image.
     */
    .node_id = CONFIG_FIBRIL_NODE_ID,
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
    .hal = fcan_zephyr_can_hal_get(&hal),
    .allocator = {.alloc = heap_alloc, .ctx = NULL},
    .master_lost_us = MASTER_LOST_US,
  };
  fcan_apply_schema_capacities(&cfg.limits, counts);

  if (fcan_init(node, &cfg) != FCAN_OK) {
    LOG_ERR("fcan_init failed (fault=%d)", (int)fcan_fault(node));
    return -EIO;
  }
  LOG_INF(
    "fcan_init OK: node_id=0x%02x schema_hash=0x%016llx blob_len=%u", cfg.node_id,
    (unsigned long long)fcan_schema_hash, (unsigned)fcan_schema_blob_len);

  if (fcan_register_all(node) != FCAN_OK) {
    LOG_ERR("fcan_register_all failed (fault=%d)", (int)fcan_fault(node));
    return -EIO;
  }
  LOG_INF("fcan_register_all OK");

  fcan_zephyr_can_hal_attach_node(&hal, node);

  bool any_tick = false;

  STRUCT_SECTION_FOREACH(fibril_fcan_func, f) {
    if (f->start != NULL) {
      int ret = f->start();

      if (ret != 0) {
        LOG_ERR("%s: start failed (%d)", f->name, ret);
        return ret;
      }
    }

    any_tick = any_tick || (f->tick != NULL);
  }

  /* Only pay for the 1 kHz wake if something is waiting for it. With every
   * function publishing on change, scheduler_wait can sleep until fcan's
   * next deadline instead.
   */
  if (any_tick) {
    k_timer_init(&tick_timer, tick_expiry, NULL);
    k_timer_user_data_set(&tick_timer, node);
    k_timer_start(&tick_timer, K_MSEC(1), K_MSEC(1));
  }

  LOG_INF(
    "entering main loop (state=%s, periodic ticks %s)", state_str(fcan_state(node)),
    any_tick ? "on" : "off");

  fcan_node_state_t last_state = fcan_state(node);
  uint32_t last_tick_ms = k_uptime_get_32();

  while (true) {
    /* Block until the HAL posts a wake (RX enqueue, TX complete, topic
     * commit, or the 1 ms timer above) or fcan's next deadline falls due,
     * drain RX onto this thread so fcan_on_can_rx() never races fcan_poll(),
     * then advance the state machine and the TX scheduler.
     */
    fcan_zephyr_can_hal_scheduler_wait(&hal, node);
    fcan_zephyr_can_hal_drain_rx(&hal);
    fcan_poll(node);

    const uint32_t now_ms = k_uptime_get_32();

    /* scheduler_wait returns more often than 1 kHz during bursts; gate the
     * function ticks on the millisecond boundary so a burst does not run
     * them faster than they were written for.
     */
    if (any_tick && (int32_t)(now_ms - last_tick_ms) > 0) {
      STRUCT_SECTION_FOREACH(fibril_fcan_func, f) {
        if (f->tick != NULL) {
          f->tick();
        }
      }
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

  return 0;
}
