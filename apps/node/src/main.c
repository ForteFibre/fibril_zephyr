/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * fibril_can slave entry point.
 *
 * Names no block type and no transport. The functions this image carries
 * register themselves in the fibril_fcan_func section (see
 * include/fibril_can_node/func.h), and how their frames reach the bus is
 * decided by the devicetree (see include/fcan_transport/transport.h). Adding
 * a valve, or putting the same node behind a CAN hub, does not touch this
 * file.
 */

#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>

#include <fibril_can/fcan.h>
#include <fibril_can/fcan_protocol.h>

#include <fcan_transport/transport.h>
#include <fibril_can_node/func.h>

#include "schema_gen.h"

/* The C emitter puts these in schema_blob.c but does not declare them. */
extern const uint8_t fcan_schema_blob[];
extern const size_t fcan_schema_blob_len;
extern const uint64_t fcan_schema_hash;

LOG_MODULE_REGISTER(fibril_node, CONFIG_APP_LOG_LEVEL);

#define MASTER_LOST_US 300000U /* SPEC §5.11 */

/* fcan_init carves every variable-length buffer from this once. Sized with
 * headroom rather than tuned: the image can gain a block type without a
 * matching edit here, and running out is a boot failure, not a slow path.
 */
K_HEAP_DEFINE(fcan_heap, 16 * 1024);

static void * heap_alloc(size_t size, size_t align, void * ctx)
{
  ARG_UNUSED(ctx);
  return k_heap_aligned_alloc(&fcan_heap, align, size, K_NO_WAIT);
}

static void run_ticks(void)
{
  STRUCT_SECTION_FOREACH(fibril_fcan_func, f) {
    if (f->tick != NULL) {
      f->tick();
    }
  }
}

int main(void)
{
  /* Indexed by the codegen's FCAN_ARRAY_*, so a function fills its own slot
   * whatever order the schema puts the block arrays in.
   */
  uint8_t counts[FCAN_NUM_BLOCK_ARRAYS] = {0};
  bool any_tick = false;

  STRUCT_SECTION_FOREACH(fibril_fcan_func, f) {
    int ret = (f->init != NULL) ? f->init() : 0;

    if (ret != 0) {
      LOG_ERR("%s: init failed (%d)", f->name, ret);
      return ret;
    }

    counts[f->array] = f->count;
    any_tick = any_tick || (f->tick != NULL);
    LOG_INF("%s: %u instance(s) on block array %u", f->name, f->count, f->array);
  }

  int rc = fcan_transport_init();

  if (rc != 0) {
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
    .hal = fcan_transport_hal(),
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

  rc = fcan_transport_attach(node);
  if (rc != 0) {
    return rc;
  }

  STRUCT_SECTION_FOREACH(fibril_fcan_func, f) {
    int ret = (f->start != NULL) ? f->start() : 0;

    if (ret != 0) {
      LOG_ERR("%s: start failed (%d)", f->name, ret);
      return ret;
    }
  }

  LOG_INF("handing over to the transport (periodic ticks %s)", any_tick ? "on" : "off");

  fcan_transport_run(node, any_tick ? run_ticks : NULL);

  return 0;
}
