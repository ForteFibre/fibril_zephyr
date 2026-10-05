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

#include <cstddef>
#include <cstdint>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>

#include <fcan_transport/transport.h>
#include <fibril_can_node/func.h>

#include "schema_gen.hpp"

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
    if (f->tick != nullptr) {
      f->tick();
    }
  }
}

int main(void)
{
  /* Indexed by fcan_gen::<type>::block_array_index, so a function fills its
   * own slot whatever order the schema puts the block arrays in.
   *
   * Static because config_user documents that the pointer has to outlive the
   * node, and fcan_transport_run() does return on a backend whose frames a
   * driver thread drives. The runtime copies the counts today, so an
   * automatic would work; one byte of .bss is cheaper than depending on that.
   */
  static uint8_t counts[fcan_gen::num_block_arrays];
  bool any_tick = false;

  STRUCT_SECTION_FOREACH(fibril_fcan_func, f) {
    int ret = (f->init != nullptr) ? f->init() : 0;

    if (ret != 0) {
      LOG_ERR("%s: init failed (%d)", f->name, ret);
      return ret;
    }

    counts[f->array] = f->count;
    any_tick = any_tick || (f->tick != nullptr);
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
  auto * node = static_cast<fcan_node_t *>(
    k_heap_aligned_alloc(&fcan_heap, 8, fcan_node_storage_size(), K_NO_WAIT));

  if (node == nullptr) {
    LOG_ERR("fcan_heap OOM allocating node (%u bytes)", (unsigned)fcan_node_storage_size());
    return -ENOMEM;
  }

  fcan_gen::config_user user{};

  /* Build-time for now. On a board with a DIP switch or a config ROM this is
   * read at boot so that identical boards share one image.
   */
  user.node_id = CONFIG_FIBRIL_NODE_ID;
  user.boot_id = sys_rand32_get();
  user.instance_counts = counts;
  user.hal = fcan_transport_hal();
  user.allocator = {.alloc = heap_alloc, .ctx = nullptr};
  user.master_lost_us = MASTER_LOST_US;

  /* The schema-determined half of the config, and the capacities derived from
   * the counts, come from the generated header.
   */
  fcan_config_t cfg = fcan_gen::make_config(user);

  if (fcan_init(node, &cfg) != FCAN_OK) {
    LOG_ERR("fcan_init failed (fault=%d)", (int)fcan_fault(node));
    return -EIO;
  }
  LOG_INF(
    "fcan_init OK: node_id=0x%02x schema_hash=0x%016llx blob_len=%u", user.node_id,
    (unsigned long long)fcan_schema_hash, (unsigned)fcan_schema_blob_len);

  /* Also what installs the service and parameter dispatchers the functions
   * register their handlers into, so it has to run before any start hook.
   */
  if (fcan_gen::register_all(node) != FCAN_OK) {
    LOG_ERR("register_all failed (fault=%d)", (int)fcan_fault(node));
    return -EIO;
  }
  LOG_INF("register_all OK");

  /* Before attach, not after. Attaching is what lets frames reach the node —
   * behind a hub the driver thread starts polling it, and a gs_usb build
   * enumerates on the same call — so a service handler could otherwise run
   * while a function is still publishing its initial state and overwrite a
   * just-commanded output. Committing here is not a lost publish: the runtime
   * holds the request until the node is allowed to transmit.
   */
  STRUCT_SECTION_FOREACH(fibril_fcan_func, f) {
    int ret = (f->start != nullptr) ? f->start() : 0;

    if (ret != 0) {
      LOG_ERR("%s: start failed (%d)", f->name, ret);
      return ret;
    }
  }

  rc = fcan_transport_attach(node);
  if (rc != 0) {
    return rc;
  }

  LOG_INF("handing over to the transport (periodic ticks %s)", any_tick ? "on" : "off");

  fcan_transport_run(node, any_tick ? run_ticks : nullptr);

  return 0;
}
