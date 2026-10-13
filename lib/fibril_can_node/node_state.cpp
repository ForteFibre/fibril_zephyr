/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * The image's one fibril_can node. Creating it here, rather than in the
 * application, is what lets fibril_fcan_node_state() find it without a
 * registration step the application could forget. The pointer goes back to
 * the application only: the functions get the one runtime call that is safe
 * from a tick, not the whole runtime API.
 */

#include <errno.h>

#include <zephyr/logging/log.h>

#include <fibril_can_node/func.h>

LOG_MODULE_REGISTER(fcan_node, CONFIG_FIBRIL_CAN_NODE_LOG_LEVEL);

/* Set once at boot, before the transport runs any tick, and read only from
 * ticks, so a plain pointer is enough.
 */
static fcan_node_t * the_node;

int fibril_fcan_node_init(const fcan_config_t * cfg, fcan_node_t ** out)
{
  if (the_node != nullptr) {
    return -EALREADY;
  }

  /* From the allocator fcan_init is about to carve its arena from, so the
   * node needs no heap of its own. 8-byte alignment satisfies the seqlock and
   * u64 hash fields the runtime keeps inside.
   */
  auto * node = static_cast<fcan_node_t *>(
    cfg->allocator.alloc(fcan_node_storage_size(), 8, cfg->allocator.ctx));

  if (node == nullptr) {
    LOG_ERR("out of memory for the node (%u bytes)", (unsigned)fcan_node_storage_size());
    return -ENOMEM;
  }

  if (fcan_init(node, cfg) != FCAN_OK) {
    LOG_ERR("fcan_init failed (fault=%d)", (int)fcan_fault(node));
    return -EIO;
  }

  the_node = node;
  *out = node;

  return 0;
}

fcan_node_state_t fibril_fcan_node_state(void)
{
  if (the_node == nullptr) {
    return FCAN_STATE_UNPROVISIONED;
  }

  return fcan_state(the_node);
}
