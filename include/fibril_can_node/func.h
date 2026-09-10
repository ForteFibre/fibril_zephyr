/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FIBRIL_CAN_NODE_FUNC_H_
#define FIBRIL_CAN_NODE_FUNC_H_

#include <stdint.h>

#include <zephyr/sys/iterable_sections.h>

/**
 * @brief One block type's implementation, as seen by the node application.
 * @ingroup fibril_can_node
 *
 * The application walks these to size fcan_config_t::instance_counts and to
 * drive each function, so it never names a block type. Adding a function is
 * then a change to this library alone.
 */
struct fibril_fcan_func
{
  /** Block type name, for logs. */
  const char * name;
  /** FCAN_ARRAY_<TYPE> from the codegen: the block array this fills. */
  uint8_t array;
  /** Instances this board actually wires, at most the schema's max_count. */
  uint8_t count;
  /** Claim the hardware. Called before fcan_init(). Optional. */
  int (*init)(void);
  /**
   * First contact with the bus, called once after fcan_register_all().
   * Optional. A function whose S2M topics are commit-triggered publishes
   * their initial value here; the runtime holds the request until the node
   * reaches RUNNING.
   */
  int (*start)(void);
  /**
   * Publish periodic S2M topics. Called at roughly 1 kHz from a single
   * thread, which is not necessarily the one running fcan_poll() — behind a
   * CAN hub the poll belongs to the driver thread. Publishing is safe either
   * way, because a topic commit is built for a committer and a scheduler in
   * different contexts, but a tick must not assume it can observe the node's
   * state without racing the poll.
   *
   * Optional, and worth leaving out: a function whose state only changes on
   * a service call should publish from the handler instead, which costs
   * nothing while nothing happens.
   */
  void (*tick)(void);
};

/**
 * @brief Register a block type implementation.
 * @ingroup fibril_can_node
 *
 * Place inside the codegen's `#if defined(FCAN_<TYPE>_MAX_COUNT)` guard so
 * that a schema without this block type drops the function entirely.
 */
#define FIBRIL_FCAN_FUNC_DEFINE(_ident, ...)                                   \
  static const STRUCT_SECTION_ITERABLE(fibril_fcan_func, _ident) = {           \
    .name = #_ident,                                                           \
    __VA_ARGS__                                                                \
  }

#endif /* FIBRIL_CAN_NODE_FUNC_H_ */
