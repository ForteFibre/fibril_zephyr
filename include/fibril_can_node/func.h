/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FIBRIL_CAN_NODE_FUNC_H_
#define FIBRIL_CAN_NODE_FUNC_H_

#include <stdint.h>

#include <zephyr/sys/iterable_sections.h>

#include <fibril_can/fcan.h>

#ifdef __cplusplus
extern "C" {
#endif

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
  /** Instances this board actually wires, at most FCAN_<TYPE>_MAX_COUNT. */
  uint8_t count;
  /** Claim the hardware. Called before fcan_init(). Optional. */
  int (*init)(void);
  /**
   * First contact with the bus, called once between fcan_register_all() and
   * fcan_transport_attach(). Optional. A function whose S2M topics are
   * commit-triggered publishes their initial value here; the runtime holds
   * the request until the node reaches RUNNING.
   *
   * Nothing can reach the node yet — attaching is what starts delivery — so
   * a service handler cannot run concurrently with this and undo what it
   * publishes.
   */
  int (*start)(void);
  /**
   * Publish periodic S2M topics. Called at roughly 1 kHz from a single
   * thread, which is not necessarily the one running fcan_poll() — behind a
   * CAN hub the poll belongs to the driver thread. Publishing is safe either
   * way, because a topic commit is built for a committer and a scheduler in
   * different contexts. The node's state is readable through
   * fibril_fcan_node_state(); nothing else about the node is, so a tick must
   * not assume it shares a thread with the poll.
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
 * The source holding this is compiled only when the devicetree describes the
 * function's hardware, which is also what puts the block type in the schema.
 */
#define FIBRIL_FCAN_FUNC_DEFINE(_ident, ...)                                   \
  static const STRUCT_SECTION_ITERABLE(fibril_fcan_func, _ident) = {           \
    .name = #_ident,                                                           \
    __VA_ARGS__                                                                \
  }

/**
 * @brief Create the image's fibril_can node and run fcan_init() on it.
 * @ingroup fibril_can_node
 *
 * For the application, in place of allocating a node and calling fcan_init()
 * itself. The node's storage comes from @p cfg's allocator. There is one node
 * per image, and only the node made here is what fibril_fcan_node_state()
 * reports on.
 *
 * @param cfg  configuration for fcan_init().
 * @param out  receives the node on success, for fcan_gen::register_all() and
 *             the transport. Not for the functions.
 *
 * @retval 0 on success.
 * @retval -EALREADY if the node was already made.
 * @retval -ENOMEM if the allocator could not hold the node.
 * @retval -EIO if fcan_init() failed; the fault is logged.
 */
int fibril_fcan_node_init(const fcan_config_t * cfg, fcan_node_t ** out);

/**
 * @brief The node's state, safe to read from a tick.
 * @ingroup fibril_can_node
 *
 * Only @c FCAN_STATE_RUNNING means the master is driving the node. It turns
 * @c FCAN_STATE_SUSPENDED once no master frame has arrived for the
 * application's master_lost_us, and back on the next one; before a master
 * starts the node it is UNPROVISIONED or PROVISIONED. A function whose
 * outputs must not outlive the master's commands checks for RUNNING rather
 * than for SUSPENDED.
 *
 * @c FCAN_STATE_UNPROVISIONED until fibril_fcan_node_init() succeeds.
 */
fcan_node_state_t fibril_fcan_node_state(void);

#ifdef __cplusplus
}
#endif

#endif /* FIBRIL_CAN_NODE_FUNC_H_ */
