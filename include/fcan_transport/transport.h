/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FCAN_TRANSPORT_TRANSPORT_H_
#define FCAN_TRANSPORT_TRANSPORT_H_

#include <fibril_can/fcan.h>

/**
 * @addtogroup fcan_transport
 *
 * A node is a slave whether its frames go straight onto a CAN controller or
 * pass through a hub that bridges several segments. What differs is where the
 * HAL comes from and which thread drives @c fcan_poll, and neither is
 * something a block type's implementation should have to know.
 *
 * The backend is chosen by the devicetree: a "fibril,can-hub" node selects the
 * hub, its absence selects the CAN controller named by @c zephyr,canbus.
 *
 * @{
 */

/**
 * @brief Claim the transport's device. Call before fcan_init().
 *
 * @retval 0 on success, negative errno otherwise.
 */
int fcan_transport_init(void);

/**
 * @brief HAL to put in fcan_config_t::hal.
 *
 * Only meaningful after a successful fcan_transport_init().
 */
fcan_hal_t fcan_transport_hal(void);

/**
 * @brief Hand the initialised node to the transport and bring it up.
 *
 * Call after fcan_init() and fcan_register_all().
 *
 * @retval 0 on success, negative errno otherwise.
 */
int fcan_transport_attach(fcan_node_t * node);

/**
 * @brief Give the calling thread to the transport.
 *
 * @param node  the attached node.
 * @param tick  periodic application work, or NULL when nothing is periodic.
 *              Called at roughly 1 kHz from this thread.
 *
 * Whether this returns depends on the backend: one that owns the poll loop
 * never does, one whose frames are driven by a driver thread returns as soon
 * as there is no @p tick left to run.
 */
void fcan_transport_run(fcan_node_t * node, void (*tick)(void));

/** @} */

#endif /* FCAN_TRANSPORT_TRANSPORT_H_ */
