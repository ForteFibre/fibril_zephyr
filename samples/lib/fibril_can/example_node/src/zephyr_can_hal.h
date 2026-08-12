/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr CAN driver <-> fcan_hal_t bridge for the fibril_can example node.
 *
 * The HAL owns two responsibilities that live on opposite sides of the
 * kernel boundary:
 *   1. TX: fcan_runtime calls hal.send() from thread context (fcan_poll or a
 *      service handler) — we forward each call to can_send() with FD framing.
 *   2. RX: the CAN driver invokes our rx callback from a driver-owned thread
 *      (for zephyr,can-loopback it's the tx_thread; on real controllers it's
 *      typically a workqueue). fcan_on_can_rx() is safe from ISR but is not
 *      re-entrant against fcan_poll(). Instead of adding a lock we drop the
 *      frame into a k_msgq and let main() drain it right before fcan_poll().
 */
#ifndef ZEPHYR_CAN_HAL_H_
#define ZEPHYR_CAN_HAL_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>

#include <fibril_can/fcan.h>
#include <fibril_can/fcan_hal.h>

/* One queued frame — mirrors the fcan_on_can_rx() argument set so we don't
 * hold on to the driver's struct can_frame past the callback return. */
struct fcan_rx_item {
	uint32_t id;
	uint8_t  data[64];
	uint8_t  len;
	bool     ext;
};

struct zephyr_can_hal {
	const struct device *dev;
	fcan_node_t *node; /* set after fcan_init via zephyr_can_hal_attach_node() */
	struct k_msgq *rx_msgq;
	uint32_t tx_frames;
	uint32_t tx_drops;
	uint32_t rx_frames;
};

/* Prepare the HAL and register accept-all RX filters (standard + extended).
 * The caller owns the k_msgq so it can size / place it (a static
 * K_MSGQ_DEFINE next to main() is the expected usage).
 *
 * Returns 0 on success, or a negative errno mirroring the failing CAN API. */
int zephyr_can_hal_init(struct zephyr_can_hal *h,
			const struct device *dev,
			struct k_msgq *rx_msgq);

/* Post fcan_init: hand the runtime node to the HAL so fcan_on_can_rx() can
 * be dispatched from main() when the msgq is drained. */
void zephyr_can_hal_attach_node(struct zephyr_can_hal *h, fcan_node_t *node);

/* Vtable snapshot to hand to fcan_config_t.hal. */
fcan_hal_t zephyr_can_hal_get(struct zephyr_can_hal *h);

/* Drain the RX queue into fcan_on_can_rx(). Non-blocking. */
void zephyr_can_hal_drain_rx(struct zephyr_can_hal *h);

#endif /* ZEPHYR_CAN_HAL_H_ */
