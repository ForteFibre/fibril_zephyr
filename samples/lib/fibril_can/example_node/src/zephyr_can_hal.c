/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * See zephyr_can_hal.h for the design notes.
 */
#include "zephyr_can_hal.h"

#include <string.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(fcan_hal, LOG_LEVEL_INF);

static bool hal_send(uint32_t id, bool ext, const uint8_t *data, uint8_t len, void *ctx)
{
	struct zephyr_can_hal *h = ctx;

	/* CAN FD tops out at 64 B (SPEC §7). can_bytes_to_dlc() has no defined
	 * behavior above that, so guard here rather than trusting it to clamp. */
	if (len > CANFD_MAX_DLEN) {
		LOG_ERR("send: oversize len=%u (max %u)", len, (unsigned)CANFD_MAX_DLEN);
		return false;
	}

	struct can_frame frame = {
		.id = id,
		/* BRS is set to allow the loopback and real FDCAN controllers to
		 * negotiate the fast-phase bit rate — the loopback ignores it,
		 * on real hardware it depends on the DTS bitrate-data setting. */
		.flags = CAN_FRAME_FDF | CAN_FRAME_BRS | (ext ? CAN_FRAME_IDE : 0U),
		.dlc = can_bytes_to_dlc(len),
	};
	memcpy(frame.data, data, len);
	/* can_bytes_to_dlc rounds up (e.g. 9 bytes -> DLC for 12). Zero-pad the
	 * tail so we do not leak stack contents to the wire. */
	const uint8_t dlc_bytes = can_dlc_to_bytes(frame.dlc);
	if (dlc_bytes > len) {
		memset(&frame.data[len], 0, dlc_bytes - len);
	}

	int rc = can_send(h->dev, &frame, K_NO_WAIT, NULL, NULL);
	if (rc == 0) {
		h->tx_frames++;
		return true;
	}
	/* -EAGAIN means the TX queue is full; the runtime's scheduler retries
	 * on the next fcan_poll tick. Anything else is a driver-level fault
	 * we surface via log. */
	h->tx_drops++;
	if (rc != -EAGAIN) {
		LOG_WRN("send: can_send rc=%d id=0x%x len=%u", rc, id, len);
	}
	return false;
}

static uint32_t hal_micros(void *ctx)
{
	ARG_UNUSED(ctx);
	/* Monotonic microseconds truncated to 32 bits. fcan_runtime takes
	 * differences modulo 2^32 so wraparound every ~71 minutes is fine. */
	return (uint32_t)k_ticks_to_us_floor64(k_uptime_ticks());
}

static void hal_rx_cb(const struct device *dev, struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(dev);
	struct zephyr_can_hal *h = user_data;

	const uint8_t bytes = can_dlc_to_bytes(frame->dlc);
	if (bytes > sizeof(((struct fcan_rx_item){0}).data)) {
		LOG_WRN("rx: dropping oversize frame len=%u id=0x%x", bytes, frame->id);
		return;
	}

	struct fcan_rx_item item = {
		.id = frame->id,
		.len = bytes,
		.ext = (frame->flags & CAN_FRAME_IDE) != 0,
	};
	memcpy(item.data, frame->data, bytes);

	/* Drop rather than block: RX runs on the CAN driver's rx thread. If we
	 * cannot enqueue we prefer to lose a frame over stalling the driver. */
	int rc = k_msgq_put(h->rx_msgq, &item, K_NO_WAIT);
	if (rc == 0) {
		h->rx_frames++;
	} else {
		LOG_WRN("rx: msgq full, dropping id=0x%x", frame->id);
	}
}

int zephyr_can_hal_init(struct zephyr_can_hal *h,
			const struct device *dev,
			struct k_msgq *rx_msgq)
{
	memset(h, 0, sizeof(*h));
	h->dev = dev;
	h->rx_msgq = rx_msgq;

	/* Accept every 11-bit ID. */
	const struct can_filter accept_std = {.id = 0U, .mask = 0U, .flags = 0U};
	int fid = can_add_rx_filter(dev, hal_rx_cb, h, &accept_std);
	if (fid < 0) {
		LOG_ERR("can_add_rx_filter (std) rc=%d", fid);
		return fid;
	}
	/* And every 29-bit ID — SPEC §5 uses extended IDs throughout. */
	const struct can_filter accept_ext = {
		.id = 0U, .mask = 0U, .flags = CAN_FILTER_IDE,
	};
	fid = can_add_rx_filter(dev, hal_rx_cb, h, &accept_ext);
	if (fid < 0) {
		LOG_ERR("can_add_rx_filter (ext) rc=%d", fid);
		return fid;
	}
	return 0;
}

void zephyr_can_hal_attach_node(struct zephyr_can_hal *h, fcan_node_t *node)
{
	h->node = node;
}

fcan_hal_t zephyr_can_hal_get(struct zephyr_can_hal *h)
{
	return (fcan_hal_t){
		.send = hal_send,
		.micros = hal_micros,
		.ctx = h,
	};
}

void zephyr_can_hal_drain_rx(struct zephyr_can_hal *h)
{
	if (h->node == NULL) {
		return; /* pre-fcan_init: the runtime cannot accept frames yet. */
	}

	struct fcan_rx_item item;
	while (k_msgq_get(h->rx_msgq, &item, K_NO_WAIT) == 0) {
		fcan_on_can_rx(h->node, item.id, item.ext, item.data, item.len);
	}
}
