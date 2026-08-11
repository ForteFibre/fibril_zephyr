/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Demonstrates driving a "fibril,can-router" Zephyr CAN device.
 *
 * The production topology hangs a CANnectivity gs_usb node off the same
 * device so the router's uplink lives on USB; frames the PC sends over gs_usb
 * arrive as can_send() calls on this device, and frames the router forwards
 * toward the master are delivered to can_add_rx_filter() callbacks. See
 * docs/09-router.md in the fibril_can repository.
 *
 * With no host attached, main() plays the master. A broadcast DISCOVER
 * (SPEC §5.11) fans out to every downlink without any prior learning, so the
 * fwd_up_to_down counters advance from a cold start and prove the whole path
 * -- can_send() -> ingress msgq -> router thread -> fcan_router_on_rx ->
 * downlink can_send() -- is wired end to end.
 */

#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <fibril_can/fcan_protocol.h>
#include <fibril_can/router/fcan_router.h>
#include <fibril_can_zephyr/can_router.h>

LOG_MODULE_REGISTER(can_router_sample, LOG_LEVEL_INF);

#define ROUTER_NODE DT_NODELABEL(fcan_router)
#define DOWNLINK_COUNT DT_PROP_LEN(ROUTER_NODE, downlinks)
#define REPORT_PERIOD K_MSEC(1000)

static const struct device *const router = DEVICE_DT_GET(ROUTER_NODE);

static void uplink_rx(const struct device *dev, struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	/* Nothing sources uplink-bound frames in the standalone configuration,
	 * so this only fires under cannectivity+gs_usb (or when a downlink
	 * peer sends). The filter is here so the surface matches production. */
	LOG_INF("uplink RX: id=0x%08x dlc=%u", frame->id, frame->dlc);
}

static void report_diag(fcan_router_t *r)
{
	fcan_router_diag_t d;

	fcan_router_get_diag(r, &d);

	LOG_INF("diag: unknown_stdid=%u unknown_node=%u drop_send=%u ingress_drops=%u",
		d.drop_unknown_stdid, d.drop_unknown_node,
		d.drop_send_failed,
		fcan_router_zephyr_ingress_drops(router));

	for (int k = 0; k < DOWNLINK_COUNT; k++) {
		LOG_INF("  downlink %d: up->dn=%u dn->up=%u", k,
			d.fwd_up_to_down[k], d.fwd_down_to_up[k]);
	}
}

int main(void)
{
	fcan_router_t *r;
	int ret;

	if (!device_is_ready(router)) {
		LOG_ERR("%s not ready", router->name);
		return -ENODEV;
	}

	r = fcan_router_zephyr_get(router);
	if (r == NULL) {
		LOG_ERR("router init failed");
		return -ENODEV;
	}

	/* can_set_mode is effectively a no-op on this device (the driver forces
	 * FD internally), but calling it mirrors the sequence a well-behaved
	 * gs_usb host follows before opening the channel. */
	(void)can_set_mode(router, CAN_MODE_FD);

	ret = can_start(router);
	if (ret < 0) {
		LOG_ERR("can_start: %d", ret);
		return ret;
	}

	/* gs_usb installs one accept-all filter per format; do the same so the
	 * sample exercises both the standard and extended filter slots. */
	const struct can_filter accept_std = {.id = 0U, .mask = 0U};
	const struct can_filter accept_ext = {
		.id = 0U, .mask = 0U, .flags = CAN_FILTER_IDE,
	};

	ret = can_add_rx_filter(router, uplink_rx, NULL, &accept_std);
	if (ret < 0) {
		LOG_ERR("can_add_rx_filter (std): %d", ret);
		return ret;
	}
	ret = can_add_rx_filter(router, uplink_rx, NULL, &accept_ext);
	if (ret < 0) {
		LOG_ERR("can_add_rx_filter (ext): %d", ret);
		return ret;
	}

	const uint32_t discover_id = fcan_make_ext_id(FCAN_CHAN_CONFIG,
						      FCAN_NODE_ID_BROADCAST,
						      FCAN_CMD_DISCOVER, 0U, 0U);
	struct can_frame frame = {
		.id = discover_id,
		.flags = CAN_FRAME_FDF | CAN_FRAME_BRS | CAN_FRAME_IDE,
		.dlc = can_bytes_to_dlc(4),
		.data = {0x01, 0x02, 0x03, 0x04},
	};

	while (true) {
		ret = can_send(router, &frame, K_MSEC(100), NULL, NULL);
		if (ret < 0) {
			LOG_ERR("can_send: %d", ret);
		}
		report_diag(r);
		k_sleep(REPORT_PERIOD);
	}

	return 0;
}
