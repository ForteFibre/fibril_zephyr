/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr entry point for the fibril_can router + self node + gs_usb sample.
 *
 * What the sample proves
 * ----------------------
 * 1. fcan_codegen'd wrappers link against the runtime on the Zephyr target.
 * 2. fcan_router (as a "fibril,can-router" Zephyr CAN device) hosts an
 *    embedded self node using fcan_router_get_self_hal()/
 *    fcan_router_attach_self() -- the three-step sequence documented in
 *    fibril_can/docs/09-router.md.
 * 3. The router device is registered as a gs_usb channel, so a host on USB
 *    sees a single CAN line that transparently combines the self node's
 *    traffic with any slaves attached to the fdcan1 downlink.
 * 4. fcan_router_poll() runs on the router driver thread and drives
 *    fcan_poll(self) internally, so the app main loop only has to publish
 *    telemetry and observe diagnostics.
 *
 * What the sample does NOT prove
 * ------------------------------
 * - Round-trip provisioning against fibril_can_bridge or fibril_can_web
 *   (needs a host process; see the README for the manual bring-up steps).
 * - Hardware timing on downlinks beyond what fdcan1's 1M/5M bitrate exercises
 *   in isolation.
 */

#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>

#include <cannectivity/usb/class/gs_usb.h>

#include <fibril_can/fcan.h>
#include <fibril_can/fcan_protocol.h>
#include <fibril_can/router/fcan_router.h>
#include <fibril_can_zephyr/can_router.h>

#include "schema_gen.h"
#include "app_logic.h"
#include "rgb_state.h"
#include "usbd_setup.h"

/* The C emitter puts these symbols in schema_blob.c but doesn't declare them
 * in schema_gen.h -- the C example does the same manual extern block. */
extern const uint8_t  fcan_schema_blob[];
extern const size_t   fcan_schema_blob_len;
extern const uint64_t fcan_schema_hash;

LOG_MODULE_REGISTER(router_gs_usb_self, LOG_LEVEL_INF);

#define ROUTER_NODE    DT_NODELABEL(fcan_router)
#define ROUTER_DEV     DEVICE_DT_GET(ROUTER_NODE)

/* Default 0x10 matches the fibril_can example schema's assumed slave ID; the
 * host bridge picks this up during ANNOUNCE. Override at build time with
 * -DCONFIG_EXTRA_CFLAGS=-DNODE_ID=0x?? if you need to run more than one of
 * these boards on the same bus. */
#ifndef NODE_ID
#define NODE_ID        0x10
#endif
#define MASTER_LOST_US 300000U  /* SPEC §5.11 -- 300 ms watchdog */

/* fcan_init calls the allocator once to carve every variable-length buffer
 * the runtime needs. Same 16 KiB budget as the example_node sample; the extra
 * headroom absorbs future schema growth without a re-tune. */
K_HEAP_DEFINE(fcan_heap, 16 * 1024);

static void *heap_alloc(size_t size, size_t align, void *ctx)
{
	ARG_UNUSED(ctx);
	return k_heap_aligned_alloc(&fcan_heap, align, size, K_NO_WAIT);
}

static const char *state_str(fcan_node_state_t s)
{
	switch (s) {
	case FCAN_STATE_UNPROVISIONED: return "UNPROVISIONED";
	case FCAN_STATE_PROVISIONED:   return "PROVISIONED";
	case FCAN_STATE_RUNNING:       return "RUNNING";
	case FCAN_STATE_FAULT:         return "FAULT";
	default:                       return "?";
	}
}

int main(void)
{
	const struct device *router = ROUTER_DEV;
	const struct device *gs_usb = DEVICE_DT_GET(DT_NODELABEL(gs_usb0));

	if (!device_is_ready(router)) {
		LOG_ERR("router device %s not ready", router->name);
		return -ENODEV;
	}

	fcan_router_t *r = fcan_router_zephyr_get(router);
	if (r == NULL) {
		LOG_ERR("fcan_router_zephyr_get returned NULL (router init failed)");
		return -ENODEV;
	}

	/* fcan_node_storage_size() is a runtime function (returns
	 * sizeof(fcan_node_t) -- opaque on purpose), so we heap-allocate the
	 * node instead of trying to size a static array. 8-byte alignment
	 * satisfies the seqlock and u64-hash fields the runtime keeps inside. */
	fcan_node_t *self = k_heap_aligned_alloc(&fcan_heap, 8,
						 fcan_node_storage_size(),
						 K_NO_WAIT);
	if (self == NULL) {
		LOG_ERR("fcan_heap OOM allocating node (%u bytes)",
			(unsigned)fcan_node_storage_size());
		return -ENOMEM;
	}

	/* MotorDriver has max_count=4 in the schema; ship 1 instance to keep
	 * the RAM footprint small. Imu is fixed count=1. */
	static const uint8_t counts[] = {1U, 1U};
	BUILD_ASSERT(ARRAY_SIZE(counts) == FCAN_NUM_BLOCK_ARRAYS,
		     "counts[] must match the schema's block-array count");

	fcan_config_t cfg = {
		.node_id = NODE_ID,
		.boot_id = sys_rand32_get(),
		.schema_blob = fcan_schema_blob,
		.schema_blob_len = (uint16_t)fcan_schema_blob_len,
		.schema_hash = fcan_schema_hash,
		.protocol_version = FCAN_PROTOCOL_VERSION,
		.block_count = FCAN_NUM_BLOCK_ARRAYS,
		.instance_counts = counts,
		.limits = {
			.max_frames = FCAN_MAX_FRAMES,
			.max_copy_entries = FCAN_MAX_COPY_ENTRIES,
			.service_buffer = FCAN_SERVICE_BUFFER,
			.service_reassembly = FCAN_SERVICE_REASSEMBLY,
			/* capacity fields filled below */
		},
		/* HAL comes from the router, not from a can_hal wrapper around
		 * a physical Zephyr CAN device: the router internally proxies
		 * this HAL onto its uplink, and the router thread drives
		 * fcan_poll(self) via fcan_router_poll(). */
		.hal = fcan_router_get_self_hal(r),
		.allocator = {.alloc = heap_alloc, .ctx = NULL},
		.master_lost_us = MASTER_LOST_US,
		/* .t_listen_us = 0 -> runtime picks the safe default. */
	};
	fcan_apply_schema_capacities(&cfg.limits, counts);

	if (fcan_init(self, &cfg) != FCAN_OK) {
		LOG_ERR("fcan_init failed (fault=%d)", (int)fcan_fault(self));
		return -EIO;
	}
	LOG_INF("fcan_init OK: node_id=0x%02x schema_hash=0x%016llx blob_len=%u",
		cfg.node_id, (unsigned long long)fcan_schema_hash,
		(unsigned)fcan_schema_blob_len);

	/* Attach BEFORE the router device is started (i.e. before the host can
	 * open the gs_usb channel). fcan_router_attach_self mutates state the
	 * router thread reads; docs/09-router.md documents this ordering. */
	if (fcan_router_attach_self(r, self) != FCAN_OK) {
		LOG_ERR("fcan_router_attach_self failed (fault=%d)",
			(int)fcan_fault(self));
		return -EIO;
	}
	LOG_INF("fcan_router_attach_self OK");

	if (fcan_register_all(self) != FCAN_OK) {
		LOG_ERR("fcan_register_all failed (fault=%d)",
			(int)fcan_fault(self));
		return -EIO;
	}
	LOG_INF("fcan_register_all OK");

	app_logic_init();
	rgb_state_init();

	if (!device_is_ready(gs_usb)) {
		LOG_ERR("gs_usb device %s not ready", gs_usb->name);
		return -ENODEV;
	}

	int rc = usbd_setup_init();
	if (rc != 0) {
		LOG_ERR("usbd_setup_init failed (%d)", rc);
		return rc;
	}

	const struct device *channels[] = {router};
	rc = gs_usb_register(gs_usb, channels, ARRAY_SIZE(channels), NULL, NULL);
	if (rc != 0) {
		LOG_ERR("gs_usb_register failed (%d)", rc);
		return rc;
	}
	LOG_INF("gs_usb_register OK (1 channel: %s)", router->name);

	rc = usbd_setup_enable();
	if (rc != 0) {
		return rc;
	}
	LOG_INF("usbd_enable OK");

	LOG_INF("entering main loop (tick=1 ms, state=%s)",
		state_str(fcan_state(self)));

	fcan_node_state_t last_state = fcan_state(self);
	uint32_t last_log_ms = 0;
	uint32_t last_led_ms = 0;
	while (true) {
		/* Threading model: fcan_poll(self) runs on the router driver
		 * thread (fcan_router_poll drives it at the end of each pass),
		 * so this main-thread app_logic_tick() must only use fcan APIs
		 * that are documented cross-thread-safe -- topic begin/commit/
		 * read and param_read are seqlock-protected per fcan_seqlock.h.
		 * Do NOT call fcan_svc_complete from here: it races the router
		 * thread's fcan_service_poll on the reassembly slots.
		 * app_logic.c completes the `home` service synchronously to
		 * keep that invariant, at the cost of skipping the ACCEPTED
		 * demo the example_node sample carries. */
		app_logic_tick(0.001f);

		const fcan_node_state_t s = fcan_state(self);
		if (s != last_state) {
			LOG_INF("state: %s -> %s (fault=%d)",
				state_str(last_state), state_str(s),
				(int)fcan_fault(self));
			last_state = s;
		}

		/* LED indicator @ ~100 Hz. GPIO toggles at every 1 ms tick would
		 * be wasteful; the blink phase is derived from k_uptime so a
		 * coarser refresh does not slur the animation. */
		const uint32_t now_ms = k_uptime_get_32();
		if ((now_ms - last_led_ms) >= 10U) {
			rgb_state_update(s, fcan_fault(self));
			last_led_ms = now_ms;
		}

		/* Router-oriented heartbeat log. fcan_router_get_diag() covers
		 * both forwarding directions and every drop reason; we surface
		 * the summary counters the operator needs to spot an
		 * ingress-drop condition or a runaway downlink. */
		if ((now_ms - last_log_ms) >= 1000U) {
			fcan_router_diag_t d;
			fcan_router_get_diag(r, &d);
			uint32_t ing_drops = fcan_router_zephyr_ingress_drops(router);
			LOG_INF("alive: state=%s up->dn[0]=%u dn->up[0]=%u "
				"drop_send=%u ingress_drops=%u",
				state_str(s),
				d.fwd_up_to_down[0], d.fwd_down_to_up[0],
				d.drop_send_failed, ing_drops);
			last_log_ms = now_ms;
		}

		k_msleep(1);
	}
	/* unreachable */
}
