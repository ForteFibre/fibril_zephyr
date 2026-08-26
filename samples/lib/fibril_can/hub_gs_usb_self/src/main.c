/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr entry point for the fibril_can hub + self node + gs_usb sample.
 *
 * What the sample proves
 * ----------------------
 * 1. fcan_codegen'd wrappers link against the runtime on the Zephyr target.
 * 2. fcan_hub (as a "fibril,can-hub" Zephyr CAN device) hosts an
 *    embedded self node using fcan_hub_get_self_hal() /
 *    fcan_hub_attach_self() -- the three-step sequence documented in
 *    fibril_can/docs/09-hub.md.
 * 3. The hub device is registered as a gs_usb channel, so a host on USB
 *    sees a single logical CAN line that transparently combines the self
 *    node's traffic with any nodes attached to the fdcan1 peer segment.
 * 4. fcan_hub_poll() runs on the hub driver thread and drives
 *    fcan_poll(self) internally, so the app main loop only has to publish
 *    telemetry and observe diagnostics.
 *
 * What the sample does NOT prove
 * ------------------------------
 * - Round-trip provisioning against fibril_can_bridge or fibril_can_web
 *   (needs a host process; see the README for the manual bring-up steps).
 * - Hardware timing on the peer bus beyond what fdcan1's 1M/5M bitrate
 *   exercises in isolation.
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
#include <fibril_can/hub/fcan_hub.h>
#include <fibril_can_zephyr/can_hub.h>

#include "schema_gen.h"
#include "app_logic.h"
#include "rgb_state.h"
#include "usbd_setup.h"

/* The C emitter puts these symbols in schema_blob.c but doesn't declare them
 * in schema_gen.h -- the C example does the same manual extern block. */
extern const uint8_t  fcan_schema_blob[];
extern const size_t   fcan_schema_blob_len;
extern const uint64_t fcan_schema_hash;

LOG_MODULE_REGISTER(hub_gs_usb_self, LOG_LEVEL_INF);

#define HUB_NODE       DT_NODELABEL(fcan_hub)
#define HUB_DEV        DEVICE_DT_GET(HUB_NODE)

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
	case FCAN_STATE_SUSPENDED:     return "SUSPENDED";
	default:                       return "?";
	}
}

int main(void)
{
	const struct device *hub = HUB_DEV;
	const struct device *gs_usb = DEVICE_DT_GET(DT_NODELABEL(gs_usb0));

	if (!device_is_ready(hub)) {
		LOG_ERR("hub device %s not ready", hub->name);
		return -ENODEV;
	}

	fcan_hub_t *h = fcan_hub_zephyr_get(hub);
	if (h == NULL) {
		LOG_ERR("fcan_hub_zephyr_get returned NULL (hub init failed)");
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
		/* HAL comes from the hub, not from a can_hal wrapper around a
		 * physical Zephyr CAN device: the hub broadcasts self TX onto
		 * every live port, and the hub thread drives fcan_poll(self)
		 * via fcan_hub_poll(). */
		.hal = fcan_hub_get_self_hal(h),
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

	/* Attach BEFORE the hub device is started (i.e. before the host can
	 * open the gs_usb channel). fcan_hub_attach_self mutates state the
	 * hub thread reads; docs/09-hub.md documents this ordering. */
	if (fcan_hub_attach_self(h, self) != FCAN_OK) {
		LOG_ERR("fcan_hub_attach_self failed (fault=%d)",
			(int)fcan_fault(self));
		return -EIO;
	}
	LOG_INF("fcan_hub_attach_self OK");

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

	const struct device *channels[] = {hub};
	rc = gs_usb_register(gs_usb, channels, ARRAY_SIZE(channels), NULL, NULL);
	if (rc != 0) {
		LOG_ERR("gs_usb_register failed (%d)", rc);
		return rc;
	}
	LOG_INF("gs_usb_register OK (1 channel: %s)", hub->name);

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
		/* Threading model: fcan_poll(self) runs on the hub driver
		 * thread (fcan_hub_poll drives it at the end of each pass),
		 * so this main-thread app_logic_tick() must only use fcan APIs
		 * that are documented cross-thread-safe -- topic begin/commit/
		 * read and param_read are seqlock-protected per fcan_seqlock.h.
		 * Do NOT call fcan_svc_complete from here: it races the hub
		 * thread's fcan_service_poll on the reassembly slots.
		 * app_logic.c completes the `home` service synchronously to
		 * keep that invariant, at the cost of skipping the ACCEPTED
		 * demo the example_node sample carries. */
		app_logic_tick(0.01f);

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

		/* Hub-oriented heartbeat log. fcan_hub_get_diag() reports
		 * per-port fwd_count and the aggregate send-failure counter
		 * (idle slots are excluded). port 0 is the external face
		 * (gs_usb), port 1 is the fdcan1 peer segment. */
		if ((now_ms - last_log_ms) >= 1000U) {
			fcan_hub_diag_t d;
			fcan_hub_get_diag(h, &d);
			uint32_t ing_drops = fcan_hub_zephyr_ingress_drops(hub);
			LOG_INF("alive: state=%s fwd[0]=%u fwd[1]=%u "
				"to_self=%u drop_send=%u ingress_drops=%u",
				state_str(s),
				d.fwd_count[0], d.fwd_count[1],
				d.delivered_to_self,
				d.drop_send_failed, ing_drops);
			last_log_ms = now_ms;
		}

		k_msleep(10);
	}
	/* unreachable */
}
