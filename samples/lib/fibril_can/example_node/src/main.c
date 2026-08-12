/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr entry point for the fibril_can codegen-driven slave sample.
 *
 * What the sample proves
 * ----------------------
 * 1. fcan_codegen produced schema_gen.{h,c} + schema_blob.c match the runtime
 *    ABI on the Zephyr target (no host-only assumptions leak in).
 * 2. fcan_init consumes the codegen's FCAN_* macros + apply_schema_capacities
 *    and returns FCAN_OK.
 * 3. fcan_register_all wires every topic/param/service without a link error
 *    — every user-implemented service handler in app_logic.c must be present
 *    or this file will fail to link.
 * 4. fcan_poll drives HEARTBEAT emissions and the state machine forward while
 *    the app_logic tick publishes S2M telemetry (visible via the HAL's
 *    tx_frames counter). A CAN master is not required for this to be a
 *    meaningful smoke test — SPEC §5.3 keeps the slave in UNPROVISIONED until
 *    an ANNOUNCE_ACK arrives, which never comes on a bare loopback bus.
 *
 * What the sample does NOT prove
 * ------------------------------
 * - Round-trip provisioning against fibril_can_bridge (needs vcan + a ROS 2
 *   bridge process; see docs/02-getting-started.md in the fibril_can repo).
 * - Timing correctness on real FDCAN hardware. can_loopback drops TX on the
 *   floor unless CAN_MODE_LOOPBACK is set, which we intentionally leave off
 *   to keep the state machine deterministic on native_sim.
 */

#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>

#include <fibril_can/fcan.h>
#include <fibril_can/fcan_protocol.h>

#include "schema_gen.h"
#include "zephyr_can_hal.h"
#include "app_logic.h"

/* The C emitter puts these symbols in schema_blob.c but doesn't declare them
 * in schema_gen.h — the C example does the same manual extern block. */
extern const uint8_t  fcan_schema_blob[];
extern const size_t   fcan_schema_blob_len;
extern const uint64_t fcan_schema_hash;

LOG_MODULE_REGISTER(fcan_example_node, LOG_LEVEL_INF);

#define CAN_NODE       DT_CHOSEN(zephyr_canbus)
#define CAN_BUS_DEV    DEVICE_DT_GET(CAN_NODE)

/* SPEC §5.3: node_id is a firmware-owned decision. Default 0x10 keeps this
 * sample self-contained; override without patching the source by passing
 * -DCONFIG_EXTRA_CFLAGS=-DNODE_ID=0x?? on the west build line, or by adding
 * the same via a board-specific overlay conf. */
#ifndef NODE_ID
#define NODE_ID        0x10
#endif
#define MASTER_LOST_US 300000U  /* §5.11 — 300 ms watchdog */

/* fcan_init calls the allocator once to carve every variable-length buffer
 * the runtime needs. Size the arena from a rough upper bound: 1 MotorDriver +
 * 1 Imu with service_buffer=0 fits comfortably in a few kilobytes; the extra
 * headroom absorbs future schema growth without a re-tune. */
K_HEAP_DEFINE(fcan_heap, 16 * 1024);

/* Enough slots for a burst of provisioning frames arriving faster than main
 * can drain them. Loss on overflow is preferable to blocking the CAN driver
 * thread, and the runtime tolerates dropped frames — masters retry. */
#define FCAN_RX_QUEUE_DEPTH 32
K_MSGQ_DEFINE(fcan_rx_msgq, sizeof(struct fcan_rx_item), FCAN_RX_QUEUE_DEPTH, 4);

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
	const struct device *can_dev = CAN_BUS_DEV;
	static struct zephyr_can_hal hal;

	if (!device_is_ready(can_dev)) {
		LOG_ERR("CAN device %s not ready", can_dev->name);
		return -ENODEV;
	}

	int rc = can_set_mode(can_dev, CAN_MODE_FD);
	if (rc != 0) {
		LOG_ERR("can_set_mode(FD) rc=%d", rc);
		return rc;
	}

	rc = zephyr_can_hal_init(&hal, can_dev, &fcan_rx_msgq);
	if (rc != 0) {
		return rc;
	}

	rc = can_start(can_dev);
	if (rc != 0) {
		LOG_ERR("can_start rc=%d", rc);
		return rc;
	}

	/* fcan_node_storage_size() is a runtime function (returns
	 * sizeof(fcan_node_t) — opaque on purpose), so we heap-allocate the
	 * node instead of trying to size a static array. 8-byte alignment
	 * satisfies the seqlock and u64-hash fields the runtime keeps inside. */
	fcan_node_t *node = k_heap_aligned_alloc(&fcan_heap, 8,
						 fcan_node_storage_size(),
						 K_NO_WAIT);
	if (node == NULL) {
		LOG_ERR("fcan_heap OOM allocating node (%u bytes)",
			(unsigned)fcan_node_storage_size());
		return -ENOMEM;
	}

	/* MotorDriver has max_count=4 in the schema; ship 1 instance to keep
	 * the RAM footprint small on native_sim. Imu is fixed count=1. */
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
		.hal = zephyr_can_hal_get(&hal),
		.allocator = {.alloc = heap_alloc, .ctx = NULL},
		.master_lost_us = MASTER_LOST_US,
		/* .t_listen_us = 0 -> runtime picks the safe default. */
	};
	fcan_apply_schema_capacities(&cfg.limits, counts);

	if (fcan_init(node, &cfg) != FCAN_OK) {
		LOG_ERR("fcan_init failed (fault=%d)", (int)fcan_fault(node));
		return -EIO;
	}
	LOG_INF("fcan_init OK: node_id=0x%02x schema_hash=0x%016llx blob_len=%u",
		cfg.node_id, (unsigned long long)fcan_schema_hash,
		(unsigned)fcan_schema_blob_len);

	zephyr_can_hal_attach_node(&hal, node);

	if (fcan_register_all(node) != FCAN_OK) {
		LOG_ERR("fcan_register_all failed (fault=%d)", (int)fcan_fault(node));
		return -EIO;
	}
	LOG_INF("fcan_register_all OK");

	app_logic_init();

	LOG_INF("entering main loop (tick=1 ms, state=%s)",
		state_str(fcan_state(node)));

	fcan_node_state_t last_state = fcan_state(node);
	uint32_t last_log_ms = 0;
	while (true) {
		/* Drain driver-thread deliveries onto this thread BEFORE polling.
		 * Everything the runtime sees on RX comes through this queue, so
		 * fcan_on_can_rx() never races fcan_poll(). */
		zephyr_can_hal_drain_rx(&hal);
		fcan_poll(node);
		app_logic_tick(0.001f);

		const fcan_node_state_t s = fcan_state(node);
		if (s != last_state) {
			LOG_INF("state: %s -> %s (fault=%d)",
				state_str(last_state), state_str(s),
				(int)fcan_fault(node));
			last_state = s;
		}

		/* Heartbeat log so the sample provides an obvious "still alive"
		 * signal on a bus with no master. Rate-limit to once a second. */
		const uint32_t now_ms = k_uptime_get_32();
		if ((now_ms - last_log_ms) >= 1000U) {
			LOG_INF("alive: state=%s tx=%u tx_drops=%u rx=%u",
				state_str(s), hal.tx_frames, hal.tx_drops,
				hal.rx_frames);
			last_log_ms = now_ms;
		}

		k_msleep(1);
	}
	/* unreachable */
}
