/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr entry point for the fibril_can_benchmark latency-probe slave sample.
 * Implements the MCU firmware contract in
 * fibril_can_benchmark/docs/firmware-requirements.md; the on-wire behaviour
 * matches the reference host slave (host_latency_slave.cpp) byte-for-byte.
 *
 * Design points
 * -------------
 * - Main loop order is `drain_rx -> probe_logic_tick -> fcan_poll -> sleep`,
 *   the §3.4.1 recommendation for latency-critical firmware: change-driven
 *   echo commits during the tick are emitted by the same fcan_poll pass.
 * - Hot path holds no LOG / printf / malloc calls (§5). Status logging runs
 *   on a lower-priority thread that samples atomic HAL counters once a
 *   second.
 * - master_lost_us = 0 (§7) so the echo stream does not stall when a master
 *   heartbeat slips.
 *
 * What this sample does NOT verify
 * --------------------------------
 * - Round-trip against fibril_can_bridge / probe_node.py — that needs vcan
 *   and a ROS 2 workspace, both outside the Zephyr harness. Use the reference
 *   host slave for that setup.
 * - Real-hardware timing budgets. native_sim's cycle rate is coarse; §4.2's
 *   1 μs resolution requirement is met on the MCU targets (fdcan cycle
 *   counter runs at CPU clock) but not on native_sim.
 */

#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/time_units.h>

#include <fibril_can/fcan.h>
#include <fibril_can/fcan_protocol.h>

#include <fibril_can_zephyr/can_hal.h>

#include "schema_gen.h"
#include "probe_logic.h"

extern const uint8_t  fcan_schema_blob[];
extern const size_t   fcan_schema_blob_len;
extern const uint64_t fcan_schema_hash;

LOG_MODULE_REGISTER(fcan_latency_probe, LOG_LEVEL_INF);

#define CAN_NODE    DT_CHOSEN(zephyr_canbus)
#define CAN_BUS_DEV DEVICE_DT_GET(CAN_NODE)

/* §6: probe launch defaults --node-id to 0x20 — match it here so the sample
 * is a drop-in target for the reference launch. Override without patching
 * source by passing -DCONFIG_EXTRA_CFLAGS=-DNODE_ID=0x?? on the west build
 * line. */
#ifndef NODE_ID
#define NODE_ID     0x20
#endif

#define MASTER_LOST_US 0U  /* §7: disabled by default. */

/* §3.4.3 — higher tick rate reduces uniform[0, tick_period] detection lag.
 * 1 kHz matches the schema's echo period_us so periodic phase noise is
 * minimal; 5 kHz gave the biggest additional improvement in the README
 * "Finding" table. Override via -DTICK_HZ=5000 on the build line. */
#ifndef TICK_HZ
#define TICK_HZ     1000U
#endif

K_HEAP_DEFINE(fcan_heap, 8 * 1024);

#define FCAN_RX_QUEUE_DEPTH 32
K_MSGQ_DEFINE(fcan_rx_msgq, sizeof(struct fcan_zephyr_can_rx_item),
	      FCAN_RX_QUEUE_DEPTH, 4);

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

/* Handed to the status thread once main() finishes bring-up. Reads happen
 * from a lower-priority thread; the values themselves are word-sized so a
 * torn read is at worst a stale count in the log line. */
static fcan_node_t                *g_node;
static struct fcan_zephyr_can_hal *g_hal;

static void status_thread_entry(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	fcan_node_state_t last = FCAN_STATE_UNPROVISIONED;
	while (true) {
		if (g_node != NULL && g_hal != NULL) {
			const fcan_node_state_t s = fcan_state(g_node);
			if (s != last) {
				LOG_INF("state: %s -> %s (fault=%d)",
					state_str(last), state_str(s),
					(int)fcan_fault(g_node));
				last = s;
			}
			LOG_INF("alive: state=%s tx=%u tx_drops=%u rx=%u",
				state_str(s), g_hal->tx_frames,
				g_hal->tx_drops, g_hal->rx_frames);
		}
		k_sleep(K_SECONDS(1));
	}
}
K_THREAD_DEFINE(status_tid, 1024, status_thread_entry, NULL, NULL, NULL,
		7, 0, 0);

int main(void)
{
	const struct device *can_dev = CAN_BUS_DEV;
	static struct fcan_zephyr_can_hal hal;

	if (!device_is_ready(can_dev)) {
		LOG_ERR("CAN device %s not ready", can_dev->name);
		return -ENODEV;
	}

	int rc = can_set_mode(can_dev, CAN_MODE_FD);
	if (rc != 0) {
		LOG_ERR("can_set_mode(FD) rc=%d", rc);
		return rc;
	}

	rc = fcan_zephyr_can_hal_init(&hal, can_dev, &fcan_rx_msgq);
	if (rc != 0) {
		return rc;
	}

	rc = can_start(can_dev);
	if (rc != 0) {
		LOG_ERR("can_start rc=%d", rc);
		return rc;
	}

	fcan_node_t *node = k_heap_aligned_alloc(&fcan_heap, 8,
						 fcan_node_storage_size(),
						 K_NO_WAIT);
	if (node == NULL) {
		LOG_ERR("fcan_heap OOM allocating node (%u bytes)",
			(unsigned)fcan_node_storage_size());
		return -ENOMEM;
	}

	/* Single LatencyProbe instance matches
	 * fibril_can_benchmark/schema/latency_probe.yaml `instances: [{count:1}]`. */
	static const uint8_t counts[] = {1U};
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
		},
		.hal = fcan_zephyr_can_hal_get(&hal),
		.allocator = {.alloc = heap_alloc, .ctx = NULL},
		.master_lost_us = MASTER_LOST_US,
	};
	fcan_apply_schema_capacities(&cfg.limits, counts);

	if (fcan_init(node, &cfg) != FCAN_OK) {
		LOG_ERR("fcan_init failed (fault=%d)", (int)fcan_fault(node));
		return -EIO;
	}
	LOG_INF("fcan_init OK: node_id=0x%02x schema_hash=0x%016llx blob_len=%u tick=%u Hz",
		cfg.node_id, (unsigned long long)fcan_schema_hash,
		(unsigned)fcan_schema_blob_len, (unsigned)TICK_HZ);

	fcan_zephyr_can_hal_attach_node(&hal, node);

	if (fcan_register_all(node) != FCAN_OK) {
		LOG_ERR("fcan_register_all failed (fault=%d)",
			(int)fcan_fault(node));
		return -EIO;
	}
	LOG_INF("fcan_register_all OK");

	probe_logic_init();

	g_node = node;
	g_hal = &hal;

	LOG_INF("entering main loop");

	/* K_TIMEOUT_ABS_TICKS takes int64_t; k_ticks_t narrows to uint32_t on
	 * some configs, so keep both accumulators as int64_t. */
	const int64_t period_ticks =
		(int64_t)k_us_to_ticks_ceil64(1000000ULL / TICK_HZ);
	int64_t next = k_uptime_ticks();
	while (true) {
		fcan_zephyr_can_hal_drain_rx(&hal);
		probe_logic_tick();
		fcan_poll(node);

		next += period_ticks;
		k_sleep(K_TIMEOUT_ABS_TICKS(next));
	}
	/* unreachable */
}
