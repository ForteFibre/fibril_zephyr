/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Router-variant tick logic. On-wire behaviour is identical to
 * samples/lib/fibril_can/latency_probe_node/src/probe_logic.c — the fcan
 * topic begin/commit/read API is seqlock-protected so it is safe to call
 * from the app thread while the router driver thread runs fcan_poll(self)
 * asynchronously.
 *
 * Firmware-requirements alignment mirrors the single-bus sample:
 *
 *   §3.1  seq / t_master_send_ns copied inside the same app tick as the ping
 *         arrival. The "arrival" event lands in router-thread context
 *         (fcan_router_on_rx forwards uplink frames into self); the app
 *         tick reads the up-to-date state via latencyprobe_ping_read.
 *   §3.2  ticks with no ping yet emit zeros so the probe can filter warmup
 *         samples by seq == 0.
 *   §3.4  periodic echo fires every tick; the runtime scheduler running
 *         inside fcan_poll(self) on the router thread handles next_due gating.
 *   §4    t_slave_*_us stamped in app-tick context — the requirement only
 *         asks for a monotonic microsecond source, not for a common clock
 *         with any other component of the pipeline.
 *
 * No heap, no logging, no floating point (§5).
 */

#include "probe_logic.h"

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/time_units.h>

#include "schema_gen.h"

#define PROBE_INST 0U

static uint32_t last_committed_seq;
static bool     any_committed;

static inline uint32_t now_us(void)
{
	return k_cyc_to_us_floor32(k_cycle_get_32());
}

void probe_logic_init(void)
{
	last_committed_seq = 0U;
	any_committed = false;
}

void probe_logic_tick(void)
{
	latencyprobe_ping_t ping;
	const bool got_ping = latencyprobe_ping_read(PROBE_INST, &ping);
	const uint32_t t_recv = got_ping ? now_us() : 0U;

	latencyprobe_echo_t *out = latencyprobe_echo_begin(PROBE_INST);
	if (out != NULL) {
		if (got_ping) {
			out->seq = ping.seq;
			out->t_master_send_ns = ping.t_master_send_ns;
			out->t_slave_recv_us = t_recv;
		} else {
			out->seq = 0U;
			out->t_master_send_ns = 0U;
			out->t_slave_recv_us = 0U;
		}
		out->t_slave_send_us = now_us();
		latencyprobe_echo_commit(PROBE_INST);
	}

	if (got_ping && (!any_committed || ping.seq != last_committed_seq)) {
		latencyprobe_echo_change_t *chg = latencyprobe_echo_change_begin(PROBE_INST);
		if (chg != NULL) {
			chg->seq = ping.seq;
			chg->t_master_send_ns = ping.t_master_send_ns;
			chg->t_slave_recv_us = t_recv;
			chg->t_slave_send_us = now_us();
			latencyprobe_echo_change_commit(PROBE_INST);
		}
		last_committed_seq = ping.seq;
		any_committed = true;
	}
}
