/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * On-tick copy of the received ping into the two S2M echo streams. Wire-visible
 * behaviour tracks fibril_can_benchmark/docs/firmware-requirements.md:
 *
 *   §3.1  seq / t_master_send_ns are copied inside the same tick as the ping
 *         arrival — no cross-tick buffering.
 *   §3.2  ticks with no ping yet emit zeros so the probe can filter warmup
 *         samples by seq == 0.
 *   §3.3  latencyprobe_ping_read() collapses multiple frames per tick to the
 *         last one; nothing extra to do here.
 *   §3.4  periodic echo fires every tick regardless of ping arrival; the
 *         runtime scheduler handles the actual next_due gate.
 *   §4    t_slave_recv_us is stamped as soon as we know we have a fresh ping;
 *         t_slave_send_us is stamped immediately before commit.
 *
 * No heap allocation, no logging, no floating point — this runs on the hot
 * path and firmware-requirements §5 forbids all three.
 */

#include "probe_logic.h"

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/time_units.h>

#include "schema_gen.h"

#define PROBE_INST 0U

/* Track the last seq we committed to echo_change so we only fire the
 * app-trigger stream when something new arrives. `any_committed` guards the
 * first ping (seq could legitimately be 0). */
static uint32_t last_committed_seq;
static bool     any_committed;

/* Monotonic microseconds. On a real MCU the DWT / SysTick cycle counter
 * runs at CPU frequency and easily satisfies §4.2's 1 μs resolution; on
 * native_sim the cycle rate is lower — good enough for smoke testing but
 * do not treat those samples as production timing.
 *
 * The 32-bit wrap (~71 min) is intentional — §4.2 states the probe side
 * handles wraparound in its differences, so the slave just hands over the
 * low 32 bits. */
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
