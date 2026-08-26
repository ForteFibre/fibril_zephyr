/*
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef LATENCY_PROBE_LOGIC_H_
#define LATENCY_PROBE_LOGIC_H_

void probe_logic_init(void);

/* Same-tick copy of the latest ping into echo (periodic) and echo_change
 * (app-trigger). Must run under the fcan runtime's owning thread — see
 * main.c for the drain_rx -> probe_logic_tick -> fcan_poll order that
 * firmware-requirements §3.4.1 recommends. */
void probe_logic_tick(void);

#endif /* LATENCY_PROBE_LOGIC_H_ */
