/*
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ROUTER_LATENCY_PROBE_LOGIC_H_
#define ROUTER_LATENCY_PROBE_LOGIC_H_

void probe_logic_init(void);

/* Same-tick copy of the latest ping into echo (periodic) and echo_change
 * (app-trigger). In the router variant this runs on the app thread; the
 * router driver thread runs fcan_poll(self) asynchronously (see main.c
 * for the threading model — the two loops are decoupled at
 * CONFIG_CAN_FCAN_ROUTER_POLL_INTERVAL_US granularity). */
void probe_logic_tick(void);

#endif /* ROUTER_LATENCY_PROBE_LOGIC_H_ */
