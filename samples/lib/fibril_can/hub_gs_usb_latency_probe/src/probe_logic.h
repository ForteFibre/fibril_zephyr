/*
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef HUB_LATENCY_PROBE_LOGIC_H_
#define HUB_LATENCY_PROBE_LOGIC_H_

void probe_logic_init(void);

/* Same-tick copy of the latest ping into echo (periodic) and echo_change
 * (app-trigger). In the hub variant this runs on the app thread; the
 * hub driver thread runs fcan_poll(self) asynchronously (see main.c
 * for the threading model — the two loops are decoupled at
 * CONFIG_CAN_FCAN_HUB_POLL_INTERVAL_US granularity). */
void probe_logic_tick(void);

#endif /* HUB_LATENCY_PROBE_LOGIC_H_ */
