/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Optional visual state indicator for the fibril_can example_node. Maps the
 * fcan_node runtime state (and fault code) onto the board's on-board LEDs so
 * a technician can see what the slave is doing without wiring up a serial
 * console.
 *
 * Boards that expose `led_r`, `led_g`, `led_b` gpio-leds children (currently:
 * fibril_robomaster_miniv1) drive the indicator. Boards that don't -- notably
 * native_sim -- fall back to a no-op implementation so the sample keeps
 * building unchanged.
 */

#ifndef FCAN_EXAMPLE_RGB_STATE_H_
#define FCAN_EXAMPLE_RGB_STATE_H_

#include <fibril_can/fcan_protocol.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Configure the R/G/B outputs and drive them low (all off). Idempotent; safe
 * to call before the runtime is initialised. On boards without the three LED
 * nodes this is a no-op. */
void rgb_state_init(void);

/* Refresh the LED output for the given fcan runtime snapshot. Intended to be
 * called from the main loop at 100 Hz-ish; the internal blink phase is driven
 * from k_uptime_get_32() so callers do not need to keep any state.
 *
 * Priority: any non-NONE fault turns red on regardless of state. Otherwise:
 *   UNPROVISIONED -> blue, blinking at 1 Hz (500 ms on, 500 ms off)
 *   PROVISIONED   -> blue, solid
 *   RUNNING       -> green, solid
 *   FAULT         -> red, solid   (also reached via the fault-code branch)
 */
void rgb_state_update(fcan_node_state_t state, fcan_fault_t fault);

#ifdef __cplusplus
}
#endif

#endif /* FCAN_EXAMPLE_RGB_STATE_H_ */
