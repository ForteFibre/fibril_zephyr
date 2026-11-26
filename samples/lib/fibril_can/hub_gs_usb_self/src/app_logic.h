/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Toy motor + IMU application layered on top of the codegen-generated typed
 * wrappers. Keeps the schema hooked up end to end without pretending to be a
 * real control loop — the point is to demonstrate every direction:
 *   - S->M state / diagnostics publishes via <block>_<topic>_begin/commit
 *   - M->S command / brake reads via <block>_<topic>_read
 *   - Service handlers implemented as required (non-weak) symbols so a
 *     missing one produces a link error rather than a silent runtime skip
 *   - PARAM_SET notifications via the weak on_params_changed override
 */
#ifndef APP_LOGIC_H_
#define APP_LOGIC_H_

/* Reset the toy dynamics state. Call once before the main loop. */
void app_logic_init(void);

/* Advance dynamics by dt_seconds and publish updated telemetry. Call from
 * the same thread as fcan_poll(). */
void app_logic_tick(float dt_seconds);

#endif /* APP_LOGIC_H_ */
