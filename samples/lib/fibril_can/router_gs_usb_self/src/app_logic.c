/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal toy application on top of the codegen wrappers. One MotorDriver
 * instance (motor0) integrates a 1-D dynamics model driven by the last M2S
 * `command` frame, publishes S2M `state` every tick and `diagnostics` at
 * ~10 Hz. One IMU instance publishes constant gyro/accel samples so the
 * S2M path is exercised for both block types.
 *
 * Divergence from example_node/src/app_logic.c
 * --------------------------------------------
 * Unlike example_node -- which runs fcan_poll and app_logic_tick on the same
 * main thread -- the router+self topology puts fcan_poll(self) on the router
 * driver thread (via fcan_router_poll) while app_logic_tick() runs on main.
 * Topic begin/commit/read and param_read stay safe across that boundary via
 * fcan_seqlock.h. fcan_svc_complete() does NOT: it races the router thread's
 * fcan_service_poll on the reassembly slots. So the `home` service handler
 * completes synchronously here (returns FCAN_SVC_OK with a filled response)
 * instead of stashing a handle for a later motordriver_home_complete call.
 * The deferred-response path is still demonstrated by the example_node sample.
 */

#include "app_logic.h"

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/logging/log.h>

#include <fibril_can/fcan.h>
#include <fibril_can/fcan_protocol.h>

#include "schema_gen.h"

LOG_MODULE_REGISTER(fcan_app, LOG_LEVEL_INF);

#define MOTOR_INST      0U
#define IMU_INST        0U
#define DIAG_DIVIDER    100U  /* diagnostics @ 10 Hz when tick=1 kHz */

struct axis_state {
	float velocity;
	float position;
	float last_setpoint;
	uint8_t latched_mode;
	uint32_t fault_code;
	bool braked;
	uint32_t uptime_ticks;
};

static struct axis_state g_axis;

void app_logic_init(void)
{
	g_axis = (struct axis_state){0};
}

void app_logic_tick(float dt_seconds)
{
	/* 1. Absorb any queued master commands. */
	motordriver_command_t cmd;
	if (motordriver_command_read(MOTOR_INST, &cmd)) {
		g_axis.last_setpoint = cmd.setpoint;
		g_axis.latched_mode = cmd.mode;
	}
	motordriver_brake_t brk;
	if (motordriver_brake_read(MOTOR_INST, &brk)) {
		g_axis.braked = brk.engaged;
	}

	/* 2. Toy dynamics. Params are master-owned; we only read them. */
	const float kp = motordriver_param_kp(MOTOR_INST);
	const float torque_constant = motordriver_param_torque_constant(MOTOR_INST);
	const float drive = g_axis.braked ? 0.0f : kp * g_axis.last_setpoint;
	g_axis.velocity += drive * dt_seconds;
	g_axis.position += g_axis.velocity * dt_seconds;

	/* 3. High-rate telemetry. */
	motordriver_state_t *state = motordriver_state_begin(MOTOR_INST);
	if (state != NULL) {
		state->velocity = g_axis.velocity;
		state->position = g_axis.position;
		state->current = (torque_constant > 0.0f) ? drive / torque_constant : 0.0f;
		state->torque = drive;
		state->output_voltage = drive;
		state->bus_voltage = 24.0f;
		state->temperature = 25.0f;
		state->error_flags = g_axis.fault_code;
		motordriver_state_commit(MOTOR_INST);
	}

	/* 4. Low-rate diagnostics. No deferred-response completion here (see the
	 *    file header): motordriver_home returns synchronously in this sample
	 *    so nothing needs to complete a stashed handle from the app thread. */
	g_axis.uptime_ticks++;
	if ((g_axis.uptime_ticks % DIAG_DIVIDER) == 0U) {
		motordriver_diagnostics_t *diag = motordriver_diagnostics_begin(MOTOR_INST);
		if (diag != NULL) {
			diag->fault_code = g_axis.fault_code;
			diag->uptime_s = (uint32_t)((float)g_axis.uptime_ticks * dt_seconds);
			diag->mcu_temperature = 30.0f;
			diag->mode = g_axis.latched_mode;
			motordriver_diagnostics_commit(MOTOR_INST);
		}
	}

	/* 5. Constant IMU sample so the Imu S2M path is not dead code. */
	imu_sample_t *imu = imu_sample_begin(IMU_INST);
	if (imu != NULL) {
		imu->gyro[0] = 0.0f;
		imu->gyro[1] = 0.0f;
		imu->gyro[2] = 0.0f;
		imu->accel[0] = 0.0f;
		imu->accel[1] = 0.0f;
		imu->accel[2] = 9.81f;
		imu_sample_commit(IMU_INST);
	}
}

/* ---- Service handlers. codegen emits a dispatcher that calls each of these
 *      by symbol; any omission is a hard link error. See
 *      fibril_can_example/src/app_logic.cpp for the reference implementations. */

fcan_svc_status_t motordriver_set_mode(uint8_t inst,
				       const motordriver_set_mode_req_t *req,
				       motordriver_set_mode_resp_t *resp,
				       fcan_call_handle_t h)
{
	ARG_UNUSED(h);
	if (inst == MOTOR_INST) {
		g_axis.latched_mode = req->mode;
	}
	resp->accepted = true;
	return FCAN_SVC_OK;
}

fcan_svc_status_t motordriver_set_pid(uint8_t inst,
				      const motordriver_set_pid_req_t *req,
				      motordriver_set_pid_resp_t *resp,
				      fcan_call_handle_t h)
{
	ARG_UNUSED(inst);
	ARG_UNUSED(req);
	ARG_UNUSED(h);
	/* Canonical PID gain updates flow through PARAM_SET (§8). This handler
	 * exists so the schema's service surface is complete but does not
	 * touch the working values. */
	resp->accepted = true;
	return FCAN_SVC_OK;
}

fcan_svc_status_t motordriver_calibrate(uint8_t inst,
					const motordriver_calibrate_req_t *req,
					motordriver_calibrate_resp_t *resp,
					fcan_call_handle_t h)
{
	ARG_UNUSED(req);
	ARG_UNUSED(h);
	resp->offset = (inst == MOTOR_INST) ? g_axis.position : 0.0f;
	return FCAN_SVC_OK;
}

fcan_svc_status_t motordriver_clear_faults(uint8_t inst,
					   const motordriver_clear_faults_req_t *req,
					   motordriver_clear_faults_resp_t *resp,
					   fcan_call_handle_t h)
{
	ARG_UNUSED(req);
	ARG_UNUSED(h);
	if (inst == MOTOR_INST) {
		g_axis.fault_code = 0;
	}
	resp->success = true;
	return FCAN_SVC_OK;
}

fcan_svc_status_t motordriver_home(uint8_t inst,
				   const motordriver_home_req_t *req,
				   motordriver_home_resp_t *resp,
				   fcan_call_handle_t h)
{
	ARG_UNUSED(req);
	ARG_UNUSED(h);
	if (inst == MOTOR_INST) {
		g_axis.position = 0.0f;
		g_axis.velocity = 0.0f;
	}
	/* Synchronous completion: fill the response and return OK so the runtime
	 * sends it from this call's context (router driver thread). The
	 * deferred-response path (FCAN_SVC_ACCEPTED + motordriver_home_complete
	 * later) would trip on the router-thread vs app-thread race documented at
	 * the top of this file. */
	resp->success = true;
	return FCAN_SVC_OK;
}

void motordriver_on_params_changed(uint8_t inst, uint32_t changed_mask)
{
	LOG_INF("PARAM_SET applied: inst=%u mask=0x%08x kp=%f",
		inst, changed_mask, (double)motordriver_param_kp(inst));
}
