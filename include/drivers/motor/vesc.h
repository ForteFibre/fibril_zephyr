/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef FIBRIL_ZEPHYR_INCLUDE_DRIVERS_MOTOR_VESC_H_
#define FIBRIL_ZEPHYR_INCLUDE_DRIVERS_MOTOR_VESC_H_

#include <drivers/motor.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file
 * @brief Runtime binding and diagnostics for the VESC CAN driver.
 * @defgroup motor_vesc VESC CAN driver
 * @ingroup drivers
 * @{
 */

#define VESC_MOTOR_SLOT_COUNT 8U
#define VESC_CAN_ID_UNASSIGNED (-1)
#define VESC_ID_BITMAP_WORDS 8U

struct vesc_slot_runtime_config {
  int16_t vesc_can_id;
  float max_current_a;
  uint32_t feedback_timeout_ms;
  uint8_t reconnect_frames;
  uint32_t command_period_us;
};

struct vesc_slot_diagnostics {
  int16_t requested_can_id;
  int16_t applied_can_id;
  bool slot_configured;
  bool last_config_request_ok;
  bool online;
  bool stale;
  uint32_t last_rx_age_ms;
  uint32_t status_mask;
  uint32_t rx_count;
  uint32_t timeout_count;
  uint32_t reconnect_count;
  uint32_t can_id_change_count;
  uint8_t fault_code;
  uint32_t fault_count;
  float motor_current_a;
  float input_current_a;
  float erpm;
  float duty;
  float input_voltage_v;
  float fet_temperature_c;
  float motor_temperature_c;
};

struct vesc_bus_diagnostics {
  uint32_t observed_ids[VESC_ID_BITMAP_WORDS];
  uint32_t online_ids[VESC_ID_BITMAP_WORDS];
  uint32_t stale_ids[VESC_ID_BITMAP_WORDS];
  uint32_t assigned_ids[VESC_ID_BITMAP_WORDS];
  uint32_t unexpected_ids[VESC_ID_BITMAP_WORDS];
  uint32_t missing_ids[VESC_ID_BITMAP_WORDS];
  uint32_t rx_count;
  uint32_t tx_count;
  uint32_t tx_error_count;
  uint32_t invalid_rx_count;
  uint32_t config_error_count;
};

typedef void (*vesc_position_listener_t)(
  const struct device * motor, int32_t position_counts, uint32_t cycle, void * user_data);

int vesc_configure_slot(
  const struct device * motor, const struct vesc_slot_runtime_config * config);
int vesc_configure_bus_timeout(const struct device * motor, uint32_t feedback_timeout_ms);
int vesc_get_slot_config(
  const struct device * motor, struct vesc_slot_runtime_config * config);
int vesc_get_slot_diagnostics(
  const struct device * motor, struct vesc_slot_diagnostics * diagnostics);
int vesc_get_bus_diagnostics(
  const struct device * motor, struct vesc_bus_diagnostics * diagnostics);
int vesc_set_position_listener(
  const struct device * motor, vesc_position_listener_t listener, void * user_data);

/** @} */

#ifdef __cplusplus
}
#endif
#endif
