/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef FIBRIL_ZEPHYR_INCLUDE_DRIVERS_MOTOR_VESC_PROTOCOL_H_
#define FIBRIL_ZEPHYR_INCLUDE_DRIVERS_MOTOR_VESC_PROTOCOL_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/can.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Protocol values are isolated here because the selected VESC CAN FOC ESC's
 * manual does not identify its installed firmware. These match
 * ForteFibre/ros_vesc@716a9dc and must be checked against that firmware before
 * hardware commissioning. */
enum vesc_can_packet_id {
  VESC_CAN_PACKET_SET_CURRENT = 1,
  VESC_CAN_PACKET_PROCESS_SHORT_BUFFER = 8,
  VESC_CAN_PACKET_STATUS = 9,
  VESC_CAN_PACKET_STATUS_4 = 16,
  VESC_CAN_PACKET_STATUS_5 = 27,
};

#define VESC_POSITION_COUNTS_PER_REV 18000
#define VESC_POSITION_HALF_TURN (VESC_POSITION_COUNTS_PER_REV / 2)
#define VESC_STATUS_MASK_STATUS BIT(0)
#define VESC_STATUS_MASK_STATUS_4 BIT(1)
#define VESC_STATUS_MASK_STATUS_5 BIT(2)
#define VESC_STATUS_MASK_FAULT BIT(3)

struct vesc_status {
  uint8_t vesc_can_id;
  float erpm;
  float motor_current_a;
  float duty;
};

struct vesc_status_4 {
  uint8_t vesc_can_id;
  float fet_temperature_c;
  float motor_temperature_c;
  float input_current_a;
  int32_t position_counts;
};

struct vesc_status_5 {
  uint8_t vesc_can_id;
  float tachometer;
  float input_voltage_v;
};

struct vesc_fault_status {
  uint8_t vesc_can_id;
  uint8_t fault_code;
};

int vesc_encode_set_current(uint8_t vesc_can_id, float current_a, struct can_frame * frame);
int vesc_encode_get_fault(
  uint8_t vesc_can_id, uint8_t controller_id, struct can_frame * frame);
bool vesc_decode_status(const struct can_frame * frame, struct vesc_status * status);
bool vesc_decode_status_4(const struct can_frame * frame, struct vesc_status_4 * status);
bool vesc_decode_status_5(const struct can_frame * frame, struct vesc_status_5 * status);
bool vesc_decode_fault_status(
  const struct can_frame * frame, uint8_t controller_id, struct vesc_fault_status * status);
int64_t vesc_position_wrap_delta(int32_t current, int32_t previous);

#ifdef __cplusplus
}
#endif
#endif
