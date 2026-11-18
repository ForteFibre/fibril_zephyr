/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */
#include <drivers/motor/vesc_protocol.h>

#include <errno.h>
#include <math.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#define VESC_CURRENT_SCALE 1000.0F
#define VESC_STATUS_CURRENT_SCALE 10.0F
#define VESC_STATUS_DUTY_SCALE 1000.0F
#define VESC_STATUS_TEMPERATURE_SCALE 10.0F
#define VESC_STATUS_POSITION_SCALE 50.0F
#define VESC_STATUS_VOLTAGE_SCALE 10.0F
#define VESC_COMM_GET_VALUES_SELECTIVE 50U
#define VESC_GET_VALUES_SELECTIVE_FAULT_MASK BIT(15)

static uint32_t packet_can_id(uint8_t packet_id, uint8_t vesc_can_id)
{
  return ((uint32_t)packet_id << 8) | vesc_can_id;
}

int vesc_encode_set_current(uint8_t vesc_can_id, float current_a, struct can_frame * frame)
{
  if ((frame == NULL) || !isfinite(current_a)) {
    return -EINVAL;
  }

  double scaled = (double)current_a * (double)VESC_CURRENT_SCALE;
  scaled = CLAMP(scaled, (double)INT32_MIN, (double)INT32_MAX);
  *frame = (struct can_frame){
    .id = packet_can_id(VESC_CAN_PACKET_SET_CURRENT, vesc_can_id),
    .flags = CAN_FRAME_IDE,
    .dlc = 4,
  };
  sys_put_be32((uint32_t)(int32_t)scaled, frame->data);
  return 0;
}

int vesc_encode_get_fault(
  uint8_t vesc_can_id, uint8_t controller_id, struct can_frame * frame)
{
  if (frame == NULL) return -EINVAL;
  *frame = (struct can_frame){
    .id = packet_can_id(VESC_CAN_PACKET_PROCESS_SHORT_BUFFER, vesc_can_id),
    .flags = CAN_FRAME_IDE,
    .dlc = 7,
  };
  frame->data[0] = controller_id;
  frame->data[1] = 0U;
  frame->data[2] = VESC_COMM_GET_VALUES_SELECTIVE;
  sys_put_be32(VESC_GET_VALUES_SELECTIVE_FAULT_MASK, &frame->data[3]);
  return 0;
}

static bool status_frame(const struct can_frame * frame, uint8_t packet, uint8_t min_dlc)
{
  return (frame != NULL) && ((frame->flags & CAN_FRAME_IDE) != 0U) &&
         ((frame->flags & CAN_FRAME_RTR) == 0U) && (frame->dlc >= min_dlc) &&
         ((frame->id >> 16) == 0U) && (((frame->id >> 8) & 0xffU) == packet);
}

bool vesc_decode_status(const struct can_frame * frame, struct vesc_status * status)
{
  if ((status == NULL) || !status_frame(frame, VESC_CAN_PACKET_STATUS, 8U)) {
    return false;
  }
  status->vesc_can_id = (uint8_t)frame->id;
  status->erpm = (float)(int32_t)sys_get_be32(frame->data);
  status->motor_current_a = (float)(int16_t)sys_get_be16(&frame->data[4]) /
                            VESC_STATUS_CURRENT_SCALE;
  status->duty = (float)(int16_t)sys_get_be16(&frame->data[6]) / VESC_STATUS_DUTY_SCALE;
  return true;
}

bool vesc_decode_status_4(const struct can_frame * frame, struct vesc_status_4 * status)
{
  if ((status == NULL) || !status_frame(frame, VESC_CAN_PACKET_STATUS_4, 8U)) {
    return false;
  }
  status->vesc_can_id = (uint8_t)frame->id;
  status->fet_temperature_c = (float)(int16_t)sys_get_be16(frame->data) /
                              VESC_STATUS_TEMPERATURE_SCALE;
  status->motor_temperature_c = (float)(int16_t)sys_get_be16(&frame->data[2]) /
                                VESC_STATUS_TEMPERATURE_SCALE;
  status->input_current_a = (float)(int16_t)sys_get_be16(&frame->data[4]) /
                            VESC_STATUS_CURRENT_SCALE;
  status->position_counts = (int32_t)(int16_t)sys_get_be16(&frame->data[6]);
  return true;
}

bool vesc_decode_status_5(const struct can_frame * frame, struct vesc_status_5 * status)
{
  if ((status == NULL) || !status_frame(frame, VESC_CAN_PACKET_STATUS_5, 6U)) {
    return false;
  }
  status->vesc_can_id = (uint8_t)frame->id;
  status->tachometer = (float)(int32_t)sys_get_be32(frame->data);
  status->input_voltage_v = (float)(int16_t)sys_get_be16(&frame->data[4]) /
                            VESC_STATUS_VOLTAGE_SCALE;
  return true;
}

bool vesc_decode_fault_status(
  const struct can_frame * frame, uint8_t controller_id, struct vesc_fault_status * status)
{
  if ((status == NULL) || !status_frame(frame, VESC_CAN_PACKET_PROCESS_SHORT_BUFFER, 8U) ||
      ((uint8_t)frame->id != controller_id) ||
      (frame->data[0] == VESC_CAN_ID_BROADCAST) || (frame->data[1] != 1U) ||
      (frame->data[2] != VESC_COMM_GET_VALUES_SELECTIVE) ||
      (sys_get_be32(&frame->data[3]) != VESC_GET_VALUES_SELECTIVE_FAULT_MASK)) {
    return false;
  }
  status->vesc_can_id = frame->data[0];
  status->fault_code = frame->data[7];
  return true;
}

int64_t vesc_position_wrap_delta(int32_t current, int32_t previous)
{
  int64_t delta = (int64_t)current - previous;
  if (delta > VESC_POSITION_HALF_TURN) {
    delta -= VESC_POSITION_COUNTS_PER_REV;
  } else if (delta < -VESC_POSITION_HALF_TURN) {
    delta += VESC_POSITION_COUNTS_PER_REV;
  }
  return delta;
}
