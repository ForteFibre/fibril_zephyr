/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */
#include <math.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can/can_fake.h>
#include <zephyr/fff.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>
#include <drivers/encoder.h>
#include <drivers/motor.h>
#include <drivers/motor/vesc.h>
#include <drivers/motor/vesc_protocol.h>

#define CAPTURE_MAX 256
static const struct device * const can_dev = DEVICE_DT_GET(DT_NODELABEL(test_can));
static const struct device * const motor0 = DEVICE_DT_GET(DT_NODELABEL(motor0));
static const struct device * const motor1 = DEVICE_DT_GET(DT_NODELABEL(motor1));
static const struct device * const enc0 = DEVICE_DT_GET(DT_NODELABEL(enc0));
static can_rx_callback_t rx_callback;
static void * rx_user_data;
static struct can_frame captured[CAPTURE_MAX];
static size_t captured_count;
DEFINE_FFF_GLOBALS;

static int add_filter(const struct device * dev, can_rx_callback_t callback, void * user_data,
                      const struct can_filter * filter)
{
  ARG_UNUSED(filter);
  if (dev != can_dev) return -EINVAL;
  rx_callback = callback; rx_user_data = user_data; return 1;
}

static int send_frame(const struct device * dev, const struct can_frame * frame, k_timeout_t timeout,
                      can_tx_callback_t callback, void * user_data)
{
  ARG_UNUSED(timeout);
  if (dev != can_dev) return -EINVAL;
  if (captured_count < CAPTURE_MAX) captured[captured_count] = *frame;
  captured_count++;
  if (callback != NULL) callback(dev, 0, user_data);
  return 0;
}

static int start_can(const struct device * dev) { return dev == can_dev ? 0 : -EINVAL; }

static void install_fakes(void)
{
  fake_can_add_rx_filter_fake.custom_fake = add_filter;
  fake_can_send_fake.custom_fake = send_frame;
  fake_can_start_fake.custom_fake = start_can;
}

static int fake_init(void) { install_fakes(); return 0; }
SYS_INIT(fake_init, PRE_KERNEL_1, 0);

static void inject(struct can_frame * frame)
{
  zassert_not_null(rx_callback);
  rx_callback(can_dev, frame, rx_user_data);
}

static void inject_status(uint8_t id)
{
  struct can_frame frame = {.id = ((uint32_t)VESC_CAN_PACKET_STATUS << 8) | id,
                            .flags = CAN_FRAME_IDE, .dlc = 8};
  sys_put_be32(1200, frame.data); sys_put_be16(25, &frame.data[4]);
  sys_put_be16(100, &frame.data[6]); inject(&frame);
}

static void inject_position(uint8_t id, int16_t counts)
{
  struct can_frame frame = {.id = ((uint32_t)VESC_CAN_PACKET_STATUS_4 << 8) | id,
                            .flags = CAN_FRAME_IDE, .dlc = 8};
  sys_put_be16((uint16_t)counts, &frame.data[6]); inject(&frame);
}

static void inject_fault(uint8_t id, uint8_t fault)
{
  struct can_frame frame = {.id = ((uint32_t)VESC_CAN_PACKET_PROCESS_SHORT_BUFFER << 8) | id,
                            .flags = CAN_FRAME_IDE, .dlc = 8};
  frame.data[0] = 255; frame.data[1] = 1; frame.data[2] = 50;
  sys_put_be32(BIT(15), &frame.data[3]); frame.data[7] = fault; inject(&frame);
}

static struct vesc_slot_runtime_config config(int id, float max_current, uint32_t timeout,
                                               uint8_t reconnect)
{
  return (struct vesc_slot_runtime_config){id, max_current, timeout, reconnect, 1000};
}

static int32_t frame_current(const struct can_frame * frame)
{
  return (int32_t)sys_get_be32(frame->data);
}

static const struct can_frame * last_for(uint8_t id)
{
  for (size_t i = MIN(captured_count, (size_t)CAPTURE_MAX); i > 0; --i) {
    if ((captured[i - 1].id & 0xffffU) == (((uint32_t)VESC_CAN_PACKET_SET_CURRENT << 8) | id))
      return &captured[i - 1];
  }
  return NULL;
}

static void before(void * fixture)
{
  ARG_UNUSED(fixture); install_fakes(); captured_count = 0;
  struct vesc_slot_runtime_config off = config(-1, 1.0F, 20, 3);
  zassert_ok(vesc_configure_slot(motor0, &off));
  zassert_ok(vesc_configure_slot(motor1, &off));
}

ZTEST_SUITE(vesc, NULL, NULL, before, NULL, NULL);

ZTEST(vesc, test_protocol_current_id_sign_and_saturation)
{
  struct can_frame frame;
  zassert_ok(vesc_encode_set_current(201, -2.5F, &frame));
  zassert_equal(frame.id, ((uint32_t)VESC_CAN_PACKET_SET_CURRENT << 8) | 201);
  zassert_true(frame.flags & CAN_FRAME_IDE);
  zassert_equal(frame_current(&frame), -2500);
  zassert_equal(vesc_encode_set_current(1, INFINITY, &frame), -EINVAL);
}

ZTEST(vesc, test_assignment_rejects_duplicate_and_invalid_without_losing_old_config)
{
  struct vesc_bus_diagnostics before_diag;
  zassert_ok(vesc_get_bus_diagnostics(motor0, &before_diag));
  struct vesc_slot_runtime_config a = config(37, 5.0F, 20, 3);
  struct vesc_slot_runtime_config b = config(37, 5.0F, 20, 3);
  zassert_ok(vesc_configure_slot(motor0, &a));
  zassert_equal(vesc_configure_slot(motor1, &b), -EINVAL);
  b = config(256, 5.0F, 20, 3);
  zassert_equal(vesc_configure_slot(motor0, &b), -EINVAL);
  b = config(38, 11.0F, 20, 3);
  zassert_equal(vesc_configure_slot(motor0, &b), -EINVAL);
  struct vesc_slot_runtime_config applied;
  zassert_ok(vesc_get_slot_config(motor0, &applied));
  zassert_equal(applied.vesc_can_id, 37);
  struct vesc_bus_diagnostics diag;
  zassert_ok(vesc_get_bus_diagnostics(motor0, &diag));
  zassert_equal(diag.config_error_count, before_diag.config_error_count + 3);
}

ZTEST(vesc, test_id_change_sends_zero_to_old_id_and_unassigned_sends_nothing)
{
  struct vesc_slot_runtime_config c = config(37, 5.0F, 20, 1);
  zassert_ok(vesc_configure_slot(motor0, &c)); inject_status(37);
  struct vesc_slot_diagnostics before_diag;
  zassert_ok(vesc_get_slot_diagnostics(motor0, &before_diag));
  zassert_ok(motor_enable(motor0)); zassert_ok(motor_set_output(motor0, MOTOR_OUTPUT_MODE_CURRENT, 2));
  k_msleep(3); captured_count = 0;
  c.vesc_can_id = 201; zassert_ok(vesc_configure_slot(motor0, &c));
  zassert_not_null(last_for(37)); zassert_equal(frame_current(last_for(37)), 0);
  captured_count = 0; c.vesc_can_id = -1; zassert_ok(vesc_configure_slot(motor0, &c));
  k_msleep(3); zassert_not_null(last_for(201));
  zassert_equal(frame_current(last_for(201)), 0, "unassign did not stop old ID");
  struct vesc_slot_diagnostics after_diag;
  zassert_ok(vesc_get_slot_diagnostics(motor0, &after_diag));
  zassert_equal(after_diag.can_id_change_count, before_diag.can_id_change_count + 2);
}

ZTEST(vesc, test_unknown_id_is_observed_but_not_controlled_and_bitmaps_match)
{
  struct vesc_slot_runtime_config c = config(37, 5.0F, 20, 1);
  zassert_ok(vesc_configure_slot(motor0, &c)); inject_status(99); k_msleep(2);
  struct vesc_bus_diagnostics d; zassert_ok(vesc_get_bus_diagnostics(motor0, &d));
  zassert_true(d.observed_ids[99 / 32] & BIT(99 % 32));
  zassert_true(d.unexpected_ids[99 / 32] & BIT(99 % 32));
  zassert_true(d.missing_ids[37 / 32] & BIT(37 % 32));
  zassert_is_null(last_for(99));
}

ZTEST(vesc, test_wrap_forward_reverse_and_exact_half_turn)
{
  zassert_equal(vesc_position_wrap_delta(10, 17990), 20);
  zassert_equal(vesc_position_wrap_delta(17990, 10), -20);
  zassert_equal(vesc_position_wrap_delta(9000, 0), 9000);
  zassert_equal(vesc_position_wrap_delta(0, 9000), -9000);
}

ZTEST(vesc, test_encoder_keeps_position_across_timeout_and_reconnect)
{
  struct vesc_slot_runtime_config c = config(37, 5.0F, 10, 1);
  zassert_ok(vesc_configure_slot(motor0, &c)); inject_position(37, 17990);
  inject_position(37, 10); struct encoder_feedback fb;
  zassert_ok(encoder_get_feedback(enc0, &fb)); zassert_equal(fb.position, 18010);
  k_msleep(12); inject_position(37, 20);
  zassert_ok(encoder_get_feedback(enc0, &fb)); zassert_equal(fb.position, 18020);
}

ZTEST(vesc, test_timeout_reconnect_requires_frames_and_new_enable)
{
  struct vesc_slot_runtime_config c = config(37, 5.0F, 30, 3);
  zassert_ok(vesc_configure_slot(motor0, &c));
  struct vesc_slot_diagnostics before_diag;
  zassert_ok(vesc_get_slot_diagnostics(motor0, &before_diag));
  inject_status(37); inject_status(37); zassert_equal(motor_enable(motor0), -EAGAIN);
  inject_status(37); zassert_ok(motor_enable(motor0));
  zassert_ok(motor_set_output(motor0, MOTOR_OUTPUT_MODE_CURRENT, 12.0F)); k_msleep(3);
  zassert_equal(frame_current(last_for(37)), 10000, "absolute driver current clamp");
  zassert_ok(motor_set_output(motor0, MOTOR_OUTPUT_MODE_CURRENT, -12.0F)); k_msleep(3);
  zassert_equal(frame_current(last_for(37)), -10000, "negative current clamp");
  k_msleep(35); zassert_equal(frame_current(last_for(37)), 0);
  struct vesc_bus_diagnostics bus_diag;
  zassert_ok(vesc_get_bus_diagnostics(motor0, &bus_diag));
  zassert_false(bus_diag.online_ids[37 / 32] & BIT(37 % 32));
  zassert_true(bus_diag.stale_ids[37 / 32] & BIT(37 % 32));
  zassert_true(bus_diag.missing_ids[37 / 32] & BIT(37 % 32));
  inject_status(37); inject_status(37); inject_status(37);
  zassert_ok(motor_set_output(motor0, MOTOR_OUTPUT_MODE_CURRENT, 2.0F)); k_msleep(3);
  zassert_equal(frame_current(last_for(37)), 0, "restarted without a new target/enable");
  zassert_ok(motor_enable(motor0)); k_msleep(3);
  zassert_equal(frame_current(last_for(37)), 2000);
  struct vesc_slot_diagnostics slot_diag;
  zassert_ok(vesc_get_slot_diagnostics(motor0, &slot_diag));
  zassert_equal(slot_diag.timeout_count, before_diag.timeout_count + 1);
  zassert_equal(slot_diag.reconnect_count, before_diag.reconnect_count + 1);
}

ZTEST(vesc, test_fault_stops_and_is_counted)
{
  struct vesc_slot_runtime_config c = config(37, 5.0F, 20, 1);
  zassert_ok(vesc_configure_slot(motor0, &c)); inject_status(37);
  zassert_ok(motor_enable(motor0)); zassert_ok(motor_set_output(motor0, MOTOR_OUTPUT_MODE_CURRENT, 2));
  k_msleep(2); inject_fault(37, 3); k_msleep(2);
  zassert_equal(frame_current(last_for(37)), 0);
  struct vesc_slot_diagnostics d; zassert_ok(vesc_get_slot_diagnostics(motor0, &d));
  zassert_equal(d.fault_code, 3); zassert_equal(d.fault_count, 1);
}
