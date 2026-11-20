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
static int tx_complete_error;
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
  if (callback != NULL) callback(dev, tx_complete_error, user_data);
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
  struct can_frame frame = {.id = ((uint32_t)VESC_CAN_PACKET_PROCESS_SHORT_BUFFER << 8) | 254,
                            .flags = CAN_FRAME_IDE, .dlc = 8};
  frame.data[0] = id; frame.data[1] = 1; frame.data[2] = 50;
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

static bool wait_for_current(uint8_t id, int32_t expected_ma, uint8_t timeout_ms)
{
  for (uint8_t elapsed = 0U; elapsed < timeout_ms; ++elapsed) {
    const struct can_frame * frame = last_for(id);
    if ((frame != NULL) && (frame_current(frame) == expected_ma)) return true;
    inject_status(id);
    k_msleep(1);
  }
  return false;
}

static void before(void * fixture)
{
  ARG_UNUSED(fixture); install_fakes(); captured_count = 0;
  tx_complete_error = 0;
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
  zassert_equal(vesc_encode_set_current(254, 0.0F, &frame), -EINVAL);
  zassert_equal(vesc_encode_set_current(255, 0.0F, &frame), -EINVAL);
}

ZTEST(vesc, test_fault_request_and_reply_use_dedicated_controller_id)
{
  struct can_frame frame;
  zassert_ok(vesc_encode_get_fault(37, 254, &frame));
  zassert_equal(frame.id, ((uint32_t)VESC_CAN_PACKET_PROCESS_SHORT_BUFFER << 8) | 37);
  zassert_equal(frame.data[0], 254);
  zassert_equal(vesc_encode_get_fault(37, 255, &frame), -EINVAL);
  zassert_equal(vesc_encode_get_fault(254, 254, &frame), -EINVAL);

  frame = (struct can_frame){
    .id = ((uint32_t)VESC_CAN_PACKET_PROCESS_SHORT_BUFFER << 8) | 254,
    .flags = CAN_FRAME_IDE,
    .dlc = 8,
  };
  frame.data[0] = 37;
  frame.data[1] = 1;
  frame.data[2] = 50;
  sys_put_be32(BIT(15), &frame.data[3]);
  frame.data[7] = 3;
  struct vesc_fault_status status;
  zassert_true(vesc_decode_fault_status(&frame, 254, &status));
  zassert_equal(status.vesc_can_id, 37);
  zassert_equal(status.fault_code, 3);
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
  b = config(254, 5.0F, 20, 3);
  zassert_equal(vesc_configure_slot(motor0, &b), -EINVAL);
  b = config(255, 5.0F, 20, 3);
  zassert_equal(vesc_configure_slot(motor0, &b), -EINVAL);
  b = config(38, 11.0F, 20, 3);
  zassert_equal(vesc_configure_slot(motor0, &b), -EINVAL);
  b = config(38, 5.0F, 20, 3); b.command_period_us = 20001;
  zassert_equal(vesc_configure_slot(motor0, &b), -EINVAL);
  struct vesc_slot_runtime_config applied;
  zassert_ok(vesc_get_slot_config(motor0, &applied));
  zassert_equal(applied.vesc_can_id, 37);
  struct vesc_bus_diagnostics diag;
  zassert_ok(vesc_get_bus_diagnostics(motor0, &diag));
  zassert_equal(diag.config_error_count, before_diag.config_error_count + 6);
}

ZTEST(vesc, test_id_change_stops_seen_old_id_and_never_transmits_to_unseen_new_id)
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
  k_msleep(3); zassert_is_null(last_for(201));
  struct vesc_slot_diagnostics after_diag;
  zassert_ok(vesc_get_slot_diagnostics(motor0, &after_diag));
  zassert_equal(after_diag.can_id_change_count, before_diag.can_id_change_count + 2);
}

ZTEST(vesc, test_id_change_invalidates_the_old_motors_encoder_position)
{
  struct vesc_slot_runtime_config c = config(37, 5.0F, 20, 1);
  zassert_ok(vesc_configure_slot(motor0, &c));
  inject_position(37, 100); inject_position(37, 110);
  struct encoder_feedback fb;
  zassert_ok(encoder_get_feedback(enc0, &fb));
  c.vesc_can_id = 38;
  zassert_ok(vesc_configure_slot(motor0, &c));
  zassert_equal(encoder_get_feedback(enc0, &fb), -ENODATA);
  inject_position(38, 9000);
  zassert_ok(encoder_get_feedback(enc0, &fb));
  zassert_equal(fb.valid_mask & ENCODER_FEEDBACK_VELOCITY, 0U,
                "one sample invented velocity for the new motor");
  inject_position(38, 9010);
  zassert_ok(encoder_get_feedback(enc0, &fb));
  zassert_equal(fb.position, 9010);
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

ZTEST(vesc, test_notify_boot_is_recognized_without_marking_feedback_online)
{
  struct can_frame frame = {
    .id = ((uint32_t)VESC_CAN_PACKET_NOTIFY_BOOT << 8) | 0U,
    .flags = CAN_FRAME_IDE,
    .dlc = 2,
    .data = {'6', '0'},
  };
  struct vesc_notify_boot boot;
  zassert_true(vesc_decode_notify_boot(&frame, &boot));
  zassert_equal(boot.vesc_can_id, 0U);
  zassert_equal(boot.name_length, 2U);
  zassert_mem_equal(boot.hardware_name, "60", 2U);

  inject(&frame);
  struct vesc_bus_diagnostics diag;
  zassert_ok(vesc_get_bus_diagnostics(motor0, &diag));
  zassert_equal(diag.invalid_rx_count, 0U);
  zassert_true(diag.observed_ids[0] & BIT(0));
  zassert_equal(diag.online_ids[0] & BIT(0), 0U);

  frame.dlc = 0U;
  zassert_false(vesc_decode_notify_boot(&frame, &boot));
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

ZTEST(vesc, test_status_4_freshness_expires_while_other_status_keeps_motor_online)
{
  struct vesc_slot_runtime_config c = config(37, 5.0F, 10, 1);
  zassert_ok(vesc_configure_slot(motor0, &c));
  inject_position(37, 100); inject_position(37, 110);
  struct encoder_feedback fb;
  zassert_ok(encoder_get_feedback(enc0, &fb));
  k_msleep(12); inject_status(37); k_msleep(2);
  zassert_equal(encoder_get_feedback(enc0, &fb), -EAGAIN);
  struct motor_feedback motor_fb;
  zassert_ok(motor_get_feedback(motor0, &motor_fb));
  zassert_true(motor_fb.online);
  struct vesc_slot_diagnostics d;
  zassert_ok(vesc_get_slot_diagnostics(motor0, &d));
  zassert_equal(d.fresh_status_mask & VESC_STATUS_MASK_STATUS_4, 0U);
  zassert_true(d.fresh_status_mask & VESC_STATUS_MASK_STATUS);
  inject_position(37, 120);
  zassert_ok(encoder_get_feedback(enc0, &fb));
}

ZTEST(vesc, test_generic_motor_feedback_exposes_md_current_and_health)
{
  struct vesc_slot_runtime_config c = config(37, 5.0F, 20, 1);
  zassert_ok(vesc_configure_slot(motor0, &c));
  inject_status(37);

  struct motor_feedback feedback;
  zassert_ok(motor_get_feedback(motor0, &feedback));
  zassert_true(feedback.online);
  zassert_false(feedback.stale);
  zassert_true(feedback.valid_mask & MOTOR_FEEDBACK_CURRENT);
  zassert_within(feedback.current, 2.5F, 0.001F);

  inject_fault(37, 3);
  zassert_ok(motor_get_feedback(motor0, &feedback));
  zassert_false(feedback.online);
}

ZTEST(vesc, test_reconnect_frames_do_not_accumulate_across_a_timeout)
{
  struct vesc_slot_runtime_config c = config(37, 5.0F, 10, 3);
  zassert_ok(vesc_configure_slot(motor0, &c));
  inject_status(37);
  k_msleep(12);
  inject_status(37); inject_status(37);
  zassert_equal(motor_enable(motor0), -EAGAIN);
  inject_status(37);
  zassert_ok(motor_enable(motor0));
}

ZTEST(vesc, test_timeout_reconnect_requires_frames_and_new_enable)
{
  struct vesc_slot_runtime_config c = config(37, 5.0F, 30, 3);
  zassert_ok(vesc_configure_slot(motor0, &c));
  struct vesc_slot_diagnostics before_diag;
  zassert_ok(vesc_get_slot_diagnostics(motor0, &before_diag));
  inject_status(37); inject_status(37); zassert_equal(motor_enable(motor0), -EAGAIN);
  inject_status(37); zassert_ok(motor_enable(motor0));
  zassert_ok(motor_set_output(motor0, MOTOR_OUTPUT_MODE_CURRENT, 12.0F));
  zassert_true(wait_for_current(37, 5000, 10), "slot current clamp");
  zassert_ok(motor_set_output(motor0, MOTOR_OUTPUT_MODE_CURRENT, -12.0F));
  zassert_true(wait_for_current(37, -5000, 10), "negative slot current clamp");
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

ZTEST(vesc, test_async_transmit_errors_are_counted)
{
  struct vesc_slot_runtime_config c = config(37, 5.0F, 20, 1);
  struct vesc_bus_diagnostics before_diag;
  zassert_ok(vesc_get_bus_diagnostics(motor0, &before_diag));
  tx_complete_error = -EIO;
  zassert_ok(vesc_configure_slot(motor0, &c));
  inject_status(37);
  k_msleep(3);
  struct vesc_bus_diagnostics after_diag;
  zassert_ok(vesc_get_bus_diagnostics(motor0, &after_diag));
  zassert_true(after_diag.tx_complete_error_count > before_diag.tx_complete_error_count);
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
  zassert_true(d.fresh_status_mask & VESC_STATUS_MASK_FAULT);
  zassert_true(d.fault_age_ms < UINT32_MAX);
}
