/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT vesc_bus

#include <drivers/motor.h>
#include <drivers/motor/vesc.h>
#include <drivers/motor/vesc_protocol.h>

#include <errno.h>
#include <math.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(motor_vesc, CONFIG_MOTOR_LOG_LEVEL);

#define VESC_MAX_COMMAND_PERIOD_US 20000U

enum vesc_diag_state_reason {
  VESC_STATE_ONLINE = 1,
  VESC_STATE_TIMEOUT,
  VESC_STATE_FAULT,
  VESC_STATE_ENABLE,
  VESC_STATE_DISABLE,
  VESC_STATE_REBIND,
};

#if defined(CONFIG_MOTOR_VESC_DIAGNOSTICS)
#define VESC_DIAG_QUEUE_DEPTH 128U

enum vesc_diag_event_kind {
  VESC_DIAG_BOOT,
  VESC_DIAG_RX,
  VESC_DIAG_TX_CURRENT,
  VESC_DIAG_TX_FAULT,
  VESC_DIAG_CONFIG,
  VESC_DIAG_STATE,
  VESC_DIAG_SUMMARY,
};

struct vesc_diag_event {
  enum vesc_diag_event_kind kind;
  int64_t timestamp_us;
  struct can_frame frame;
  int32_t value_a;
  int32_t value_b;
  uint8_t slot;
  bool assigned;
};
#endif

struct vesc_bus_config {
  const struct device * can_dev;
  uint32_t bitrate;
  float absolute_max_current_a;
  uint32_t max_feedback_timeout_ms;
  uint32_t min_command_period_us;
  uint8_t max_reconnect_frames;
  uint8_t controller_id;
};

struct vesc_motor_config {
  const struct device * bus;
  uint8_t slot;
};

struct vesc_slot_data {
  struct vesc_slot_runtime_config config;
  struct vesc_slot_diagnostics diag;
  float command_a;
  int64_t last_rx_ms;
  int64_t last_status_ms;
  int64_t last_status_4_ms;
  int64_t last_status_5_ms;
  int64_t last_fault_rx_ms;
  int64_t last_tx_us;
  int64_t last_fault_request_ms;
  uint8_t reconnect_countdown;
  bool enabled;
  bool seen;
  bool awaiting_reconnect;
  vesc_position_listener_t position_listener;
  void * position_user_data;
};

struct vesc_bus_data {
  const struct device * dev;
  const struct device * slots[VESC_MOTOR_SLOT_COUNT];
  struct vesc_slot_data slot_data[VESC_MOTOR_SLOT_COUNT];
  struct vesc_bus_diagnostics diag;
  uint32_t status_ids[VESC_ID_BITMAP_WORDS];
  int64_t id_last_rx_ms[256];
  uint32_t observed_timeout_ms;
  struct k_spinlock lock;
  struct k_work_delayable work;
  atomic_t tx_complete_error_count;
  bool can_started;
#if defined(CONFIG_MOTOR_VESC_DIAGNOSTICS)
  struct k_msgq diag_queue;
  char diag_queue_buffer[VESC_DIAG_QUEUE_DEPTH * sizeof(struct vesc_diag_event)] __aligned(4);
  struct k_work diag_work;
  struct k_work_q diag_work_queue;
  K_KERNEL_STACK_MEMBER(diag_work_queue_stack, CONFIG_MOTOR_VESC_DIAGNOSTICS_WORKQ_STACK_SIZE);
  uint8_t diag_rx_samples[256][4];
  uint8_t diag_tx_current_samples[256];
  uint8_t diag_tx_fault_samples[256];
  atomic_t diag_dropped;
  int64_t diag_last_summary_ms;
  int64_t diag_last_position_sample_ms;
#endif
};

static inline void bitmap_set(uint32_t bitmap[VESC_ID_BITMAP_WORDS], uint8_t id)
{
  bitmap[id / 32U] |= BIT(id % 32U);
}

static inline bool bitmap_test(const uint32_t bitmap[VESC_ID_BITMAP_WORDS], uint8_t id)
{
  return (bitmap[id / 32U] & BIT(id % 32U)) != 0U;
}

#if defined(CONFIG_MOTOR_VESC_DIAGNOSTICS)
static void diag_work_handler(struct k_work * work);

static void diag_submit(struct vesc_bus_data * data, const struct vesc_diag_event * event)
{
  if (k_msgq_put(&data->diag_queue, event, K_NO_WAIT) == 0) {
    (void)k_work_submit_to_queue(&data->diag_work_queue, &data->diag_work);
  } else {
    atomic_inc(&data->diag_dropped);
  }
}

static uint8_t diag_packet_bucket(const struct can_frame * frame)
{
  const uint8_t packet = (uint8_t)(frame->id >> 8);
  if (packet == VESC_CAN_PACKET_STATUS) return 0U;
  if (packet == VESC_CAN_PACKET_STATUS_4) return 1U;
  if (packet == VESC_CAN_PACKET_STATUS_5) return 2U;
  return 3U;
}

static void diag_rx(
  struct vesc_bus_data * data, const struct can_frame * frame, int64_t now_ms, bool assigned)
{
  const uint8_t source = (uint8_t)frame->id;
  const uint8_t bucket = diag_packet_bucket(frame);
  uint8_t * samples = &data->diag_rx_samples[source][bucket];
  bool log_frame = *samples < CONFIG_MOTOR_VESC_DIAGNOSTICS_FIRST_FRAMES;
  if (log_frame) (*samples)++;
#if defined(CONFIG_FIBRIL_CAN_NODE_VESC_HARDWARE_SELFTEST)
  if (assigned && (bucket == 1U) &&
      ((now_ms - data->diag_last_position_sample_ms) >= 100)) {
    data->diag_last_position_sample_ms = now_ms;
    log_frame = true;
  }
#endif
  if (!log_frame) return;
  const struct vesc_diag_event event = {
    .kind = VESC_DIAG_RX, .timestamp_us = now_ms * 1000, .frame = *frame, .assigned = assigned,
  };
  diag_submit(data, &event);
}

static void diag_event(
  struct vesc_bus_data * data, enum vesc_diag_event_kind kind, uint8_t slot,
  int32_t value_a, int32_t value_b)
{
  const struct vesc_diag_event event = {
    .kind = kind, .timestamp_us = k_ticks_to_us_floor64(k_uptime_ticks()),
    .slot = slot, .value_a = value_a, .value_b = value_b,
  };
  diag_submit(data, &event);
}
#else
#define diag_rx(data, frame, now_ms, assigned) do { } while (false)
#define diag_event(data, kind, slot, value_a, value_b) do { } while (false)
#endif

static void tx_done(const struct device * dev, int error, void * user_data)
{
  ARG_UNUSED(dev);
  struct vesc_bus_data * data = user_data;
  if ((error != 0) && (data != NULL)) {
    atomic_inc(&data->tx_complete_error_count);
  }
}

static int slot_index(const struct device * motor, struct vesc_bus_data ** bus_data)
{
  if ((motor == NULL) || (motor->api == NULL)) {
    return -EINVAL;
  }
  const struct vesc_motor_config * config = motor->config;
  struct vesc_bus_data * data = config->bus->data;
  if ((config->slot >= VESC_MOTOR_SLOT_COUNT) || (data->slots[config->slot] != motor)) {
    return -EINVAL;
  }
  if (bus_data != NULL) {
    *bus_data = data;
  }
  return config->slot;
}

static int send_current(
  const struct device * bus, uint8_t id, float requested_a, float current_a)
{
  const struct vesc_bus_config * config = bus->config;
  struct vesc_bus_data * data = bus->data;
  struct can_frame frame;
  int ret = vesc_encode_set_current(id, current_a, &frame);
  if (ret == 0) {
    ret = can_send(config->can_dev, &frame, K_NO_WAIT, tx_done, data);
  }
  if (ret == 0) {
    data->diag.tx_count++;
#if defined(CONFIG_MOTOR_VESC_DIAGNOSTICS)
    if (data->diag_tx_current_samples[id] < CONFIG_MOTOR_VESC_DIAGNOSTICS_FIRST_FRAMES) {
      data->diag_tx_current_samples[id]++;
      const struct vesc_diag_event event = {
        .kind = VESC_DIAG_TX_CURRENT,
        .timestamp_us = k_ticks_to_us_floor64(k_uptime_ticks()),
        .frame = frame,
        .value_a = (int32_t)(requested_a * 1000.0F),
        .value_b = (int32_t)(current_a * 1000.0F),
      };
      diag_submit(data, &event);
    }
#endif
  } else {
    data->diag.tx_error_count++;
  }
  return ret;
}

static void send_fault_request(const struct device * bus, uint8_t id)
{
  const struct vesc_bus_config * config = bus->config;
  struct vesc_bus_data * data = bus->data;
  struct can_frame frame;
  int ret = vesc_encode_get_fault(id, config->controller_id, &frame);
  if (ret == 0) {
    ret = can_send(config->can_dev, &frame, K_NO_WAIT, tx_done, data);
  }
  if (ret == 0) data->diag.tx_count++; else data->diag.tx_error_count++;
#if defined(CONFIG_MOTOR_VESC_DIAGNOSTICS)
  if ((ret == 0) &&
      (data->diag_tx_fault_samples[id] < CONFIG_MOTOR_VESC_DIAGNOSTICS_FIRST_FRAMES)) {
    data->diag_tx_fault_samples[id]++;
    const struct vesc_diag_event event = {
      .kind = VESC_DIAG_TX_FAULT,
      .timestamp_us = k_ticks_to_us_floor64(k_uptime_ticks()),
      .frame = frame,
    };
    diag_submit(data, &event);
  }
#endif
}

static void stop_slot(const struct device * bus, struct vesc_slot_data * slot)
{
  /* Close the local output gate before the old physical ID is stopped. */
  slot->enabled = false;
  slot->command_a = 0.0F;
  if ((slot->config.vesc_can_id >= 0) && slot->seen) {
    (void)send_current(bus, (uint8_t)slot->config.vesc_can_id, 0.0F, 0.0F);
  }
}

static struct vesc_slot_data * find_slot(struct vesc_bus_data * data, uint8_t id)
{
  for (uint8_t i = 0; i < VESC_MOTOR_SLOT_COUNT; ++i) {
    if (data->slot_data[i].config.vesc_can_id == id) {
      return &data->slot_data[i];
    }
  }
  return NULL;
}

#if defined(CONFIG_MOTOR_VESC_DIAGNOSTICS)
static const char * diag_invalid_reason(const struct can_frame * frame)
{
  const uint8_t packet = (uint8_t)(frame->id >> 8);
  if ((frame->flags & CAN_FRAME_IDE) == 0U) return "standard-id";
  if ((frame->flags & CAN_FRAME_RTR) != 0U) return "remote-frame";
  if ((frame->id >> 16) != 0U) return "id-above-vesc-layout";
  if ((packet == VESC_CAN_PACKET_STATUS) && (frame->dlc < 8U)) return "short-STATUS";
  if ((packet == VESC_CAN_PACKET_STATUS_4) && (frame->dlc < 8U)) return "short-STATUS_4";
  if ((packet == VESC_CAN_PACKET_STATUS_5) && (frame->dlc < 6U)) return "short-STATUS_5";
  if ((packet == VESC_CAN_PACKET_NOTIFY_BOOT) && (frame->dlc < 1U)) return "short-NOTIFY_BOOT";
  if (packet == VESC_CAN_PACKET_PROCESS_SHORT_BUFFER) return "malformed-or-other-short-buffer";
  return "unknown-packet";
}

static void diag_payload(const struct can_frame * frame, char * out, size_t size)
{
  size_t pos = 0U;
  const uint8_t len = MIN(frame->dlc, (uint8_t)CAN_MAX_DLEN);
  for (uint8_t i = 0U; (i < len) && (pos < size); ++i) {
    pos += (size_t)snprintk(&out[pos], size - pos, "%s%02x", i == 0U ? "" : " ",
                            frame->data[i]);
  }
}

static void diag_bitmap(const char * name, const uint32_t bitmap[VESC_ID_BITMAP_WORDS])
{
  LOG_INF(
    "VESC bitmap %s=%08x:%08x:%08x:%08x:%08x:%08x:%08x:%08x", name,
    bitmap[7], bitmap[6], bitmap[5], bitmap[4], bitmap[3], bitmap[2], bitmap[1], bitmap[0]);
}

static void diag_log_rx(const struct vesc_bus_config * config, const struct vesc_diag_event * event)
{
  const struct can_frame * frame = &event->frame;
  const uint8_t packet = (uint8_t)(frame->id >> 8);
  const uint8_t source = (uint8_t)frame->id;
  char raw[3U * CAN_MAX_DLEN] = {0};
  struct vesc_status status;
  struct vesc_status_4 status4;
  struct vesc_status_5 status5;
  struct vesc_fault_status fault;
  struct vesc_notify_boot boot;
  diag_payload(frame, raw, sizeof(raw));
  if (vesc_decode_status(frame, &status)) {
    LOG_INF(
      "VESC RX t_us=%lld id=0x%08x packet=%u source=%u dlc=%u raw=[%s] decode=ok "
      "type=STATUS assigned=%u erpm=%d motor_current_ma=%d duty_milli=%d",
      event->timestamp_us, frame->id, packet, source, frame->dlc, raw, event->assigned,
      (int32_t)status.erpm, (int32_t)(status.motor_current_a * 1000.0F),
      (int32_t)(status.duty * 1000.0F));
  } else if (vesc_decode_status_4(frame, &status4)) {
    LOG_INF(
      "VESC RX t_us=%lld id=0x%08x packet=%u source=%u dlc=%u raw=[%s] decode=ok "
      "type=STATUS_4 assigned=%u fet_deci_c=%d motor_deci_c=%d input_current_ma=%d "
      "position_counts=%d",
      event->timestamp_us, frame->id, packet, source, frame->dlc, raw, event->assigned,
      (int32_t)(status4.fet_temperature_c * 10.0F),
      (int32_t)(status4.motor_temperature_c * 10.0F),
      (int32_t)(status4.input_current_a * 1000.0F), status4.position_counts);
  } else if (vesc_decode_status_5(frame, &status5)) {
    LOG_INF(
      "VESC RX t_us=%lld id=0x%08x packet=%u source=%u dlc=%u raw=[%s] decode=ok "
      "type=STATUS_5 assigned=%u tachometer=%d input_voltage_deci_v=%d",
      event->timestamp_us, frame->id, packet, source, frame->dlc, raw, event->assigned,
      (int32_t)status5.tachometer, (int32_t)(status5.input_voltage_v * 10.0F));
  } else if (vesc_decode_fault_status(frame, config->controller_id, &fault)) {
    LOG_INF(
      "VESC RX t_us=%lld id=0x%08x packet=%u source=%u dlc=%u raw=[%s] decode=ok "
      "type=FAULT response_vesc=%u fault=%u assigned=%u",
      event->timestamp_us, frame->id, packet, source, frame->dlc, raw,
      fault.vesc_can_id, fault.fault_code, event->assigned);
  } else if (vesc_decode_notify_boot(frame, &boot)) {
    char hardware_name[CAN_MAX_DLEN + 1U] = {0};
    memcpy(hardware_name, boot.hardware_name, boot.name_length);
    LOG_INF(
      "VESC RX t_us=%lld id=0x%08x packet=%u source=%u dlc=%u raw=[%s] decode=ok "
      "type=NOTIFY_BOOT hardware_name=%s assigned=%u",
      event->timestamp_us, frame->id, packet, source, frame->dlc, raw, hardware_name,
      event->assigned);
  } else {
    LOG_WRN(
      "VESC RX t_us=%lld id=0x%08x packet=%u source=%u dlc=%u raw=[%s] decode=fail "
      "reason=%s assigned=%u", event->timestamp_us, frame->id, packet, source, frame->dlc,
      raw, diag_invalid_reason(frame), event->assigned);
  }
}

static void diag_log_config(struct vesc_bus_data * data, const struct vesc_diag_event * event)
{
  struct vesc_slot_runtime_config config;
  struct vesc_slot_diagnostics diag;
  k_spinlock_key_t key = k_spin_lock(&data->lock);
  config = data->slot_data[event->slot].config;
  diag = data->slot_data[event->slot].diag;
  k_spin_unlock(&data->lock, key);
  LOG_INF(
    "VESC config t_us=%lld slot=%u accepted=%u requested_id=%d applied_id=%d "
    "max_current_ma=%d feedback_timeout_ms=%u reconnect_frames=%u command_period_us=%u",
    event->timestamp_us, event->slot, event->value_a, diag.requested_can_id,
    diag.applied_can_id, (int32_t)(config.max_current_a * 1000.0F),
    config.feedback_timeout_ms, config.reconnect_frames, config.command_period_us);
}

static void diag_log_state(struct vesc_bus_data * data, const struct vesc_diag_event * event)
{
  struct vesc_slot_data slot;
  k_spinlock_key_t key = k_spin_lock(&data->lock);
  slot = data->slot_data[event->slot];
  k_spin_unlock(&data->lock, key);
  LOG_INF(
    "VESC state t_us=%lld slot=%u id=%d online=%u stale=%u reconnect_frames=%u/%u "
    "fault=%u encoder_valid=%u enabled=%u reason=%d",
    event->timestamp_us, event->slot, slot.config.vesc_can_id, slot.diag.online,
    slot.diag.stale, slot.reconnect_countdown, slot.config.reconnect_frames,
    slot.diag.fault_code,
    (slot.diag.fresh_status_mask & VESC_STATUS_MASK_STATUS_4) != 0U,
    slot.enabled, event->value_a);
}

static void diag_log_summary(struct vesc_bus_data * data)
{
  struct vesc_bus_diagnostics diag;
  struct can_bus_err_cnt err_cnt = {0};
  enum can_state state = CAN_STATE_STOPPED;
  k_spinlock_key_t key = k_spin_lock(&data->lock);
  diag = data->diag;
  k_spin_unlock(&data->lock, key);
  (void)can_get_state(((const struct vesc_bus_config *)data->dev->config)->can_dev,
                      &state, &err_cnt);
  LOG_INF(
    "VESC summary t_ms=%lld controller_state=%d tx_err=%u rx_err=%u rx=%u invalid=%u "
    "tx=%u tx_queue_err=%u tx_complete_err=%u diag_dropped=%u",
    k_uptime_get(), state, err_cnt.tx_err_cnt, err_cnt.rx_err_cnt, diag.rx_count,
    diag.invalid_rx_count, diag.tx_count, diag.tx_error_count,
    (uint32_t)atomic_get(&data->tx_complete_error_count),
    (uint32_t)atomic_get(&data->diag_dropped));
  diag_bitmap("observed", diag.observed_ids);
  diag_bitmap("online", diag.online_ids);
  diag_bitmap("stale", diag.stale_ids);
  diag_bitmap("assigned", diag.assigned_ids);
  diag_bitmap("unexpected", diag.unexpected_ids);
  diag_bitmap("missing", diag.missing_ids);
}

static void diag_work_handler(struct k_work * work)
{
  struct vesc_bus_data * data = CONTAINER_OF(work, struct vesc_bus_data, diag_work);
  const struct vesc_bus_config * config = data->dev->config;
  struct vesc_diag_event event;
  while (k_msgq_get(&data->diag_queue, &event, K_NO_WAIT) == 0) {
    if (event.kind == VESC_DIAG_BOOT) {
      enum can_state state = CAN_STATE_STOPPED;
      struct can_bus_err_cnt err_cnt = {0};
      (void)can_get_state(config->can_dev, &state, &err_cnt);
      LOG_INF(
        "VESC boot can=%s bitrate=%u controller_state=%d fault_reply_controller_id=%u "
        "absolute_max_current_ma=%d",
        config->can_dev->name, config->bitrate, state, config->controller_id,
        (int32_t)(config->absolute_max_current_a * 1000.0F));
      for (uint8_t i = 0U; i < VESC_MOTOR_SLOT_COUNT; ++i) {
        diag_log_config(data, &(struct vesc_diag_event){
          .kind = VESC_DIAG_CONFIG, .timestamp_us = event.timestamp_us, .slot = i,
          .value_a = 1,
        });
      }
    } else if (event.kind == VESC_DIAG_RX) {
      diag_log_rx(config, &event);
    } else if ((event.kind == VESC_DIAG_TX_CURRENT) ||
               (event.kind == VESC_DIAG_TX_FAULT)) {
      char raw[3U * CAN_MAX_DLEN] = {0};
      diag_payload(&event.frame, raw, sizeof(raw));
      LOG_INF(
        "VESC TX t_us=%lld packet=%u destination=%u dlc=%u raw=[%s] type=%s "
        "requested_ma=%d clamped_ma=%d encoded=%d reply_destination=%d",
        event.timestamp_us, (uint8_t)(event.frame.id >> 8), (uint8_t)event.frame.id,
        event.frame.dlc, raw,
        event.kind == VESC_DIAG_TX_CURRENT ? "SET_CURRENT" : "FAULT_REQUEST",
        event.value_a, event.value_b,
        event.kind == VESC_DIAG_TX_CURRENT ? (int32_t)sys_get_be32(event.frame.data) : 0,
        event.kind == VESC_DIAG_TX_FAULT ? event.frame.data[0] : -1);
    } else if (event.kind == VESC_DIAG_CONFIG) {
      diag_log_config(data, &event);
    } else if (event.kind == VESC_DIAG_STATE) {
      diag_log_state(data, &event);
    } else if (event.kind == VESC_DIAG_SUMMARY) {
      diag_log_summary(data);
    }
  }
}
#endif

static void note_rx(
  const struct device * bus, uint8_t id, uint32_t mask, int64_t now_ms,
  bool connection_status, struct vesc_slot_data ** slot_out)
{
  struct vesc_bus_data * data = bus->data;
  struct vesc_slot_data * slot = find_slot(data, id);
  bitmap_set(data->diag.observed_ids, id);
  if (connection_status) {
    bitmap_set(data->status_ids, id);
    data->id_last_rx_ms[id] = now_ms;
  }
  data->diag.rx_count++;
  if (slot != NULL) {
    const bool was_online = slot->diag.online;
    slot->diag.status_mask |= mask;
    slot->diag.fresh_status_mask |= mask;
    slot->diag.rx_count++;
    if (connection_status) {
      if (slot->seen && ((now_ms - slot->last_rx_ms) > slot->config.feedback_timeout_ms)) {
        slot->reconnect_countdown = 0U;
      }
      slot->seen = true;
      slot->last_rx_ms = now_ms;
      slot->diag.stale = false;
    }
    if (connection_status && !slot->diag.online) {
      if (slot->reconnect_countdown < slot->config.reconnect_frames) {
        slot->reconnect_countdown++;
      }
      if (slot->reconnect_countdown >= slot->config.reconnect_frames) {
        slot->diag.online = true;
        if (!was_online && slot->awaiting_reconnect) {
          slot->diag.reconnect_count++;
          slot->awaiting_reconnect = false;
        }
        if (!was_online) {
          diag_event(
            data, VESC_DIAG_STATE, (uint8_t)(slot - data->slot_data), VESC_STATE_ONLINE, 0);
        }
      }
    }
  }
  *slot_out = slot;
}

static void vesc_rx(const struct device * can_dev, struct can_frame * frame, void * user_data)
{
  const struct device * bus = user_data;
  const struct vesc_bus_config * config = bus->config;
  struct vesc_bus_data * data = bus->data;
  struct vesc_slot_data * slot = NULL;
  struct vesc_status status;
  struct vesc_status_4 status4;
  struct vesc_status_5 status5;
  struct vesc_fault_status fault;
  struct vesc_notify_boot boot;
  const int64_t now_ms = k_uptime_get();
  const uint32_t cycle = k_cycle_get_32();
  k_spinlock_key_t key;

  ARG_UNUSED(can_dev);
  key = k_spin_lock(&data->lock);
  if (vesc_decode_status(frame, &status)) {
    note_rx(bus, status.vesc_can_id, VESC_STATUS_MASK_STATUS, now_ms, true, &slot);
    if (slot != NULL) {
      slot->last_status_ms = now_ms;
      slot->diag.erpm = status.erpm;
      slot->diag.motor_current_a = status.motor_current_a;
      slot->diag.duty = status.duty;
    }
  } else if (vesc_decode_status_4(frame, &status4)) {
    note_rx(bus, status4.vesc_can_id, VESC_STATUS_MASK_STATUS_4, now_ms, true, &slot);
    if (slot != NULL) {
      slot->last_status_4_ms = now_ms;
      slot->diag.fet_temperature_c = status4.fet_temperature_c;
      slot->diag.motor_temperature_c = status4.motor_temperature_c;
      slot->diag.input_current_a = status4.input_current_a;
      if (slot->position_listener != NULL) {
        slot->position_listener(
          data->slots[slot - data->slot_data], status4.position_counts, cycle, false,
          slot->position_user_data);
      }
    }
  } else if (vesc_decode_status_5(frame, &status5)) {
    note_rx(bus, status5.vesc_can_id, VESC_STATUS_MASK_STATUS_5, now_ms, true, &slot);
    if (slot != NULL) {
      slot->last_status_5_ms = now_ms;
      slot->diag.input_voltage_v = status5.input_voltage_v;
    }
  } else if (vesc_decode_fault_status(frame, config->controller_id, &fault)) {
    note_rx(bus, fault.vesc_can_id, VESC_STATUS_MASK_FAULT, now_ms, false, &slot);
    if (slot != NULL) {
      slot->last_fault_rx_ms = now_ms;
      if ((fault.fault_code != 0U) && (slot->diag.fault_code == 0U)) {
        slot->diag.fault_count++;
      }
      slot->diag.fault_code = fault.fault_code;
      if (fault.fault_code != 0U) {
        stop_slot(bus, slot);
        diag_event(
          data, VESC_DIAG_STATE, (uint8_t)(slot - data->slot_data), VESC_STATE_FAULT,
          fault.fault_code);
      }
    }
  } else if (vesc_decode_notify_boot(frame, &boot)) {
    note_rx(bus, boot.vesc_can_id, 0U, now_ms, false, &slot);
  } else {
    data->diag.invalid_rx_count++;
  }
  diag_rx(data, frame, now_ms, slot != NULL);
  k_spin_unlock(&data->lock, key);
}

static void rebuild_bitmaps(const struct device * bus, int64_t now_ms)
{
  struct vesc_bus_data * data = bus->data;
  memset(data->diag.online_ids, 0, sizeof(data->diag.online_ids));
  memset(data->diag.stale_ids, 0, sizeof(data->diag.stale_ids));
  memset(data->diag.assigned_ids, 0, sizeof(data->diag.assigned_ids));
  for (uint16_t id = 0; id < 256U; ++id) {
    if (bitmap_test(data->status_ids, (uint8_t)id)) {
      const struct vesc_slot_data * slot = find_slot(data, (uint8_t)id);
      const uint32_t timeout_ms = (slot != NULL) ? slot->config.feedback_timeout_ms :
                                                  data->observed_timeout_ms;
      if ((now_ms - data->id_last_rx_ms[id]) <= timeout_ms) {
        bitmap_set(data->diag.online_ids, (uint8_t)id);
      } else {
        bitmap_set(data->diag.stale_ids, (uint8_t)id);
      }
    }
  }
  for (uint8_t i = 0; i < VESC_MOTOR_SLOT_COUNT; ++i) {
    if (data->slot_data[i].config.vesc_can_id >= 0) {
      bitmap_set(data->diag.assigned_ids, (uint8_t)data->slot_data[i].config.vesc_can_id);
    }
  }
  for (uint8_t i = 0; i < VESC_ID_BITMAP_WORDS; ++i) {
    data->diag.unexpected_ids[i] = data->diag.online_ids[i] & ~data->diag.assigned_ids[i];
    data->diag.missing_ids[i] = 0U;
  }
  for (uint8_t i = 0; i < VESC_MOTOR_SLOT_COUNT; ++i) {
    const struct vesc_slot_data * slot = &data->slot_data[i];
    if ((slot->config.vesc_can_id >= 0) && !slot->diag.online) {
      bitmap_set(data->diag.missing_ids, (uint8_t)slot->config.vesc_can_id);
    }
  }
}

static void service_work(struct k_work * work)
{
  struct k_work_delayable * delayable = k_work_delayable_from_work(work);
  struct vesc_bus_data * data = CONTAINER_OF(delayable, struct vesc_bus_data, work);
  const struct device * bus = data->dev;
  const struct vesc_bus_config * config = bus->config;
  const int64_t now_ms = k_uptime_get();
  const int64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());
  k_spinlock_key_t key = k_spin_lock(&data->lock);

  for (uint8_t i = 0; i < VESC_MOTOR_SLOT_COUNT; ++i) {
    struct vesc_slot_data * slot = &data->slot_data[i];
    if (slot->config.vesc_can_id < 0) {
      continue;
    }
    const bool timed_out = slot->seen &&
      ((now_ms - slot->last_rx_ms) > slot->config.feedback_timeout_ms);
    if (timed_out && !slot->diag.stale) {
      const bool was_online = slot->diag.online;
      slot->diag.online = false;
      slot->diag.stale = true;
      slot->diag.timeout_count++;
      slot->reconnect_countdown = 0U;
      slot->awaiting_reconnect = was_online;
      if (was_online) stop_slot(bus, slot);
      diag_event(data, VESC_DIAG_STATE, i, VESC_STATE_TIMEOUT, 0);
    }
    slot->diag.last_rx_age_ms = slot->seen ?
      (uint32_t)MIN(now_ms - slot->last_rx_ms, (int64_t)UINT32_MAX) : UINT32_MAX;
    slot->diag.fresh_status_mask = 0U;
#define UPDATE_PACKET_FRESH(mask, member, age_member)                                           \
    do {                                                                                        \
      if ((slot->diag.status_mask & (mask)) != 0U) {                                            \
        const int64_t age = now_ms - slot->member;                                              \
        slot->diag.age_member = (uint32_t)MIN(age, (int64_t)UINT32_MAX);                        \
        if (age <= slot->config.feedback_timeout_ms) slot->diag.fresh_status_mask |= (mask);    \
      } else {                                                                                  \
        slot->diag.age_member = UINT32_MAX;                                                     \
      }                                                                                         \
    } while (false)
    UPDATE_PACKET_FRESH(VESC_STATUS_MASK_STATUS, last_status_ms, status_age_ms);
    UPDATE_PACKET_FRESH(VESC_STATUS_MASK_STATUS_4, last_status_4_ms, status_4_age_ms);
    UPDATE_PACKET_FRESH(VESC_STATUS_MASK_STATUS_5, last_status_5_ms, status_5_age_ms);
    UPDATE_PACKET_FRESH(VESC_STATUS_MASK_FAULT, last_fault_rx_ms, fault_age_ms);
#undef UPDATE_PACKET_FRESH
    /* VESC STATUS is unsolicited. Wait for fresh feedback before sending
     * periodic traffic to a slot that may have no peer to ACK it.
     */
    if (!slot->seen || timed_out) {
      continue;
    }
    if ((now_us - slot->last_tx_us) >= slot->config.command_period_us) {
      float command = 0.0F;
      if (slot->enabled && slot->diag.online && !slot->diag.stale &&
          (slot->diag.fault_code == 0U)) {
        const float limit = MIN(slot->config.max_current_a, config->absolute_max_current_a);
        command = CLAMP(slot->command_a, -limit, limit);
      }
      (void)send_current(
        bus, (uint8_t)slot->config.vesc_can_id, slot->command_a, command);
      slot->last_tx_us = now_us;
    }
    if ((now_ms - slot->last_fault_request_ms) >= 100) {
      send_fault_request(bus, (uint8_t)slot->config.vesc_can_id);
      slot->last_fault_request_ms = now_ms;
    }
  }
  rebuild_bitmaps(bus, now_ms);
#if defined(CONFIG_MOTOR_VESC_DIAGNOSTICS)
  if ((now_ms - data->diag_last_summary_ms) >= CONFIG_MOTOR_VESC_DIAGNOSTICS_SUMMARY_MS) {
    data->diag_last_summary_ms = now_ms;
    diag_event(data, VESC_DIAG_SUMMARY, 0U, 0, 0);
  }
#endif
  k_spin_unlock(&data->lock, key);
  k_work_schedule(&data->work, K_USEC(CONFIG_MOTOR_VESC_WORK_PERIOD_US));
}

static int motor_enable_impl(const struct device * motor)
{
  struct vesc_bus_data * data;
  const int idx = slot_index(motor, &data);
  if (idx < 0) return idx;
  k_spinlock_key_t key = k_spin_lock(&data->lock);
  struct vesc_slot_data * slot = &data->slot_data[idx];
  const bool ready = slot->diag.slot_configured && slot->diag.online && !slot->diag.stale &&
                     (slot->diag.fault_code == 0U);
  slot->enabled = ready;
  diag_event(data, VESC_DIAG_STATE, (uint8_t)idx, VESC_STATE_ENABLE, ready);
  k_spin_unlock(&data->lock, key);
  return ready ? 0 : -EAGAIN;
}

static int motor_disable_impl(const struct device * motor)
{
  struct vesc_bus_data * data;
  const int idx = slot_index(motor, &data);
  if (idx < 0) return idx;
  k_spinlock_key_t key = k_spin_lock(&data->lock);
  stop_slot(((const struct vesc_motor_config *)motor->config)->bus, &data->slot_data[idx]);
  diag_event(data, VESC_DIAG_STATE, (uint8_t)idx, VESC_STATE_DISABLE, 0);
  k_spin_unlock(&data->lock, key);
  return 0;
}

static int motor_set_output_impl(
  const struct device * motor, enum motor_output_mode mode, float value)
{
  struct vesc_bus_data * data;
  const struct vesc_bus_config * config;
  const int idx = slot_index(motor, &data);
  if (idx < 0) return idx;
  if (mode != MOTOR_OUTPUT_MODE_CURRENT) return -ENOTSUP;
  if (!isfinite(value)) return -EINVAL;
  config = ((const struct vesc_motor_config *)motor->config)->bus->config;
  k_spinlock_key_t key = k_spin_lock(&data->lock);
  const float limit = MIN(data->slot_data[idx].config.max_current_a,
                          config->absolute_max_current_a);
  data->slot_data[idx].command_a = CLAMP(value, -limit, limit);
  k_spin_unlock(&data->lock, key);
  return 0;
}

static int motor_get_feedback_impl(const struct device * motor, void * feedback_ptr)
{
  struct vesc_bus_data * data;
  struct motor_feedback * feedback = feedback_ptr;
  const int idx = slot_index(motor, &data);
  if ((idx < 0) || (feedback == NULL)) return -EINVAL;
  k_spinlock_key_t key = k_spin_lock(&data->lock);
  const struct vesc_slot_data * slot = &data->slot_data[idx];
  *feedback = (struct motor_feedback){
    .valid_mask = (slot->diag.fresh_status_mask & VESC_STATUS_MASK_STATUS) ?
      (MOTOR_FEEDBACK_CURRENT | MOTOR_FEEDBACK_VELOCITY) : 0U,
    .current = slot->diag.motor_current_a,
    /* ERPM to mechanical rad/s needs pole pairs, which the selected VESC and
     * firmware must define. MdMotor gets velocity from the position shim. */
    .velocity = 0.0F,
    .online = slot->diag.slot_configured && slot->diag.online &&
              (slot->diag.fault_code == 0U),
    .stale = slot->diag.stale,
    .timestamp_ms = slot->last_status_ms,
  };
  k_spin_unlock(&data->lock, key);
  return 0;
}

static int motor_get_info_impl(const struct device * motor, struct motor_info * info)
{
  if (info == NULL) return -EINVAL;
  struct vesc_bus_data * data;
  const int idx = slot_index(motor, &data);
  if (idx < 0) return idx;
  k_spinlock_key_t key = k_spin_lock(&data->lock);
  *info = (struct motor_info){.output_modes = MOTOR_OUTPUT_MODE_CURRENT,
                              .counts_per_rev = VESC_POSITION_COUNTS_PER_REV,
                              .current_scale_a = data->slot_data[idx].config.max_current_a};
  k_spin_unlock(&data->lock, key);
  return 0;
}

static const struct motor_driver_api motor_api = {
  .enable = motor_enable_impl, .disable = motor_disable_impl,
  .set_output = motor_set_output_impl, .get_feedback = motor_get_feedback_impl,
  .get_info = motor_get_info_impl,
};

int vesc_configure_slot(
  const struct device * motor, const struct vesc_slot_runtime_config * requested)
{
  struct vesc_bus_data * data;
  const int idx = slot_index(motor, &data);
  if ((idx < 0) || (requested == NULL)) return -EINVAL;
  const struct device * bus = ((const struct vesc_motor_config *)motor->config)->bus;
  const struct vesc_bus_config * limits = bus->config;
  k_spinlock_key_t key = k_spin_lock(&data->lock);
  struct vesc_slot_data * slot = &data->slot_data[idx];
  slot->diag.requested_can_id = requested->vesc_can_id;
  bool valid = (requested->vesc_can_id >= VESC_CAN_ID_UNASSIGNED) &&
    (requested->vesc_can_id <= (int16_t)VESC_CAN_ID_MAX_MOTOR) &&
    ((requested->vesc_can_id < 0) ||
     (requested->vesc_can_id != limits->controller_id)) &&
    isfinite(requested->max_current_a) &&
    (((requested->vesc_can_id == VESC_CAN_ID_UNASSIGNED) &&
      (requested->max_current_a >= 0.0F)) ||
     ((requested->max_current_a > 0.0F) &&
      (requested->max_current_a <= limits->absolute_max_current_a))) &&
    (requested->feedback_timeout_ms > 0U) &&
    (requested->feedback_timeout_ms <= limits->max_feedback_timeout_ms) &&
    (requested->reconnect_frames > 0U) &&
    (requested->reconnect_frames <= limits->max_reconnect_frames) &&
    (requested->command_period_us >= limits->min_command_period_us) &&
    (requested->command_period_us <= VESC_MAX_COMMAND_PERIOD_US);
  for (uint8_t i = 0; valid && (i < VESC_MOTOR_SLOT_COUNT); ++i) {
    if ((i != idx) && (requested->vesc_can_id >= 0) &&
        (data->slot_data[i].config.vesc_can_id == requested->vesc_can_id)) {
      valid = false;
    }
  }
  if (!valid) {
    slot->diag.last_config_request_ok = false;
    data->diag.config_error_count++;
    diag_event(data, VESC_DIAG_CONFIG, (uint8_t)idx, 0, requested->vesc_can_id);
    k_spin_unlock(&data->lock, key);
    return -EINVAL;
  }
  const int16_t old_id = slot->config.vesc_can_id;
  if (old_id != requested->vesc_can_id) {
    stop_slot(bus, slot);
    if (slot->position_listener != NULL) {
      slot->position_listener(motor, 0, k_cycle_get_32(), true, slot->position_user_data);
    }
    slot->diag.can_id_change_count++;
    slot->diag.online = false;
    slot->diag.stale = false;
    slot->seen = false;
    slot->reconnect_countdown = 0U;
    slot->awaiting_reconnect = false;
    slot->diag.status_mask = 0U;
    slot->diag.fresh_status_mask = 0U;
    slot->diag.fault_code = 0U;
    diag_event(data, VESC_DIAG_STATE, (uint8_t)idx, VESC_STATE_REBIND, old_id);
  }
  slot->config = *requested;
  slot->diag.applied_can_id = requested->vesc_can_id;
  slot->diag.slot_configured = requested->vesc_can_id >= 0;
  slot->diag.last_config_request_ok = true;
  diag_event(data, VESC_DIAG_CONFIG, (uint8_t)idx, 1, requested->vesc_can_id);
  k_spin_unlock(&data->lock, key);
  return 0;
}

int vesc_configure_bus_timeout(const struct device * motor, uint32_t feedback_timeout_ms)
{
  struct vesc_bus_data * data;
  const int idx = slot_index(motor, &data);
  if (idx < 0) return idx;
  const struct device * bus = ((const struct vesc_motor_config *)motor->config)->bus;
  const struct vesc_bus_config * limits = bus->config;
  if ((feedback_timeout_ms == 0U) ||
      (feedback_timeout_ms > limits->max_feedback_timeout_ms)) return -EINVAL;
  k_spinlock_key_t key = k_spin_lock(&data->lock);
  data->observed_timeout_ms = feedback_timeout_ms;
  k_spin_unlock(&data->lock, key);
  return 0;
}

int vesc_get_slot_config(
  const struct device * motor, struct vesc_slot_runtime_config * config)
{
  struct vesc_bus_data * data;
  const int idx = slot_index(motor, &data);
  if ((idx < 0) || (config == NULL)) return -EINVAL;
  k_spinlock_key_t key = k_spin_lock(&data->lock);
  *config = data->slot_data[idx].config;
  k_spin_unlock(&data->lock, key);
  return 0;
}

int vesc_get_slot_diagnostics(
  const struct device * motor, struct vesc_slot_diagnostics * diagnostics)
{
  struct vesc_bus_data * data;
  const int idx = slot_index(motor, &data);
  if ((idx < 0) || (diagnostics == NULL)) return -EINVAL;
  k_spinlock_key_t key = k_spin_lock(&data->lock);
  *diagnostics = data->slot_data[idx].diag;
  k_spin_unlock(&data->lock, key);
  return 0;
}

int vesc_get_bus_diagnostics(
  const struct device * motor, struct vesc_bus_diagnostics * diagnostics)
{
  struct vesc_bus_data * data;
  const int idx = slot_index(motor, &data);
  if ((idx < 0) || (diagnostics == NULL)) return -EINVAL;
  k_spinlock_key_t key = k_spin_lock(&data->lock);
  *diagnostics = data->diag;
  diagnostics->tx_complete_error_count =
    (uint32_t)atomic_get(&data->tx_complete_error_count);
  diagnostics->tx_error_count += diagnostics->tx_complete_error_count;
  k_spin_unlock(&data->lock, key);
  return 0;
}

int vesc_set_position_listener(
  const struct device * motor, vesc_position_listener_t listener, void * user_data)
{
  struct vesc_bus_data * data;
  const int idx = slot_index(motor, &data);
  if (idx < 0) return idx;
  k_spinlock_key_t key = k_spin_lock(&data->lock);
  data->slot_data[idx].position_listener = listener;
  data->slot_data[idx].position_user_data = user_data;
  k_spin_unlock(&data->lock, key);
  return 0;
}

static int bus_init(const struct device * bus)
{
  const struct vesc_bus_config * config = bus->config;
  struct vesc_bus_data * data = bus->data;
  struct can_filter filter = {.id = 0U, .mask = 0U, .flags = CAN_FILTER_IDE};
  if (!device_is_ready(config->can_dev)) return -ENODEV;
  data->dev = bus;
#if defined(CONFIG_MOTOR_VESC_DIAGNOSTICS)
  k_msgq_init(
    &data->diag_queue, data->diag_queue_buffer, sizeof(struct vesc_diag_event),
    VESC_DIAG_QUEUE_DEPTH);
  k_work_init(&data->diag_work, diag_work_handler);
  k_work_queue_start(
    &data->diag_work_queue, data->diag_work_queue_stack,
    K_THREAD_STACK_SIZEOF(data->diag_work_queue_stack),
    CONFIG_MOTOR_VESC_DIAGNOSTICS_WORKQ_PRIORITY, NULL);
  k_thread_name_set(&data->diag_work_queue.thread, "vesc_diag");
#endif
  data->observed_timeout_ms = config->max_feedback_timeout_ms;
  for (uint8_t i = 0; i < VESC_MOTOR_SLOT_COUNT; ++i) {
    data->slot_data[i].config = (struct vesc_slot_runtime_config){
      .vesc_can_id = VESC_CAN_ID_UNASSIGNED, .max_current_a = 1.0F,
      .feedback_timeout_ms = config->max_feedback_timeout_ms,
      .reconnect_frames = 3U, .command_period_us = MAX(config->min_command_period_us, 2000U),
    };
    data->slot_data[i].diag.requested_can_id = VESC_CAN_ID_UNASSIGNED;
    data->slot_data[i].diag.applied_can_id = VESC_CAN_ID_UNASSIGNED;
  }
  int ret = can_add_rx_filter(config->can_dev, vesc_rx, (void *)bus, &filter);
  if (ret < 0) return ret;
#if defined(CONFIG_MOTOR_VESC_DIAGNOSTICS)
  /* Queue boot metadata before CAN RX can fill the diagnostic queue. */
  diag_event(data, VESC_DIAG_BOOT, 0U, 0, 0);
#endif
  ret = can_start(config->can_dev);
  if ((ret != 0) && (ret != -EALREADY)) return ret;
  data->can_started = true;
  k_work_init_delayable(&data->work, service_work);
  k_work_schedule(&data->work, K_USEC(CONFIG_MOTOR_VESC_WORK_PERIOD_US));
  return 0;
}

#define SLOT_PTR(child) [DT_REG_ADDR(child)] = DEVICE_DT_GET(child),
#define MOTOR_DEFINE(child)                                                                    \
  static const struct vesc_motor_config motor_config_##child = {                               \
    .bus = DEVICE_DT_GET(DT_PARENT(child)), .slot = DT_REG_ADDR(child)};                        \
  DEVICE_DT_DEFINE(child, NULL, NULL, NULL, &motor_config_##child, POST_KERNEL,                 \
                   CONFIG_MOTOR_INIT_PRIORITY, &motor_api);

#define BUS_DEFINE(inst)                                                                       \
  DT_INST_FOREACH_CHILD(inst, MOTOR_DEFINE)                                                     \
  static struct vesc_bus_data bus_data_##inst = {                                               \
    .slots = {DT_INST_FOREACH_CHILD(inst, SLOT_PTR)}};                                          \
  static const struct vesc_bus_config bus_config_##inst = {                                     \
    .can_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, cans)),                                      \
    .bitrate = DT_PROP_OR(DT_INST_PHANDLE(inst, cans), bitrate, 0),                             \
    .absolute_max_current_a = DT_INST_PROP(inst, absolute_max_current_ma) / 1000.0F,            \
    .max_feedback_timeout_ms = DT_INST_PROP(inst, max_feedback_timeout_ms),                     \
    .min_command_period_us = DT_INST_PROP(inst, min_command_period_us),                         \
    .max_reconnect_frames = DT_INST_PROP(inst, max_reconnect_frames),                           \
    .controller_id = DT_INST_PROP(inst, controller_id),                                        \
  };                                                                                            \
  DEVICE_DT_INST_DEFINE(inst, bus_init, NULL, &bus_data_##inst, &bus_config_##inst,             \
                        POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(BUS_DEFINE)
