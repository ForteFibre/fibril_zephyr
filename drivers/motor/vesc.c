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
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(motor_vesc, CONFIG_MOTOR_LOG_LEVEL);

struct vesc_bus_config {
  const struct device * can_dev;
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
  bool can_started;
};

static inline void bitmap_set(uint32_t bitmap[VESC_ID_BITMAP_WORDS], uint8_t id)
{
  bitmap[id / 32U] |= BIT(id % 32U);
}

static inline bool bitmap_test(const uint32_t bitmap[VESC_ID_BITMAP_WORDS], uint8_t id)
{
  return (bitmap[id / 32U] & BIT(id % 32U)) != 0U;
}

static void tx_done(const struct device * dev, int error, void * user_data)
{
  ARG_UNUSED(dev);
  ARG_UNUSED(error);
  ARG_UNUSED(user_data);
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

static int send_current(const struct device * bus, uint8_t id, float current_a)
{
  const struct vesc_bus_config * config = bus->config;
  struct vesc_bus_data * data = bus->data;
  struct can_frame frame;
  int ret = vesc_encode_set_current(id, current_a, &frame);
  if (ret == 0) {
    ret = can_send(config->can_dev, &frame, K_NO_WAIT, tx_done, NULL);
  }
  if (ret == 0) {
    data->diag.tx_count++;
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
    ret = can_send(config->can_dev, &frame, K_NO_WAIT, tx_done, NULL);
  }
  if (ret == 0) data->diag.tx_count++; else data->diag.tx_error_count++;
}

static void stop_slot(const struct device * bus, struct vesc_slot_data * slot)
{
  /* Close the local output gate before the old physical ID is stopped. */
  slot->enabled = false;
  slot->command_a = 0.0F;
  if (slot->config.vesc_can_id >= 0) {
    (void)send_current(bus, (uint8_t)slot->config.vesc_can_id, 0.0F);
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
    slot->diag.rx_count++;
    if (connection_status) {
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
  const int64_t now_ms = k_uptime_get();
  const uint32_t cycle = k_cycle_get_32();
  k_spinlock_key_t key;

  ARG_UNUSED(can_dev);
  key = k_spin_lock(&data->lock);
  if (vesc_decode_status(frame, &status)) {
    note_rx(bus, status.vesc_can_id, VESC_STATUS_MASK_STATUS, now_ms, true, &slot);
    if (slot != NULL) {
      slot->diag.erpm = status.erpm;
      slot->diag.motor_current_a = status.motor_current_a;
      slot->diag.duty = status.duty;
    }
  } else if (vesc_decode_status_4(frame, &status4)) {
    note_rx(bus, status4.vesc_can_id, VESC_STATUS_MASK_STATUS_4, now_ms, true, &slot);
    if (slot != NULL) {
      slot->diag.fet_temperature_c = status4.fet_temperature_c;
      slot->diag.motor_temperature_c = status4.motor_temperature_c;
      slot->diag.input_current_a = status4.input_current_a;
      if (slot->position_listener != NULL) {
        slot->position_listener(
          data->slots[slot - data->slot_data], status4.position_counts, cycle,
          slot->position_user_data);
      }
    }
  } else if (vesc_decode_status_5(frame, &status5)) {
    note_rx(bus, status5.vesc_can_id, VESC_STATUS_MASK_STATUS_5, now_ms, true, &slot);
    if (slot != NULL) {
      slot->diag.input_voltage_v = status5.input_voltage_v;
    }
  } else if (vesc_decode_fault_status(frame, config->controller_id, &fault)) {
    note_rx(bus, fault.vesc_can_id, VESC_STATUS_MASK_FAULT, now_ms, false, &slot);
    if (slot != NULL) {
      if ((fault.fault_code != 0U) && (slot->diag.fault_code == 0U)) {
        slot->diag.fault_count++;
      }
      slot->diag.fault_code = fault.fault_code;
      if (fault.fault_code != 0U) {
        stop_slot(bus, slot);
      }
    }
  } else {
    data->diag.invalid_rx_count++;
  }
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
    if (timed_out && slot->diag.online) {
      slot->diag.online = false;
      slot->diag.stale = true;
      slot->diag.timeout_count++;
      slot->reconnect_countdown = 0U;
      slot->awaiting_reconnect = true;
      stop_slot(bus, slot);
    }
    slot->diag.last_rx_age_ms = slot->seen ?
      (uint32_t)MIN(now_ms - slot->last_rx_ms, (int64_t)UINT32_MAX) : UINT32_MAX;
    if ((now_us - slot->last_tx_us) >= slot->config.command_period_us) {
      float command = 0.0F;
      if (slot->enabled && slot->diag.online && !slot->diag.stale &&
          (slot->diag.fault_code == 0U)) {
        command = CLAMP(slot->command_a, -config->absolute_max_current_a,
                        config->absolute_max_current_a);
      }
      (void)send_current(bus, (uint8_t)slot->config.vesc_can_id, command);
      slot->last_tx_us = now_us;
    }
    if ((now_ms - slot->last_fault_request_ms) >= 100) {
      send_fault_request(bus, (uint8_t)slot->config.vesc_can_id);
      slot->last_fault_request_ms = now_ms;
    }
  }
  rebuild_bitmaps(bus, now_ms);
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
  data->slot_data[idx].command_a = CLAMP(value, -config->absolute_max_current_a,
                                         config->absolute_max_current_a);
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
    .valid_mask = (slot->diag.status_mask & VESC_STATUS_MASK_STATUS) ?
      (MOTOR_FEEDBACK_CURRENT | MOTOR_FEEDBACK_VELOCITY) : 0U,
    .current = slot->diag.motor_current_a,
    /* ERPM to mechanical rad/s needs pole pairs, which the selected VESC and
     * firmware must define. MdMotor gets velocity from the position shim. */
    .velocity = 0.0F,
    .online = slot->diag.slot_configured && slot->diag.online &&
              (slot->diag.fault_code == 0U),
    .stale = slot->diag.stale,
    .timestamp_ms = slot->last_rx_ms,
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
    (requested->vesc_can_id <= UINT8_MAX) && isfinite(requested->max_current_a) &&
    (((requested->vesc_can_id == VESC_CAN_ID_UNASSIGNED) &&
      (requested->max_current_a >= 0.0F)) ||
     ((requested->max_current_a > 0.0F) &&
      (requested->max_current_a <= limits->absolute_max_current_a))) &&
    (requested->feedback_timeout_ms > 0U) &&
    (requested->feedback_timeout_ms <= limits->max_feedback_timeout_ms) &&
    (requested->reconnect_frames > 0U) &&
    (requested->reconnect_frames <= limits->max_reconnect_frames) &&
    (requested->command_period_us >= limits->min_command_period_us);
  for (uint8_t i = 0; valid && (i < VESC_MOTOR_SLOT_COUNT); ++i) {
    if ((i != idx) && (requested->vesc_can_id >= 0) &&
        (data->slot_data[i].config.vesc_can_id == requested->vesc_can_id)) {
      valid = false;
    }
  }
  if (!valid) {
    slot->diag.last_config_request_ok = false;
    data->diag.config_error_count++;
    k_spin_unlock(&data->lock, key);
    return -EINVAL;
  }
  const int16_t old_id = slot->config.vesc_can_id;
  if (old_id != requested->vesc_can_id) {
    stop_slot(bus, slot);
    slot->diag.can_id_change_count++;
    slot->diag.online = false;
    slot->diag.stale = false;
    slot->seen = false;
    slot->reconnect_countdown = 0U;
    slot->awaiting_reconnect = false;
    slot->diag.status_mask = 0U;
    slot->diag.fault_code = 0U;
  }
  slot->config = *requested;
  slot->diag.applied_can_id = requested->vesc_can_id;
  slot->diag.slot_configured = requested->vesc_can_id >= 0;
  slot->diag.last_config_request_ok = true;
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
    .absolute_max_current_a = DT_INST_PROP(inst, absolute_max_current_ma) / 1000.0F,            \
    .max_feedback_timeout_ms = DT_INST_PROP(inst, max_feedback_timeout_ms),                     \
    .min_command_period_us = DT_INST_PROP(inst, min_command_period_us),                         \
    .max_reconnect_frames = DT_INST_PROP(inst, max_reconnect_frames),                           \
    .controller_id = DT_INST_PROP(inst, controller_id),                                        \
  };                                                                                            \
  DEVICE_DT_INST_DEFINE(inst, bus_init, NULL, &bus_data_##inst, &bus_config_##inst,             \
                        POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(BUS_DEFINE)
