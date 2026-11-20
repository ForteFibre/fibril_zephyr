/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT fibril_fcan_vesc

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <drivers/motor/vesc.h>
#include <fibril_can_node/func.h>
#include "schema_gen.hpp"

LOG_MODULE_REGISTER(fcan_vesc, CONFIG_FIBRIL_CAN_NODE_LOG_LEVEL);

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
             "exactly one enabled fibril,fcan-vesc node");
#define MOTOR_BY_IDX(node_id, prop, idx) DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx))
static const struct device * const motors[] = {
  DT_INST_FOREACH_PROP_ELEM_SEP(0, motors, MOTOR_BY_IDX, (, ))
};
BUILD_ASSERT(ARRAY_SIZE(motors) == VESC_MOTOR_SLOT_COUNT, "VESC requires exactly eight slots");

static struct k_spinlock lock;
static bool config_pending = true;

static void request_config()
{
  k_spinlock_key_t key = k_spin_lock(&lock);
  config_pending = true;
  k_spin_unlock(&lock, key);
}

static void apply_config()
{
#if defined(CONFIG_FIBRIL_CAN_NODE_VESC_HARDWARE_SELFTEST)
  constexpr uint32_t feedback_timeout_ms = 100U;
  constexpr uint8_t reconnect_frames = 3U;
  constexpr uint32_t command_period_us = 5000U;
  if (vesc_configure_bus_timeout(motors[0], feedback_timeout_ms) != 0) {
    LOG_ERR("VESC self-test bus timeout rejected");
  }
  for (uint8_t i = 0; i < VESC_MOTOR_SLOT_COUNT; ++i) {
    const struct vesc_slot_runtime_config config{
      (int16_t)(i == 0U ? 0 : VESC_CAN_ID_UNASSIGNED),
      1.0F,
      feedback_timeout_ms,
      reconnect_frames,
      command_period_us,
    };
    const int ret = vesc_configure_slot(motors[i], &config);
    if (ret != 0) {
      LOG_ERR("VESC self-test slot %u configuration rejected (%d)", i, ret);
    }
  }
  LOG_WRN("VESC hardware self-test active: slot 0 -> CAN ID 0, injected target limited to 0");
#else
  const fcan_gen::vescbus bus(0);
  if (vesc_configure_bus_timeout(motors[0], bus.param_feedback_timeout_ms()) != 0) {
    LOG_WRN("VESC bus feedback_timeout_ms rejected");
  }
  for (uint8_t i = 0; i < VESC_MOTOR_SLOT_COUNT; ++i) {
    const fcan_gen::vescmotor motor(i);
    struct vesc_slot_runtime_config config{
      motor.param_vesc_can_id(),
      motor.param_max_current_a(),
      motor.param_feedback_timeout_ms() != 0U ? motor.param_feedback_timeout_ms() :
                                                bus.param_feedback_timeout_ms(),
      motor.param_reconnect_frames() != 0U ? motor.param_reconnect_frames() :
                                             bus.param_reconnect_frames(),
      motor.param_command_period_us() != 0U ? motor.param_command_period_us() :
                                              bus.param_command_period_us(),
    };
    const int ret = vesc_configure_slot(motors[i], &config);
    if (ret != 0) {
      LOG_WRN("VESC slot %u configuration rejected (%d)", i, ret);
    }
  }
#endif
}

static int init()
{
  for (const auto * motor : motors) if (!device_is_ready(motor)) return -ENODEV;
  return 0;
}

static int start()
{
  fcan_gen::vescbus(0).on_params_changed([](uint32_t) noexcept { request_config(); });
  for (uint8_t i = 0; i < VESC_MOTOR_SLOT_COUNT; ++i) {
    fcan_gen::vescmotor(i).on_params_changed([](uint32_t) noexcept { request_config(); });
  }
  apply_config();
  return 0;
}

static void tick()
{
  k_spinlock_key_t key = k_spin_lock(&lock);
  const bool pending = config_pending;
  config_pending = false;
  k_spin_unlock(&lock, key);
  if (pending) apply_config();
}

FIBRIL_FCAN_FUNC_DEFINE(vesc, .array = fcan_gen::vescmotor::block_array_index,
  .count = VESC_MOTOR_SLOT_COUNT, .init = init, .start = start, .tick = tick);

FIBRIL_FCAN_FUNC_DEFINE(vesc_bus, .array = fcan_gen::vescbus::block_array_index,
  .count = 1);
