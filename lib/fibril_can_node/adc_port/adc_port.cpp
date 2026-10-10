/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * The inputs MdMotor's calibration watches. Binds the schema's AdcPort block
 * type to the devicetree's children, one instance per child, and samples
 * every one of them each adc_port::period_ticks for md_motor to read.
 */

#define DT_DRV_COMPAT fibril_fcan_adc_port

#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <drivers/motor.h>
#include <fibril_can_node/adc_port.hpp>
#include <fibril_can_node/func.h>

#include "schema_gen.hpp"

LOG_MODULE_REGISTER(fcan_adc_port, CONFIG_FIBRIL_CAN_NODE_LOG_LEVEL);

BUILD_ASSERT(
  DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
  "exactly one enabled fibril,fcan-adc-port node; put every port in its children");

#define CHECK_ONE_SOURCE(child)                                                                    \
  BUILD_ASSERT(                                                                                    \
    DT_NODE_HAS_PROP(child, io_channels) != DT_NODE_HAS_PROP(child, motor),                        \
    "fibril,fcan-adc-port: " DT_NODE_PATH(child) " needs exactly one of io-channels and motor");

DT_INST_FOREACH_CHILD(0, CHECK_ONE_SOURCE)

/* Indexed by port. A port reads whichever of the two it has; the other entry
 * is empty. */
#define ADC_SPEC_OR_EMPTY(child)                                                                   \
  COND_CODE_1(DT_NODE_HAS_PROP(child, io_channels), (ADC_DT_SPEC_GET(child)), ({}))

#define MOTOR_OR_NULL(child)                                                                       \
  COND_CODE_1(                                                                                     \
    DT_NODE_HAS_PROP(child, motor), (DEVICE_DT_GET(DT_PHANDLE(child, motor))), (nullptr))

static const struct adc_dt_spec adcs[] = {DT_INST_FOREACH_CHILD_SEP(0, ADC_SPEC_OR_EMPTY, (, ))};

static const struct device * const motors[] = {
  DT_INST_FOREACH_CHILD_SEP(0, MOTOR_OR_NULL, (, ))};

BUILD_ASSERT(ARRAY_SIZE(adcs) <= fcan_gen::adcport::max_count,
             "more ports listed than CONFIG_FIBRIL_CAN_NODE_ADC_PORT_MAX");

/* CanMotorMbed's RoboMasterTorqueSensor::read(). */
static constexpr float motor_full_scale = 10000.0F;

static float values[ARRAY_SIZE(adcs)];
static bool valid[ARRAY_SIZE(adcs)];
static uint32_t sample_generation;
static uint32_t ticks;

static bool sample_adc(const struct adc_dt_spec * spec, float & value)
{
  int16_t raw = 0;
  struct adc_sequence seq = {};
  int ret;

  seq.buffer = &raw;
  seq.buffer_size = sizeof(raw);

  ret = adc_sequence_init_dt(spec, &seq);
  if (ret == 0) {
    ret = adc_read_dt(spec, &seq);
  }
  if (ret != 0) {
    LOG_WRN_ONCE("%s channel %u: read failed (%d)", spec->dev->name, spec->channel_id, ret);
    return false;
  }

  value = (float)raw / (float)BIT_MASK(spec->resolution);

  return true;
}

static bool sample_motor(const struct device * motor, float & value)
{
  struct motor_feedback fb;

  if ((motor_get_feedback(motor, &fb) != 0) || ((fb.valid_mask & MOTOR_FEEDBACK_CURRENT) == 0U) ||
      !fb.online || fb.stale) {
    return false;
  }

  value = (float)fb.current / motor_full_scale;

  return true;
}

size_t adc_port::count()
{
  return ARRAY_SIZE(adcs);
}

uint32_t adc_port::generation()
{
  return sample_generation;
}

bool adc_port::read(uint8_t port, float & value)
{
  if ((port >= ARRAY_SIZE(adcs)) || !valid[port]) {
    return false;
  }

  value = values[port];

  return true;
}

float adc_port::threshold(uint8_t port)
{
  return fcan_gen::adcport(port).param_threshold();
}

static int adc_port_func_init(void)
{
  for (size_t i = 0; i < ARRAY_SIZE(adcs); i++) {
    if (motors[i] != nullptr) {
      if (!device_is_ready(motors[i])) {
        LOG_ERR("port %u: motor %s not ready", (unsigned)i, motors[i]->name);
        return -ENODEV;
      }
      continue;
    }

    if (!adc_is_ready_dt(&adcs[i])) {
      LOG_ERR("port %u: %s not ready", (unsigned)i, adcs[i].dev->name);
      return -ENODEV;
    }

    const int ret = adc_channel_setup_dt(&adcs[i]);

    if (ret != 0) {
      LOG_ERR("port %u: channel setup failed (%d)", (unsigned)i, ret);
      return ret;
    }
  }

  return 0;
}

static void adc_port_func_tick(void)
{
  if (++ticks < adc_port::period_ticks) {
    return;
  }
  ticks = 0;

  for (size_t i = 0; i < ARRAY_SIZE(adcs); i++) {
    valid[i] = (motors[i] != nullptr) ? sample_motor(motors[i], values[i])
                                      : sample_adc(&adcs[i], values[i]);
  }

  sample_generation++;
}

/* Not `adc_port`, which names the namespace md_motor reads through. */
FIBRIL_FCAN_FUNC_DEFINE(
  adc_ports,
  .array = fcan_gen::adcport::block_array_index,
  .count = (uint8_t)ARRAY_SIZE(adcs),
  .init = adc_port_func_init,
  .tick = adc_port_func_tick);
