/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Quadrature encoder decoded by an STM32 general purpose timer.
 *
 * The timer counts edges on its own; this driver only reads the counter often
 * enough that the accumulated position stays continuous, and turns the raw
 * counter into the values a control loop wants. Reading is deliberately not
 * done on demand: a consumer that stops asking for a while, or two consumers
 * asking at different rates, would let the counter wrap unobserved and the
 * accumulated position could not be recovered.
 */

#define DT_DRV_COMPAT fibril_stm32_qdec

#include <errno.h>

#include <drivers/encoder.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <stm32_ll_tim.h>

#include "encoder_accum.h"

LOG_MODULE_REGISTER(encoder_qdec_stm32, CONFIG_ENCODER_LOG_LEVEL);

/* The name of the direct input selection changed between Cube versions. */
#ifdef CONFIG_STM32_HAL2
#define QDEC_STM32_ACTIVEINPUT_DIRECT LL_TIM_ACTIVEINPUT_DIRECT
#else
#define QDEC_STM32_ACTIVEINPUT_DIRECT LL_TIM_ACTIVEINPUT_DIRECTTI
#endif

struct qdec_stm32_config
{
  const struct pinctrl_dev_config * pincfg;
  struct stm32_pclken pclken;
  TIM_TypeDef * timer;
  uint32_t encoder_mode;
  uint32_t poll_interval_us;
  uint8_t filter_level;
  bool invert;
};

struct qdec_stm32_data
{
  struct k_spinlock lock;
  struct encoder_feedback feedback;
  struct encoder_accum accum;
  struct k_timer timer;
};

static uint64_t qdec_stm32_read_counter(const struct device * dev)
{
  const struct qdec_stm32_config * config = dev->config;

  return (uint64_t)LL_TIM_GetCounter(config->timer);
}

/**
 * @brief Sample the counter and fold it into the accumulated position.
 *
 * Runs from the k_timer expiry, which is interrupt context. That is on purpose:
 * the work is one register read and a handful of integer operations, and
 * handing it to a work queue would only add the scheduling delay to the
 * interval the velocity is divided by.
 */
static void qdec_stm32_sample(struct k_timer * timer)
{
  const struct device * dev = k_timer_user_data_get(timer);
  struct qdec_stm32_data * data = dev->data;
  struct encoder_accum_sample sample;
  k_spinlock_key_t key;

  /* Both reads belong inside the lock. A reset that lands between reading the
   * counter and folding it in would otherwise have its zeroed counter
   * subtracted from a reading taken before it, inventing motion.
   */
  key = k_spin_lock(&data->lock);

  encoder_accum_update(&data->accum, qdec_stm32_read_counter(dev), k_cycle_get_32(), &sample);

  data->feedback.valid_mask = ENCODER_FEEDBACK_POSITION;
  data->feedback.position = sample.position;

  if (sample.has_velocity) {
    data->feedback.valid_mask |= ENCODER_FEEDBACK_VELOCITY;
    data->feedback.velocity = sample.velocity;
    data->feedback.sample_interval_us = sample.sample_interval_us;
  }

  data->feedback.online = true;
  data->feedback.stale = false;
  data->feedback.timestamp_ms = k_uptime_get();

  k_spin_unlock(&data->lock, key);
}

static int qdec_stm32_get_feedback(const struct device * dev, void * feedback)
{
  struct qdec_stm32_data * data = dev->data;
  struct encoder_feedback * out = feedback;
  k_spinlock_key_t key;
  int ret = 0;

  if (out == NULL) {
    return -EINVAL;
  }

  key = k_spin_lock(&data->lock);

  /* The counter itself cannot fail, so the only state a reader has to be told
   * about is that no sample has been taken yet.
   */
  if (data->feedback.valid_mask == 0U) {
    ret = -ENODATA;
  }

  *out = data->feedback;

  k_spin_unlock(&data->lock, key);

  return ret;
}

static int qdec_stm32_get_resolution(const struct device * dev, uint8_t * resolution)
{
  ARG_UNUSED(dev);
  ARG_UNUSED(resolution);

  /* A resolution here means the width of a single-turn absolute position, and
   * a quadrature encoder without an index pulse does not have one.
   */
  return -ENOTSUP;
}

static int qdec_stm32_set_zero(const struct device * dev)
{
  ARG_UNUSED(dev);

  /* Answered here rather than by leaving the vtable slot empty, which would
   * give -ENOSYS and read as an unimplemented driver. The device genuinely has
   * nowhere to store a zero point; encoder_set_position() is the way to make
   * the current position read as zero.
   */
  return -ENOTSUP;
}

static int qdec_stm32_set_position(const struct device * dev, int64_t position)
{
  struct qdec_stm32_data * data = dev->data;
  k_spinlock_key_t key;
  int ret;

  key = k_spin_lock(&data->lock);

  ret = encoder_accum_set_position(&data->accum, position);

  if (ret == 0) {
    data->feedback.position = position;
  }

  k_spin_unlock(&data->lock, key);

  return ret;
}

static int qdec_stm32_reset(const struct device * dev)
{
  const struct qdec_stm32_config * config = dev->config;
  struct qdec_stm32_data * data = dev->data;
  k_spinlock_key_t key;

  /* Zeroing the counter and rebuilding the accumulator have to happen together,
   * or the sample that the k_timer takes in between is folded in as motion.
   */
  key = k_spin_lock(&data->lock);

  LL_TIM_SetCounter(config->timer, 0);
  encoder_accum_reset(&data->accum, 0, 0);

  data->feedback.position = 0;
  data->feedback.velocity = 0;
  data->feedback.sample_interval_us = 0U;
  data->feedback.valid_mask = ENCODER_FEEDBACK_POSITION;
  data->feedback.position_epoch++;

  k_spin_unlock(&data->lock, key);

  return 0;
}

static const struct encoder_driver_api qdec_stm32_api = {
  .get_feedback = qdec_stm32_get_feedback,
  .get_resolution = qdec_stm32_get_resolution,
  .set_zero = qdec_stm32_set_zero,
  .set_position = qdec_stm32_set_position,
  .reset = qdec_stm32_reset,
};

static void qdec_stm32_init_channel(const struct device * dev, uint32_t channel)
{
  const struct qdec_stm32_config * config = dev->config;

  LL_TIM_IC_SetActiveInput(config->timer, channel, QDEC_STM32_ACTIVEINPUT_DIRECT);
  LL_TIM_IC_SetFilter(config->timer, channel, config->filter_level * LL_TIM_IC_FILTER_FDIV1_N2);
  LL_TIM_IC_SetPrescaler(config->timer, channel, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetPolarity(config->timer, channel, LL_TIM_IC_POLARITY_RISING);
}

static int qdec_stm32_init(const struct device * dev)
{
  const struct qdec_stm32_config * config = dev->config;
  struct qdec_stm32_data * data = dev->data;
  uint8_t width_bits;
  int ret;

  ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
  if (ret < 0) {
    LOG_ERR("%s: could not apply pinctrl state (%d)", dev->name, ret);
    return ret;
  }

  ret = clock_control_on(
    DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE), (clock_control_subsys_t)&config->pclken);
  if (ret < 0) {
    LOG_ERR("%s: could not enable the timer clock (%d)", dev->name, ret);
    return ret;
  }

  /* Count over the whole counter so that it wraps at a power of two and the
   * accumulator can resolve the wrap with a plain modular difference.
   */
  width_bits = IS_TIM_32B_COUNTER_INSTANCE(config->timer) ? 32U : 16U;
  LL_TIM_SetAutoReload(config->timer, (width_bits == 32U) ? UINT32_MAX : UINT16_MAX);

  /* Despite the name this writes SMCR.SMS, which is where the encoder mode
   * lives, and clears SMCR.ECE at the same time. The SMS mask includes bit 3
   * (0x00010007), so the x1 modes survive it.
   */
  LL_TIM_SetClockSource(config->timer, config->encoder_mode);

  qdec_stm32_init_channel(dev, LL_TIM_CHANNEL_CH1);
  qdec_stm32_init_channel(dev, LL_TIM_CHANNEL_CH2);

  LL_TIM_CC_EnableChannel(config->timer, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
  LL_TIM_SetCounter(config->timer, 0);
  LL_TIM_EnableCounter(config->timer);

  encoder_accum_init(&data->accum, width_bits, config->invert);
  /* An incremental encoder has no absolute reference, so wherever the shaft
   * happens to be at boot is position zero.
   */
  encoder_accum_reset(&data->accum, 0, 0);

  k_timer_init(&data->timer, qdec_stm32_sample, NULL);
  k_timer_user_data_set(&data->timer, (void *)dev);
  k_timer_start(
    &data->timer, K_USEC(config->poll_interval_us), K_USEC(config->poll_interval_us));

  return 0;
}

#define QDEC_STM32_INIT(inst)                                                                  \
  BUILD_ASSERT(                                                                                \
    DT_PROP(DT_INST_PARENT(inst), st_prescaler) == 0,                                          \
    "st,prescaler must be 0: the prescaler divides in encoder mode too, which drops counts");  \
                                                                                               \
  BUILD_ASSERT(                                                                                \
    !(DT_INST_PROP(inst, encoder_mode) & ~TIM_SMCR_SMS),                                       \
    "encoder-mode is not supported by this MCU");                                              \
                                                                                               \
  BUILD_ASSERT(DT_INST_PROP(inst, counts_per_revolution) > 0, "counts-per-revolution must be positive"); \
                                                                                               \
  BUILD_ASSERT(DT_INST_PROP(inst, poll_interval_us) > 0, "poll-interval-us must be positive"); \
                                                                                               \
  PINCTRL_DT_INST_DEFINE(inst);                                                                \
                                                                                               \
  static const struct qdec_stm32_config qdec_stm32_config_##inst = {                           \
    .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                            \
    .pclken = STM32_CLOCK_INFO(0, DT_INST_PARENT(inst)),                                       \
    .timer = (TIM_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(inst)),                                 \
    .encoder_mode = DT_INST_PROP(inst, encoder_mode),                                          \
    .poll_interval_us = DT_INST_PROP(inst, poll_interval_us),                                  \
    .filter_level = DT_INST_PROP(inst, input_filter_level),                                    \
    .invert = DT_INST_PROP(inst, invert_direction),                                            \
  };                                                                                           \
                                                                                               \
  static struct qdec_stm32_data qdec_stm32_data_##inst;                                        \
                                                                                               \
  DEVICE_DT_INST_DEFINE(                                                                       \
    inst, qdec_stm32_init, NULL, &qdec_stm32_data_##inst, &qdec_stm32_config_##inst,           \
    POST_KERNEL, CONFIG_ENCODER_INIT_PRIORITY, &qdec_stm32_api);

DT_INST_FOREACH_STATUS_OKAY(QDEC_STM32_INIT)
