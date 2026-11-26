/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include "encoder_accum.h"

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>

int64_t encoder_accum_wrap_delta(uint8_t width_bits, uint64_t current, uint64_t previous)
{
  __ASSERT((width_bits >= 1U) && (width_bits <= 32U), "counter width out of range");

  const uint64_t span = BIT64(width_bits);
  const uint64_t diff = (current - previous) & (span - 1U);

  if (diff > (span / 2U)) {
    return (int64_t)diff - (int64_t)span;
  }

  return (int64_t)diff;
}

/** @brief Signed motion since the previous reading, with the inversion applied. */
static int64_t encoder_accum_delta(const struct encoder_accum * accum, uint64_t raw)
{
  int64_t delta = encoder_accum_wrap_delta(accum->width_bits, raw, accum->prev_raw);

  return accum->invert ? -delta : delta;
}

void encoder_accum_init(struct encoder_accum * accum, uint8_t width_bits, bool invert)
{
  __ASSERT((width_bits >= 1U) && (width_bits <= 32U), "counter width out of range");

  accum->width_bits = width_bits;
  accum->mask = BIT64(width_bits) - 1U;
  accum->invert = invert;

  accum->valid = false;
  accum->has_prev_cycle = false;
  accum->prev_raw = 0U;
  accum->raw_count = 0;
  accum->bias = 0;
  accum->prev_cycle = 0U;
}

void encoder_accum_reset(struct encoder_accum * accum, int64_t start, uint64_t raw)
{
  accum->raw_count = start;
  accum->bias = 0;
  accum->prev_raw = raw & accum->mask;
  accum->valid = true;
  accum->has_prev_cycle = false;
}

void encoder_accum_invalidate(struct encoder_accum * accum)
{
  accum->valid = false;
}

void encoder_accum_update(
  struct encoder_accum * accum, uint64_t raw, uint32_t now_cycle,
  struct encoder_accum_sample * sample)
{
  __ASSERT(accum->valid, "update on an accumulator with nothing to continue from");

  int64_t delta = encoder_accum_delta(accum, raw);

  accum->prev_raw = raw & accum->mask;
  accum->raw_count += delta;

  sample->position = encoder_accum_position(accum);
  sample->velocity = 0;
  sample->sample_interval_us = 0U;
  sample->has_velocity = false;

  if (accum->has_prev_cycle) {
    uint32_t interval_us = (uint32_t)k_cyc_to_us_floor64(now_cycle - accum->prev_cycle);

    if (interval_us > 0U) {
      /* A 32 bit counter can report half a span of motion in one interval,
       * which works out well past what int32_t holds: 2^31 counts over 1000 us
       * is 2.1e12 counts/s. Saturating keeps the sign and the "impossibly
       * fast" reading instead of wrapping to an arbitrary small number.
       */
      int64_t velocity = (delta * (int64_t)USEC_PER_SEC) / (int64_t)interval_us;

      sample->sample_interval_us = interval_us;
      sample->velocity = (int32_t)CLAMP(velocity, INT32_MIN, INT32_MAX);
      sample->has_velocity = true;
    }
  }

  accum->prev_cycle = now_cycle;
  accum->has_prev_cycle = true;
}

int encoder_accum_set_position(struct encoder_accum * accum, int64_t position)
{
  if ((position > ENCODER_ACCUM_POSITION_LIMIT) || (position < -ENCODER_ACCUM_POSITION_LIMIT)) {
    return -EINVAL;
  }

  if (!accum->valid) {
    return -ENODATA;
  }

  accum->bias = position - accum->raw_count;

  return 0;
}
