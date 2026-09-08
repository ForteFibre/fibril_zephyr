/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FIBRIL_ZEPHYR_DRIVERS_ENCODER_ENCODER_ACCUM_H_
#define FIBRIL_ZEPHYR_DRIVERS_ENCODER_ENCODER_ACCUM_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * @file
 * @brief Continuous position accumulator shared by the encoder drivers.
 *
 * Every encoder in this repository reports a counter that wraps: a single-turn
 * absolute reading on the AMT21x, a hardware timer counter on a quadrature
 * input. Turning that into the accumulated @ref encoder_feedback.position is
 * the same problem in both cases, and it is the part of an encoder driver that
 * is easiest to get wrong, so it lives here where a test can reach it without a
 * device.
 *
 * A driver owns the epoch counter and the valid_mask rather than this helper,
 * because what makes an accumulator lose continuity differs between devices.
 * The helper only says whether it currently has something to continue from.
 */

/**
 * @brief Largest magnitude encoder_accum_set_position() accepts.
 *
 * Both the offset itself and every later sum of the raw count and the offset
 * have to stay inside int64_t, and signed overflow is undefined. Capping the
 * requested position well short of the type leaves the accumulator room to keep
 * counting: at the roughly 1.2 M counts/s a RoboMaster rotor reaches, the
 * remaining headroom is measured in millennia.
 */
#define ENCODER_ACCUM_POSITION_LIMIT (INT64_C(1) << 62)

/** @brief Accumulator state. Treat the fields as private to the helper. */
struct encoder_accum
{
  /** Width of the underlying counter in bits. */
  uint8_t width_bits;
  /** BIT64(width_bits) - 1, for reducing a reading to the counter's width. */
  uint64_t mask;
  /** Whether the sign of the motion is flipped. */
  bool invert;

  /** False until a reading has been established to continue from. */
  bool valid;
  /** False until a second reading has arrived and an interval can be measured. */
  bool has_prev_cycle;

  uint64_t prev_raw;
  /** What the readings add up to, before the offset. */
  int64_t raw_count;
  /** Offset requested through encoder_accum_set_position(). */
  int64_t bias;
  uint32_t prev_cycle;
};

/** @brief What one call to encoder_accum_update() produced. */
struct encoder_accum_sample
{
  /** Accumulated position with the offset applied. */
  int64_t position;
  /** Velocity in counts per second. Only meaningful when has_velocity. */
  int32_t velocity;
  /** Measured interval the velocity was derived over. */
  uint32_t sample_interval_us;
  /**
   * @brief False when no velocity could be derived for this sample.
   *
   * That is the case for the first reading after the accumulator was reset,
   * and for a reading that lands in the same cycle as the previous one. A
   * caller should leave the previously reported velocity alone and drop the
   * ENCODER_FEEDBACK_VELOCITY bit for this sample rather than report a zero.
   */
  bool has_velocity;
};

/**
 * @brief Prepare an accumulator for a counter of a given width.
 *
 * The accumulator starts invalid; encoder_accum_reset() gives it something to
 * continue from.
 *
 * @param accum Accumulator to initialise.
 * @param width_bits Width of the underlying counter, 1 to 32.
 * @param invert Report motion with the opposite sign.
 */
void encoder_accum_init(struct encoder_accum * accum, uint8_t width_bits, bool invert);

/**
 * @brief Start the accumulator over at a known position.
 *
 * Any offset set through encoder_accum_set_position() is dropped, and the next
 * update produces no velocity because there is no interval to measure yet. The
 * caller is responsible for advancing its epoch counter.
 *
 * @param accum Accumulator to reset.
 * @param start Accumulated position the current reading stands for. Absolute
 *        encoders pass the absolute count they decoded; incremental ones pass 0.
 * @param raw Raw counter value the next difference is taken from.
 */
void encoder_accum_reset(struct encoder_accum * accum, int64_t start, uint64_t raw);

/**
 * @brief Mark the accumulator as having nothing to continue from.
 *
 * Use this when the counter may have moved further than half a span unobserved,
 * which makes the next difference unresolvable. The following
 * encoder_accum_reset() rebuilds the accumulator.
 *
 * @param accum Accumulator to invalidate.
 */
void encoder_accum_invalidate(struct encoder_accum * accum);

/**
 * @brief Whether the accumulator has a reading to continue from.
 *
 * @param accum Accumulator to query.
 * @return True when encoder_accum_update() may be called.
 */
static inline bool encoder_accum_is_valid(const struct encoder_accum * accum)
{
  return accum->valid;
}

/**
 * @brief Accumulated position with the offset applied.
 *
 * @param accum Accumulator to query.
 * @return The position that the latest update produced.
 */
static inline int64_t encoder_accum_position(const struct encoder_accum * accum)
{
  return accum->raw_count + accum->bias;
}

/**
 * @brief Advance the accumulator by a new counter reading.
 *
 * The shorter of the two ways round is taken as the real motion, which holds as
 * long as the counter moves by less than half a span between two readings. A
 * difference of exactly half a span is taken as forward motion.
 *
 * The velocity is derived from that difference rather than from the change in
 * the reported position. The two are the same in the steady state, but taking
 * the difference of positions would turn an encoder_accum_set_position() that
 * lands between two readings into a velocity spike.
 *
 * @param accum Accumulator to advance. Must be valid.
 * @param raw Raw counter value, reduced to the counter's width by the helper.
 * @param now_cycle Instant the reading was taken, from k_cycle_get_32().
 * @param sample Destination for the results.
 */
void encoder_accum_update(
  struct encoder_accum * accum, uint64_t raw, uint32_t now_cycle,
  struct encoder_accum_sample * sample);

/**
 * @brief Redefine the accumulated position.
 *
 * @param accum Accumulator to offset.
 * @param position Value the present position should read as.
 * @retval 0 Success.
 * @retval -EINVAL @p position exceeds @ref ENCODER_ACCUM_POSITION_LIMIT.
 * @retval -ENODATA The accumulator has nothing to offset from.
 */
int encoder_accum_set_position(struct encoder_accum * accum, int64_t position);

/**
 * @brief Signed motion between two raw counter values of a given width.
 *
 * The shorter of the two ways round is taken, and a difference of exactly half
 * a span counts as forward. Exposed separately from the accumulator because a
 * driver sometimes has to compare two readings without advancing anything, such
 * as when checking whether a wrap fell between two transactions.
 *
 * @param width_bits Width of the counter, 1 to 32.
 * @param current Later reading.
 * @param previous Earlier reading.
 * @return Signed difference, within plus or minus half a span.
 */
int64_t encoder_accum_wrap_delta(uint8_t width_bits, uint64_t current, uint64_t previous);

#endif
