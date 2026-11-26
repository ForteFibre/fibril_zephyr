/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cmath>

#include <zephyr/ztest.h>

#include <motor_control/sensing.hpp>

using motor_control::CurrentBaseline;
using motor_control::LevelWatch;

using Event = LevelWatch::Event;
using State = CurrentBaseline::State;

ZTEST(sensing, test_level_watch_is_silent_until_started)
{
  LevelWatch w;
  zassert_equal(w.update(1.0f, 0.5f), Event::NONE);
  zassert_false(w.active());
}

ZTEST(sensing, test_level_watch_reports_the_first_level_after_start)
{
  LevelWatch w;
  w.start();
  zassert_equal(w.update(0.2f, 0.5f), Event::LOW);
  zassert_equal(w.update(0.3f, 0.5f), Event::NONE);

  w.start();
  zassert_equal(w.update(0.3f, 0.5f), Event::LOW);
}

ZTEST(sensing, test_level_watch_reports_each_crossing_once)
{
  LevelWatch w;
  w.start();
  w.update(0.0f, 0.5f);
  zassert_equal(w.update(0.5f, 0.5f), Event::HIGH);
  zassert_equal(w.update(0.9f, 0.5f), Event::NONE);
  zassert_equal(w.update(0.49f, 0.5f), Event::LOW);
  zassert_equal(w.update(0.1f, 0.5f), Event::NONE);
}

ZTEST(sensing, test_level_watch_compares_the_magnitude)
{
  LevelWatch w;
  w.start();
  zassert_equal(w.update(-0.8f, 0.5f), Event::HIGH);
  zassert_equal(w.update(0.8f, 0.5f), Event::NONE);
}

ZTEST(sensing, test_level_watch_skips_non_finite_samples_and_keeps_the_level)
{
  LevelWatch w;
  w.start();
  w.update(0.9f, 0.5f);
  zassert_equal(w.update(NAN, 0.5f), Event::NONE);
  zassert_equal(w.update(0.1f, NAN), Event::NONE);
  zassert_equal(w.update(0.9f, 0.5f), Event::NONE);
}

ZTEST(sensing, test_level_watches_on_one_input_each_see_the_crossing)
{
  LevelWatch a;
  LevelWatch b;
  a.start();
  b.start();
  a.update(0.0f, 0.5f);
  b.update(0.0f, 0.5f);
  zassert_equal(a.update(1.0f, 0.5f), Event::HIGH);
  zassert_equal(b.update(1.0f, 0.5f), Event::HIGH);
}

ZTEST(sensing, test_level_watch_stops)
{
  LevelWatch w;
  w.start();
  w.update(0.0f, 0.5f);
  w.stop();
  zassert_equal(w.update(1.0f, 0.5f), Event::NONE);
}

ZTEST(sensing, test_baseline_averages_over_the_run)
{
  CurrentBaseline b;
  b.start(4);
  zassert_true(b.running());
  zassert_equal(b.update(true, true, 1.0f), State::RUNNING);
  zassert_equal(b.update(true, true, 2.0f), State::RUNNING);
  zassert_equal(b.update(true, true, 3.0f), State::RUNNING);
  zassert_equal(b.update(true, true, 6.0f), State::DONE);
  zassert_within(b.offset(), 3.0f, 1e-6f);
  zassert_false(b.running());
  zassert_equal(b.update(true, true, 9.0f), State::IDLE);
}

ZTEST(sensing, test_baseline_aborts_on_motion_and_keeps_the_last_offset)
{
  CurrentBaseline b;
  b.start(1);
  b.update(true, true, 0.5f);

  b.start(10);
  b.update(true, true, 7.0f);
  zassert_equal(b.update(false, true, 7.0f), State::ABORTED);
  zassert_false(b.running());
  zassert_within(b.offset(), 0.5f, 1e-6f);
}

ZTEST(sensing, test_baseline_aborts_without_a_current_reading)
{
  CurrentBaseline b;
  b.start(10);
  zassert_equal(b.update(true, false, 0.0f), State::ABORTED);

  b.start(10);
  zassert_equal(b.update(true, true, NAN), State::ABORTED);
}

ZTEST(sensing, test_baseline_of_zero_ticks_takes_one_sample)
{
  CurrentBaseline b;
  b.start(0);
  zassert_equal(b.update(true, true, 2.0f), State::DONE);
  zassert_within(b.offset(), 2.0f, 1e-6f);
}

ZTEST(sensing, test_baseline_idles_until_started)
{
  CurrentBaseline b;
  zassert_equal(b.update(true, true, 1.0f), State::IDLE);
  zassert_within(b.offset(), 0.0f, 1e-6f);
}

ZTEST_SUITE(sensing, NULL, NULL, NULL, NULL, NULL);
