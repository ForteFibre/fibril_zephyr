/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal quadrature encoder reader. Enumerates every fibril,stm32-qdec node
 * that the devicetree marks status = "okay" and prints its feedback once per
 * second, along with the position converted to revolutions so that turning the
 * shaft by hand is enough to check the wiring and counts-per-revolution.
 */

#include <inttypes.h>
#include <stdlib.h>

#include <drivers/encoder.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(qdec_read, LOG_LEVEL_INF);

#define REPORT_PERIOD K_MSEC(1000)

#define ENCODER_LIST_ENTRY(node_id) DEVICE_DT_GET(node_id),
#define CPR_LIST_ENTRY(node_id) DT_PROP(node_id, counts_per_revolution),

static const struct device *const encoders[] = {
	DT_FOREACH_STATUS_OKAY(fibril_stm32_qdec, ENCODER_LIST_ENTRY)
};

/* Counts to physical units is the application's job, so the sample reads the
 * per-device constant out of the devicetree rather than asking the driver.
 */
static const uint32_t counts_per_revolution[] = {
	DT_FOREACH_STATUS_OKAY(fibril_stm32_qdec, CPR_LIST_ENTRY)
};

BUILD_ASSERT(ARRAY_SIZE(encoders) > 0,
	     "Enable at least one fibril,stm32-qdec node in the devicetree");

static void report(const struct device *dev, uint32_t cpr)
{
	struct encoder_feedback fb;
	int ret = encoder_get_feedback(dev, &fb);

	if (ret == -ENODATA) {
		LOG_INF("%s: no sample yet", dev->name);
		return;
	}

	/* Whole revolutions and thousandths separately rather than scaling the
	 * position by 1000 first, which would overflow int64_t at the large
	 * positions encoder_set_position() accepts. Integers rather than a float
	 * so the sample works without CONFIG_CBPRINTF_FP_SUPPORT.
	 */
	int64_t magnitude = (fb.position < 0) ? -fb.position : fb.position;
	const char *sign = (fb.position < 0) ? "-" : "";
	int64_t rev = magnitude / (int64_t)cpr;
	int64_t rev_frac = ((magnitude % (int64_t)cpr) * 1000) / (int64_t)cpr;

	/* Velocity has no value until a second sample gives it an interval to be
	 * measured over, so printing it unconditionally would show a stale number
	 * after every rebuild of the accumulator.
	 */
	if ((fb.valid_mask & ENCODER_FEEDBACK_VELOCITY) != 0U) {
		int32_t mrev_per_s = (int32_t)((int64_t)fb.velocity * 1000 / (int64_t)cpr);

		LOG_INF("%s: pos=%" PRId64 " (%s%" PRId64 ".%03" PRId64 " rev) vel=%d counts/s "
			"(%d.%03d rev/s) interval=%uus epoch=%u",
			dev->name, fb.position, sign, rev, rev_frac, fb.velocity,
			mrev_per_s / 1000, abs(mrev_per_s % 1000), fb.sample_interval_us,
			fb.position_epoch);
	} else {
		LOG_INF("%s: pos=%" PRId64 " (%s%" PRId64 ".%03" PRId64 " rev) vel=n/a epoch=%u",
			dev->name, fb.position, sign, rev, rev_frac, fb.position_epoch);
	}
}


int main(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(encoders); i++) {
		if (!device_is_ready(encoders[i])) {
			LOG_ERR("%s: not ready", encoders[i]->name);
			return -ENODEV;
		}

		LOG_INF("%s: %u counts per revolution", encoders[i]->name,
			counts_per_revolution[i]);
	}

	while (true) {
		for (size_t i = 0; i < ARRAY_SIZE(encoders); i++) {
			report(encoders[i], counts_per_revolution[i]);
		}

		k_sleep(REPORT_PERIOD);
	}

	return 0;
}
