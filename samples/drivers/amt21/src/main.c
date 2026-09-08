/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal AMT21x reader. Enumerates every cui,amt21-encoder child that the
 * devicetree marks status = "okay" and prints its feedback once per second.
 */

#include <inttypes.h>
#include <stdlib.h>

#include <drivers/encoder.h>
#include <drivers/encoder/amt21.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(amt21_read, LOG_LEVEL_INF);

#define REPORT_PERIOD K_MSEC(1000)

#define ENCODER_LIST_ENTRY(node_id) DEVICE_DT_GET(node_id),

static const struct device * const encoders[] = {
	DT_FOREACH_STATUS_OKAY(cui_amt21_encoder, ENCODER_LIST_ENTRY)
};

BUILD_ASSERT(ARRAY_SIZE(encoders) > 0,
	     "Enable at least one cui,amt21-encoder node in the devicetree");

static void report(const struct device * dev)
{
	struct encoder_feedback fb;
	int ret = encoder_get_feedback(dev, &fb);

	/* A stale reading still carries the last good value, so it is worth
	 * printing along with the reason it is not fresh.
	 */
	/* Velocity has no value until a second reading gives it an interval to be
	 * measured over, so printing it unconditionally would show a stale number
	 * after every rebuild of the accumulator.
	 */
	if ((fb.valid_mask & ENCODER_FEEDBACK_VELOCITY) != 0U) {
		LOG_INF("%s: ret=%d pos=%" PRId64 " vel=%d single=%u turns=%d epoch=%u online=%d "
			"stale=%d errors=%u",
			dev->name, ret, fb.position, fb.velocity, fb.single_turn, fb.turns,
			fb.position_epoch, (int)fb.online, (int)fb.stale, fb.error_count);
	} else {
		LOG_INF("%s: ret=%d pos=%" PRId64 " vel=n/a single=%u turns=%d epoch=%u online=%d "
			"stale=%d errors=%u",
			dev->name, ret, fb.position, fb.single_turn, fb.turns,
			fb.position_epoch, (int)fb.online, (int)fb.stale, fb.error_count);
	}

#if defined(CONFIG_ENCODER_AMT21_STATS)
	struct amt21_stats stats;

	if (amt21_get_stats(dev, &stats) == 0) {
		LOG_INF("  transactions=%u ok=%u timeout=%u checksum=%u desync=%u echo=%u bus=%u retries=%u",
			stats.transactions, stats.successes,
			stats.errors[AMT21_ERROR_TIMEOUT],
			stats.errors[AMT21_ERROR_CHECKSUM],
			stats.errors[AMT21_ERROR_DESYNC],
			stats.errors[AMT21_ERROR_ECHO],
			stats.errors[AMT21_ERROR_BUS],
			stats.retries);
	}
#endif
}

int main(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(encoders); ++i) {
		if (!device_is_ready(encoders[i])) {
			LOG_ERR("%s is not ready", encoders[i]->name);
			return -ENODEV;
		}
	}

	while (true) {
		for (size_t i = 0; i < ARRAY_SIZE(encoders); ++i) {
			report(encoders[i]);
		}
		k_sleep(REPORT_PERIOD);
	}

	return 0;
}
