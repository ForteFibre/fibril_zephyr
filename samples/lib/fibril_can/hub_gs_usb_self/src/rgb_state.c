/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * See rgb_state.h for the design notes.
 */

#include "rgb_state.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(fcan_rgb, LOG_LEVEL_INF);

/* All three LED nodes must be present-and-enabled for the indicator to make
 * sense. Missing any of them is not an error -- native_sim and boards that
 * chose different aliases just get the no-op path. */
#define FCAN_RGB_HAVE_LEDS                                                     \
	(DT_NODE_HAS_STATUS(DT_NODELABEL(led_r), okay) &&                      \
	 DT_NODE_HAS_STATUS(DT_NODELABEL(led_g), okay) &&                      \
	 DT_NODE_HAS_STATUS(DT_NODELABEL(led_b), okay))

#if FCAN_RGB_HAVE_LEDS

/* 500 ms on / 500 ms off. Slow enough that a technician sees "blinking =
 * waiting" without it looking like a fault indicator. */
#define BLINK_PERIOD_MS 1000U
#define BLINK_ON_MS     500U

static const struct gpio_dt_spec led_r =
	GPIO_DT_SPEC_GET(DT_NODELABEL(led_r), gpios);
static const struct gpio_dt_spec led_g =
	GPIO_DT_SPEC_GET(DT_NODELABEL(led_g), gpios);
static const struct gpio_dt_spec led_b =
	GPIO_DT_SPEC_GET(DT_NODELABEL(led_b), gpios);

static bool g_ready;

static void apply(bool r, bool g, bool b)
{
	/* gpio_pin_set() honours GPIO_ACTIVE_LOW from the DT flags, so we pass
	 * the logical value (1 = LED on) and the driver flips it for us. */
	(void)gpio_pin_set_dt(&led_r, r ? 1 : 0);
	(void)gpio_pin_set_dt(&led_g, g ? 1 : 0);
	(void)gpio_pin_set_dt(&led_b, b ? 1 : 0);
}

void rgb_state_init(void)
{
	if (!gpio_is_ready_dt(&led_r) || !gpio_is_ready_dt(&led_g) ||
	    !gpio_is_ready_dt(&led_b)) {
		LOG_WRN("one or more LED GPIOs not ready; indicator disabled");
		g_ready = false;
		return;
	}

	int rc = gpio_pin_configure_dt(&led_r, GPIO_OUTPUT_INACTIVE);
	rc |= gpio_pin_configure_dt(&led_g, GPIO_OUTPUT_INACTIVE);
	rc |= gpio_pin_configure_dt(&led_b, GPIO_OUTPUT_INACTIVE);
	if (rc != 0) {
		LOG_WRN("LED configure failed (rc=%d); indicator disabled", rc);
		g_ready = false;
		return;
	}

	g_ready = true;
}

void rgb_state_update(fcan_node_state_t state, fcan_fault_t fault)
{
	if (!g_ready) {
		return;
	}

	if (fault != FCAN_FAULT_NONE) {
		apply(true, false, false);
		return;
	}

	switch (state) {
	case FCAN_STATE_RUNNING:
		apply(false, true, false);
		break;
	case FCAN_STATE_PROVISIONED:
		apply(false, false, true);
		break;
	case FCAN_STATE_FAULT:
		apply(true, false, false);
		break;
	case FCAN_STATE_UNPROVISIONED:
	default: {
		const uint32_t phase = k_uptime_get_32() % BLINK_PERIOD_MS;
		apply(false, false, phase < BLINK_ON_MS);
		break;
	}
	}
}

#else /* !FCAN_RGB_HAVE_LEDS */

void rgb_state_init(void)
{
}

void rgb_state_update(fcan_node_state_t state, fcan_fault_t fault)
{
	(void)state;
	(void)fault;
}

#endif /* FCAN_RGB_HAVE_LEDS */
