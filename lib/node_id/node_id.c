/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include <node_id/node_id.h>

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define NODE_ID_SWITCH DT_CHOSEN(fibril_node_id)

/* 0x7F, the broadcast address. */
#define NODE_ID_BROADCAST 0x7FU

BUILD_ASSERT(DT_NODE_HAS_COMPAT(NODE_ID_SWITCH, fibril_id_switch),
             "the fibril,node-id chosen node must be a fibril,id-switch");
BUILD_ASSERT(DT_PROP_LEN(NODE_ID_SWITCH, gpios) <= 7,
             "a fibril_can node id is 7 bits; the switch has more");

static const struct gpio_dt_spec bits[] = {
  DT_FOREACH_PROP_ELEM_SEP(NODE_ID_SWITCH, gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))
};

/* Long enough for the internal pull-ups (tens of kilohms) to charge the switch
 * lines' stray capacitance, which takes microseconds; a millisecond costs
 * nothing at boot. */
#define NODE_ID_SETTLE_MS 1

int node_id_read(uint8_t * id)
{
  uint8_t value = 0U;
  int ret;

  for (size_t i = 0; i < ARRAY_SIZE(bits); i++) {
    if (!gpio_is_ready_dt(&bits[i])) {
      return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&bits[i], GPIO_INPUT);
    if (ret < 0) {
      return ret;
    }
  }

  k_msleep(NODE_ID_SETTLE_MS);

  for (size_t i = 0; i < ARRAY_SIZE(bits); i++) {
    ret = gpio_pin_get_dt(&bits[i]);
    if (ret < 0) {
      return ret;
    }

    value |= (uint8_t)((ret != 0) ? BIT(i) : 0U);
  }

  if (value == NODE_ID_BROADCAST) {
    return -ERANGE;
  }

  *id = value;

  return 0;
}
