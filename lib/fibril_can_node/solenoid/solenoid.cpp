/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Solenoid valves on the bus. Binds the schema's Solenoid block type to the
 * GPIOs the devicetree names, one instance per entry of `gpios`.
 */

#define DT_DRV_COMPAT fibril_fcan_solenoid

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <fibril_can_node/func.h>

#include "schema_gen.hpp"

LOG_MODULE_REGISTER(fcan_solenoid, CONFIG_FIBRIL_CAN_NODE_LOG_LEVEL);

using set_req = fcan_gen::solenoid::set_req;
using set_resp = fcan_gen::solenoid::set_resp;

/* One node describes every valve the board drives, and the list below reads
 * instance 0 only. A second enabled node would build and then be invisible
 * from the bus, so refuse it here rather than ship an image that drives half
 * the valves.
 */
BUILD_ASSERT(
  DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
  "exactly one enabled fibril,fcan-solenoid node; put every valve in its gpios");

static const struct gpio_dt_spec valves[] = {
  DT_INST_FOREACH_PROP_ELEM_SEP(0, gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))
};

/* max_count is a Kconfig ceiling rather than the devicetree's length, because
 * CMake cannot read a phandle-array. An extra valve would otherwise be
 * silently unreachable from the bus.
 */
BUILD_ASSERT(ARRAY_SIZE(valves) <= fcan_gen::solenoid::max_count,
             "more gpios wired than CONFIG_FIBRIL_CAN_NODE_SOLENOID_MAX");

/* Mirrors what was last driven. Read back from the GPIO would report the pin,
 * not the request, and on a high-side driver those differ while the supply is
 * down — which is exactly the fault this topic should make visible.
 */
static bool engaged[ARRAY_SIZE(valves)];

static int solenoid_init(void)
{
  for (size_t i = 0; i < ARRAY_SIZE(valves); i++) {
    if (!gpio_is_ready_dt(&valves[i])) {
      LOG_ERR("valve %u: %s not ready", (unsigned)i, valves[i].port->name);
      return -ENODEV;
    }

    /* Inactive at boot: a valve that latches on through a reset can drive an
     * actuator into whatever it is resting against.
     */
    int ret = gpio_pin_configure_dt(&valves[i], GPIO_OUTPUT_INACTIVE);

    if (ret != 0) {
      LOG_ERR("valve %u: configure failed (%d)", (unsigned)i, ret);
      return ret;
    }
  }

  return 0;
}

/* The topic is declared period_us=0, so the frame goes out on commit rather
 * than on a schedule. A valve only moves when a service call moves it, so
 * there is nothing for a periodic tick to observe between calls.
 */
static void publish_state(uint8_t inst)
{
  if (auto pub = fcan_gen::solenoid(inst).publish_state()) {
    pub->on = engaged[inst];
  }
}

static fcan_gen::svc_status set_valve(
  uint8_t inst, const set_req & req, set_resp & resp)
{
  int ret = gpio_pin_set_dt(&valves[inst], req.on ? 1 : 0);

  if (ret != 0) {
    LOG_ERR("valve %u: set %d failed (%d)", inst, (int)req.on, ret);
    resp.success = false;
    return FCAN_SVC_APP_ERROR;
  }

  engaged[inst] = req.on;
  resp.success = true;

  /* Report before replying. The reply says the request was accepted; the
   * topic says what the output is now driving, and a master that watches
   * only the topic should not have to wait for a poll to learn it.
   */
  publish_state(inst);

  LOG_INF("valve %u -> %s", inst, req.on ? "on" : "off");

  return FCAN_SVC_OK;
}

/* Handlers are registered per instance, so the bound index is the only thing
 * a handler captures and an out-of-range call never reaches one — the runtime
 * answers BAD_INDEX for a slot nobody claimed.
 *
 * Committing before the node reaches RUNNING is not a lost publish: the
 * runtime keeps the request and sends it on the first poll that is allowed
 * to transmit. So the master's first read is the real state of the valves,
 * not a default.
 */
static int solenoid_start(void)
{
  for (uint8_t i = 0; i < (uint8_t)ARRAY_SIZE(valves); i++) {
    fcan_gen::solenoid(i).on_set(
      [i](const set_req & req, set_resp & resp, fcan_gen::call_handle) noexcept {
        return set_valve(i, req, resp);
      });

    publish_state(i);
  }

  return 0;
}

FIBRIL_FCAN_FUNC_DEFINE(
  solenoid,
  .array = fcan_gen::solenoid::block_array_index,
  .count = (uint8_t)ARRAY_SIZE(valves),
  .init = solenoid_init,
  .start = solenoid_start);
