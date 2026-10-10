/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief The calibration inputs AdcPort samples, for md_motor to watch.
 *
 * Call only from the function tick. The samples are refreshed there and are
 * not guarded against any other thread.
 */

#ifndef FIBRIL_CAN_NODE_ADC_PORT_HPP_
#define FIBRIL_CAN_NODE_ADC_PORT_HPP_

#include <cstddef>
#include <cstdint>

/**
 * @addtogroup fibril_can_node
 * @{
 */

namespace adc_port
{

/** Ticks between two samplings of every port: CanMotorMbed's 5 ms. */
constexpr uint32_t period_ticks = 5;

#if defined(CONFIG_FIBRIL_CAN_NODE_ADC_PORT)

/** Ports the devicetree lists. */
size_t count();

/** Advances each time every port has been sampled again. */
uint32_t generation();

/** The port's latest sample. False when the port does not exist or the
 * last sampling got nothing from it. */
bool read(uint8_t port, float & value);

/** The port's `threshold` parameter. */
float threshold(uint8_t port);

#else

/* No ports: calibration is refused, and nothing has to be compiled out at the
 * call sites. */
inline size_t count() { return 0; }
inline uint32_t generation() { return 0; }
inline bool read(uint8_t, float &) { return false; }
inline float threshold(uint8_t) { return 0.0F; }

#endif

}  // namespace adc_port

/** @} */

#endif /* FIBRIL_CAN_NODE_ADC_PORT_HPP_ */
