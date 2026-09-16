/*
 * Copyright (c) 2026 Forte Fibre
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FIBRIL_ZEPHYR_INCLUDE_DRIVERS_SENSOR_AS5047P_H_
#define FIBRIL_ZEPHYR_INCLUDE_DRIVERS_SENSOR_AS5047P_H_

/**
 * @file
 * @brief Header file for extended sensor API of AS5047P sensor
 */

#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Additional channels supported by the AS5047P.
 */
enum as5047p_sensor_channel {
	/** Raw 14-bit angular position count. */
	AS5047P_CHAN_RAW_ANGLE = SENSOR_CHAN_PRIV_START,
};

/**
 * @brief Additional attributes supported by the AS5047P.
 */
enum as5047p_sensor_attribute {
	/** Raw 14-bit zero position split across ZPOSM/ZPOSL. */
	AS5047P_ATTR_ZERO_POSITION = SENSOR_ATTR_PRIV_START,
	/** Raw SETTINGS1 register value. */
	AS5047P_ATTR_SETTINGS1,
	/** Raw SETTINGS2 register value. */
	AS5047P_ATTR_SETTINGS2,
};

#ifdef __cplusplus
}
#endif

#endif /* FIBRIL_ZEPHYR_INCLUDE_DRIVERS_SENSOR_AS5047P_H_ */
