/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FIBRIL_ZEPHYR_INCLUDE_NODE_ID_NODE_ID_H_
#define FIBRIL_ZEPHYR_INCLUDE_NODE_ID_NODE_ID_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file
 * @brief The node id a board's switch is set to.
 *
 * @defgroup node_id Node id switch
 * @ingroup lib
 * @{
 */

/**
 * @brief Read the switch the `fibril,node-id` chosen node points at.
 *
 * Configures the switch's GPIOs as inputs and reads them once. The value is
 * what the switch is set to, with the first GPIO of the binding as the least
 * significant bit.
 *
 * @param id Destination for the node id.
 * @retval 0 Success.
 * @retval -ERANGE The switch reads as 0x7F, the broadcast address.
 * @retval -ENODEV A GPIO controller of the switch is not ready.
 * @retval negative_errno Failed to configure or read a GPIO.
 */
int node_id_read(uint8_t * id);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
