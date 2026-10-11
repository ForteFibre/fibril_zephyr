/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Bus rates shared by every board's snippets, so that a bus changes its rate
 * in one place.
 */

#ifndef FIBRIL_DTS_CAN_BITRATE_H
#define FIBRIL_DTS_CAN_BITRATE_H

/* The fibril_can bus: 1 Mbps arbitration and 5 Mbps data. Every board in this
 * repository clocks its FDCAN from an 80 MHz PLL_Q, which divides both
 * cleanly (80 and 16 tq/bit). A transport snippet gives these to the
 * controller that reaches the bus. */
#define FIBRIL_FCAN_BITRATE      1000000
#define FIBRIL_FCAN_BITRATE_DATA 5000000

/* A classic CAN actuator bus (RoboMaster, RobStride). No data phase, so an FD
 * frame cannot reach a bus that cannot carry one. */
#define FIBRIL_ACTUATOR_CAN_BITRATE 1000000

#endif /* FIBRIL_DTS_CAN_BITRATE_H */
