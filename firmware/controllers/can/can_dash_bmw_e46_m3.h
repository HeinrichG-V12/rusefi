// file can_dash_bmw_e46_m3.h
//
// MSS54 DME emulation for the BMW E46 M3.
// Kept separate from can_bmw.h / can_dash.cpp's generic "BMW E46" profile
// (canDashboardBmwE46()) on purpose - own CAN IDs, own dashboard function,
// same pattern as can_dash_honda.* / can_dash_haltech.*

#pragma once

#include "can.h"
#include "can_msg_tx.h"

/**
 * MSS54 (E46 M3) DME broadcast set:
 *  DME1 0x316 - RPM / ASC message
 *  DME2 0x329 - coolant temp / baro / TPS / brake status
 *  DME3 0x338 - Alpina roadster specific, NOT part of M3 emulation, intentionally not sent
 *  DME4 0x545 - TODO: payload not yet reverse engineered
 */
#define CAN_BMW_E46_M3_DME1 0x316
#define CAN_BMW_E46_M3_DME2 0x329
#define CAN_BMW_E46_M3_DME4 0x545

void canDashboardBmwE46M3(CanCycle cycle);
