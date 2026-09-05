// file can_dash_bmw_e46_m3.cpp
//
// Emulates the MSS54 DME CAN broadcast of a BMW E46 M3, as its own dashboard
// profile (CAN_BUS_BMW_E46_M3 / TunerStudio "BMW E46 M3"), independent of the
// existing generic canDashboardBmwE46() / can_bmw.h.

#include "pch.h"

#if EFI_CAN_SUPPORT || EFI_UNIT_TEST
#include "can_dash_bmw_e46_m3.h"

//todo: we use a 10ms fixed cycle, trace against a real MSS54 needed to confirm exact period per message
void canDashboardBmwE46M3(CanCycle cycle) {
	if (cycle.isInterval(CI::_10ms)) {
		{ // DME1 0x316
			CanTxMessage msg(CanCategory::NBC, CAN_BMW_E46_M3_DME1, 8, DEFAULT_BUS_INDEX);
			msg[0] = 0x05; // ASC message
			msg[1] = 0x0C; // Indexed Engine Torque in % of C_TQ_STND TBD
			msg.setShortValue((int) (Sensor::getOrZero(SensorType::Rpm) * 6.4), 2);
			msg[4] = 0x0C;
			msg[5] = 0x15;
			msg[6] = 0x00;
			msg[7] = 0x35;
		}

		{ // DME2 0x329
			CanTxMessage msg(CanCategory::NBC, CAN_BMW_E46_M3_DME2, 8, DEFAULT_BUS_INDEX);
			msg[0] = 0x11;
			msg[1] = (Sensor::getOrZero(SensorType::Clt) + 48.373) / 0.75;
			msg[2] = 0x00; // baro sensor
			msg[3] = 0x08;
			msg[4] = 0x00; // TPS_VIRT_CRU_CAN, not used.
			msg[5] = 0x00; // TPS out, but we set to 0 just in case.
			msg[6] = 0x00; // brake system status Ok.
			msg[7] = 0x00; // not used
		}

		// DME3 0x338 (Alpina roadster) intentionally not sent - out of scope for M3 emulation.

		{ // DME4 0x545 - skeleton only, byte layout TBD
			// TODO: reverse-engineer payload from bus logs before relying on this frame.
			// Placeholder is transmitted so the frame exists on the bus / cycle-time and
			// DLC can be validated against a real MSS54 trace.
			CanTxMessage msg(CanCategory::NBC, CAN_BMW_E46_M3_DME4, 8, DEFAULT_BUS_INDEX);
			msg[0] = 0x00; // TODO
			msg[1] = 0x00; // TODO
			msg[2] = 0x00; // TODO
			msg[3] = 0x00; // TODO
			msg[4] = 0x00; // TODO
			msg[5] = 0x00; // TODO
			msg[6] = 0x00; // TODO
			msg[7] = 0x00; // TODO
		}
	}
}

#endif // EFI_CAN_SUPPORT || EFI_UNIT_TEST
