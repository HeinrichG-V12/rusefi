// this section was generated automatically by rusEFI tool config_definition_base-all.jar based on (unknown script) controllers/actuators/etb_bank_balance.txt
// by class com.rusefi.output.CHeaderConsumer
// begin
#pragma once
#include "rusefi_types.h"
// start of etb_bank_balance_s
struct etb_bank_balance_s {
	/**
	 * "ETB balance: state"
	 * offset 0
	 */
	uint8_t etbBalanceState = (uint8_t)0;
	/**
	 * need 4 byte alignment
	 * units: units
	 * offset 1
	 */
	uint8_t alignmentFill_at_1[3] = {};
	/**
	 * "ETB balance: MAF delta"
	 * units: %
	 * offset 4
	 */
	float etbBalanceDeltaPercent = (float)0;
	/**
	 * "ETB balance: trim"
	 * units: %
	 * offset 8
	 */
	float etbBalanceTrim = (float)0;
};
static_assert(sizeof(etb_bank_balance_s) == 12);

// end
// this section was generated automatically by rusEFI tool config_definition_base-all.jar based on (unknown script) controllers/actuators/etb_bank_balance.txt
