/**
 * @file etb_bank_balance.h
 *
 * Cylinder-bank throttle synchronization ("ETB balancing") for engines with
 * two independent electronic throttle bodies and two MAF sensors, one pair
 * per bank (e.g. BMW M70/M73 V12 with a MSS54-style DME-per-bank + EML setup).
 *
 * rusEFI has no notion of a "bank" for computation - MafAirmass::getMaf()
 * simply sums MAF1+MAF2 into one shared airmass/ti (see maf_airmass.cpp).
 * BMW's real EML doesn't compare ti either - it syncs on airflow directly
 * (see the Bosch EML III cylinder-bank-sync documentation referenced in
 * rusefi/rusefi issue #3680, "Throttle synchronization"). This module does
 * the same: it compares the two MAF channels directly, at idle, and nudges
 * the two ETB targets apart (one up, one down) until both banks draw the
 * same air.
 *
 * This is a PROTOTYPE:
 *  - Parameters are constexpr constants in the .cpp, not yet exposed as
 *    tunable engine_configuration_s fields / a TunerStudio page.
 *  - No persistence across power cycles (LongTermFuelTrim-style storage
 *    would be the natural next step, see long_term_fuel_trim.cpp).
 *  - Modeled as a bounded adaptation (like EtbImpl::doAutocal()'s ACPhase
 *    state machine in electronic_throttle_impl.h), not a free-running PID,
 *    to match how the Bosch original behaves (a limited "Grundadaption"
 *    window, not continuous closed-loop control of a mechanical actuator).
 *
 * @date Sep 2026
 */
#pragma once

#include "engine_module.h"
#include "rusefi_types.h"
#include "rusefi_enums.h"

enum class EtbBankBalanceState : uint8_t {
	Disabled,  // preconditions not met (not idling, not calibrated, sensors invalid, ...) - trim held
	Adapting,  // preconditions met, |delta| outside the deadband - actively nudging the trim
	Converged, // preconditions met, |delta| inside the deadband - trim held
	Fault,     // |delta| too large to plausibly be a throttle mismatch - not trimmed, flagged instead
};

class EtbBankBalance : public EngineModule {
public:
	// EngineModule
	void onSlowCallback() override;
	void onEngineStop() override;

	// Signed trim in percent to be added to a throttle's target position.
	// Positive for DC_Throttle1, mirrored (negative) for DC_Throttle2, 0 otherwise.
	percent_t getTrim(dc_function_e function) const;

	EtbBankBalanceState getState() const { return m_state; }
	// Filtered (maf1 - maf2) / (maf1 + maf2) * 100, for logging/console/TS
	float getDeltaPercent() const { return m_deltaPercent; }

private:
	bool checkPreconditions(float rpm) const;
	void resetFilters();

	EtbBankBalanceState m_state = EtbBankBalanceState::Disabled;

	float m_maf1Filtered = 0;
	float m_maf2Filtered = 0;
	bool m_filtersInitialized = false;

	float m_deltaPercent = 0;  // for diagnostics
	float m_trim = 0;          // current symmetric trim, bank1 sign, percent
};

// Global instance, wired into EtbController::getThrottleTrim() (see electronic_throttle_impl.h)
float getEtbBankBalanceTrim(dc_function_e function);

// For diagnostics/console/future TS live-data wiring (state, delta%, ...).
EtbBankBalance& getEtbBankBalanceInstance();
