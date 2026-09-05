/**
 * @file etb_bank_balance.cpp
 *
 * See etb_bank_balance.h for the background. Short version: BMW/Bosch EML III
 * cylinder-bank sync, ported to rusEFI's single-process architecture -
 * compare MAF1 vs MAF2 directly at idle, trim the two ETBs apart (gegenläufig)
 * until both banks draw the same air.
 *
 * Bosch EML III conditions this on (M73 EML documentation):
 *  - Leerlauf aktiv (idle active)
 *  - RPM in a narrow band around idle target
 *  - DK-Anschläge erkannt (throttle end-stops learned, i.e. ETB autocal done)
 *  - DeltaTL between 0.4% and 10% (below: converged, nothing to do;
 *    above: not a throttle mismatch, don't trim - flag it instead)
 *  - both banks active, no fault
 *  - base adaptation running on both DMEs
 *
 * rusEFI mapping used here:
 *  - idle active            -> IdleController::isIdlingOrTaper()
 *  - rpm band                -> 0 < rpm < engineConfiguration->idlePidRpmUpperLimit
 *  - throttle stops learned  -> etbGetState() == EtbStatus::None on both banks
 *                                (excludes Uninitialized/AutoCalibrate/fault states)
 *  - DeltaTL 0.4%..10%       -> deadband + fault threshold below
 *  - both banks active       -> Sensor::hasSensor(Maf2) && second ETB configured
 *  - base adaptation active  -> not implemented in this prototype (would map to
 *                                "closed-loop idle already settled"); see header.
 *
 * No CAN/cross-ECU messaging is needed - MAF1/MAF2 and both ETB targets live
 * in the same process, so the whole "DME measures, sends to EML, EML adjusts"
 * round-trip collapses into one function call.
 */

#include "pch.h"

#include "etb_bank_balance.h"
#include "electronic_throttle.h"

#if EFI_ELECTRONIC_THROTTLE_BODY

namespace {

// --- Prototype tuning constants -------------------------------------------
// Bosch's numbers (0.4% / 10%) are for their "TL" load signal; ours is a
// simple relative MAF delta, so treat these as starting points, not gospel.

// Below this |delta|, banks are considered matched - stop nudging.
constexpr float kDeadbandPercent = 0.4f;

// Above this |delta|, this doesn't look like a throttle mismatch anymore
// (vacuum leak, dead MAF, stuck throttle, ...) - don't trim, flag instead.
constexpr float kFaultPercent = 10.0f;

// Symmetric trim clamp - matches the existing +-10% clamp already applied
// around getThrottleTrim() in EtbController::getSetpointEtb().
constexpr float kMaxTrim = 10.0f;

// Max trim change per onSlowCallback tick (~20Hz) -> about 0.4%/s.
// Deliberately slow: this drives a mechanical throttle, not a fast loop.
constexpr float kMaxTrimStepPerTick = 0.02f;

// Proportional gain from delta% to trim step - small on purpose, the slew
// rate limit above is what actually shapes the response.
constexpr float kGain = 0.01f;

// EMA filter coefficient for the raw MAF signals (~20Hz callback), roughly
// a 1s time constant - we want the same kind of "settled" signal BMW's
// filtered ti represents, not raw per-tick MAF noise.
constexpr float kMafFilterAlpha = 0.2f;

} // namespace

static EtbBankBalance instance;

bool EtbBankBalance::checkPreconditions(float rpm) const {
	if (rpm <= 0) {
		return false;
	}

#if EFI_IDLE_CONTROL
	if (!engine->module<IdleController>()->isIdlingOrTaper()) {
		return false;
	}
#endif // EFI_IDLE_CONTROL

	if (rpm >= engineConfiguration->idlePidRpmUpperLimit) {
		return false;
	}

	// Only makes sense with a real second bank: two throttles, two MAFs.
	if (!Sensor::hasSensor(SensorType::Maf2)) {
		return false;
	}

	if (etbGetState(0) != EtbStatus::None || etbGetState(1) != EtbStatus::None) {
		// covers: not configured, not calibrated (autocal not yet run),
		// TPS/PPS errors, jam detected, currently autotuning, etc.
		return false;
	}

	auto maf1 = Sensor::get(SensorType::Maf);
	auto maf2 = Sensor::get(SensorType::Maf2);
	if (!maf1.Valid || !maf2.Valid) {
		return false;
	}

	return true;
}

void EtbBankBalance::resetFilters() {
	m_filtersInitialized = false;
}

void EtbBankBalance::onEngineStop() {
	// Don't carry a stale filtered MAF delta into the next start; the trim
	// itself is intentionally left alone (it's a learned mechanical offset,
	// still valid next time we're at idle).
	resetFilters();
	m_state = EtbBankBalanceState::Disabled;
}

void EtbBankBalance::onSlowCallback() {
	float rpm = Sensor::getOrZero(SensorType::Rpm);

	if (!checkPreconditions(rpm)) {
		m_state = EtbBankBalanceState::Disabled;
		resetFilters();
		return;
	}

	float maf1 = Sensor::get(SensorType::Maf).value_or(0);
	float maf2 = Sensor::get(SensorType::Maf2).value_or(0);

	if (!m_filtersInitialized) {
		m_maf1Filtered = maf1;
		m_maf2Filtered = maf2;
		m_filtersInitialized = true;
	} else {
		m_maf1Filtered += (maf1 - m_maf1Filtered) * kMafFilterAlpha;
		m_maf2Filtered += (maf2 - m_maf2Filtered) * kMafFilterAlpha;
	}

	float sum = m_maf1Filtered + m_maf2Filtered;
	if (sum < 1.0f) {
		// Both MAFs read ~zero - avoid a division blowup, just wait.
		m_state = EtbBankBalanceState::Disabled;
		return;
	}

	// Positive delta = bank 1 flows more than bank 2.
	m_deltaPercent = 100.0f * (m_maf1Filtered - m_maf2Filtered) / sum;

	float absDelta = std::abs(m_deltaPercent);

	if (absDelta > kFaultPercent) {
		// Bosch doesn't trim through this - it's not plausibly a throttle
		// mismatch anymore. Freeze the trim and flag it; let something
		// upstream (OBD code / console warning) surface this to the user.
		if (m_state != EtbBankBalanceState::Fault) {
			efiPrintf("ETB bank balance: delta %.1f%% exceeds fault threshold (%.1f%%), disabling trim", m_deltaPercent, kFaultPercent);
		}
		m_state = EtbBankBalanceState::Fault;
		return;
	}

	if (absDelta < kDeadbandPercent) {
		m_state = EtbBankBalanceState::Converged;
		return;
	}

	m_state = EtbBankBalanceState::Adapting;

	float step = clampF(-kMaxTrimStepPerTick, m_deltaPercent * kGain, kMaxTrimStepPerTick);
	m_trim = clampF(-kMaxTrim, m_trim + step, kMaxTrim);
}

percent_t EtbBankBalance::getTrim(dc_function_e function) const {
	switch (function) {
		case DC_Throttle1: return m_trim;
		case DC_Throttle2: return -m_trim;
		default: return 0;
	}
}

float getEtbBankBalanceTrim(dc_function_e function) {
	return instance.getTrim(function);
}

EtbBankBalance& getEtbBankBalanceInstance() {
	return instance;
}

#else // EFI_ELECTRONIC_THROTTLE_BODY

float getEtbBankBalanceTrim(dc_function_e /*function*/) {
	return 0;
}

#endif // EFI_ELECTRONIC_THROTTLE_BODY
