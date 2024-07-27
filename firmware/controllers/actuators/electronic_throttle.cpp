/**
 * @file	electronic_throttle.cpp
 * @brief	Electronic Throttle driver
 *
 * @see test test_etb.cpp
 *
 *
 * Limited user documentation at https://github.com/rusefi/rusefi/wiki/HOWTO_electronic_throttle_body
 *
 *
 *  ETB is controlled according to pedal position input (pedal position sensor is a potentiometer)
 *    pedal 0% means pedal not pressed / idle
 *    pedal 100% means pedal all the way down
 *  (not TPS - not the one you can calibrate in TunerStudio)
 *
 *
 * See also pid.cpp
 *
 * Relevant console commands:
 *
 * ETB_BENCH_ENGINE
 * set engine_type 58
 *
 * enable verbose_etb
 * disable verbose_etb
 * etbinfo
 *
 * http://rusefi.com/forum/viewtopic.php?f=5&t=592
 *
 * @date Dec 7, 2013
 * @author Andrey Belomutskiy, (c) 2012-2020
 *
 * This file is part of rusEfi - see http://rusefi.com
 *
 * rusEfi is free software; you can redistribute it and/or modify it under the terms of
 * the GNU General Public License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * rusEfi is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "pch.h"

#include "electronic_throttle_impl.h"

#if EFI_ELECTRONIC_THROTTLE_BODY

#include "dc_motor.h"
#include "dc_motors.h"
#include "defaults.h"

#if defined(HAS_OS_ACCESS)
#error "Unexpected OS ACCESS HERE"
#endif

#if HW_PROTEUS
#include "proteus_meta.h"
#endif // HW_PROTEUS

#ifndef ETB_MAX_COUNT
#define ETB_MAX_COUNT 2
#endif /* ETB_MAX_COUNT */

static pedal2tps_t pedal2tpsMap{"p2t"};
static Map3D<ETB2_TRIM_SIZE, ETB2_TRIM_SIZE, int8_t, uint8_t, uint8_t> throttle2TrimTable{"t2t"};
static Map3D<TRACTION_CONTROL_ETB_DROP_SIZE, TRACTION_CONTROL_ETB_DROP_SIZE, int8_t, uint16_t, uint8_t> tcEtbDropTable{"tce"};

constexpr float etbPeriodSeconds = 1.0f / ETB_LOOP_FREQUENCY;

//static bool startupPositionError = false;

//#define STARTUP_NEUTRAL_POSITION_ERROR_THRESHOLD 5

static const float hardCodedetbHitachiBiasBins[8] = {0.0, 19.0, 21.0, 22.0, 23.0, 25.0, 30.0, 100.0};

static const float hardCodedetbHitachiBiasValues[8] = {-18.0, -17.0, -15.0, 0.0, 16.0, 20.0, 20.0, 20.0};

/* Generated by TS2C on Thu Aug 20 21:10:02 EDT 2020*/
void setHitachiEtbBiasBins() {
	copyArray(config->etbBiasBins, hardCodedetbHitachiBiasBins);
	copyArray(config->etbBiasValues, hardCodedetbHitachiBiasValues);
}

static SensorType functionToPositionSensor(dc_function_e func) {
	switch(func) {
		case DC_Throttle1: return SensorType::Tps1;
		case DC_Throttle2: return SensorType::Tps2;
		case DC_IdleValve: return SensorType::IdlePosition;
		case DC_Wastegate: return SensorType::WastegatePosition;
		default: return SensorType::Invalid;
	}
}

static SensorType functionToTpsSensor(dc_function_e func) {
	switch(func) {
		case DC_Throttle1:  return SensorType::Tps1;
		default: return SensorType::Tps2;
	}
}

static SensorType functionToTpsSensorPrimary(dc_function_e func) {
	switch(func) {
		case DC_Throttle1:  return SensorType::Tps1Primary;
		default: return SensorType::Tps2Primary;
	}
}

static SensorType functionToTpsSensorSecondary(dc_function_e func) {
	switch(func) {
		case DC_Throttle1:  return SensorType::Tps1Secondary;
		default: return SensorType::Tps2Secondary;
	}
}

#if EFI_TUNER_STUDIO
static TsCalMode functionToCalModePriMin(dc_function_e func) {
	switch (func) {
		case DC_Throttle1: return TsCalMode::Tps1Min;
		default: return TsCalMode::Tps2Min;
	}
}

static TsCalMode functionToCalModePriMax(dc_function_e func) {
	switch (func) {
		case DC_Throttle1: return TsCalMode::Tps1Max;
		default: return TsCalMode::Tps2Max;
	}
}

static TsCalMode functionToCalModeSecMin(dc_function_e func) {
	switch (func) {
		case DC_Throttle1: return TsCalMode::Tps1SecondaryMin;
		default: return TsCalMode::Tps2SecondaryMin;
	}
}

static TsCalMode functionToCalModeSecMax(dc_function_e func) {
	switch (func) {
		case DC_Throttle1: return TsCalMode::Tps1SecondaryMax;
		default: return TsCalMode::Tps2SecondaryMax;
	}
}
#endif // EFI_TUNER_STUDIO

static percent_t directPwmValue = NAN;

#define ETB_DUTY_LIMIT 0.9
// this macro clamps both positive and negative percentages from about -100% to 100%
#define ETB_PERCENT_TO_DUTY(x) (clampF(-ETB_DUTY_LIMIT, 0.01f * (x), ETB_DUTY_LIMIT))

bool EtbController::init(dc_function_e function, DcMotor *motor, pid_s *pidParameters, const ValueProvider3D* pedalProvider, bool hasPedal) {
	if (function == DC_None) {
		// if not configured, don't init.
		etbErrorCode = (int8_t)TpsState::None;
		return false;
	}

	m_function = function;
	m_positionSensor = functionToPositionSensor(function);

	// If we are a throttle, require redundant TPS sensor
	if (isEtbMode()) {
		// We don't need to init throttles, so nothing to do here.
		if (!hasPedal) {
			etbErrorCode = (int8_t)TpsState::None;
			return false;
		}

		// If no sensor is configured for this throttle, skip initialization.
		if (!Sensor::hasSensor(functionToTpsSensor(function))) {
			etbErrorCode = (int8_t)TpsState::TpsError;
			return false;
		}

		if (!Sensor::isRedundant(m_positionSensor)) {
			firmwareError(
				ObdCode::OBD_TPS_Configuration,
				"Use of electronic throttle requires %s to be redundant.",
				Sensor::getSensorName(m_positionSensor)
			);

			etbErrorCode = (int8_t)TpsState::Redundancy;
			return false;
		}

		if (!Sensor::isRedundant(SensorType::AcceleratorPedal)) {
			firmwareError(
				ObdCode::OBD_TPS_Configuration,
				"Use of electronic throttle requires accelerator pedal to be redundant."
			);
			etbErrorCode = (int8_t)TpsState::Redundancy;
			return false;
		}
	}

	m_motor = motor;
	m_pid.initPidClass(pidParameters);
	m_pedalProvider = pedalProvider;

	// Ignore 3% position error before complaining
	m_errorAccumulator.init(3.0f, etbPeriodSeconds);

	reset();

	return true;
}

void EtbController::reset() {
	m_shouldResetPid = true;
	etbDutyRateOfChange = etbDutyAverage = 0;
	m_dutyRocAverage.reset();
	m_dutyAverage.reset();
	etbTpsErrorCounter = 0;
	etbPpsErrorCounter = 0;
}

void EtbController::onConfigurationChange(pid_s* previousConfiguration) {
	if (m_motor && !m_pid.isSame(previousConfiguration)) {
		m_shouldResetPid = true;
	}
	m_dutyRocAverage.init(engineConfiguration->etbRocExpAverageLength);
	m_dutyAverage.init(engineConfiguration->etbExpAverageLength);
	doInitElectronicThrottle();
}

void EtbController::showStatus() {
	m_pid.showPidStatus("ETB");
}

expected<percent_t> EtbController::observePlant() {
	return Sensor::get(m_positionSensor);
}

void EtbController::setIdlePosition(percent_t pos) {
	m_idlePosition = pos;
}

void EtbController::setWastegatePosition(percent_t pos) {
	m_wastegatePosition = pos;
}

expected<percent_t> EtbController::getSetpoint() {
	switch (m_function) {
		case DC_Throttle1:
		case DC_Throttle2:
			return getSetpointEtb();
		case DC_IdleValve:
			return getSetpointIdleValve();
		case DC_Wastegate:
			return getSetpointWastegate();
		default:
			return unexpected;
	}
}

expected<percent_t> EtbController::getSetpointIdleValve() const {
	// VW ETB idle mode uses an ETB only for idle (a mini-ETB sets the lower stop, and a normal cable
	// can pull the throttle up off the stop.), so we directly control the throttle with the idle position.
#if EFI_TUNER_STUDIO && (EFI_PROD_CODE || EFI_SIMULATOR)
	engine->outputChannels.etbTarget = m_idlePosition;
#endif // EFI_TUNER_STUDIO
	return clampPercentValue(m_idlePosition);
}

expected<percent_t> EtbController::getSetpointWastegate() const {
	return clampPercentValue(m_wastegatePosition);
}

float getSanitizedPedal() {
	auto pedalPosition = Sensor::get(SensorType::AcceleratorPedal);
	// If the pedal has failed, just use 0 position.
	// This is safer than disabling throttle control - we can at least push the throttle closed
	// and let the engine idle.
	return clampPercentValue(pedalPosition.value_or(0));
}

PUBLIC_API_WEAK float boardAdjustEtbTarget(float currentEtbTarget) {
  return currentEtbTarget;
}

expected<percent_t> EtbController::getSetpointEtb() {
	// Autotune runs with 50% target position
	if (m_isAutotune) {
		return 50.0f;
	}

//	// A few extra preconditions if throttle control is invalid
//	if (startupPositionError) {
//		return unexpected;
//	}

	// If the pedal map hasn't been set, we can't provide a setpoint.
	if (!m_pedalProvider) {
		return unexpected;
	}

  float sanitizedPedal = getSanitizedPedal();

	float rpm = Sensor::getOrZero(SensorType::Rpm);
	etbCurrentTarget = m_pedalProvider->getValue(rpm, sanitizedPedal);

	percent_t etbIdlePosition = clampPercentValue(m_idlePosition);
	percent_t etbIdleAddition = PERCENT_DIV * engineConfiguration->etbIdleThrottleRange * etbIdlePosition;

	// Interpolate so that the idle adder just "compresses" the throttle's range upward.
	// [0, 100] -> [idle, 100]
	// 0% target from table -> idle position as target
	// 100% target from table -> 100% target position
	targetWithIdlePosition = interpolateClamped(0, etbIdleAddition, 100, 100, etbCurrentTarget);

	percent_t targetPosition = boardAdjustEtbTarget(targetWithIdlePosition + getLuaAdjustment());

#if EFI_ANTILAG_SYSTEM
	if (engine->antilagController.isAntilagCondition) {
		targetPosition += engineConfiguration->ALSEtbAdd;
	}
#endif /* EFI_ANTILAG_SYSTEM */

  float vehicleSpeed = Sensor::getOrZero(SensorType::VehicleSpeed);
  float wheelSlip = Sensor::getOrZero(SensorType::WheelSlipRatio);
  tcEtbDrop = tcEtbDropTable.getValue(wheelSlip, vehicleSpeed);

	// Apply any adjustment that this throttle alone needs
	// Clamped to +-10 to prevent anything too wild
	trim = clampF(-10, getThrottleTrim(rpm, targetPosition), 10);
	targetPosition += trim + tcEtbDrop;

	// Clamp before rev limiter to avoid ineffective rev limit due to crazy out of range position target
	targetPosition = clampPercentValue(targetPosition);

	// Lastly, apply ETB rev limiter
	auto etbRpmLimit = engineConfiguration->etbRevLimitStart;
	if (etbRpmLimit != 0) {
		auto fullyLimitedRpm = etbRpmLimit + engineConfiguration->etbRevLimitRange;

		float targetPositionBefore = targetPosition;
		// Linearly taper throttle to closed from the limit across the range
		targetPosition = interpolateClamped(etbRpmLimit, targetPosition, fullyLimitedRpm, 0, rpm);

		// rev limit active if the position was changed by rev limiter
		etbRevLimitActive = absF(targetPosition - targetPositionBefore) > 0.1f;
	}

	float minPosition = engineConfiguration->etbMinimumPosition;

	// Keep the throttle just barely off the lower stop, and less than the user-configured maximum
	float maxPosition = engineConfiguration->etbMaximumPosition;
	// Don't allow max position over 100
	maxPosition = minF(maxPosition, 100);

	targetPosition = clampF(minPosition, targetPosition, maxPosition);
	etbCurrentAdjustedTarget = targetPosition;

#if EFI_TUNER_STUDIO
	if (m_function == DC_Throttle1) {
		engine->outputChannels.etbTarget = targetPosition;
	}
#endif // EFI_TUNER_STUDIO

	return targetPosition;
}

void EtbController::setLuaAdjustment(float adjustment) {
	luaAdjustment = adjustment;
	m_luaAdjustmentTimer.reset();
}

/**
 * positive adjustment opens TPS, negative closes TPS
 */
float EtbController::getLuaAdjustment() const {
	// If the lua position hasn't been set in 0.2 second, don't adjust!
	// This avoids a stuck throttle due to hung/rogue/etc Lua script
	if (m_luaAdjustmentTimer.getElapsedSeconds() > 0.2f) {
		return 0;
	} else {
		return luaAdjustment;
	}
}

percent_t EtbController2::getThrottleTrim(float rpm, percent_t targetPosition) const {
	return m_throttle2Trim.getValue(rpm, targetPosition);
}

expected<percent_t> EtbController::getOpenLoop(percent_t target) {
	// Don't apply open loop for wastegate/idle valve, only real ETB
	if (m_function != DC_Wastegate
		&& m_function != DC_IdleValve) {
		etbFeedForward = interpolate2d(target, config->etbBiasBins, config->etbBiasValues);
	} else {
		etbFeedForward = 0;
	}

	return etbFeedForward;
}

expected<percent_t> EtbController::getClosedLoopAutotune(percent_t target, percent_t actualThrottlePosition) {
	// Estimate gain at current position - this should be well away from the spring and in the linear region
	// GetSetpoint sets this to 50%
	bool isPositive = actualThrottlePosition > target;

	float autotuneAmplitude = 20;

	// End of cycle - record & reset
	if (!isPositive && m_lastIsPositive) {
		efitick_t now = getTimeNowNt();

		// Determine period
		float tu = m_autotuneCycleStart.getElapsedSecondsAndReset(now);

		// Determine amplitude
		float a = m_maxCycleTps - m_minCycleTps;

		// Filter - it's pretty noisy since the ultimate period is not very many loop periods
		constexpr float alpha = 0.05;
		m_a  = alpha * a  + (1 - alpha) * m_a;
		m_tu = alpha * tu + (1 - alpha) * m_tu;

		// Reset bounds
		m_minCycleTps = 100;
		m_maxCycleTps = 0;

		// Math is for Åström–Hägglund (relay) auto tuning
		// https://warwick.ac.uk/fac/cross_fac/iatl/reinvention/archive/volume5issue2/hornsey

		// Publish to TS state
#if EFI_TUNER_STUDIO
		// Amplitude of input (duty cycle %)
		float b = 2 * autotuneAmplitude;

		// Ultimate gain per A-H relay tuning rule
		float ku = 4 * b / (CONST_PI * m_a);

		// The multipliers below are somewhere near the "no overshoot"
		// and "some overshoot" flavors of the Ziegler-Nichols method
		// Kp
		float kp = 0.35f * ku;
		float ki = 0.25f * ku / m_tu;
		float kd = 0.08f * ku * m_tu;

		// Every 5 cycles (of the throttle), cycle to the next value
		if (m_autotuneCounter >= 5) {
			m_autotuneCounter = 0;
			m_autotuneCurrentParam = (m_autotuneCurrentParam + 1) % 3; // three ETB calibs: P-I-D
		}

		m_autotuneCounter++;

		// Multiplex 3 signals on to the {mode, value} format
		engine->outputChannels.calibrationMode = (uint8_t)static_cast<TsCalMode>((uint8_t)TsCalMode::EtbKp + m_autotuneCurrentParam);

		switch (m_autotuneCurrentParam) {
		case 0:
			engine->outputChannels.calibrationValue = kp;
			break;
		case 1:
			engine->outputChannels.calibrationValue = ki;
			break;
		case 2:
			engine->outputChannels.calibrationValue = kd;
			break;
		}

		// Also output to debug channels if configured
		if (engineConfiguration->debugMode == DBG_ETB_AUTOTUNE) {
			// a - amplitude of output (TPS %)
			engine->outputChannels.debugFloatField1 = m_a;
			// b - amplitude of input (Duty cycle %)
			engine->outputChannels.debugFloatField2 = b;
			// Tu - oscillation period (seconds)
			engine->outputChannels.debugFloatField3 = m_tu;

			engine->outputChannels.debugFloatField4 = ku;
			engine->outputChannels.debugFloatField5 = kp;
			engine->outputChannels.debugFloatField6 = ki;
			engine->outputChannels.debugFloatField7 = kd;
		}
#endif
	}

	m_lastIsPositive = isPositive;

	// Find the min/max of each cycle
	if (actualThrottlePosition < m_minCycleTps) {
		m_minCycleTps = actualThrottlePosition;
	}

	if (actualThrottlePosition > m_maxCycleTps) {
		m_maxCycleTps = actualThrottlePosition;
	}

	// Bang-bang control the output to induce oscillation
	return autotuneAmplitude * (isPositive ? -1 : 1);
}

expected<percent_t> EtbController::getClosedLoop(percent_t target, percent_t observation) {
	if (m_shouldResetPid) {
		m_pid.reset();
		m_shouldResetPid = false;
	}

	if (m_isAutotune) {
		return getClosedLoopAutotune(target, observation);
	} else {
		// Check that we're not over the error limit
		etbIntegralError = m_errorAccumulator.accumulate(target - observation);

		// Allow up to 10 percent-seconds of error
		if (etbIntegralError > 10.0f) {
			// TODO: figure out how to handle uncalibrated ETB
			//getLimpManager()->reportEtbProblem();
		}

		// Normal case - use PID to compute closed loop part
		return m_pid.getOutput(target, observation, etbPeriodSeconds);
	}
}

void EtbController::setOutput(expected<percent_t> outputValue) {
#if EFI_TUNER_STUDIO
	// Only report first-throttle stats
	if (m_function == DC_Throttle1) {
		engine->outputChannels.etb1DutyCycle = outputValue.value_or(0);
	}
#endif

	if (!m_motor) {
		return;
	}

	// If not ETB, or ETB is allowed, output is valid, and we aren't paused, output to motor.
	if (!isEtbMode() ||
	   (getLimpManager()->allowElectronicThrottle()
		&& outputValue
		&& !engineConfiguration->pauseEtbControl)) {
		m_motor->enable();
		m_motor->set(ETB_PERCENT_TO_DUTY(outputValue.Value));
	} else {
		// Otherwise disable the motor.
		m_motor->disable("no-ETB");
	}
}

bool EtbController::checkStatus() {
#if EFI_TUNER_STUDIO
	// Only debug throttle #1
	if (m_function == DC_Throttle1) {
		m_pid.postState(engine->outputChannels.etbStatus);
	} else if (m_function == DC_Wastegate) {
		m_pid.postState(engine->outputChannels.wastegateDcStatus);
	}
#endif /* EFI_TUNER_STUDIO */

	if (!isEtbMode()) {
		// no validation for h-bridge or idle mode
		return true;
	}
	// ETB-specific code belo. The whole mix-up between DC and ETB is shameful :(

	m_pid.iTermMin = engineConfiguration->etb_iTermMin;
	m_pid.iTermMax = engineConfiguration->etb_iTermMax;

	// Only allow autotune with stopped engine, and on the first throttle
	// Update local state about autotune
	m_isAutotune = Sensor::getOrZero(SensorType::Rpm) == 0
		&& engine->etbAutoTune
		&& m_function == DC_Throttle1;

	bool shouldCheckSensorFunction = engine->module<SensorChecker>()->analogSensorsShouldWork();

	if (!m_isAutotune && shouldCheckSensorFunction) {
		bool isTpsError = !Sensor::get(m_positionSensor).Valid;

		// If we have an error that's new, increment the counter
		if (isTpsError && !hadTpsError) {
			etbTpsErrorCounter++;
		}

		hadTpsError = isTpsError;

		bool isPpsError = !Sensor::get(SensorType::AcceleratorPedal).Valid;

		// If we have an error that's new, increment the counter
		if (isPpsError && !hadPpsError) {
			etbPpsErrorCounter++;
		}

		hadPpsError = isPpsError;
	} else {
		// Either sensors are expected to not work, or autotune is running, so reset the error counter
		etbTpsErrorCounter = 0;
		etbPpsErrorCounter = 0;
	}

#ifndef ETB_INTERMITTENT_LIMIT
#define ETB_INTERMITTENT_LIMIT 50
#endif

	TpsState localReason = TpsState::None;
	if (etbTpsErrorCounter > ETB_INTERMITTENT_LIMIT) {
		localReason = TpsState::IntermittentTps;
#if EFI_SHAFT_POSITION_INPUT
	} else if (engineConfiguration->disableEtbWhenEngineStopped && !engine->triggerCentral.engineMovedRecently()) {
		localReason = TpsState::EngineStopped;
#endif // EFI_SHAFT_POSITION_INPUT
	} else if (etbPpsErrorCounter > ETB_INTERMITTENT_LIMIT) {
		localReason = TpsState::IntermittentPps;
	} else if (engine->engineState.lua.luaDisableEtb) {
		localReason = TpsState::Lua;
	}

	etbErrorCode = (int8_t)localReason;

	return localReason == TpsState::None;
}

void EtbController::update() {
#if !EFI_UNIT_TEST
	// If we didn't get initialized, fail fast
	if (!m_motor) {
		return;
	}
#endif // EFI_UNIT_TEST

	if (!std::isnan(directPwmValue)) {
		m_motor->set(directPwmValue);
		etbErrorCode = (int8_t)TpsState::Manual;
		return;
	}

	bool isOk = checkStatus();

	if (!isOk) {
		// If engine is stopped and so configured, skip the ETB update entirely
		// This is quieter and pulls less power than leaving it on all the time
		m_motor->disable("etb status");
		return;
	}

	auto output = ClosedLoopController::update();

	if (!output) {
		return;
	}

	checkOutput(output.Value);
}

void EtbController::checkOutput(percent_t output) {
	etbDutyAverage = m_dutyAverage.average(absF(output));

	etbDutyRateOfChange = m_dutyRocAverage.average(absF(output - prevOutput));
	prevOutput = output;

#if EFI_UNIT_TEST
	auto integratorLimit = engineConfiguration->etbJamIntegratorLimit;

	if (integratorLimit != 0) {
	  float integrator = absF(m_pid.getIntegration());
		auto nowNt = getTimeNowNt();

		if (integrator > integratorLimit) {
			if (m_jamDetectTimer.hasElapsedSec(engineConfiguration->etbJamTimeout)) {
				// ETB is jammed!
				jamDetected = true;

				// TODO: do something about it!
			}
		} else {
			m_jamDetectTimer.reset(getTimeNowNt());
			jamDetected = false;
		}

		jamTimer = m_jamDetectTimer.getElapsedSeconds(nowNt);
	}
#endif // EFI_UNIT_TEST

}

void EtbController::autoCalibrateTps() {
	// Only auto calibrate throttles
	if (m_function == DC_Throttle1 || m_function == DC_Throttle2) {
		m_isAutocal = true;
	}
}

#if !EFI_UNIT_TEST
/**
 * Things running on a timer (instead of a thread) don't participate it the RTOS's thread priority system,
 * and operate essentially "first come first serve", which risks starvation.
 * Since ETB is a safety critical device, we need the hard RTOS guarantee that it will be scheduled over other less important tasks.
 */
#include "periodic_thread_controller.h"
#else
#define chThdSleepMilliseconds(x) {}
#endif // EFI_UNIT_TEST

#include <utility>

template <typename TBase>
struct EtbImpl final : public TBase {
	template <typename... TArgs>
	EtbImpl(TArgs&&... args) : TBase(std::forward<TArgs>(args)...) { }

	void update() override {
#if EFI_TUNER_STUDIO
	if (TBase::m_isAutocal) {
		// Don't allow if engine is running!
		if (Sensor::getOrZero(SensorType::Rpm) > 0) {
			TBase::m_isAutocal = false;
			return;
		}

		auto motor = TBase::getMotor();
		if (!motor) {
			TBase::m_isAutocal = false;
			return;
		}

		auto myFunction = TBase::getFunction();

		// First grab open
		motor->set(0.5f);
		motor->enable();
		chThdSleepMilliseconds(1000);
		float primaryMax = Sensor::getRaw(functionToTpsSensorPrimary(myFunction));
		float secondaryMax = Sensor::getRaw(functionToTpsSensorSecondary(myFunction));

		// Let it return
		motor->set(0);
		chThdSleepMilliseconds(200);

		// Now grab closed
		motor->set(-0.5f);
		chThdSleepMilliseconds(1000);
		float primaryMin = Sensor::getRaw(functionToTpsSensorPrimary(myFunction));
		float secondaryMin = Sensor::getRaw(functionToTpsSensorSecondary(myFunction));

		// Finally disable and reset state
		motor->disable("autotune");

		// Check that the calibrate actually moved the throttle
		if (absF(primaryMax - primaryMin) < 0.5f) {
			firmwareError(ObdCode::OBD_TPS_Configuration, "Auto calibrate failed, check your wiring!\r\nClosed voltage: %.1fv Open voltage: %.1fv", primaryMin, primaryMax);
			TBase::m_isAutocal = false;
			return;
		}

		// Write out the learned values to TS, waiting briefly after setting each to let TS grab it
		engine->outputChannels.calibrationMode = (uint8_t)functionToCalModePriMax(myFunction);
		engine->outputChannels.calibrationValue = convertVoltageTo10bitADC(primaryMax);
		chThdSleepMilliseconds(500);
		engine->outputChannels.calibrationMode = (uint8_t)functionToCalModePriMin(myFunction);
		engine->outputChannels.calibrationValue = convertVoltageTo10bitADC(primaryMin);
		chThdSleepMilliseconds(500);

		engine->outputChannels.calibrationMode = (uint8_t)functionToCalModeSecMax(myFunction);
		engine->outputChannels.calibrationValue = convertVoltageTo10bitADC(secondaryMax);
		chThdSleepMilliseconds(500);
		engine->outputChannels.calibrationMode = (uint8_t)functionToCalModeSecMin(myFunction);
		engine->outputChannels.calibrationValue = convertVoltageTo10bitADC(secondaryMin);
		chThdSleepMilliseconds(500);

		engine->outputChannels.calibrationMode = (uint8_t)TsCalMode::None;

		TBase::m_isAutocal = false;
		return;
	}
#endif /* EFI_TUNER_STUDIO */

		TBase::update();
	}
};

// real implementation (we mock for some unit tests)
static EtbImpl<EtbController1> etb1;
static EtbImpl<EtbController2> etb2(throttle2TrimTable);

static_assert(ETB_COUNT == 2);
static EtbController* etbControllers[] = { &etb1, &etb2 };

#if !EFI_UNIT_TEST

struct DcThread final : public PeriodicController<512> {
	DcThread() : PeriodicController("DC", PRIO_ETB, ETB_LOOP_FREQUENCY) {}

	void PeriodicTask(efitick_t) override {
		// Simply update all controllers
		for (int i = 0 ; i < ETB_COUNT; i++) {
			etbControllers[i]->update();
		}
	}
};

static DcThread dcThread CCM_OPTIONAL;

#endif // !EFI_UNIT_TEST

void etbPidReset() {
	for (int i = 0 ; i < ETB_COUNT; i++) {
		if (auto controller = engine->etbControllers[i]) {
			controller->reset();
		}
	}
}

#if !EFI_UNIT_TEST

/**
 * At the moment there are TWO ways to use this
 * set_etb_duty X
 * set etb X
 * manual duty cycle control without PID. Percent value from 0 to 100
 */
void setThrottleDutyCycle(percent_t level) {
	efiPrintf("setting ETB duty=%f%%", level);
	if (std::isnan(level)) {
		directPwmValue = NAN;
		return;
	}

	float dc = ETB_PERCENT_TO_DUTY(level);
	directPwmValue = dc;
	for (int i = 0 ; i < ETB_COUNT; i++) {
		setDcMotorDuty(i, dc);
	}
	efiPrintf("duty ETB duty=%f", dc);
}
#endif /* EFI_PROD_CODE */

void etbAutocal(size_t throttleIndex) {
	if (throttleIndex >= ETB_COUNT) {
		return;
	}

	if (auto etb = engine->etbControllers[throttleIndex]) {
		etb->autoCalibrateTps();
	}
}

/**
 * This specific throttle has default position of about 7% open
 */
static const float boschBiasBins[] = {
	0, 1, 5, 7, 14, 65, 66, 100
};
static const float boschBiasValues[] = {
	-15, -15, -10, 0, 19, 20, 26, 28
};

void setBoschVAGETB() {
	engineConfiguration->tpsMin = 890; // convert 12to10 bit (ADC/4)
	engineConfiguration->tpsMax = 70; // convert 12to10 bit (ADC/4)

	engineConfiguration->tps1SecondaryMin = 102;
	engineConfiguration->tps1SecondaryMax = 891;

	engineConfiguration->etb.pFactor = 5.12;
	engineConfiguration->etb.iFactor = 47;
	engineConfiguration->etb.dFactor = 0.088;
	engineConfiguration->etb.offset = 0;
}

void setBoschVNH2SP30Curve() {
	copyArray(config->etbBiasBins, boschBiasBins);
	copyArray(config->etbBiasValues, boschBiasValues);
}

void setDefaultEtbParameters() {
	engineConfiguration->etbIdleThrottleRange = 15;

	engineConfiguration->etbExpAverageLength = 50;
	engineConfiguration->etbRocExpAverageLength = 50;

	setLinearCurve(config->pedalToTpsPedalBins, /*from*/0, /*to*/100, 1);
	setRpmTableBin(config->pedalToTpsRpmBins);

	for (int pedalIndex = 0;pedalIndex<PEDAL_TO_TPS_SIZE;pedalIndex++) {
		for (int rpmIndex = 0;rpmIndex<PEDAL_TO_TPS_SIZE;rpmIndex++) {
			config->pedalToTpsTable[pedalIndex][rpmIndex] = config->pedalToTpsPedalBins[pedalIndex];
		}
	}

	// Default is to run each throttle off its respective hbridge
	engineConfiguration->etbFunctions[0] = DC_Throttle1;
	engineConfiguration->etbFunctions[1] = DC_Throttle2;

	engineConfiguration->etbFreq = DEFAULT_ETB_PWM_FREQUENCY;

	// voltage, not ADC like with TPS
	setPPSCalibration(0, 5, 5, 0);

	engineConfiguration->etb = {
		1,		// Kp
		10,		// Ki
		0.05,	// Kd
		0,		// offset
		0,		// Update rate, unused
		-100, 100 // min/max
	};

	engineConfiguration->etb_iTermMin = -30;
	engineConfiguration->etb_iTermMax = 30;
}

void onConfigurationChangeElectronicThrottleCallback(engine_configuration_s *previousConfiguration) {
	for (int i = 0; i < ETB_COUNT; i++) {
		etbControllers[i]->onConfigurationChange(&previousConfiguration->etb);
	}
}

static const float defaultBiasBins[] = {
	0, 1, 2, 4, 7, 98, 99, 100
};
static const float defaultBiasValues[] = {
	-20, -18, -17, 0, 20, 21, 22, 25
};

void setDefaultEtbBiasCurve() {
	copyArray(config->etbBiasBins, defaultBiasBins);
	copyArray(config->etbBiasValues, defaultBiasValues);
}

void unregisterEtbPins() {
	// todo: we probably need an implementation here?!
}

static pid_s* getPidForDcFunction(dc_function_e function) {
	switch (function) {
		case DC_Wastegate: return &engineConfiguration->etbWastegatePid;
		default: return &engineConfiguration->etb;
	}
}

PUBLIC_API_WEAK ValueProvider3D* pedal2TpsProvider() {
  return &pedal2tpsMap;
}

void doInitElectronicThrottle() {
	bool hasPedal = Sensor::hasSensor(SensorType::AcceleratorPedalPrimary);

#if EFI_UNIT_TEST
	printf("doInitElectronicThrottle %s\n", boolToString(hasPedal));
#endif // EFI_UNIT_TEST

	// these status flags are consumed by TS see tunerstudio.template.ini TODO should those be outputs/live data not configuration?!
	engineConfiguration->etb1configured = engineConfiguration->etb2configured = false;

	// todo: technical debt: we still have DC motor code initialization in ETB-specific file while DC motors are used not just as ETB
	// like DC motor wastegate code flow should probably NOT go through electronic_throttle.cpp right?
	// todo: rename etbFunctions to something-without-etb for same reason?
	for (int i = 0 ; i < ETB_COUNT; i++) {
		auto func = engineConfiguration->etbFunctions[i];
		if (func == DC_None) {
			// do not touch HW pins if function not selected, this way Lua can use DC motor hardware pins directly
			continue;
		}
		auto motor = initDcMotor("ETB disable",
				engineConfiguration->etbIo[i], i, engineConfiguration->etb_use_two_wires);

		auto controller = engine->etbControllers[i];
    criticalAssertVoid(controller != nullptr, "null ETB");

		auto pid = getPidForDcFunction(func);

		bool dcConfigured = controller->init(func, motor, pid, pedal2TpsProvider(), hasPedal);
		bool etbConfigured = dcConfigured && controller->isEtbMode();
		if (i == 0) {
		    engineConfiguration->etb1configured = etbConfigured;
		} else if (i == 1) {
		    engineConfiguration->etb2configured = etbConfigured;
		}
	}

	if (!engineConfiguration->etb1configured && !engineConfiguration->etb2configured) {
		// It's not valid to have a PPS without any ETBs - check that at least one ETB was enabled along with the pedal
		if (hasPedal) {
			criticalError("A pedal position sensor was configured, but no electronic throttles are configured.");
		}
	}

#if 0 && ! EFI_UNIT_TEST
	percent_t startupThrottlePosition = getTPS();
	if (absF(startupThrottlePosition - engineConfiguration->etbNeutralPosition) > STARTUP_NEUTRAL_POSITION_ERROR_THRESHOLD) {
		/**
		 * Unexpected electronic throttle start-up position is worth a critical error
		 */
		firmwareError(ObdCode::OBD_Throttle_Actuator_Control_Range_Performance_Bank_1, "startup ETB position %.2f not %d",
				startupThrottlePosition,
				engineConfiguration->etbNeutralPosition);
		startupPositionError = true;
	}
#endif /* EFI_UNIT_TEST */

#if !EFI_UNIT_TEST
	static bool started = false;
	if (started == false) {
		dcThread.start();
		started = true;
	}
#endif
}

void initElectronicThrottle() {
	if (hasFirmwareError()) {
		return;
	}

	for (int i = 0; i < ETB_COUNT; i++) {
		engine->etbControllers[i] = etbControllers[i];
	}

#if EFI_PROD_CODE
	addConsoleAction("etbinfo", [](){
	  efiPrintf("etbAutoTune=%d", engine->etbAutoTune);
	  efiPrintf("TPS=%.2f", Sensor::getOrZero(SensorType::Tps1));

	  efiPrintf("ETB1 duty=%.2f",
			(float)engine->outputChannels.etb1DutyCycle);

	  efiPrintf("ETB freq=%d",
			engineConfiguration->etbFreq);

	  for (int i = 0; i < ETB_COUNT; i++) {
		  efiPrintf("ETB%d", i);
		  efiPrintf(" dir1=%s", hwPortname(engineConfiguration->etbIo[i].directionPin1));
		  efiPrintf(" dir2=%s", hwPortname(engineConfiguration->etbIo[i].directionPin2));
		  efiPrintf(" control=%s", hwPortname(engineConfiguration->etbIo[i].controlPin));
		  efiPrintf(" disable=%s", hwPortname(engineConfiguration->etbIo[i].disablePin));
		  showDcMotorInfo(i);
	  }
	});

#endif /* EFI_PROD_CODE */

	pedal2tpsMap.initTable(config->pedalToTpsTable, config->pedalToTpsRpmBins, config->pedalToTpsPedalBins);
	throttle2TrimTable.initTable(config->throttle2TrimTable, config->throttle2TrimRpmBins, config->throttle2TrimTpsBins);
	tcEtbDropTable.initTable(engineConfiguration->tractionControlEtbDrop, engineConfiguration->tractionControlSlipBins, engineConfiguration->tractionControlSpeedBins);

	doInitElectronicThrottle();
}

void setEtbIdlePosition(percent_t pos) {
	for (int i = 0; i < ETB_COUNT; i++) {
		if (auto etb = engine->etbControllers[i]) {
			etb->setIdlePosition(pos);
		}
	}
}

void setEtbWastegatePosition(percent_t pos) {
	for (int i = 0; i < ETB_COUNT; i++) {
		if (auto etb = engine->etbControllers[i]) {
			etb->setWastegatePosition(pos);
		}
	}
}

void setEtbLuaAdjustment(percent_t pos) {
	for (int i = 0; i < ETB_COUNT; i++) {
		if (auto etb = engine->etbControllers[i]) {
			etb->setLuaAdjustment(pos);
		}
	}
}

void setToyota89281_33010_pedal_position_sensor() {
	setPPSCalibration(0, 4.1, 0.73, 4.9);
}

void setHitachiEtbCalibration() {
	setToyota89281_33010_pedal_position_sensor();

	setHitachiEtbBiasBins();

	engineConfiguration->etb.pFactor = 2.7999;
	engineConfiguration->etb.iFactor = 25.5;
	engineConfiguration->etb.dFactor = 0.053;
	engineConfiguration->etb.offset = 0.0;
	engineConfiguration->etb.periodMs = 5.0;
	engineConfiguration->etb.minValue = -100.0;
	engineConfiguration->etb.maxValue = 100.0;

	// Nissan 60mm throttle
	engineConfiguration->tpsMin = engineConfiguration->tps2Min = 113;
	engineConfiguration->tpsMax = engineConfiguration->tps2Max = 846;
	engineConfiguration->tps1SecondaryMin = engineConfiguration->tps2SecondaryMin = 897;
	engineConfiguration->tps1SecondaryMax = engineConfiguration->tps2SecondaryMax = 161;
}

void setProteusHitachiEtbDefaults() {
#if HW_PROTEUS
	setHitachiEtbCalibration();

	// EFI_ADC_12: "Analog Volt 3"
	engineConfiguration->tps1_2AdcChannel = PROTEUS_IN_TPS1_2;
	// EFI_ADC_13: "Analog Volt 4"
	engineConfiguration->tps2_1AdcChannel = PROTEUS_IN_TPS2_1;
	// EFI_ADC_0: "Analog Volt 5"
	engineConfiguration->tps2_2AdcChannel = PROTEUS_IN_ANALOG_VOLT_5;
	setPPSInputs(PROTEUS_IN_ANALOG_VOLT_6, PROTEUS_IN_PPS2);
#endif // HW_PROTEUS
}

#endif /* EFI_ELECTRONIC_THROTTLE_BODY */

template<>
const electronic_throttle_s* getLiveData(size_t idx) {
#if EFI_ELECTRONIC_THROTTLE_BODY
	if (idx >= efi::size(etbControllers)) {
		return nullptr;
	}

	return etbControllers[idx];
#else
	return nullptr;
#endif
}
