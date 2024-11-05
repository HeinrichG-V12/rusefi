//
// Created by kifir on 11/4/24.
//

#include "pch.h"

#include "flat_shift_condition_test_base.h"

FlatShiftConditionTestBase::FlatShiftConditionTestBase(const int8_t torqueReductionIgnitionCut)
: m_torqueReductionIgnitionCut(torqueReductionIgnitionCut) {
}

void FlatShiftConditionTestBase::SetUp() {
    ShiftTorqueReductionTestBase::SetUp();

    setUpTestConfig(ShiftTorqueReductionTestConfig()
        .setTorqueReductionEnabled(true)
        .setTorqueReductionActivationMode(torqueReductionActivationMode_e::TORQUE_REDUCTION_BUTTON)
        .setTriggerPin(TEST_TORQUE_REDUCTION_BUTTON_PIN)
        .setLimitTorqueReductionTime(false)
        .setTorqueReductionIgnitionCut(m_torqueReductionIgnitionCut)
    );
}

void FlatShiftConditionTestBase::satisfyFlatShiftCondition() {
    setMockState(TEST_TORQUE_REDUCTION_BUTTON_PIN, true);
    updateApp(TEST_TORQUE_REDUCTION_ARMING_APP);

    periodicFastCallback();

    EXPECT_TRUE(engine->shiftTorqueReductionController.isFlatShiftConditionSatisfied);
}

void FlatShiftConditionTestBase::unsatisfyFlatShiftCondition() {
    setMockState(TEST_TORQUE_REDUCTION_BUTTON_PIN, false);

    periodicFastCallback();

    EXPECT_FALSE(engine->shiftTorqueReductionController.isFlatShiftConditionSatisfied);
}