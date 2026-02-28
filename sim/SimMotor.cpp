//
// Created by west on 2/26/26.
//

#include "../Config.h"
#include "SimMotor.h"

using namespace std;

SimMotor::SimMotor() : omega(0.0) {}

void SimMotor::setSpeed(const double speed) {
    omega = speed;
}

double SimMotor::getTorque() const {
    using namespace Config::Motor;

    const double backEmf = omega / K_V;

    const double appliedVoltage = getVoltage();

    const double statorCurrent = (appliedVoltage - backEmf) / R;

    const double motorTorque = K_T * statorCurrent;

    return motorTorque;
}
