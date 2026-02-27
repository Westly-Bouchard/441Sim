//
// Created by west on 2/26/26.
//

#include "SimMotor.h"

#include <algorithm>

using namespace std;

SimMotor::SimMotor(const double kV, const double kT, const double r, const bool inverted) : kV(kV), kT(kT), r(r),
    setpoint(0), omega(0.0), inverted(inverted) {}

void SimMotor::setInput(const double pwm) {
    // setpoint = std::clamp(pwm, -255, 255);
    setpoint = pwm;
}


void SimMotor::setSpeed(const double speed) {

    omega = speed;
}

double SimMotor::getTorque() const {
    return setpoint;
    const double backEmf = omega / kV;

    const double appliedVoltage = (setpoint / 255.0) * vBus;

    const double statorCurrent = (appliedVoltage - backEmf) / r;

    double motorTorque = kT * statorCurrent;

    if (inverted) {
        motorTorque *= -1.0;
    }

    return motorTorque;
}
