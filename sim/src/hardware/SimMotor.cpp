//
// Created by Westly Bouchard on 3/2/26.
//

#include "hardware/SimMotor.h"

#include <algorithm>
#include <cmath>

using namespace std;

SimMotor::SimMotor(const MotorConfig& c) : config(c), pwm(0), velocity(0.0) {}

void SimMotor::writePWM(const int value) {
    pwm.store(value);
}

double SimMotor::getTorque(const double speed, const double noise) {
    velocity = speed;
    const double backEmf = speed / config.kV;
    const double appliedVoltage = (pwm.load() / 255.0) * config.vBus;
    const double noisyVoltage = appliedVoltage * noise;
    const double statorCurrent = (noisyVoltage - backEmf) / config.r;
    const double electricalTorque = config.kT * statorCurrent;

    return electricalTorque;
}

double SimMotor::getTorque(const double speed) {
    return getTorque(speed, 0.0);
}

double SimMotor::getSpeed() const {
    return velocity;
}

double SimMotor::getPWM() const {
    return pwm.load();
}
