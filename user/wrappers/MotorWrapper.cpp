//
// Created by Westly Bouchard on 2/27/26.
//

#include "MotorWrapper.h"

#include "../../managers.h"


MotorWrapper::MotorWrapper() {
    MotorManager::getInstance().registerMotor(this);
}


void MotorWrapper::pinMode(int pin, int mode) {
    static int counter = 0;

    if (counter == 0) {
        forwardsPin = {pin, 0};
    } else if (counter == 1) {
        backwardsPin = {pin, 0};
    } else if (counter == 2) {
        analogPin = {pin,0};
    }

    counter++;
}

void MotorWrapper::digitalWrite(int pin, int val) {
    if (pin == forwardsPin.first) {
        forwardsPin.second = val;
    } else if (pin ==  backwardsPin.first) {
        backwardsPin.second = val;
    }
}

void MotorWrapper::analogWrite(int pin, int val) {
    if (pin == analogPin.first) {
        analogPin.second = val;
    }
}

int MotorWrapper::getSetpoint() const {
    if (forwardsPin.second == 1 && backwardsPin.second == 0) {
        // Going forward, return the positive pwm value on the analog pin
        return analogPin.second;
    }
    if (forwardsPin.second == 0 && backwardsPin.second == 1) {
        return -1 * analogPin.second;
    }

    return 0;
}
