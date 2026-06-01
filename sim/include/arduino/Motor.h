//
// Created by Westly Bouchard on 3/2/26.
//

#ifndef MOTOR_H
#define MOTOR_H

#include "ArduinoRuntime.h"

/**
 * Motor on a robot
 */
class Motor {
public:
    /**
     * Construct a motor
     * For simulation purposes, pinA is the only important one
     * @param pinA Direction 1 pin
     * @param pinB Direction 2 pin
     * @param enable Enable pin (pwm)
     */
    Motor(const int enable, const int pinA, const int pinB) : pin(pinA) {}

    /**
     * Simulation specific constructor
     * To be used if the codebase is only in simulation, where
     * other motor pins don't matter
     * @param pin Pin to identify the motor
     */
    explicit Motor(const int pin) : pin(pin) {}

    /**
     * Run the motor at a given speed
     * @param pwm Signed speed between -255 and 255
     */
    void run(const int pwm) const {
        ArduinoRuntime::getInstance().setPwm(pin, pwm);
    }

    /**
     * Stop the motor by writing pwm zero
     */
    void stop() const {
        ArduinoRuntime::getInstance().setPwm(pin, 0);
    }

private:
    /**
     * Identifier to get the correct handle from the simulator when writing
     */
    int pin;
};

#endif //MOTOR_H
