//
// Created by Westly Bouchard on 3/2/26.
//

#include "arduino/ArduinoRuntime.h"

#include <chrono>
#include <thread>

#include "sim/SimulatorBase.h"
#include "hardware/SimButton.h"

ArduinoRuntime::ArduinoRuntime() : clock(SimulatorBase::getClock()) {}

ArduinoRuntime &ArduinoRuntime::getInstance() {
    static ArduinoRuntime instance;
    return instance;
}

void ArduinoRuntime::freeze() {
    m.lock();
}

void ArduinoRuntime::unfreeze() {
    m.unlock();
}

void ArduinoRuntime::bindPWM(const int pin, WriteablePWM &pwm) {
    pwmMap.emplace(pin, Handle{pwm});
}

void ArduinoRuntime::bindEncoder(const int pin, ReadableEncoder &encoder) {
    encoderMap.emplace(pin, Handle{encoder});
}

void ArduinoRuntime::bindTOF(ReadableDistance &tof) {
    i2cTOF = std::make_unique<Handle<ReadableDistance>>(tof);
}

void ArduinoRuntime::createButton(const std::string& name, int pin) {
    // Default to regular input, if user uses pinMode later with INPUT_PULLUP
    // it will modify the pair.second here to signal that to the runtime
    digitalInputs.emplace(pin, std::make_pair(std::make_unique<SimButton>(name), false));
}

void ArduinoRuntime::setInputPullup(const int pin) {
    digitalInputs.at(pin).second = INPUT_PULLUP;
}

bool ArduinoRuntime::getDigitalInputState(const int pin) const {
    auto& [device, pullup] = digitalInputs.at(pin);
    return !device->digitalRead() != !pullup;
}

Handle<WriteablePWM> ArduinoRuntime::getPWM(const int pin) const {
    return pwmMap.at(pin);
}

void ArduinoRuntime::setPwm(const int pin, const int pwm) {
    m.lock();
    pwmMap.at(pin).get().writePWM(pwm);
    m.unlock();
}

Handle<ReadableEncoder> ArduinoRuntime::getEncoder(const int pin) const {
    return encoderMap.at(pin);
}

Handle<ReadableDistance> ArduinoRuntime::getTOF() const {
    return *i2cTOF;
}

Handle<ReadableTime> ArduinoRuntime::getClock() const {
    return clock;
}

void pinMode(const int pin, const int mode) {
    if (mode == INPUT_PULLUP) {
        ArduinoRuntime::getInstance().setInputPullup(pin);
    }
}

bool digitalRead(const int pin) {
    return ArduinoRuntime::getInstance().getDigitalInputState(pin);
}

/**
 * Arduino time functions
 */

/* Static cast to int is appropriate here because we only want to count whole
 * micro or milliseconds that have passed
 */

int millis() {
    return static_cast<int>(ArduinoRuntime::getInstance().getClock().get().readTime() * 1'000);
}

int micros() {
    return static_cast<int>(ArduinoRuntime::getInstance().getClock().get().readTime() * 1'000'000);
}

void delay(const int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void delayMicroseconds(const int us) {
    std::this_thread::sleep_for(std::chrono::microseconds(us));
}