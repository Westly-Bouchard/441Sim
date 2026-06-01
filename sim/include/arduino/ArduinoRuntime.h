//
// Created by Westly Bouchard on 3/2/26.
//

#ifndef ARDUINORUNTIME_H
#define ARDUINORUNTIME_H

#include <unordered_map>
#include <cmath>

// Redefine PI to match how Arduino defines it
#define PI M_PI

#include <memory>
#include <mutex>

#include "Handle.hpp"
#include "Wire.h"
#include "capability/DigitallyReadable.h"
#include "capability/ReadableDistance.h"
#include "capability/ReadableEncoder.h"
#include "capability/ReadableTime.h"
#include "capability/WriteablePWM.h"

/**
 * Setup function from user code
 */
void setup();

/**
 * Loop function from user code
 */
void loop();

/**
 * This is the structure that the user code uses to interact with
 * the simulator.
 */
class ArduinoRuntime {
public:
    /**
     * Get singleton runtime instance
     * @return Runtime instance
     */
    static ArduinoRuntime& getInstance();

    /**
     * Deleted funcs because singleton
     */
    ArduinoRuntime(const ArduinoRuntime&) = delete;
    ArduinoRuntime& operator=(const ArduinoRuntime&) = delete;

    // This interface should really only be available to the simulator, not to user code
    // I'm not really sure how to fix this tbh
    void freeze();
    void unfreeze();

    /**
     * Make a certain pin PWM writeable by the user's code
     * @param pin Pin to bind
     * @param pwm Handle to capability
     */
    void bindPWM(int pin, WriteablePWM& pwm);

    /**
     * Bind an encoder to the runtime
     * Pin is the ID used to access and read the counts later
     * @param pin Encoder A Pin
     * @param encoder Readable encoder capability
     */
    void bindEncoder(int pin, ReadableEncoder& encoder);

    /**
     * Bind a time of flight sensor to the runtime
     * @param tof Readable distance capability
     */
    void bindTOF(ReadableDistance& tof);

    void createButton(const std::string& name, int pin);

    void setInputPullup(int pin);

    bool getDigitalInputState(int pin) const;

    /**
     * Get a handle to a writeable PWM to write a PWM to it
     * @param pin Pin capability is bound to
     * @return Handle to capability
     */
    [[nodiscard]] Handle<WriteablePWM> getPWM(int pin) const;

    void setPwm(int pin, int pwm);

    /**
     * Get a handle to an encoder to read its value
     * @param pin Pin capability is bound to
     * @return Handle to capability
     */
    [[nodiscard]] Handle<ReadableEncoder> getEncoder(int pin) const;

    /**
     * Get a handle to the time of flight sensor to read its distance
     * @return Handle to TOF capability
     */
    [[nodiscard]] Handle<ReadableDistance> getTOF() const;

    /**
     * Get a handle to the simulator's clock to read the current time
     * @return Handle to simulated clock
     */
    [[nodiscard]] Handle<ReadableTime> getClock() const;

private:
    /**
     * Private ctor and dtor because singleton
     */
    ArduinoRuntime();
    ~ArduinoRuntime() = default;

    std::mutex m;

    /**
     * Maps that store motors and encoders by their bound pins
     */
    std::unordered_map<int, Handle<WriteablePWM>> pwmMap;
    std::unordered_map<int, Handle<ReadableEncoder>> encoderMap;

    /**
     * Right now the runtime only supports one TOF sensor, this could
     * be changed in the future, though
     */
    std::unique_ptr<Handle<ReadableDistance>> i2cTOF;

    /**
     * Handle to simulator's clock
     * Not a unique pointer because this is created when the runtime
     * is constructed.
     */
    Handle<ReadableTime> clock;

    std::unordered_map<int, std::pair<std::unique_ptr<DigitallyReadable>, bool>> digitalInputs;
};

/**
 * For simulated I2C peripherals
 * They just need a way to call `Wire.begin()` which, in this context,
 * doesn't actually do anything.
 */
inline SimWire Wire;

/**
 * Arduino digital input and output functions
 */
#define INPUT 0
#define INPUT_PULLUP 1
// #define OUTPUT 2
void pinMode(int pin, int mode);

bool digitalRead(int pin);

/**
 * Arduino time functions
 */
int millis();

int micros();

void delay(int ms);

void delayMicroseconds(int us);

/**
 * Some of the Arduino standard functions
 */
template <typename T>
T constrain(T in, T low, T high) {
    if (in < low) { return low; }
    if (in > high) { return high; }
    return in;
}

#endif //ARDUINORUNTIME_H
