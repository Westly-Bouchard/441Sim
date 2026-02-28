//
// Created by Westly Bouchard on 2/27/26.
//

#include "arduinoLib.h"

#include <iostream>

using namespace ArduinoLib;
using namespace std;

void ArduinoInit() {
    Serial = ArdSerial();

    arduinoTime = std::chrono::system_clock::now();
}

void ArduinoLib::pinMode(int pin, int mode) {
    /* Do nothing */
}

int ArduinoLib::digitalRead(int pin) {
    // For now
    return HIGH;
}

void ArduinoLib::delay(int millis) {
    // For now do nothing
}

int ArduinoLib::millis() {
    const auto now = std::chrono::system_clock::now();

    return std::chrono::duration_cast<std::chrono::milliseconds>(now - arduinoTime).count();
}