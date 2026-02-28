//
// Created by Westly Bouchard on 2/27/26.
//

#ifndef ARDUINOLIB_H
#define ARDUINOLIB_H

#include <cmath>
#include <chrono>
#include <iostream>

#include "../../managers.h"

#define INPUT_PULLUP 1

#define HIGH 1
#define LOW 0

namespace ArduinoLib {
    class ArdSerial {
    public:
        ArdSerial() = default;

        static void begin(int baud) {}

        template <typename T>
        static void print(T toPrint) {
            std::cout << toPrint;
        }

        template <typename T>
        static void println(T toPrint) {
            std::cout << toPrint << std::endl;
        }
        static void println() {
            std::cout << std::endl;
        }
    };

    void pinMode(int pin, int mode);

    int digitalRead(int pin);

    void delay(int millis);

    int millis();
}

void ArduinoInit();

static ArduinoLib::ArdSerial Serial;

static std::chrono::time_point<std::chrono::system_clock> arduinoTime;

#endif //ARDUINOLIB_H
