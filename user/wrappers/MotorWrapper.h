//
// Created by Westly Bouchard on 2/27/26.
//

#ifndef MOTORWRAPPER_H
#define MOTORWRAPPER_H

#include <utility>

#define OUTPUT 0

class MotorWrapper {
public:
    MotorWrapper();

    int getSetpoint() const;

protected:
    void digitalWrite(int pin, int val);
    void analogWrite(int pin, int val);

    void pinMode(int pin, int mode);

    std::pair<int, int> forwardsPin;
    std::pair<int, int> backwardsPin;
    std::pair<int, int> analogPin;
};



#endif //MOTORWRAPPER_H
