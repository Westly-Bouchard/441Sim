//
// Created by Westly Bouchard on 2/27/26.
//

#ifndef MANAGERS_H
#define MANAGERS_H

#include <vector>

class MotorWrapper;
class Encoder;

class MotorManager {
public:
    static MotorManager& getInstance() {
        static MotorManager instance;
        return instance;
    }

    void registerMotor(MotorWrapper* motor) {
        motors.push_back(motor);
    }

    MotorWrapper* getMotor(int index) {
        return motors.at(index);
    }

private:
    MotorManager() = default;

    std::vector<MotorWrapper*> motors;
};

class EncoderManager {
public:
    static EncoderManager& getInstance() {
        static EncoderManager instance;
        return instance;
    }

    void registerEncoder(Encoder* encoder) {
        encoders.push_back(encoder);
    }

    Encoder* getEncoder(const int index) const {
        return encoders.at(index);
    }

private:
    EncoderManager() = default;

    std::vector<Encoder*> encoders;
};
#endif //MANAGERS_H
