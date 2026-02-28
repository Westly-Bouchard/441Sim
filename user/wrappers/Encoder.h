//
// Created by Westly Bouchard on 2/27/26.
//

#ifndef ENCODER_H
#define ENCODER_H

#include "../../managers.h"

class Encoder {

    int count;

public:
    Encoder(int pin1, int pin2) {
        EncoderManager::getInstance().registerEncoder(this);
    }

    void setCount(int count) {
        this->count = count;
    }

    int read() {
        return count;
    }

    void write(int value) {
        this->count = value;
    }

};

#endif //ENCODER_H
