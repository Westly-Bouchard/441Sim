//
// Created by Westly Bouchard on 2/27/26.
//

#ifndef CONTROLLER_H
#define CONTROLLER_H

#ifdef SIM
#include "../wrappers/arduinoLib.h"
using namespace ArduinoLib;
#endif

// Simple P controller
// May be modified as necessary

class Controller {
public:
    float Kp, Kd, maxSpeed, dt, lastError;
    int min = 50;

    Controller()
      : Kp(0.0), Kd(0.0), maxSpeed(0.0), dt(0.03), lastError(0) {}
    Controller(float kp, float maxspeed)
      : Kp(kp), maxSpeed(maxspeed) {}
    void setGains(float kp, float kd, float maxspeed) {
        Kp = kp;
        Kd = kd;
        maxSpeed = maxspeed;
    };

    float getCmd(float error) {
        float u = (Kp * error) + (Kd * (error - lastError) / dt);
        if (u > maxSpeed) {
            u = maxSpeed;
        } else if (u < -maxSpeed) {
            u = -maxSpeed;
        }

        if (u < 10 && u > -10) {
            return 0;
        }

        if (u > 0 && u < min) {
            u = min;
        } else if (u < 0 && u > -min) {
            u = -min;
        }

        lastError = error;

        return u;
    }

    float getCmd(float posError, float velError, float ff) {
        float u = ff + Kp * posError + Kd * velError;

        if (u > maxSpeed) {
            u = maxSpeed;
        } else if (u < -maxSpeed) {
            u = -maxSpeed;
        }

        if (u < 10 && u > -10) {
            return 0;
        }

        if (u > 0 && u < min) {
            u = min;
        } else if (u < 0 && u > -min) {
            u = -min;
        }

        return u;
    }
};

#endif //CONTROLLER_H
