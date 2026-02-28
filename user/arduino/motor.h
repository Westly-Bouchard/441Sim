//
// Created by Westly Bouchard on 2/27/26.
//

#ifndef MOTOR_H
#define MOTOR_H

#define ENCODER_USE_INTERRUPTS
#ifdef SIM
#include "../wrappers/arduinoLib.h"
#include "../wrappers/Encoder.h"
#include "../wrappers/MotorWrapper.h"
#else
#include <Encoder.h>
#endif

// #include "controller.h"

class Motor
#ifdef SIM
  : public MotorWrapper
#endif
{
private:
  int ena, in1, in2, enc1, enc2;

  double targetSpeed; // counts

  double lastCount;
  int lastTime;

public:
  Encoder encoder;

  Motor(int ENApin, int IN1pin, int IN2pin, int encPin1, int encPin2)
    : ena(ENApin), in1(IN1pin), in2(IN2pin), enc1(encPin1), enc2(encPin2), encoder(encPin1, encPin2) {
      targetSpeed = 0;
      lastCount = 0;
      lastTime = 0;
    }
  
  void setTarget(double cps) {
    targetSpeed = cps;
  }

  void initializePins() {
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(ena, OUTPUT);
    pinMode(enc1, INPUT_PULLUP);
    pinMode(enc2, INPUT_PULLUP);
  }

  // Encoder functions
  int32_t readCounts() {
    return encoder.read();
  }

  void resetCounts() {
    lastCount = 0;
    encoder.write(0);
  }


  // Motor driver functions
  void run(float speed) {
    int pwm, reverse;
    if (speed >= 0) {
      reverse = 0;
      pwm = int(speed);
    } else {
      reverse = 1;
      pwm = abs(int(speed));
    }
    if (pwm > 255) {
      pwm = 255;
    }

    digitalWrite(in1, !reverse);
    digitalWrite(in2, reverse);
    analogWrite(ena, pwm);
  }

  void stop() {
    digitalWrite(in1, 0);
    digitalWrite(in2, 0);
    analogWrite(ena, 0);
  }
};

#endif //MOTOR_H
