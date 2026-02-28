//
// Created by Westly Bouchard on 2/27/26.
//

#ifdef SIM
#include "../wrappers/arduinoLib.h"
using namespace ArduinoLib;
#include "arduinoMain.h"
#endif

int poses[3][8] = {
  { 140, 140, 200, 200,  100,  100,   0,    0 },
  { 0,   90,  90,  -90,  -90,  -150,  -150, 0 },
  { 90,  0,   -90, -180, -90,  -180,  -270, 0 }
};

#include "robot.h"
Robot myRobot;

int pushButton = 50;
int ran_course = 0;

void setup() {
#ifdef SIM
  ArduinoInit();
#endif

  Serial.begin(115200);

  myRobot.initializePins();

  myRobot.set_wheelbase(20.5);  // Wheel-to-wheel width in cms

  pinMode(pushButton, INPUT_PULLUP);

  // TODO: Set values for speed and heading controllers:
  // Proportional Gain (Kp), top speed
  myRobot.speedControl.setGains(900, 0.0, 255);
  myRobot.turnControl.setGains(4.5, 0.0, 150);
  myRobot.headingControl.setGains(200, 0.0, 255);
}

void loop() {
  if (!ran_course) {
    while (digitalRead(pushButton) == HIGH);
    for (int i = 0; i < 8; i++) {
      // Holds the car still for measurement until the push button is pressed
      delay(500);
      // Drive to target pose
      myRobot.goToPose(poses[0][i], poses[1][i], poses[2][i]);
      // Print current pose: x, y, theta
      Serial.println();
      Serial.print(myRobot.pose.x);
      Serial.print(", ");
      Serial.print(myRobot.pose.y);
      Serial.print(", ");
      Serial.println(myRobot.pose.theta);
    }
    ran_course = 1;
  }
}
