//
// Created by Westly Bouchard on 2/27/26.
//

#ifndef ROBOT_H
#define ROBOT_H

#ifdef SIM
#include "../wrappers/arduinoLib.h"
using namespace ArduinoLib;
#endif

#include "motor.h"
#include "pose.h"
#include "controller.h"

// Creates a Twist class to hold
// linear, angular velocity
class Twist {
public:
  float linear = 0.0;
  float angular = 0.0;
  Twist() = default;
  void set(float vel, float ang_vel) {
    linear = vel;
    angular = ang_vel;
  }
};

struct Trap {
  double posEnd;
  double accel;
  double maxVel;

  double t_a;
  double bigT;

  Trap(double posEnd, double accel, double maxVel)
    : posEnd(posEnd), accel(accel), maxVel(maxVel) {
      t_a = (maxVel / accel);
      bigT = (((posEnd) / maxVel) + (maxVel / accel)); // Milliseconds
    }

  double samplePosition(double current) {
    if (current < t_a) {
      return 0.5 * accel * pow(current, 2);
    } else if (current < bigT - t_a) {
      return 0.5 * accel * pow(t_a, 2) + maxVel * (current - t_a);
    } else if (current < bigT) {
      return posEnd - 0.5 * accel * pow((bigT - current), 2);
    } else {
      return posEnd;
    }
  }

  double sampleVelocity(double current) {
    if (current < t_a) {
      return accel * current;
    } else if (current < bigT - t_a) {
      return maxVel;
    } else if (current < bigT) {
      return accel * (bigT - current);
    } else {
      return 0;
    }
  }
};

class Robot {
public:
  Robot() = default;

  float DIST_PER_COUNT = 0.0161262051;

  float lastTheta = 0;

  double maxAccel = 8.0;
  double maxVel = 0.8;


  // TODO: Set pins for each motor:
  // ENA, In1, In2, EncA, EncB
  // Note: if motor runs backward, flip in1 and in2
  // If counts are negative, flip EncA and EncB
  Motor FL{ 10, 28, 26, 18, 46 };  //Front left motor
  Motor BL{ 12, 36, 34, 2, 42 };  //Back left motor
  Motor FR{ 9, 24, 22, 19, 40 };  //Front right motor
  Motor BR{ 11, 32, 30, 3, 44 };  //Back right motor

  void initializePins() {
    FL.initializePins();
    BL.initializePins();
    FR.initializePins();
    BR.initializePins();
  }

  // Initialize controllers for speed and heading
  Controller speedControl;
  Controller turnControl;
  Controller headingControl;


  Pose pose;

  // Create a "command velocity" with
  // desired linear, angular velocity
  Twist cmd_vel;

  void set_wheelbase(float width) {
    pose.wheelbase = width;
  }

  void resetCounts() {
    FL.resetCounts();
    BL.resetCounts();
    FR.resetCounts();
    BR.resetCounts();
  }

  void runAll(int FLpwm, int BLpwm, int FRpwm, int BRpwm) {
    FL.run(FLpwm);
    BL.run(BLpwm);
    FR.run(FRpwm);
    BR.run(BRpwm);
  }

  void updateVelocities(Pose target) {
    if ((fabs(target.x - pose.x) < xyTolerance) && (fabs(target.y - pose.y) < xyTolerance)) {
      // Turning stuff

      cmd_vel.angular = turnControl.getCmd(target.theta - pose.theta);
      cmd_vel.linear = 0;

    } else {
      // Going straight
      double posError = sqrt((target.x - pose.x) * (target.x - pose.x) + (target.y - pose.y) * (target.y - pose.y));

      cmd_vel.linear = speedControl.getCmd(posError);
      cmd_vel.angular = headingControl.getCmd(target.theta - pose.theta);
    }
  }

  void sendVelocities() {

    if (cmd_vel.linear) {
      if (cmd_vel.linear > 0) {
        FL.run((cmd_vel.linear * 0.6) - (cmd_vel.angular * 0.4));
        BL.run((cmd_vel.linear * 0.6) - (cmd_vel.angular * 0.4));

        FR.run((cmd_vel.linear * 0.6) + (cmd_vel.angular * 0.4));
        BR.run((cmd_vel.linear * 0.6) + (cmd_vel.angular * 0.4));
      } else {
        runAll(-75, -75, -75, -75);
      }
    } else if (cmd_vel.angular) {
      FL.run(-cmd_vel.angular);
      BL.run(-cmd_vel.angular);

      FR.run(cmd_vel.angular);
      BR.run(cmd_vel.angular);
    } else {
      runAll(0, 0, 0, 0);
    }
  }

  // These tolerances can be used to determine if you've reached the target
  float xyTolerance = 3.0;     // Set xy tolerance in cms
  float angleTolerance = 1.0;  // Set angle tolerance in degrees

  bool reachedPose(Pose target) {
    return (fabs(target.x - pose.x) < xyTolerance) && (fabs(target.y - pose.y) < xyTolerance) && (fabs(target.theta - pose.theta) < angleTolerance);
  }

  void goToPose(float x_target, float y_target, float theta_target) {
    // Target pose for passing to reachedPose, updateVelocities
    Pose target;
    target.set(x_target, y_target, pose.theta);
    resetCounts();

    Pose initialPose = pose;

    bool isY = fabs(x_target - pose.x) < xyTolerance;

    double posEnd = isY ? y_target - pose.y : x_target - pose.x;

    Trap trap(posEnd / 100.0, maxAccel, maxVel);

    float prevLeftCount = 0;
    float prevRightCount = 0;
    double accTime = 0;


    // Straight line driving
    while (!reachedPose(target)) {
      int32_t lastTime = -20;  // ensure entering loop
      if (millis() - lastTime > 30) {
        // Read the counts from each motor.

        float leftCount = ((FL.readCounts() + BL.readCounts()) / 2.0);
        float rightCount = ((FR.readCounts() + BR.readCounts()) / 2.0);

        float leftDistChange = (leftCount - prevLeftCount) * DIST_PER_COUNT;
        float rightDistChange = (rightCount - prevRightCount) * DIST_PER_COUNT;

        // Calculate change in left/right encoders - CONVERT TO CMS
        pose.update(leftDistChange, rightDistChange);
        Serial.println(pose.x);

        // Do controller stuff
        double vd = trap.sampleVelocity(accTime) * 100;
        double xd = trap.samplePosition(accTime) * 100;

        double vc = (leftDistChange + rightDistChange) / (2 * 0.03);
        double xc = isY ? pose.y - initialPose.y : pose.x - initialPose.x;

        double u_ff = vd / maxVel * 255;

        Serial.print(xd);
        Serial.print(" ");
        Serial.println(xc);

        Serial.print(xd-xc);
        Serial.print(" ");
        Serial.println(vd-vc);

        Serial.println(speedControl.getCmd(xd - xc, vd - vc, u_ff));

        cmd_vel.linear = speedControl.getCmd(xd - xc, vd - vc, u_ff);
        cmd_vel.angular = headingControl.getCmd(target.theta - pose.theta);

        // updateVelocities(target);
        sendVelocities();
        lastTime = millis();
        prevLeftCount = leftCount;
        prevRightCount = rightCount;
        accTime += 0.03;
      }
    }

    delay(500);

    // Turning
    target.set(pose.x, pose.y, theta_target);
    resetCounts();

    prevLeftCount = 0;
    prevRightCount = 0;


    while (!reachedPose(target)) {
      int32_t lastTime = -20;  // ensure entering loop
      if (millis() - lastTime > 30) {
        // Read the counts from each motor.

        float leftCount = ((FL.readCounts() + BL.readCounts()) / 2.0);
        float rightCount = ((FR.readCounts() + BR.readCounts()) / 2.0);

        float leftDistChange = (leftCount - prevLeftCount) * DIST_PER_COUNT;
        float rightDistChange = (rightCount - prevRightCount) * DIST_PER_COUNT;

        // Calculate change in left/right encoders - CONVERT TO CMS
        pose.update(leftDistChange, rightDistChange);

        updateVelocities(target);
        sendVelocities();
        lastTime = millis();
        prevLeftCount = leftCount;
        prevRightCount = rightCount;
      }
    }

    lastTheta = theta_target;
  }
};

#endif //ROBOT_H
