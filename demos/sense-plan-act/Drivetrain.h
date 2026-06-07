#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <Encoder.h>
#include <Motor.h>

#include <mosscap/TelemetryPosition.h>

// #include "TelemetryPosition.h"
#include "Trajectory.h"

// Some important physical constants from the robot
#define CPR 1500
#define TRACKWIDTH 0.2
#define WHEEL_RADIUS 0.04

#define FORWARD_KV 0.475
#define TURN_KV 0.427

#define FORWARD_V_MAX 0.35
#define FORWARD_A_MAX 0.5

#define TURN_V_MAX 180
#define TURN_A_MAX 360

void updateOdometry(TelemetryPosition& pose, Encoder& eL, Encoder& eR) {
    const int leftCounts = eL.readAndReset();
    const int rightCounts = eR.readAndReset();

    const double leftDistance = (double)leftCounts / CPR * 2 * PI * WHEEL_RADIUS;
    const double rightDistance = (double)rightCounts / CPR * 2 * PI * WHEEL_RADIUS;

    const double dS = 0.5 * (leftDistance + rightDistance);
    const double dTheta = (rightDistance - leftDistance) / TRACKWIDTH * 180.0 / PI;

    const double dY = dS * sin((pose.getTheta() + 0.5 * dTheta) * PI / 180.0);
    const double dX = dS * cos((pose.getTheta() + 0.5 * dTheta) * PI / 180.0);

    pose.setX(pose.getX() + dX);
    pose.setY(pose.getY() + dY);
    pose.setTheta(pose.getTheta() + dTheta);
}

// Drive forward using a trapezoidal trajectory
void driveDistance(double distMeters, Motor& mL, Motor& mR, TelemetryPosition& pose, Encoder& eL, Encoder& eR) {
    Trajectory traj(distMeters, FORWARD_V_MAX, FORWARD_A_MAX);

    int lastTime = millis();

    double startTime = lastTime / 1000.0;

    while (millis() / 1000.0 - startTime < traj.T + 0.5) {
        int currentTime = millis();

        if (currentTime - lastTime > 50) {
            updateOdometry(pose, eL, eR);

            const double t = currentTime / 1000.0 - startTime;
            double desiredVelocity = traj.sampleVelocity(t);

            int pwm = desiredVelocity / FORWARD_V_MAX * 255 * FORWARD_KV;

            mL.run(pwm);
            mR.run(pwm);

            lastTime = currentTime;
        }
    }

    mL.run(0);
    mR.run(0);
}

// Turn to a specified angle using a trapezoidal trajectory
void turnAngle(double angleDegrees, Motor& mL, Motor& mR, TelemetryPosition& pose, Encoder& eL, Encoder& eR) {
    Trajectory traj(abs(angleDegrees), TURN_V_MAX, TURN_A_MAX);

    int lastTime = millis();

    double startTime = lastTime / 1000.0;

    while (millis() / 1000.0 - startTime < traj.T + 0.5) {
        int currentTime = millis();

        if (currentTime - lastTime > 50) {
            updateOdometry(pose, eL, eR);

            const double t = currentTime / 1000.0 - startTime;
            double desiredVelocity = traj.sampleVelocity(t);

            int pwm = desiredVelocity / TURN_V_MAX * 255 * TURN_KV;

            if (angleDegrees > 0.0) {
                mL.run(-pwm);
                mR.run(pwm);
            } else {
                mL.run(pwm);
                mR.run(-pwm);
            }

            lastTime = currentTime;
        }
    }

    mL.run(0);
    mR.run(0);
}

#endif