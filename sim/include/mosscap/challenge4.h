//
// Created by west on 6/9/26.
//

#ifndef MOSSCAP_CHALLENGE4_H
#define MOSSCAP_CHALLENGE4_H

#include "defaults.h"
#include "sim/TankSim.h"

#include "Encoder.h"
#include "Motor.h"
#include "TelemetryMap.h"
#include "TelemetryPosition.h"

std::unique_ptr<SimulatorBase> simInit() {
    Renderer::setScale(3.0);

    auto config = defaultRobot;

    config.showMotorTelemetry = false;
    config.showEncoderTelemetry = false;

    config.kineticFriction = 0.1;
    config.noiseMagnitude = 0.1;

    auto robot = std::make_unique<TankSim>(config);
    robot->setPose(1.5, 1.5, 0);

    TOFConfig tofConfig{0.0, 0.0, 0.0};
    tofConfig.boundingBox = BoundingBox(3.0, 3.0);

    tofConfig.obstacles.push_back(std::make_shared<Box>(1.0, 0.5, Vec{1.5, 2.5}, 15 * PI / 180.0));
    tofConfig.obstacles.push_back(std::make_shared<Box>(0.5, 0.8, Vec{0.7, 0.75}, -5 * PI / 180.0));
    tofConfig.obstacles.push_back(std::make_shared<Box>(0.45, 0.75, Vec{2.5, 0.8}, 45 * PI / 180.0));

    robot->registerTOF(tofConfig);

    robot->registerLeftMotor(defaultMotor, 10);
    robot->registerRightMotor(defaultMotor, 20);

    robot->registerLeftEncoder(1500, 15);
    robot->registerRightEncoder(1500, 25);

    ArduinoRuntime::getInstance().createButton("Button", 1);

    return robot;
}

Motor left(11, 10, 12);
Motor right(21, 20, 22);

Encoder leftEncoder(15, 16);
Encoder rightEncoder(25, 26);

const double metersPerCount = 1.673e-4;
const double trackWidth = 0.20;

TelemetryPosition pose;

void updatePosition() {
    double leftDistance = leftEncoder.readAndReset() * metersPerCount;
    double rightDistance = rightEncoder.readAndReset() * metersPerCount;

    double d = (leftDistance + rightDistance) / 2.0;
    double dTheta = (rightDistance - leftDistance) / trackWidth;

    // Convert from degrees to radians
    double thetaOld = pose.getTheta() * PI / 180.0;

    double xNew = pose.getX() + d * cos(thetaOld + dTheta / 2.0);
    double yNew = pose.getY() + d * sin(thetaOld + dTheta / 2.0);
    double thetaNew = thetaOld + dTheta;

    // Clamp to 0->360 range. Not strictly necessary but helpful for later math
    if (thetaNew > 2 * PI) thetaNew -= 2 * PI;
    if (thetaNew < -2 * PI) thetaNew += 2 * PI;

    pose.setX(xNew);
    pose.setY(yNew);

    // Convert back to degrees
    pose.setTheta(thetaNew * 180.00 / PI);
}

void updateMap();

const int minPWM = 70;
const double TURN_KP = 2.8;

void performMappingSweep() {
    const double angle = 360.0;
    double startAngle = pose.getTheta();

    while (abs(startAngle + angle - pose.getTheta()) >= 2) {
        updatePosition();
        updateMap();

        double error = abs(startAngle + angle - pose.getTheta());

        int command = error * TURN_KP;

        if (command < 0) command -= minPWM;
        if (command > 0) command += minPWM;

        // Ensure that our command is within our PWM range
        if (command > 100) command = 100;
        if (command < -100) command = -100;

        left.run(-command);
        right.run(command);
    }

    left.run(0);
    right.run(0);
}

#endif //MOSSCAP_CHALLENGE4_H
