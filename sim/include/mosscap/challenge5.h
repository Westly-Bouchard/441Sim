//
// Created by west on 6/11/26.
//

#ifndef MOSSCAP_CHALLENGE5_H
#define MOSSCAP_CHALLENGE5_H

#include "AStarPlanner.h"
#include "TelemetryMap.h"
#include "TelemetryPosition.h"

#include "Motor.h"
#include "Encoder.h"
#include "VL53L0X.h"

#include "defaults.h"
#include "sim/TankSim.h"

using namespace std;

inline void drawGoalSquare() {
    Renderer::drawRect(1.8, 2.6, 0.0, 0.2, 0.2, {0.0f, 1.0f, 0.25f});
}

std::unique_ptr<SimulatorBase> simInit() {
    Renderer::setScale(3.0);

    Renderer::getInstance().registerDrawFunction(-1, drawGoalSquare);

    auto config = defaultRobot;

    config.showMotorTelemetry = false;
    config.showEncoderTelemetry = false;

    config.kineticFriction = 0.1;
    config.noiseMagnitude = 0.1;

    auto robot = std::make_unique<TankSim>(config);
    robot->setPose(0.5, 0.5, 0);

    TOFConfig tofConfig{0.0, 0.0, 0.0};
    tofConfig.boundingBox = BoundingBox(3.0, 3.0);

    tofConfig.obstacles.push_back(std::make_shared<Box>(0.7, 0.7, Vec{1.6, 1.8}, 0.0));
    tofConfig.obstacles.push_back(std::make_shared<Box>(0.35, 0.9, Vec{2.25, 0.5}, 15 * PI / 180.0));
    tofConfig.obstacles.push_back(std::make_shared<Box>(0.75, 0.3, Vec{0.75, 2.3}, 45 * PI / 180.0));

    robot->registerTOF(tofConfig);

    robot->registerLeftMotor(defaultMotor, 10);
    robot->registerRightMotor(defaultMotor, 20);

    robot->registerLeftEncoder(1500, 15);
    robot->registerRightEncoder(1500, 25);

    ArduinoRuntime::getInstance().createButton("Sense", 1);
    ArduinoRuntime::getInstance().createButton("Plan", 2);

    return robot;
}

inline Motor leftMotor(11, 10, 12);
inline Motor rightMotor(21, 20, 22);

inline Encoder leftEncoder(15, 16);
inline Encoder rightEncoder(25, 26);

constexpr double metersPerCount = 1.673e-4;
constexpr double trackWidth = 0.20;

inline TelemetryPosition pose;

inline void updatePosition() {
    const double leftDistance = leftEncoder.readAndReset() * metersPerCount;
    const double rightDistance = rightEncoder.readAndReset() * metersPerCount;

    const double d = (leftDistance + rightDistance) / 2.0;
    const double dTheta = (rightDistance - leftDistance) / trackWidth;

    // Convert from degrees to radians
    const double thetaOld = pose.getTheta() * PI / 180.0;

    const double xNew = pose.getX() + d * cos(thetaOld + dTheta / 2.0);
    const double yNew = pose.getY() + d * sin(thetaOld + dTheta / 2.0);
    double thetaNew = thetaOld + dTheta;

    // Clamp to 0->360 range. Not strictly necessary but helpful for later math
    if (thetaNew > 2 * PI) thetaNew -= 2 * PI;
    if (thetaNew < -2 * PI) thetaNew += 2 * PI;

    pose.setX(xNew);
    pose.setY(yNew);

    // Convert back to degrees
    pose.setTheta(thetaNew * 180.00 / PI);
}

inline VL53L0X tof;

constexpr double mapSize = 3.0;
constexpr double mapResolution = 0.1;

inline TelemetryMap map(mapSize, mapResolution);

inline void updateMap() {
    const double currentAngle = pose.getTheta();

    const double dist = tof.readRangeSingleMillimeters() / 1000.0;

    const double thetaRadians = currentAngle * PI / 180.0;

    const double globalX = pose.getX() + dist * cos(thetaRadians);
    const double globalY = pose.getY() + dist * sin(thetaRadians);

    const int mapX = floor(globalX / mapResolution);
    const int mapY = floor(globalY / mapResolution);

    for (int rOffset = -1; rOffset <= 1; rOffset++) {
        for (int cOffset = -1; cOffset <= 1; cOffset++) {
            map.set(mapX + cOffset, mapY + rOffset);
        }
    }

    map.set(mapX, mapY);
}

constexpr int minPWM = 70;
constexpr double TURN_KP = 2.8;

inline void performMappingSweep() {
    constexpr double angle = 360.0;
    const double startAngle = pose.getTheta();

    while (abs(startAngle + angle - pose.getTheta()) >= 2) {
        updatePosition();
        updateMap();

        const double error = abs(startAngle + angle - pose.getTheta());

        int command = error * TURN_KP;

        if (command < 0) command -= minPWM;
        if (command > 0) command += minPWM;

        // Ensure that our command is within our PWM range
        if (command > 200) command = 200;
        if (command < -200) command = -200;

        leftMotor.run(-command);
        rightMotor.run(command);
    }

    leftMotor.run(0);
    rightMotor.run(0);
}

#endif //MOSSCAP_CHALLENGE5_H
