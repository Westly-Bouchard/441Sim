//
// Created by west on 6/7/26.
//

#ifndef MOSSCAP_CHALLENGE2_H
#define MOSSCAP_CHALLENGE2_H

#include "defaults.h"
#include "sim/TankSim.h"

#include "TelemetryPosition.h"

std::unique_ptr<SimulatorBase> simInit() {
    Renderer::setScale(2.0);

    auto config = defaultRobot;
    config.showMotorTelemetry = false;
    config.showTOFTelemetry = false;

    config.kineticFriction = 0.1;
    config.noiseMagnitude = 0.1;

    auto robot = std::make_unique<TankSim>(config);
    robot->setPose(0.5, 1.0, 0.0);

    TOFConfig tofConfig{0, 0, 0};
    tofConfig.boundingBox = BoundingBox(2, 2);


    robot->registerTOF(tofConfig);

    robot->registerLeftMotor(defaultMotor, 10);
    robot->registerRightMotor(defaultMotor, 20);

    robot->registerLeftEncoder(1500, 15);
    robot->registerRightEncoder(1500, 25);

    ArduinoRuntime::getInstance().createButton("Forward", 1);
    ArduinoRuntime::getInstance().createButton("Left", 2);
    ArduinoRuntime::getInstance().createButton("Right", 3);

    return robot;
}

#endif //MOSSCAP_CHALLENGE2_H
