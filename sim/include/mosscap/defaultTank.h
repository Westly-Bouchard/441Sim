//
// Created by west on 5/13/26.
//

#ifndef MOSSCAP_DEFAULTTANK_H
#define MOSSCAP_DEFAULTTANK_H

#include "sim/TankSim.h"

#include "defaults.h"

std::unique_ptr<SimulatorBase> simInit() {
    // Set rendering scale
    Renderer::setScale(2.0);

    // Create robot
    auto robot = std::make_unique<TankSim>(defaultRobot);
    robot->setPose(1, 1, 0);

    TOFConfig tofConfig{0, 0, 0};
    tofConfig.boundingBox = BoundingBox(3, 3);

    robot->registerTOF(tofConfig);

    robot->registerLeftMotor(defaultMotor, 10);
    robot->registerRightMotor(defaultMotor, 20);

    robot->registerLeftEncoder(1500, 15);
    robot->registerRightEncoder(1500, 25);

    ArduinoRuntime::getInstance().createButton("Button", 1);

    return robot;
}

#endif //MOSSCAP_DEFAULTTANK_H
