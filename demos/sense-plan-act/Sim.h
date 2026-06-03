#ifndef SIM_H
#define SIM_H

#include <sim/TankSim.h>
#include <mosscap/defaults.h>

#include <memory>

void drawGoalSquare() {
    Renderer::drawRect(2.5, 2.5, 0.0, 0.2, 0.2, {0.0f, 1.0f, 0.25f});
}

std::unique_ptr<SimulatorBase> simInit() {
    // Set rendering scale
    Renderer::setScale(3.0);

    Renderer::getInstance().registerDrawFunction(-1, drawGoalSquare);

    // Create robot
    auto robot = std::make_unique<TankSim>(defaultRobot);
    robot->setPose(0.5, 0.5, 0);

    TOFConfig tofConfig{0, 0, 0};
    tofConfig.boundingBox = BoundingBox(3.0, 3.0);

    tofConfig.obstacles.push_back(std::make_shared<Box>(0.7, 0.7, Vec{1.6, 1.8}, 0.0));
    tofConfig.obstacles.push_back(std::make_shared<Box>(0.35, 0.9, Vec{2.25, 0.5}, 15 * PI / 180.0));
    tofConfig.obstacles.push_back(std::make_shared<Box>(0.75, 0.3, Vec{0.75, 2.3}, 45 * PI / 180.0));

    robot->registerTOF(tofConfig);

    robot->registerLeftMotor(defaultMotor, 10);
    robot->registerRightMotor(defaultMotor, 20);

    robot->registerLeftEncoder(1500, 15);
    robot->registerRightEncoder(1500, 25);

    ArduinoRuntime::getInstance().createButton("1. Sense", 1);
    ArduinoRuntime::getInstance().createButton("2. Plan", 2);
    ArduinoRuntime::getInstance().createButton("3. Act", 3);
    ArduinoRuntime::getInstance().createButton("4. Continue", 4);

    return robot;
}

#endif