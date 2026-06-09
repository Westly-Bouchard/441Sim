//
// Created by west on 6/8/26.
//

#ifndef MOSSCAP_CHALLENGE3_H
#define MOSSCAP_CHALLENGE3_H

#include <imgui.h>

#include "defaults.h"
#include "sim/TankSim.h"

#include "TelemetryPosition.h"

void drawPath();

std::unique_ptr<SimulatorBase> simInit() {
    // Set rendering scale
    Renderer::setScale(3.0);
    Renderer::getInstance().registerDrawFunction(-2, drawPath);

    auto config = defaultRobot;
    config.showTOFTelemetry = false;

    config.kineticFriction = 0.07;

    config.noiseMagnitude = 0.1;

    // Create robot
    auto robot = std::make_unique<TankSim>(config);
    robot->setPose(0.6, 0.75, 0);

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

static constexpr std::array<std::array<double, 2>, 8> pathPoints = {{
    {0.6, 0.75},
    {2.1, 0.75},
    {2.1, 1.45},
    {2.4, 1.45},
    {2.4, 2.25},
    {1.2, 2.25},
    {1.2, 1.75},
    {0.6, 1.75}
}};

static constexpr std::array<std::array<double, 2>, 8> pathPoints2 = {{
    {0.6, 0.75},
    {2.1, 0.6},
    {1.9, 1.45},
    {2.4, 1.6},
    {2.55, 2.25},
    {1.2, 2.4},
    {1.0, 1.75},
    {0.6, 1.55}
}};

inline void drawPath() {
    std::array<ImVec2, pathPoints.size() + 1> vertices;
    for (auto&& [p, v] : std::views::zip(pathPoints, vertices)) {
        v.x = p.at(0) * pxPerMeter;
        v.y = 3.0 * pxPerMeter - p.at(1) * pxPerMeter;
    }

    // This is kind of hacky but whatever
    vertices.at(8) = vertices.at(0);
    vertices.at(8).y += 3.1;

    ImGui::GetWindowDrawList()->AddPolyline(vertices.data(), vertices.size(), 0xFF00FF00, 0, 7.0);
}
#endif //MOSSCAP_CHALLENGE3_H
