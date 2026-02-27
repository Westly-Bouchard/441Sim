//
// Created by west on 2/26/26.
//

#ifndef INC_441SIM_DRAW_H
#define INC_441SIM_DRAW_H

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <iostream>
using namespace std;

#include "util.h"

constexpr double ROBOT_WIDTH = 245.0 / 1000.0; // Meters
constexpr double ROBOT_HEIGHT = 200.0 / 1000.0; // Meters

constexpr double PX_PER_METER = 1000.0 / 3.0;

inline void drawDebugWindow() {
    const ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(1016, 0));
    ImGui::SetNextWindowSize(ImVec2(264, 100));
    ImGui::Begin("Debug");
    ImGui::Text("%.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
    ImGui::End();
}

inline std::vector<std::pair<double, double>> points = {
    {1.0, 0.3},
    {-0.4, 0.3},
    {-0.4, 1.2},
    {-1.0, 1.2},
    {-1.0, -0.6},
    {0.0, -0.6},
    {0.0, -1.2},
    {1.0, -1.2}
};

inline void drawCourse() {
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    const ImVec2 cursor = ImGui::GetCursorScreenPos();

    for (int i = 0; i < points.size(); i++) {

        float startX = cursor.y + (points.at(i).first * PX_PER_METER) + 500;
        float startY = cursor.x + (points.at(i).second * PX_PER_METER) + 500;

        float endX, endY;

        if (i == points.size() - 1) {
            endX = cursor.y + (points.at(0).first * PX_PER_METER) + 500;
            endY = cursor.x + (points.at(0).second * PX_PER_METER) + 500;
        } else {
            endX = cursor.y + (points.at(i + 1).first * PX_PER_METER) + 500;
            endY = cursor.x + (points.at(i + 1).second * PX_PER_METER) + 500;
        }

        drawList->AddLine(
            {startX, startY},
            {endX, endY},
            ImColor(0, 255, 0),
            9.0
        );
    }
}

inline void drawRobotWindow(Pose p) {
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(1016, 1035));

    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize;

    ImGui::Begin("Robot Position", nullptr, flags);

    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    const ImVec2 cursor = ImGui::GetCursorScreenPos();

    // Area bounds
    draw_list->AddRect(
        {cursor.x - 2, cursor.y - 2},
        {cursor.x + 1002, cursor.y + 1002},
        ImColor(255, 75, 75),
        1,
        0,
        2
    );

    drawCourse();

    // Draw robot square
    // Clamp pose to bounds
    p.x = clamp(p.x, 0.0 + ROBOT_WIDTH / 2, 3.0 - ROBOT_WIDTH / 2);
    p.y = clamp(p.y, 0.0 + ROBOT_HEIGHT / 2, 3.0 - ROBOT_HEIGHT / 2);

    ImVec2 points[4];
    const double cos_a = cosf(p.theta);
    const double sin_a = sinf(p.theta);

    // Center relative corner locations, before rotation
    ImVec2 v[4] = {
        {-1.0 * ROBOT_WIDTH / 2.0, -1.0 * ROBOT_HEIGHT / 2.0},
        {ROBOT_WIDTH / 2.0, -1.0 * ROBOT_HEIGHT / 2.0},
        {ROBOT_WIDTH / 2.0, ROBOT_HEIGHT / 2.0},
        {-1.0 * ROBOT_WIDTH / 2.0, ROBOT_HEIGHT / 2.0}
    };

    // Rotate and translate each corner
    for (int i = 0; i < 4; i++) {
        points[i].x = cursor.x + (p.x + (v[i].x * cos_a - v[i].y * sin_a)) * PX_PER_METER;
        points[i].y = cursor.y + (3.0 - p.y - (v[i].x * sin_a + v[i].y * cos_a)) * PX_PER_METER;
    }

    draw_list->AddConvexPolyFilled(points, 4, ImColor(255, 255, 255));

    ImGui::End();
}

#endif //INC_441SIM_DRAW_H