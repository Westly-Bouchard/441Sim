#include <iostream>
#include <optional>
#include <vector>
#include <cmath>
using namespace std;

#include "SimRobot.h"

#include <imgui_impl_glfw.h>
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h>

#include <boost/numeric/odeint.hpp>

#include "draw.h"
#include "graphics.h"
#include "MecanumSystem.h"
#include "util.h"


int main() {
    const auto res = setUpRenderer("441Sim");

    GLFWwindow* window;
    if (res.has_value()) {
        window = res.value();
    } else {
        cerr << "Failed to set up renderer" << endl;
        return 1;
    }

    auto robot = SimRobot({1, 1, 0}, 0.002);

    double accumulator = 0.0;

    auto lastTime = std::chrono::high_resolution_clock::now();

    while (!glfwWindowShouldClose(window)) {
        /* PHYSICS SIMULATION */

        auto now = std::chrono::high_resolution_clock::now();
        const double frame_dt = std::chrono::duration<double>(now - lastTime).count();
        lastTime = now;

        accumulator += frame_dt;

        if (ImGui::IsKeyPressed(ImGuiKey_UpArrow)) {
            robot.setInputs(-0.05, -0.05, -0.05, -0.05);
        } else if (ImGui::IsKeyPressed(ImGuiKey_DownArrow)) {
            robot.setInputs(0.05, 0.05, 0.05, 0.05);
        } else if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow)) {
            robot.setInputs(0.05, -0.05, -0.05, 0.05);
        } else if (ImGui::IsKeyPressed(ImGuiKey_RightArrow)) {
            robot.setInputs(-0.05, 0.05, 0.05, -0.05);
        } else {
            robot.setInputs(0.0, 0.0, 0.0, 0.0);
        }

        robot.update(accumulator);


        /* RENDERING */
        glfwPollEvents();
        if (glfwGetWindowAttrib(window, GLFW_ICONIFIED) != 0) {
            ImGui_ImplGlfw_Sleep(10);
            continue;
        }

        newFrame();

        drawDebugWindow();

        drawRobotWindow(robot.getPose());

        render(window);
    }

    cleanup(window);
    return 0;
}