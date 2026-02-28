#include <iostream>
#include <optional>
using namespace std;

#include <imgui_impl_glfw.h>
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h>

#include <boost/numeric/odeint.hpp>

#include "draw.h"
#include "graphics.h"
#include "util.h"

#include "sim/SimRobot.h"

#include "user/arduino/arduinoMain.h"

int main() {
    const auto res = setUpRenderer("441Sim");

    GLFWwindow* window;
    if (res.has_value()) {
        window = res.value();
    } else {
        cerr << "Failed to set up renderer" << endl;
        return 1;
    }

    auto robot = SimRobot({2.5, 1.2, M_PI / 2.0}, 0.002);

    // Run arduino setup routine
    setup();

    double simAccumulator= 0.0;

    auto lastTime = std::chrono::high_resolution_clock::now();

    while (!glfwWindowShouldClose(window)) {
        /* PHYSICS SIMULATION */

        auto now = std::chrono::high_resolution_clock::now();
        const double frame_dt = std::chrono::duration<double>(now - lastTime).count();
        lastTime = now;

        simAccumulator += frame_dt;

        if (ImGui::IsKeyDown(ImGuiKey_UpArrow)) {
            robot.setInputs(255, 255, 255, 255);
        } else if (ImGui::IsKeyDown(ImGuiKey_DownArrow)) {
            robot.setInputs(-255, -255, -255, -255);
        } else if (ImGui::IsKeyDown(ImGuiKey_LeftArrow)) {
            robot.setInputs(-255, 255, -255, 255);
        } else if (ImGui::IsKeyDown(ImGuiKey_RightArrow)) {
            robot.setInputs(255, -255, 255, -255);
        } else {
            robot.setInputs(0, 0, 0, 0);
        }

        loop();

        robot.update(simAccumulator);

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