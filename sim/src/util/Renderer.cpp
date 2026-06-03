//
// Created by west on 5/6/26.
//

#include "util/Renderer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <ranges>

using namespace std;

Renderer& Renderer::getInstance() {
    static Renderer instance;
    return instance;
}

void Renderer::registerDrawable(const int layer, Drawable* drawable) {
    lock_guard lock(m);
    // Push object into tracked vector
    drawables.emplace_back(layer, drawable);

    // Sort vector so that objects can be rendered in the correct order
    ranges::sort(drawables, [](const auto& a, const auto& b) {
        return a.first < b.first;
    });
}

void Renderer::registerDrawFunction(const int layer, const function<void()>& func) {
    lock_guard lock(m);
    drawFunctions.emplace_back(layer, func);

    ranges::sort(drawFunctions, [](const auto& a, const auto& b) {
        return a.first < b.first;
    });
}

void Renderer::unregisterDrawable(Drawable* drawable) {
    lock_guard lock(m);
    erase_if(drawables, [drawable](const auto& d) { return drawable == d.second; });
}

void Renderer::setScale(const double meters) {
    scale = meters;
    pxPerMeter = 800.0 / meters;
}

void Renderer::draw() const {
    lock_guard lock(m);
    // Begin visualization window
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(800, 800));

    ImGui::Begin("Robot", nullptr,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                 ImGuiWindowFlags_NoResize);

    drawing = true;

    // Draw all drawables and drawable functions to the window
    auto it = drawFunctions.begin();

    for (const auto&[layer, d] : drawables) {
        while (it != drawFunctions.end() && it->first < layer) {
            it->second();
            ++it;
        }
        // d->lock();
        d->draw();
        // d->unlock();
    }

    while (it != drawFunctions.end()) {
        it->second();
        ++it;
    }

    drawing = false;

    // End the visualization window
    ImGui::End();
    ImGui::PopStyleVar(2);
}

void Renderer::drawRect(const double x, const double y, const double theta, const double w,
                         const double h, const ImColor color) {
    if (!drawing) return;

    std::array<ImVec2, 4> points;
    const double c = cos(theta);
    const double s = sin(theta);

    // Center relative corner locations, before rotation
    // Why does imgui use floats? So many casts...
    const std::array<ImVec2, 4> vertices = {{
        {static_cast<float>(-1.0 * w / 2.0), static_cast<float>(-1.0 * h / 2.0)},
        {static_cast<float>(w / 2.0), static_cast<float>(-1.0 * h / 2.0)},
        {static_cast<float>(w / 2.0), static_cast<float>(h / 2.0)},
        {static_cast<float>(-1.0 * w / 2.0), static_cast<float>(h / 2.0)}
    }};

    // Rotate and translate each corner
    for (auto&& [v, p] : std::views::zip(vertices, points)) {
        p.x = static_cast<float>((x + (v.x * c - v.y * s)) * pxPerMeter);
        p.y = static_cast<float>((scale - y - (v.x * s + v.y * c)) * pxPerMeter);
    }

    // Add to draw list
    ImGui::GetWindowDrawList()->AddConvexPolyFilled(points.data(), 4, color);
}