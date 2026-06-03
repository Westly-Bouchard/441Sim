#ifndef TELEMETRY_MAP_H
#define TELEMETRY_MAP_H

#include <vector>

#include <util/Renderer.h>

class TelemetryMap : public Drawable {
public:
    TelemetryMap(double s, double r) : Drawable(1), size(s), resolution(r) {
        numElements = size / resolution;

        for (int i = 0; i < numElements; i++) {
            std::vector<bool> temp;
            for (int j = 0; j < numElements; j++) {
                temp.push_back(false);
            }

            map.push_back(temp);
        }
    }

    void draw() override {
        std::lock_guard lk(drawMtx);
        for (int y = 0; y < numElements; y++) {
            for (int x = 0; x < numElements; x++) {
                if (map.at(y).at(x)) {
                    const double rectX = x * resolution + (resolution / 2.0);
                    const double rectY = y * resolution + (resolution / 2.0);
                    Renderer::drawRect(rectX, rectY, 0.0, resolution, resolution, {0.0f, 1.0f, 0.0f, 0.35f});
                }
            }
        }
    }

    bool get(int x, int y) const {
        if (x < numElements && y < numElements && x >= 0 && y >= 0) {
            return map.at(y).at(x);
        }

        return true;
    }

    void set(int x, int y) {
        drawMtx.lock();
        if (x < numElements && y < numElements && x >= 0 && y >= 0) {
            map.at(y).at(x) = true;
        }
        drawMtx.unlock();
    }

    double getSize() const { return size; }

    double getResolution() const { return resolution; }

    double getNumElements() const { return numElements; }

private:
    const double size; // Meters
    const double resolution; // Meters per cell

    double numElements{0};

    std::vector<std::vector<bool>> map;
};

#endif