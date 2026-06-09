//
// Created by west on 6/9/26.
//

#ifndef MOSSCAP_TELEMETRYMAP_H
#define MOSSCAP_TELEMETRYMAP_H

#include <vector>

#include "util/Renderer.h"

/**
 * A class that stores an occupancy map.
 *
 * This class maintains a grid of cells that can either be marked as
 * empty or full.
 *
 * This class will draw itself to the simulation window, it will draw
 * marked cells as green at 35% opacity. These cells are overlaid
 * on top of the robot (they are drawn to layer 1).
 */
class TelemetryMap : public Drawable {
public:
    /**
     * Construct a TelemetryMap with a given size and resolution.
     * @param s Map size in meters. Maps are square
     * @param r Resolution of map, meters per cell. Must be less than size
     */
    TelemetryMap(const double s, const double r) : Drawable(1), size(s), resolution(r), numElements(size /resolution) {
        for (int i = 0; i < numElements; i++) {
            std::vector<bool> temp;
            for (int j = 0; j < numElements; j++) {
                temp.push_back(false);
            }

            map.push_back(temp);
        }
    }

    void draw() const override {
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

    /**
     * Get the state of the map at the given location.
     * This function expects map indices *not* positions
     *
     * If the provided location is outside the map bounds this function
     * will return `true`.
     *
     * @param x X position (column) to access
     * @param y Y position (row) to access
     * @return Whether the map is marked at the given location
     */
    bool get(const int x, const int y) const {
        if (x < numElements && y < numElements && x >= 0 && y >= 0) {
            return map.at(y).at(x);
        }

        return true;
    }

    /**
     * Mark the map at the given location. This sets the current map cell to `true`.
     *
     * Only marked cells will be drawn to the simulation window.
     *
     * @param x X position (column) to access
     * @param y Y position (row) to access
     */
    void set(const int x, const int y) {
        if (x < numElements && y < numElements && x >= 0 && y >= 0) {
            std::lock_guard lk(drawMtx);
            map.at(y).at(x) = true;
        }
    }

    /**
     * Get map size
     * @return Map size in meters
     */
    double getSize() const { return size; }

    /**
     * Get map resolution
     * @return Map resolution in meters per cell
     */
    double getResolution() const { return resolution; }

    /**
     * Get the number of cells per row / column of the map
     * @return Number of cells per row / column of the map
     */
    double getNumElements() const { return numElements; }

private:
    const double size; // Meters
    const double resolution; // Meters per cell

    double numElements{0};

    std::vector<std::vector<bool>> map;
};
#endif //MOSSCAP_TELEMETRYMAP_H
