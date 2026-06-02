//
// Created by Westly Bouchard on 4/8/26.
//

#include <cmath>
#include <utility>

#include "hardware/SimTOF.h"

SimTOF::SimTOF(const TOFConfig& c) : config(c) {}

double SimTOF::readDist() const {
    return dist.load();
}

double SimTOF::getDist() const {
    return dist.load();
}

void SimTOF::update(const double x, const double y, const double theta) {
    // Find sensor position and orientation in global coordinates
    const double wTheta = theta + config.theta;
    const double wX = config.x * cos(theta) - config.y * sin(theta) + x;
    const double wY = config.x * sin(theta) + config.y * cos(theta) + y;

    double tempDist = std::numeric_limits<double>::max();

    // why did I do it this way
    const Vec origin{wX, wY}, direction{cos(wTheta), sin(wTheta)};

    // Start with the bounding box check
    // Would be a big problem if this condition weren't true
    if (config.detectBoundingBox) {
        if (const auto res = config.boundingBox.intersects(origin, direction)) {
            tempDist = *res;
        }
    }

    // Now check the list of obstacles to see if there's anything closer
    for (const auto& obj : config.obstacles) {
        if (const auto res = obj->intersects(origin, direction)) {
            tempDist = std::min(tempDist, *res);
        }
    }

    dist.store(tempDist);
}

void SimTOF::draw() const {
    for (const auto& obj : config.obstacles) {
        obj->draw();
    }
}
