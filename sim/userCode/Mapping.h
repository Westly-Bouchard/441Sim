#ifndef MAPPING_H
#define MAPPING_H

#include <VL53L0X.h>
#include <Wire.h>

#include "TelemetryMap.h"

#include "Drivetrain.h"

void initTof(VL53L0X& tof) {
    Wire.begin();
    tof.setTimeout(500);

    if (!tof.init()) {
        // Uh Oh!
        while(1) {}
    }
}

void updateMap(TelemetryPose& pose, TelemetryMap& map, VL53L0X& tof, Motor& mL, Motor& mR, Encoder& eL, Encoder& eR) {
    mL.run(-100);
    mR.run(100);

    double startAngle = pose.getTheta();
    double currentAngle = startAngle;

    while (currentAngle - startAngle < 360.0) {
        updateOdometry(pose, eL, eR);
        currentAngle = pose.getTheta();

        double dist = tof.readRangeSingleMillimeters() / 1000.0;

        const double th = currentAngle * PI / 180.0;

        double globalX = pose.getX() + dist * cos(th);
        double globalY = pose.getY() + dist * sin(th);

        int mapX = std::floor(globalX / map.getResolution());
        int mapY = std::floor(globalY / map.getResolution());

        // Inflate the obstacle by one cell in every direction
        for (int rOffset = -1; rOffset <= 1; rOffset++) {
            for (int cOffset = -1; cOffset <= 1; cOffset++) {
                map.set(mapX + cOffset, mapY + rOffset);
            }
        }
        map.set(mapX, mapY);
    }

    mL.run(0);
    mR.run(0);

    // Wait for robot to stop moving (keep updating odometry)
    const int start = millis();
    int current = start;

    while (current - start < 500) {
        updateOdometry(pose, eL, eR);
        current = millis();
    }
}

#endif