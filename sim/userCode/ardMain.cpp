#include "mosscap/challenge4.h"

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X tof;

const double mapSize = 3.0;
const double mapResolution = 0.1;

TelemetryMap map(mapSize, mapResolution);

void updateMap() {
    double currentAngle = pose.getTheta();

    double dist = tof.readRangeSingleMillimeters() / 1000.0;

    double thetaRadians = currentAngle * PI / 180.0;

    double globalX = pose.getX() + dist * cos(thetaRadians);
    double globalY = pose.getY() + dist * sin(thetaRadians);

    int mapX = floor(globalX / mapResolution);
    int mapY = floor(globalY / mapResolution);

    for (int rOffset = -1; rOffset <= 1; rOffset++) {
        for (int cOffset = -1; cOffset <= 1; cOffset++) {
            map.set(mapX + cOffset, mapY + rOffset);
        }
    }

    map.set(mapX, mapY);
}

void setup() {
    // Buttons to move the robot around (forward, left, and right)
    pinMode(1, INPUT);

    // Set initial position
    pose.setX(1.5);
    pose.setY(1.5);

    // Initialize time of flight sensor
    // These lines don't actually do anything, they're just here for realism
    // This is how you would set up a VL530X using its Arduino library if you=
    // were working with actual hardware.
    Wire.begin();
    tof.setTimeout(500);
    if (!tof.init()) {
        while (1) {}
    }
}

void loop() {
    // Wait for user to press the button
    while (!digitalRead(1)) {}
    while (digitalRead(1)) {}

    performMappingSweep();
}