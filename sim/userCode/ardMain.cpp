#include "mosscap/challenge2.h"

#include <Encoder.h>
#include <Motor.h>

Motor left(11, 10, 12);
Motor right(21, 20, 22);

Encoder leftEncoder(15, 16);
Encoder rightEncoder(25, 26);

TelemetryPosition pose;

const double metersPerCount = 1.673e-4;
const double trackWidth = 0.20;

void updatePosition() {
    double leftDistance = leftEncoder.readAndReset() * metersPerCount;
    double rightDistance = rightEncoder.readAndReset() * metersPerCount;

    double d = (leftDistance + rightDistance) / 2.0;
    double dTheta = (rightDistance - leftDistance) / trackWidth;

    // Convert from degrees to radians
    double thetaOld = pose.getTheta() * PI / 180.0;

    double xNew = pose.getX() + d * cos(thetaOld + dTheta / 2.0);
    double yNew = pose.getY() + d * sin(thetaOld + dTheta / 2.0);
    double thetaNew = thetaOld + dTheta;

    pose.setX(xNew);
    pose.setY(yNew);

    // Convert back to degrees
    pose.setTheta(thetaNew * 180.00 / PI);
}

void setup() {
    // Buttons to move the robot around (forward, left, and right)
    pinMode(1, INPUT);
    pinMode(2, INPUT);
    pinMode(3, INPUT);

    // Set initial position
    pose.setX(0.5);
    pose.setY(1.0);
}

void loop() {
    // Call our function to update our position
    updatePosition();

    // Movement code
    if (digitalRead(1)) {
        left.run(255);
        right.run(255);
    } else if (digitalRead(2)) {
        left.run(-255);
        right.run(255);
    } else if (digitalRead(3)) {
        left.run(255);
        right.run(-255);
    } else {
        left.run(0);
        right.run(0);
    }
}