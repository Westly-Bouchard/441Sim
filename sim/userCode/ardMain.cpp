#include "mosscap/challenge3.h"

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

    // Clamp to 0->360 range. Not strictly necessary but helpful for later math
    if (thetaNew > 2 * PI) thetaNew -= 2 * PI;
    if (thetaNew < -2 * PI) thetaNew += 2 * PI;

    pose.setX(xNew);
    pose.setY(yNew);

    // Convert back to degrees
    pose.setTheta(thetaNew * 180.00 / PI);
}

const double FORWARD_KP = 510;
const int minPWM = 70;

void driveDistance(double dist) {
    const double startX = pose.getX();
    const double startY = pose.getY();

    while (abs(sqrt(pow(pose.getX() - startX, 2) + pow(pose.getY() - startY, 2)) - dist) >= 0.01) {
        // Update estimated position
        updatePosition();

        // Compute error
        double error = dist - sqrt(pow(pose.getX() - startX, 2) + pow(pose.getY() - startY, 2));

        // Compute control signal (from the equation for a P-Controller)
        int command = error * FORWARD_KP;

        if (command < 0) command -= minPWM;
        if (command > 0) command += minPWM;

        // Ensure that our command is within our PWM range
        if (command > 255) command = 255;
        if (command < -255) command = -255;

        // Send that control signal to the motors
        left.run(command);
        right.run(command);
    }

    left.run(0);
    right.run(0);
}

const double TURN_KP = 2.8;

void turnAngle(double angle) {
    double startAngle = pose.getTheta();

    while (abs(startAngle + angle - pose.getTheta()) >= 2) {
        updatePosition();

        double error = abs(startAngle + angle - pose.getTheta());

        int command = error * TURN_KP;

        if (command < 0) command -= minPWM;
        if (command > 0) command += minPWM;

        // Ensure that our command is within our PWM range
        if (command > 255) command = 255;
        if (command < -255) command = -255;

        if (angle < 0) {
            left.run(command);
            right.run(-command);
        } else {
            left.run(-command);
            right.run(command);
        }
    }

    left.run(0);
    right.run(0);
}

void setup() {
    // Buttons to move the robot around (forward, left, and right)
    pinMode(1, INPUT);

    // Set initial position
    pose.setX(0.6);
    pose.setY(0.75);
}

void loop() {

    // Wait for user to press the button
    while (!digitalRead(1)) {}
    while (digitalRead(1)) {}

    // For each point in the list: turn towards it and drive there
    for (int i = 0; i < pathPoints.size(); i++) {
        // Determine current and target positions
        double currentX = pose.getX();
        double currentY = pose.getY();

        double targetX, targetY;
        if (i == pathPoints.size() - 1) {
            targetX = pathPoints[0][0];
            targetY = pathPoints[0][1];
        } else {
            targetX = pathPoints[i + 1][0];
            targetY = pathPoints[i + 1][1];
        }

        // Calculate number of degrees to turn
        // Convert to degrees
        double targetAngle = atan2(targetY - currentY, targetX - currentX) / PI * 180.0;

        // Always measure target angle positively from x+
        if (targetAngle < 0) targetAngle += 360.0;

        double angleDelta = targetAngle - pose.getTheta();

        // Ensure that we don't turn more than we need to (turn 90 to the left instead of 270 to the right)
        if (abs(angleDelta) > 180.0) {
            while (abs(angleDelta) > 180.0) {
                if (angleDelta > 0) {
                    angleDelta -= 180.0;
                } else {
                    angleDelta += 180.0;
                }
            }

            angleDelta *= -1;
        }

        // Turn the robot
        turnAngle(angleDelta);

        // Calculate distance to drive
        double dist = sqrt(pow(targetX - currentX, 2) + pow(targetY - currentY, 2));

        // Move the robot
        driveDistance(dist);
    }
}