#include <mosscap/challenge1.h>

#include <Motor.h>

Motor left(11, 10, 12);
Motor right(21, 20, 22);

double driveSpeed = 0.5; // Meters per second
double turnSpeed = 275; // Degrees per second

void driveDistance(double dist) {
    // Convert from seconds to milliseconds as `delay()` takes a number of milliseconds
    int time = int(dist / driveSpeed * 1000);
    left.run(255);
    right.run(255);

    delay(time);

    left.run(0);
    right.run(0);
}

void turnAngle(double angle) {
    int time = abs(angle / turnSpeed * 1000);

    if (angle > 0) {
        left.run(-255);
        right.run(255);
    } else {
        left.run(255);
        right.run(-255);
    }

    delay(time);

    left.run(0);
    right.run(0);
}

void setup() {
    // Set up button
    pinMode(1, INPUT);
}

// To keep track of which direction we're currently facing
double angle = 0;

void loop() {

    // Wait for user to press the button
    while (!digitalRead(1)) {}
    while (digitalRead(1)) {}

    // For each point in the list: turn towards it and drive there
    for (int i = 0; i < pathPoints.size(); i++) {
        // Determine current and target positions
        double currentX = pathPoints[i][0];
        double currentY = pathPoints[i][1];

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

        double angleDelta = targetAngle - angle;

        // Ensure that we don't turn more than we need to (turn 90 to the left instead of 270 to the right)
        if (abs(angleDelta) > 180.0) {
            if (angleDelta > 0) {
                angleDelta -= 180.0;
            } else {
                angleDelta += 180.0;
            }

            angleDelta *= -1;
        }

        std::cout << "Current Angle: " << angle << std::endl;
        std::cout << "Target Angle: " << targetAngle << std::endl;
        std::cout << "Angle Delta: " << angleDelta << std::endl;
        std::cout << "Current Position: (" << currentX << ", " << currentY << ")" << std::endl;
        std::cout << "Target Position: (" << targetX << ", " << targetY << ")" << std::endl;

        // Turn the robot
        turnAngle(angleDelta);

        // Update our current angle for the next iteration of the loop
        angle = targetAngle;

        // Calculate distance to drive
        double dist = sqrt(pow(targetX - currentX, 2) + pow(targetY - currentY, 2));

        // Move the robot
        driveDistance(dist);
    }
}