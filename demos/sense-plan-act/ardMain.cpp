#include "Sim.h"

#include "AStarPlanner.h"
#include "Drivetrain.h"
#include "Mapping.h"

// Button Pins
const int SENSE_BUTTON = 1;
const int PLAN_BUTTON = 2;
const int ACT_BUTTON = 3;
const int CONTINUE_BUTTON = 4;

// Left Motor Pins
const int L_M_ENA = 11;
const int L_M_IN_1 = 10;
const int L_M_IN_2 = 12;

// Right Motor Pins
const int R_M_ENA = 21;
const int R_M_IN_1 = 20;
const int R_M_IN_2 = 22;

// Left Encoder Pins
const int L_E_A = 15;
const int L_E_B = 16;

// Right Encoder Pins
const int R_E_A = 25;
const int R_E_B = 26;

// Mapping constants
const double MAP_SIZE = 3.0;
const double MAP_RESOLUTION = 0.1;

// Start and end position of the robot
const double START_X = 0.5;
const double START_Y = 0.5;

const double GOAL_X = 2.5;
const double GOAL_Y = 2.5;

const double MAX_MOVE_DIST = 0.5;

double getPhysicalDist(const Location a, const Location b) {
    const double aX = a.x * MAP_RESOLUTION + 0.5 * MAP_RESOLUTION;
    const double aY = a.y * MAP_RESOLUTION + 0.5 * MAP_RESOLUTION;

    const double bX = b.x * MAP_RESOLUTION + 0.5 * MAP_RESOLUTION;
    const double bY = b.y * MAP_RESOLUTION + 0.5 * MAP_RESOLUTION;

    return sqrt(pow(aX - bX, 2) + pow(aY - bY, 2));
}

// Physical robot hardware
Motor mL(L_M_ENA, L_M_IN_1, L_M_IN_2);
Motor mR(R_M_ENA, R_M_IN_1, R_M_IN_2);

Encoder eL(L_E_A, L_E_B);
Encoder eR(R_E_A, R_E_B);

VL53L0X tof;

// Current position of the robot from odometry calculations
TelemetryPose pose;

// Current global obstacle map
TelemetryMap map(MAP_SIZE, MAP_RESOLUTION);

// Planner
AStarPlanner planner(map);

std::vector<Location> plan() {
    // Determine map locations of current location and goal
    Location current{
        static_cast<int>(std::floor(pose.getX() / MAP_RESOLUTION)),
        static_cast<int>(std::floor(pose.getY() / MAP_RESOLUTION))
    };

    Location goal{
        static_cast<int>(std::floor(GOAL_X / MAP_RESOLUTION)),
        static_cast<int>(std::floor(GOAL_Y / MAP_RESOLUTION))
    };

    planner.reset(current, goal);
    auto path = planner.solve();

    return path;
}

void act(const std::vector<Location>& path) {
    // Traverse the path until a direction change is detected or until MAX_MOVE_DIST is reached
    if (path.size() == 0) {
        return;
    }

    Location dir;
    double dist = 0.0;

    if (path.size() == 1) {
        const int currentX = std::floor(pose.getX() / MAP_RESOLUTION);
        const int currentY = std::floor(pose.getY() / MAP_RESOLUTION);

        dir.x = path.at(0).x - currentX;
        dir.y = path.at(0).y - currentY;

        dist = getPhysicalDist({currentX, currentY}, path.at(0));
    } else {
        dir = Location{path.at(1).x - path.at(0).x, path.at(1).y - path.at(0).y};
        int stopIdx = -1;

        for (int i = 0; i < path.size(); i++) {
            if (i == 0) continue;

            const int dirX = path.at(i).x - path.at(i - 1).x;
            const int dirY = path.at(i).y - path.at(i - 1).y;

            const double tempDist = getPhysicalDist(path.at(i), path.at(0));

            if (
                dirX != dir.x || dirY != dir.y ||
                tempDist > MAX_MOVE_DIST
            ) {
                stopIdx = i - 1;
                break;
            }

            if (i == path.size() - 1) {
                stopIdx = i;
                break;
            }
        }

        dist = getPhysicalDist(path.at(stopIdx), path.at(0));
    }

    const double targetAngle = atan2(dir.y, dir.x) / PI * 180.0;
    double angleDelta = targetAngle - fmod(pose.getTheta(), 360.0);

    if (abs(angleDelta) > 180.0) {
        if (angleDelta > 0) {
            angleDelta -= 180.0;
        } else {
            angleDelta += 180.0;
        }

        angleDelta *= -1;
    }

    delay(100);

    turnAngle(angleDelta, mL, mR, pose, eL, eR);

    delay(100);

    driveDistance(dist, mL, mR, pose, eL, eR);

    delay(100);
}

void setup() {
    // Ensure buttons are inputs
    pinMode(SENSE_BUTTON, INPUT);
    pinMode(PLAN_BUTTON, INPUT);
    pinMode(ACT_BUTTON, INPUT);
    pinMode(CONTINUE_BUTTON, INPUT);

    pose.setX(START_X);
    pose.setY(START_Y);

    // First run through of the routine is directed by the user
    while (!digitalRead(SENSE_BUTTON)) {}
    while (digitalRead(SENSE_BUTTON)) {}

    updateMap(pose, map, tof, mL, mR, eL, eR);

    while (!digitalRead(PLAN_BUTTON)) {}
    while (digitalRead(PLAN_BUTTON)) {}

    const auto p = plan();

    while (!digitalRead(ACT_BUTTON)) {}
    while (digitalRead(ACT_BUTTON)) {}

    act(p);

    while (!digitalRead(CONTINUE_BUTTON)) {}
    while (digitalRead(CONTINUE_BUTTON)) {}

    // After that, run until the robot is at the target position
    while (abs(pose.getX() - GOAL_X) > 0.2 || abs(pose.getY() - GOAL_Y) > 0.2) {
        // Sense
        updateMap(pose, map, tof, mL, mR, eL, eR);

        // Plan
        const auto path = plan();

        // Act
        act(path);
    }
}

void loop() {}