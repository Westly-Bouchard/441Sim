//
// Created by west on 6/7/26.
//

#ifndef MOSSCAP_TELEMETRYPOSITION_H
#define MOSSCAP_TELEMETRYPOSITION_H

#include <util/Telemetry.h>
#include <sstream>

/**
 * A class that stores a position + orientation in the X-Y plane
 *
 * Objects of this class will also write their stored position to the simulator's
 * telemetry window automatically, allowing for easy debugging and analysis of odometry
 * implementations.
 */
class TelemetryPose : public TelemetryProvider {
public:
    /**
     * Create position object
     */
    TelemetryPose() : TelemetryProvider(1) {}

    /**
     * Get the current stored X position
     * @return X Component of Position Vector
     */
    double getX() const { return x; }

    /**
     * Get the current stored Y position
     * @return Y Component of Position Vector
     */
    double getY() const { return y; }

    /**
     * Get the current stored angle
     * @return Angle Component of Position Vector
     */
    double getTheta() const { return theta; }

    /**
     * Update the stored X position
     * @param newX Updated X component of position vector
     */
    void setX(const double newX) {
        std::lock_guard lk(linkMtx);
        x = newX;
    }

    /**
     * Update the stored Y position
     * @param newY Updated Y component of position vector
     */
    void setY(const double newY) {
        std::lock_guard lk(linkMtx);
        y = newY;
    }

    /**
     * Update the stored angle
     * @param newTheta Updated angle component of position vector
     */
    void setTheta(const double newTheta) {
        std::lock_guard lk(linkMtx);
        theta = newTheta;
    }

    /**
     * Write odometry data to simulator's telemetry window
     */
    void write() const override {
        if (Telemetry::section("Odometry")) {
            std::stringstream ss;
            ss << "X\t: " << x;
            Telemetry::text(ss.str());

            ss.str("");
            ss << "Y\t: " << y;
            Telemetry::text(ss.str());

            ss.str("");
            ss << "Theta\t: " << theta;
            Telemetry::text(ss.str());
        }
    }

private:
    /**
     * Position vector
     */
    double x{0.0}, y{0.0}, theta{0.0};
};

#endif //MOSSCAP_TELEMETRYPOSITION_H
