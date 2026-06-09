//
// Created by west on 3/22/26.
//

#ifndef INC_441SIM_MECANUMCONFIG_H
#define INC_441SIM_MECANUMCONFIG_H

#include <cmath>

/**
 * Configuration of a robot with mecanum drive base
 */
struct WMRConfig {
    const double mass;
    const double trackWidth;
    const double wheelBase;
    const double wheelRadius;
    const double inertia;

    double kineticFriction = 0.0;

    // Optionally, add noise to the system
    // This would generally be applied to the motor torques to model imperfect motors
    // and an imperfect driving surface.
    double noiseMagnitude = 0.0;

    // These allow unnecessary telemetry data to be turned off when a user is first experiencing the simulator
    bool showMotorTelemetry = true;
    bool showEncoderTelemetry = true;
    bool showTOFTelemetry = true;

    /**
     * Construct a WMR config object with provided physical constants
     * @param m Total mass of the robot in kilograms
     * @param tW Track width of the robot in meters
     * @param wB Wheelbase of the robot in meters
     * @param wR Wheel radius of the robot in meters
     */
    WMRConfig(const double m, const double tW, const double wB, const double wR) : mass(m), trackWidth(tW),
        wheelBase(wB), wheelRadius(wR), inertia(getInertia()) {}

private:
    /**
     * Calculate the approximate moment of inertia of the robot from provided constants
     * @return MOI about the z axis
     */
    [[nodiscard]] double getInertia() const {
        return 1.0 / 12.0 * mass * (pow(trackWidth, 2) + pow(wheelBase, 2));
    }
};

#endif //INC_441SIM_MECANUMCONFIG_H