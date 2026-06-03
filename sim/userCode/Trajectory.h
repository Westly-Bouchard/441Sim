#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Arduino.h>

class Trajectory {
public:
    double xF, tA, actualVMax, aMax, T;

    Trajectory(double xF, double vMax, double aMax) : xF(xF), aMax(aMax) {
        double minDistForVMax = (vMax * vMax) / aMax;

        if (xF < minDistForVMax) {
            // Triangular profile
            actualVMax = sqrt(xF * aMax);
            tA = actualVMax / aMax;

            T = 2.0 * tA;
        } else {
            // Trapezoidal profile
            actualVMax = vMax;
            tA = actualVMax / aMax;

            T = (xF / actualVMax) + (actualVMax / aMax);
        }
    }

    double samplePosition(double t) {
        if (t <= 0) return 0.0;

        if (t < tA) {
            return 0.5 * aMax * pow(t, 2);
        } else if (t < T - tA) {
            return 0.5 * aMax * pow(tA, 2) + actualVMax * (t - tA);
        } else if (t < T) {
            return xF - 0.5 * aMax * pow(T - t, 2);
        } else {
            return xF;
        }
    }

    double sampleVelocity(double t) {
        if (t <= 0) return 0.0;

        if (t < tA) {
            return aMax * t;
        } else if (t < T - tA) {
            return actualVMax;
        } else if (t < T) {
            return aMax * (T - t);
        } else {
            return 0.0;
        }
    }
};

#endif