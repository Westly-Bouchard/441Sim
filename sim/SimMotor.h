//
// Created by west on 2/26/26.
//

#ifndef INC_441SIM_SIMMOTOR_H
#define INC_441SIM_SIMMOTOR_H
#include <cmath>

inline double vBus = 6.0; // Volts

inline double stallTorque = 0.32361945; // Newton meters
// inline double stallTorque = 0.1;
inline double stallCurrent = 2.6; // Amps
inline double freeSpeed = 160.0 * (M_PI * 2 / 60); // 170 RPM, convert to radians per second
inline double freeCurrent = 0.1; // Amps

inline double kT = stallTorque / stallCurrent;
inline double r = vBus / stallCurrent;

inline double kV = freeSpeed / (vBus - freeCurrent * r);

class SimMotor {
    double kV, kT, r;

    int setpoint;

    // Current state
    double omega;

public:
    SimMotor(double kV, double kT, double r);

    void setInput(int pwm);
    void setSpeed(double speed);

    [[nodiscard]] double getTorque() const;
};


#endif //INC_441SIM_SIMMOTOR_H