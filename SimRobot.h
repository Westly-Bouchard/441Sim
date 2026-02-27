//
// Created by west on 2/26/26.
//

#ifndef INC_441SIM_SIMROBOT_H
#define INC_441SIM_SIMROBOT_H

#include <boost/numeric/odeint.hpp>

#include "MecanumSystem.h"
#include "util.h"

using stepper_t = boost::numeric::odeint::runge_kutta4<state_t>;

inline double chassisWidth = 155.0 / 1000.0; // Meters
inline double chassisDepth = 200.0 / 1000.0; // Meters

inline double chassisMass = 1.0; // Kg TODO: This is an estimate, change later

inline double sX = 60.0 / 1000.0; // Meters TODO: This is an estimate, change later
inline double sY = 104.5 / 1000.0; // Meters

inline double wheelMass = 100.0 / 1000.0; // Kg TODO: This is an estimate, change later
inline double wheelRadius = 40.0 / 1000.0; // Meters

class SimRobot {
    MecanumSystem sys;

    double dt;
    double simTime;

    stepper_t stepper;

    state_t state;

public:
    explicit SimRobot(Pose initialPose, double dt);

    void update(double& acc);

    state_t& getState();

    Pose getPose() const;

    void setInputs(double FL, double FR, double BL, double BR);
};


#endif //INC_441SIM_SIMROBOT_H