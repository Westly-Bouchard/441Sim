//
// Created by west on 2/26/26.
//

#include "SimRobot.h"
#include "../managers.h"

SimRobot::SimRobot(Pose initialPose, const double dt) : sys(
    {chassisWidth, chassisDepth},
    chassisMass,
    {sX, sY},
    wheelMass,
    wheelRadius), dt(dt), simTime(0.0) {

    stepper = stepper_t();

    state = {initialPose.x, initialPose.y, initialPose.theta, 0, 0, 0};

    motors = {
        SimMotor(kV, kT, r),
        SimMotor(kV, kT, r),
        SimMotor(kV, kT, r),
        SimMotor(kV, kT, r)
    };
}

void SimRobot::update(double& acc) {
    if (acc <= dt) return;

    // Need body frame velocities for kinematics
    const double c = cos(state.at(2));
    const double s = sin(state.at(2));

    const double bVy = state.at(4) * c - state.at(3) * s;
    const double bVx = state.at(3) * c + state.at(4) * s;

    const double omega_1 = (1 / wheelRadius) * (bVy - bVx - (sX + sY) * state.at(5));
    const double omega_2 = (1 / wheelRadius) * (bVy + bVx + (sX + sY) * state.at(5));
    const double omega_3 = (1 / wheelRadius) * (bVy + bVx - (sX + sY) * state.at(5));
    const double omega_4 = (1 / wheelRadius) * (bVy - bVx + (sX + sY) * state.at(5));
    
    motors.at(0).setSpeed(omega_1);
    motors.at(1).setSpeed(omega_2);
    motors.at(2).setSpeed(omega_3);
    motors.at(3).setSpeed(omega_4);

    const double tau_1 = motors.at(0).getTorque();
    const double tau_2 = motors.at(1).getTorque();
    const double tau_3 = motors.at(2).getTorque();
    const double tau_4 = motors.at(3).getTorque();

    sys.setTorques(tau_1, tau_2, tau_3, tau_4);

    while (acc > dt) {
        stepper.do_step(sys, state, simTime, dt);
        simTime += dt;
        acc -= dt;
    }

    // After we do the physics update, we have to update encoder counts
}

state_t& SimRobot::getState() {
    return state;
}

Pose SimRobot::getPose() const {
    return {state.at(0), state.at(1), state.at(2)};
}

void SimRobot::setInputs(const int FL, const int FR, const int BL, const int BR) {
    motors.at(0).setInput(FL);
    motors.at(1).setInput(FR);
    motors.at(2).setInput(BL);
    motors.at(3).setInput(BR);
}
