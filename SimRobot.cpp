//
// Created by west on 2/26/26.
//

#include "SimRobot.h"

SimRobot::SimRobot(Pose initialPose, const double dt) : sys(
    {chassisWidth, chassisDepth},
    chassisMass,
    {sX, sY},
    wheelMass,
    wheelRadius), dt(dt), simTime(0.0) {

    stepper = stepper_t();

    state = {initialPose.x, initialPose.y, initialPose.theta, 0, 0, 0};
}

void SimRobot::update(double& acc) {
    while (acc > dt) {
        stepper.do_step(sys, state, simTime, dt);
        simTime += dt;
        acc -= dt;
    }
}

state_t& SimRobot::getState() {
    return state;
}

Pose SimRobot::getPose() const {
    return {state.at(1), state.at(0), state.at(2)};
}

void SimRobot::setInputs(double FL, double FR, double BL, double BR) {
    sys.setTorques(FL, FR, BL, BR);
}
