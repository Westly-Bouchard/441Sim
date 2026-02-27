//
// Created by west on 2/26/26.
//

#include "MecanumSystem.h"

#include <cmath>

#include <boost/numeric/ublas/matrix.hpp>
using namespace boost::numeric::ublas;

MecanumSystem::MecanumSystem(
    const std::pair<double, double> &chassisDimensions,
    const double chassisMass,
    const std::pair<double, double> &wheelLocations,
    const double wheelMass,
    const double wheelRadius
) : chassisWidth(chassisDimensions.first), chassisDepth(chassisDimensions.second), m_pc(chassisMass),
    sX(wheelLocations.first), sY(wheelLocations.second), m_w(wheelMass), r(wheelRadius) {
    // Current approximation is just a cylinder
    I_k = 0.5 * wheelMass * pow(wheelRadius, 2);

    // Again just approximate that the robot mass is evenly distributed throughout the chassis
    I_pc = (1.0 / 12.0) * chassisMass * (pow(chassisWidth, 2) + pow(chassisDepth, 2));

    m_e = chassisMass + 4 * I_k / pow(wheelRadius, 2);
    I_e = I_pc + 4 * I_k * (pow(sX + sY, 2), pow(wheelRadius, 2));

    tau_1 = 0;
    tau_2 = 0;
    tau_3 = 0;
    tau_4 = 0;
}

void MecanumSystem::setTorques(const double FL, const double FR, const double BL, const double BR) {
    tau_1 = FL;
    tau_2 = FR;
    tau_3 = BL;
    tau_4 = BR;
}

void MecanumSystem::operator()(const state_t &x, state_t &dxdt, const double t) const {

    // Body frame forces
    const double fXBody = (tau_1 + tau_2 + tau_3 + tau_4) / r;
    const double fYBody = -1.0 * (tau_1 - tau_2 - tau_3 + tau_4) / r;
    const double tZ = -1.0 * (sX + sY) * (tau_1 - tau_2 + tau_3 - tau_4) / r;

    // Rotate them into the world frame
    const double theta = x.at(2);
    const double fXWorld = cos(theta) * fXBody - sin(theta) * fYBody;
    const double fYWorld = sin(theta) * fXBody + cos(theta) * fYBody;
    const double tZWorld = tZ;

    dxdt.at(0) = x.at(3);
    dxdt.at(1) = x.at(4);
    dxdt.at(2) = x.at(5);

    dxdt.at(3) = (1.0 / m_e) * fXWorld;
    dxdt.at(4) = (1.0 / m_e) * fYWorld;
    dxdt.at(5) = (1.0 / I_e) * tZWorld;
}
