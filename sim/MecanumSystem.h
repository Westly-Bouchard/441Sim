//
// Created by west on 2/26/26.
//

#ifndef INC_441SIM_SIM_H
#define INC_441SIM_SIM_H

#include <vector>

using state_t = std::vector<double>;

class MecanumSystem {
    double chassisWidth, chassisDepth;

    double m_pc;

    double sX, sY;

    double m_w;
    double r;

    double m_e;
    double I_e;
    double I_k;
    double I_pc;

    double tau_1, tau_2, tau_3, tau_4;

public:
    MecanumSystem(
        const std::pair<double, double> &chassisDimensions,
        double chassisMass,
        const std::pair<double, double> &wheelLocations,
        double wheelMass,
        double wheelRadius
    );

    void setTorques(double FL, double FR, double BL, double BR);

    void operator() (const state_t &x, state_t &dxdt, double t) const;
};

#endif //INC_441SIM_SIM_H