//
// Created by Westly Bouchard on 2/28/26.
//

#ifndef CONFIG_H
#define CONFIG_H

#include <cmath>

namespace Config {
    inline double V_BUS = 6.0;

    namespace Motor {
        inline double T_STALL = 0.32361945; // Newton meters
        // inline double stallTorque = 0.1;
        inline double I_STALL = 2.6; // Amps
        inline double FREE_SPEED = 170.0 * (M_PI * 2 / 60); // 170 RPM, convert to radians per second
        inline double I_FREE = 0.1; // Amps

        inline double K_T = T_STALL / I_STALL;
        inline double R = V_BUS / I_STALL;

        inline double K_V = FREE_SPEED / (V_BUS - I_FREE * R);
    }

    namespace Chassis {

    }
}

#endif //CONFIG_H
