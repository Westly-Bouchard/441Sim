//
// Created by west on 3/22/26.
//

#include "sim/MecanumSim.h"

#include <cmath>
#include <ranges>
#include <sstream>

using namespace std;

MecanumSim::MecanumSim(const WMRConfig &config)
                        :  config(config) {}

void MecanumSim::setPose(const double x, const double y, const double theta) {
    state.at(0) = x;
    state.at(1) = y;
    state.at(2) = theta * (M_PI / 180.0);
}

void MecanumSim::periodic() {
    // Update distance traveled by each wheel
    for (const auto vels = fwdKinematics(); auto&& [enc, vel] : views::zip((encoders), vels)) {
        enc->updatePosition(vel * dt);
    }

    // Update tof position
    tof->update(state.at(0), state.at(1), state.at(2));
}

void MecanumSim::operator() (const state_t &x, state_t &dxdt, double t) {
    input_t torques;
    // Wow I love modern C++ so much, this is so intelligible
    for (const auto vels = fwdKinematics(); auto &&[motor, vel, torque] : views::zip(
             motors, vels, torques)) {
        torque = motor->getTorque(vel);
    }

    // Calculate forces in the body frame
    const double bFx = (torques.at(0) + torques.at(1) + torques.at(2) +
                        torques.at(3)) / config.wheelRadius;

    const double bFy = (-torques.at(0) + torques.at(1) + torques.at(2) -
                        torques.at(3)) / config.wheelRadius;

    const double bTz = -(config.wheelBase / 2 + config.trackWidth / 2) * (torques.at(0) -
                        torques.at(1) + torques.at(2) - torques.at(3)) /
                        config.wheelRadius;

    // Transform forces into the world frame
    const double s = sin(x.at(2));
    const double c = cos(x.at(2));

    const double wFy = bFy * c + bFx * s;
    const double wFx = bFx * c - bFy * s;
    const double wTz = bTz;

    // Update the derivative vector
    dxdt.at(0) = x.at(3);
    dxdt.at(1) = x.at(4);
    dxdt.at(2) = x.at(5);

    dxdt.at(3) = (1.0 / config.mass) * wFx;
    dxdt.at(4) = (1.0 / config.mass) * wFy;
    dxdt.at(5) = (1.0 / config.inertia) * wTz;
}

std::array<double, 4> MecanumSim::fwdKinematics() const {
    // Body frame velocities for forward kinematics
    const double c = cos(state.at(2));
    const double s = sin(state.at(2));

    const double bVx = state.at(4) * c - state.at(3) * s;
    const double bVy = state.at(3) * c + state.at(4) * s;

    const double oOverR = 1 / config.wheelRadius;
    const double mid = config.wheelBase / 2 + config.trackWidth / 2;
    const double w = state.at(5);

    return {
        oOverR * (bVy - bVx - mid * w),
        oOverR * (bVy + bVx + mid * w),
        oOverR * (bVy + bVx - mid * w),
        oOverR * (bVy - bVx + mid * w)
    };
}

void MecanumSim::registerMotor(const unsigned int idx, MotorConfig c, const int pin) {
    motors.at(idx) = std::make_unique<SimMotor>(c);
    ArduinoRuntime::getInstance().bindPWM(pin, *motors.at(idx));
}

void MecanumSim::registerEncoder(const unsigned int idx, const int cpr, const int pin) {
    encoders.at(idx) = std::make_unique<SimEncoder>(cpr);
    ArduinoRuntime::getInstance().bindEncoder(pin, *encoders.at(idx));
}

void MecanumSim::registerTOF(TOFConfig c) {
    tof = std::make_unique<SimTOF>(c);
    ArduinoRuntime::getInstance().bindTOF(*tof);
}

void MecanumSim::draw() {
    // Draw robot
    Renderer::drawRect(state.at(0), state.at(1), state.at(2), 0.200, 0.245, {255, 255, 255});
}

void MecanumSim::write() const {
    stringstream ss;

    if (Telemetry::section("Robot Position")) {
        ss << "X\t: " << state.at(0);
        Telemetry::text(ss.str());

        ss.str("");
        ss << "Y\t: " << state.at(1);
        Telemetry::text(ss.str());

        double t = state.at(2);

        while (t > 2 * M_PI) t -= 2 * M_PI;

        ss.str("");
        ss << "Theta\t: " << t * 180.0 / M_PI;
        Telemetry::text(ss.str());
    }

    ss.str("");

    if (Telemetry::section("Encoders")) {
        for (auto&& [e, label] : std::views::zip(encoders, std::array{"FL", "FR", "BL", "BR"})) {
            ss << label << " counts: " << e->readCount();
            Telemetry::text(ss.str());
            ss.str("");
        }
    }

    ss.str("");

    if (Telemetry::section("TOF")) {
        ss << "Current reading: " << tof->getDist();
        Telemetry::text(ss.str());
    }
}