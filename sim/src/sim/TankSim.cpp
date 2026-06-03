//
// Created by west on 5/13/26.
//

#include "sim/TankSim.h"

using namespace std;

TankSim::TankSim(const WMRConfig &config) : config(config), noise(1.0, config.noiseMagnitude) {}

void TankSim::setPose(const double x, const double y, const double theta) {
    state.at(0) = x;
    state.at(1) = y;
    state.at(2) = theta * (M_PI / 180.0);
}

void TankSim::periodic() {
    // Update encoders and TOF sensor
    const auto [wL, wR] = fwdKinematics(state);

    leftEncoder->updatePosition(wL *dt);
    rightEncoder->updatePosition(wR *dt);

    tof->update(state.at(0), state.at(1), state.at(2));

    // Generate noise for next physics update
    // This must happen here because we want the noise to be constant across the rk4
    // Update but, of course, we still want the noise to act as noise, and so it must be
    // updated periodically
    currentNoise = { noise(gen), noise(gen) };

    // Force velocities to zero if under threshold
    const bool leftPowered = leftMotor->getPWM() != 0;
    const bool rightPowered = rightMotor->getPWM() != 0;

    constexpr double GLOBAL_STOP_THRESHOLD = 0.01;

    if (!leftPowered && !rightPowered) {
        const double vx = state.at(3);
        const double vy = state.at(4);
        const double omega = state.at(5);

        const double linear = sqrt(pow(vx, 2) + pow(vy, 2));

        if (linear < GLOBAL_STOP_THRESHOLD && abs(omega) < GLOBAL_STOP_THRESHOLD) {
            state.at(3) = 0;
            state.at(4) = 0;
            state.at(5) = 0;
        }
    }
}

void TankSim::operator()(const state_t &x, state_t &dxdt, const double t) {
    const auto [wL, wR] = fwdKinematics(x);

    const double s = sin(x.at(2));
    const double c = cos(x.at(2));

    // Calculate plant inputs
    const double tL = leftMotor->getTorque(wL, currentNoise.at(0)) - config.kineticFriction * tanh(1000.0 * wL);
    const double tR = rightMotor->getTorque(wR, currentNoise.at(1)) - config.kineticFriction * tanh(1000.0 * wR);

    // Body frame y velocity. This should always be as close to zero as possible, so we'll try to force it there
    const double bVx = x.at(3) * c + x.at(4) * s;
    const double bVy = x.at(4) * c - x.at(3) * s;

    // Compute dynamics and update derivative vector
    // Calculate forces in the body frame
    // Force in the y direction is always zero (in this simplified case)
    const double Fl = tL / config.wheelRadius;
    const double Fr = tR / config.wheelRadius;

    const double bFx = Fl + Fr;
    const double bFy = config.mass * x.at(5) * bVx - 1000.0 * bVy;
    const double bTz = (config.trackWidth / 2.0) * (Fr - Fl);

    // Transform forces into the world frame
    const double wFx = bFx * c - bFy * s;
    const double wFy = bFx * s + bFy * c;
    const double wTz = bTz;

    // Update the derivative vector
    dxdt.at(0) = x.at(3);
    dxdt.at(1) = x.at(4);
    dxdt.at(2) = x.at(5);

    // Update acceleration components of the derivative vector
    dxdt.at(3) = (1.0 / config.mass) * wFx;
    dxdt.at(4) = (1.0 / config.mass) * wFy;
    dxdt.at(5) = (1.0 / config.inertia) * wTz;
}

std::array<double, 2> TankSim::fwdKinematics(const state_t& st) const {
    // Use current state to compute the velocities of the left and right wheels
    const double wVx = st.at(3);
    const double wVy = st.at(4);
    const double wWz = st.at(5);

    const double s = sin(st.at(2));
    const double c = cos(st.at(2));

    const double bVx = wVx * c + wVy * s;

    // Should be zero
    // const double bVy = wVy * c - wVx * s;

    const double wL = (bVx - (wWz * config.trackWidth / 2.0)) / config.wheelRadius;
    const double wR = (bVx + (wWz * config.trackWidth / 2.0)) / config.wheelRadius;

    return {wL, wR};
}

std::array<double, 2> TankSim::bodyToWorld(const double x, const double y) const {
    const double s = sin(state.at(2));
    const double c = cos(state.at(2));

    const double wX = x * c - y * s + state.at(0);
    const double wY = x * s + y * c + state.at(1);

    return {wX, wY};
}

void TankSim::registerLeftMotor(MotorConfig c, const int pin) {
    leftMotor = std::make_unique<SimMotor>(c);
    ArduinoRuntime::getInstance().bindPWM(pin, *leftMotor);
}

void TankSim::registerRightMotor(MotorConfig c, const int pin) {
    rightMotor = std::make_unique<SimMotor>(c);
    ArduinoRuntime::getInstance().bindPWM(pin, *rightMotor);
}

void TankSim::registerLeftEncoder(int cpr, const int pin) {
    leftEncoder = std::make_unique<SimEncoder>(cpr);
    ArduinoRuntime::getInstance().bindEncoder(pin, *leftEncoder);
}

void TankSim::registerRightEncoder(int cpr, const int pin) {
    rightEncoder = std::make_unique<SimEncoder>(cpr);
    ArduinoRuntime::getInstance().bindEncoder(pin, *rightEncoder);
}

void TankSim::registerTOF(TOFConfig c) {
    tof = std::make_unique<SimTOF>(c);
    ArduinoRuntime::getInstance().bindTOF(*tof);
}

void TankSim::draw() {
    // Here I assume that the wheels are 3cm wide
    const double chassisWidth = config.trackWidth - 0.03;

    // Draw chassis
    Renderer::drawRect(
        state.at(0),
        state.at(1),
        state.at(2),
        config.wheelBase,
        chassisWidth,
        {0xf5, 0xf5, 0xf5}
    );

    // If the user wants to see time of flight telemetry
    if (config.showTOFTelemetry) {
        double dist = tof->getDist();

        if (dist > scale) {
            dist = scale;
        }

        const auto [tofLineX, tofLineY] = bodyToWorld(dist / 2.0 , 0);

        Renderer::drawRect(
            tofLineX,
            tofLineY,
            state.at(2),
            dist,
            0.0075,
            {50, 75, 238}
        );
    }

    // Mark the forward direction with an arrow
    const double arrowLength = config.wheelBase * 0.75;
    const double arrowThickness = config.trackWidth * 0.1;
    const double arrowHeadLength = 2.2 * sqrt(pow(arrowLength / 4.0, 2) / 2.0);

    Renderer::drawRect(
        state.at(0),
        state.at(1),
        state.at(2),
        arrowLength,
        arrowThickness,
        {0, 255, 0}
    );

    const auto [headLX, headLY] = bodyToWorld(arrowLength * (3.0/8.0), arrowLength * (1.0/8.0));
    Renderer::drawRect(
        headLX,
        headLY,
        state.at(2) - M_PI / 4.0,
        arrowHeadLength,
        arrowThickness,
        {0, 255, 0}
    );

    const auto [headRX, headRY] = bodyToWorld(arrowLength * (3.0/8.0), -arrowLength * (1.0/8.0));
    Renderer::drawRect(
        headRX,
        headRY,
        state.at(2) + M_PI / 4.0,
        arrowHeadLength,
        arrowThickness,
        {0, 255, 0}
    );

    // Draw wheels
    const auto [wheelLX, wheelLY] = bodyToWorld(0, chassisWidth / 2.0 + 0.015);
    Renderer::drawRect(
        wheelLX,
        wheelLY,
        state.at(2),
        config.wheelRadius * 2.0,
        0.03,
        {0xb4, 0xb4, 0xb4}
    );

    const auto [wheelRX, wheelRY] = bodyToWorld(0, -chassisWidth / 2.0 - 0.015);
    Renderer::drawRect(
        wheelRX,
        wheelRY,
        state.at(2),
        config.wheelRadius * 2.0,
        0.03,
        {0xb4, 0xb4, 0xb4}
    );
}

void TankSim::write() const {
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

    if (config.showEncoderTelemetry) {
        ss.str("");
        if (Telemetry::section("Encoders")) {
            ss << "Left counts: " << leftEncoder->readCount();
            Telemetry::text(ss.str());
            ss.str("");
            ss << "Right counts: " << rightEncoder->readCount();
            Telemetry::text(ss.str());
            ss.str("");
        }
    }

    if (config.showTOFTelemetry) {
        ss.str("");
        if (Telemetry::section("TOF")) {
            ss << "Current reading: " << tof->getDist();
            Telemetry::text(ss.str());
        }
    }
}