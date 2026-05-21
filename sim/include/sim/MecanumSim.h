//
// Created by west on 3/22/26.
//

#ifndef INC_441SIM_MECANUMSIM_H
#define INC_441SIM_MECANUMSIM_H

#include <array>

#include "Simulator.hpp"
#include "../config/WMRConfig.h"
#include "../hardware/SimMotor.h"
#include "../hardware/SimEncoder.h"
#include "../hardware/SimTOF.h"

class MecanumSim : public Simulator<6, 4> {
public:
    /**
     * Motor and encoder indices to make initialization easier in Sim.h
     */
    static constexpr unsigned int FL{0}, FR{1}, BL{2}, BR{3};

    /**
     * Construct a simulator with the given config
     * @param config Configuration of the system
     */
    explicit MecanumSim(const WMRConfig &config);

    /**
     * Draw robot state to ImGui window
     */
    void draw() const override;

    /**
     * Write telemetry data to ImGui window
     */
    void write() const override;

    /**
     * Set the current position of the robot
     * Primarily used in simulator setup to set the initial pose
     * @param x meters
     * @param y meters
     * @param theta degrees
     */
    void setPose(double x, double y, double theta);

    void registerMotor(unsigned int idx, MotorConfig c, int pin);

    void registerEncoder(unsigned int idx, int cpr, int pin);

    void registerTOF(TOFConfig c);

    void operator()(const state_t &x, state_t &dxdt, double t) override;

protected:
    /**
     * Update simulated hardware
     * Called only by parent class' update() method
     */
    void periodic() override;

private:
    /**
     * System configuration
     */
    const WMRConfig config;

    /**
     * Simulated hardware
     */
    std::array<std::unique_ptr<SimMotor>, 4> motors;
    std::array<std::unique_ptr<SimEncoder>, 4> encoders;

    std::unique_ptr<SimTOF> tof;

    /**
     * Get wheel velocities given current system state
     * Helper function used in updateHardware and setPlantInputs
     * @return Wheel velocities
     */
    [[nodiscard]] std::array<double, 4> fwdKinematics() const;
};

#endif //INC_441SIM_MECANUMSIM_H