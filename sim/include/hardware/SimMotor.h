//
// Created by Westly Bouchard on 3/2/26.
//

#ifndef SIMMOTOR_H
#define SIMMOTOR_H

#include <atomic>

#include "../capability/WriteablePWM.h"
#include "../config/MotorConfig.h"

/**
 * Simulated motor
 */
class SimMotor final : public WriteablePWM {
public:
    /**
     * Construct a simulated motor with the given configuration
     * @param c Config of the motor see @link MotorConfig
     */
    explicit SimMotor(const MotorConfig& c);

    /**
     * Delete copy constructor because copying simulated hardware
     * would be bad
     */
    SimMotor(const SimMotor&) = delete;

    /**
     * This is inherited from the @link WriteablePWM @link Capability
     * It is used by the ArduinoRuntime to write values to the motor
     * This function should never be called directly
     * @param value PWM Setpoint of the motor
     */
    void writePWM(int value) override;

    /**
     * Get the torque of the motor given a current speed
     * This would be called by an @link Simulator 's updateHardware function
     * @param speed Current angular velocity of the motor in rad / sec
     * @return Torque produced by the motor
     */
    [[nodiscard]] double getTorque(double speed);

    [[nodiscard]] double getTorque(double speed, double noise);

    /**
     * Get the current speed of the motor
     * This will primarily be used to write telemetry
     * @return Current angular velocity of shaft
     */
    [[nodiscard]] double getSpeed() const;

    [[nodiscard]] double getPWM() const;

private:
    /**
     * Hardware configuration
     */
    const MotorConfig config;

    /**
     * Current hardware state
     */
    std::atomic_int pwm;
    double velocity;
};

#endif //SIMMOTOR_H