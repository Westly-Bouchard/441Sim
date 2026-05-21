//
// Created by west on 3/21/26.
//

#ifndef INC_441SIM_SIMULATORNEW_H
#define INC_441SIM_SIMULATORNEW_H

// Standard library includes
#include <array>
#include <functional>
#include <thread>

// Boost includes
#include <boost/numeric/odeint.hpp>

// Project Includes
#include <random>

#include "../arduino/ArduinoRuntime.h"
#include "SimulatorBase.h"
#include "../util/Renderer.h"
#include "../util/Telemetry.h"
#include "../hardware/SimClock.h"

/**
 * Middleman that abstracts common functionality
 * To be implemented for specific robots (Mecanum, Tank, etc.)
 */
template <unsigned int numStates, unsigned int numInputs>
class Simulator : public SimulatorBase, public Drawable, public TelemetryProvider {
public:
    // Why are there so many scopes. So, so many
    using stepper_t = boost::numeric::odeint::runge_kutta4<std::array<double, numStates>>;

    using state_t = std::array<double, numStates>;
    using input_t = std::array<double, numInputs>;

    /**
     * Construct simulator, calls Arduino `setup()` function
     * @param timestep Simulation timestep, default is 0.001 (1000Hz)
     */
    explicit Simulator(const double timestep = 0.001) : Drawable(0), TelemetryProvider(0), gen(std::random_device{}()),
                                                        dt(timestep), simTime(0.0) {
    }

    ~Simulator() override {
        // Stop user code thread and join
        arduinoThreadRunning = false;
        arduinoThread.join();
    }

    /**
     * Update the simulator (physics and associated hardware)
     * Inherited from SimulatorBase
     * @param acc Accumulated time since last update
     */
    void update(double& acc) override {
        static bool first = true;
        if (first) {
            // Start Arduino thread
            arduinoThread = std::thread(&Simulator::arduinoThreadFunc, this);
            first = false;
        }

        while (acc > dt) {
            // Update hardware
            periodic();

            // Step the physics sim
            // stepper.do_step(std::ref(*plant), state, simTime, dt);
            stepper.do_step(std::ref(*this), state, simTime, dt);
            simTime += dt;

            // Update clock
            clock.updateTime(dt);
            acc -= dt;
        }
    }

    virtual void operator() (const state_t &x, state_t &dx, double t) = 0;

protected:
    /**
     * Update the simulated hardware
     * To be implemented for a specific robot
     */
    virtual void periodic() = 0;

    /**
     * Current state of the system, example values could be:
     * - x      | X position of the robot in the world frame
     * - y      | Y position of the robot in the world frame
     * - theta  | Orientation of the robot measured from world x
     * - v_x    | Velocity of the robot in the world x direction
     * - v_y    | Velocity of the robot in the world y direction
     * - omega  | Angular velocity of hte robot about world z
     */
    state_t state{};

    // Random number generator to be used when adding noise to a system
    std::mt19937 gen;

    /**
     * Timestep for the simulator
     * The simulator will update at a rate of 1/dt Hz
     */
    const double dt;

private:
    /**
     * Runs the user's Arduino code through the setup() and loop() functions
     * This function isn't really associated with this class. But running the
     * Arduino code is the simulator's job, so it's in here.
     *
     * Maybe it should be somewhere else though?
     */
    void arduinoThreadFunc() const {
        // Call setup function
        setup();

        auto lastTime = std::chrono::high_resolution_clock::now();

        while (arduinoThreadRunning) {
            // Call loop
            loop();

            // Sleep for a little bit so that we don't
            // pin a core at 100% usage at all times.
            // Will use 500Hz for now because physics runs at 1kHz
            auto now = std::chrono::high_resolution_clock::now();
            double remainingTime = (1.0 / 500) - std::chrono::duration<double>(now - lastTime).count();
            lastTime = now;

            // If that loop call took less than 1 500th of a second, sleep the thread until it's time
            // for the next call to loop.
            // If the user code blocks in loop or sleeps the thread, that's fine, we just want to limit
            // the upper bound for the frequency at which this thread will run
            if (remainingTime > 0) {
                std::this_thread::sleep_for(std::chrono::duration<double>(remainingTime));
            }
        }
    }

    /**
     * The other important state variable for the simulator
     * This is passed to the stepper during the physics update
     */
    double simTime;

    /**
     * Stepper that incrementally integrates the plant and
     * updates the system state
     */
    stepper_t stepper;

    std::atomic_bool arduinoThreadRunning{true};
    std::thread arduinoThread;
};

#endif //INC_441SIM_SIMULATORNEW_H