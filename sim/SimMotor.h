//
// Created by west on 2/26/26.
//

#ifndef INC_441SIM_SIMMOTOR_H
#define INC_441SIM_SIMMOTOR_H
#include <cmath>


// Abstract class to be implemented by a class in user code
class SimMotor {
public:
    virtual ~SimMotor() = default;

    SimMotor();

    void setSpeed(double speed);

    [[nodiscard]] double getTorque() const;

private:
    // Current state
    double omega;

protected:
    [[nodiscard]] virtual int getVoltage() const = 0;
};


#endif //INC_441SIM_SIMMOTOR_H