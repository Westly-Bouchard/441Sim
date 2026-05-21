// #include <mosscap/defaultTank.h>
// #include <Arduino.h>
//
// Motor lM(10);
// Motor rM(11);
//
// void setup() {
//     pinMode(1, INPUT);
// }
//
// void loop() {
//     if (digitalRead(1)) {
//         lM.run(-150);
//         rM.run(150);
//     } else {
//         lM.run(0);
//         rM.run(0);
//     }
// }

#include <sim/TankSim.h>
#include <Arduino.h>
#include <Motor.h>

std::unique_ptr<SimulatorBase> simInit();

#define FORWARD_PIN 41
#define TURN_LEFT_PIN 42
#define TURN_RIGHT_PIN 43
#define BACKWARD_PIN 44

Motor left(11, 10, 12);
Motor right(21, 20, 22);

void run(int pwm) {
    left.run(pwm);
    right.run(pwm);
}

void turn(int pwm) {
    left.run(-pwm);
    right.run(pwm);
}

void setup() {
    pinMode(FORWARD_PIN, INPUT);
    pinMode(BACKWARD_PIN, INPUT);
    pinMode(TURN_LEFT_PIN, INPUT);
    pinMode(TURN_RIGHT_PIN, INPUT);
}

void loop() {
    if (digitalRead(FORWARD_PIN)) {
        run(200);
    } else if (digitalRead(BACKWARD_PIN)) {
        run(-200);
    } else if (digitalRead(TURN_LEFT_PIN)) {
        turn(200);
    } else if (digitalRead(TURN_RIGHT_PIN)) {
        turn(-200);
    } else {
        run(0);
    }
}

std::unique_ptr<SimulatorBase> simInit() {
    // Set rendering scale
    Renderer::setScale(2.0);

    // Configuration of the robot
    WMRConfig config{2.0 /*kg*/, 0.20 /*m*/, 0.15 /*m*/, 0.04 /*m*/};
    config.showEncoderTelemetry = false;
    config.showTOFTelemetry = false;

    config.kineticFriction = 0.05;

    config.noiseMagnitude = 0.05;

    // Create robot
    auto robot = std::make_unique<TankSim>(config);
    robot->setPose(1, 1, 0);

    TOFConfig tofConfig{0, 0, 0};
    tofConfig.boundingBox = BoundingBox(3, 3);

    robot->registerTOF(tofConfig);

    // Configuration of motors
    // In theory you could have a different config for each motor if you wanted
    // But in this case each motor is exactly the same
    const MotorConfig mConfig{3.1, 0.1245, 2.3, 6.0};

    robot->registerLeftMotor(mConfig, 10);
    robot->registerRightMotor(mConfig, 20);

    robot->registerLeftEncoder(1500, 20);
    robot->registerRightEncoder(1500, 21);

    ArduinoRuntime::getInstance().createButton("Forward", 41);
    ArduinoRuntime::getInstance().createButton("Backward", 44);
    ArduinoRuntime::getInstance().createButton("Left", 42);
    ArduinoRuntime::getInstance().createButton("Right", 43);

    return robot;
}