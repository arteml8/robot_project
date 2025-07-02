#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <Arduino.h>
#include "MotorPIDController.h"
#include "MotionCommand.h"

class MotionController {
public:
    MotionController(MotorPIDController& pidController);
    void setup();
    void drive(float forward, float strafe, float rotate);  // Input units: meters/sec or arbitrary
    void getTicks(int* outTicks);                           // Get ticks from the encoder count via the motor pid controller
    void driveFor(float forward, float strafe, float rotate, float duration_s);
    void driveDistance(float forward, float strafe, float rotate, float meters);
    void stop();
    void resetEncoders();
    void applyCommand(const MotionCommand& cmd);
    void update();  // Call regularly

private:
    MotorPIDController& motorPID;
    float forwardSpeed;
    float strafeSpeed;
    float rotateSpeed;

    const float wheelRadius = 0.0381; // 3" diameter / 2 → meters
    const float ticksPerRevolution = 46.0; // 23.0;
    const float wheelCircumference = 2 * PI * wheelRadius;
    const float ticksPerMeter = ticksPerRevolution / wheelCircumference;

    MotionCommand currentCommand;
};

#endif