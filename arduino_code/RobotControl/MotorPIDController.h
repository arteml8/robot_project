#ifndef MOTOR_PID_CONTROLLER_H
#define MOTOR_PID_CONTROLLER_H

#include <Arduino.h>
#include "MotorController.h"
#include "EncoderReader.h"

class MotorPIDController {
public:
    MotorPIDController(MotorController& motorCtrl, EncoderReader& encoder);
    void setup();
    void update();  // Call periodically in loop()
    void setTargetSpeed(uint8_t motorIndex, float ticksPerSecond);
    void resetAll();

private:
    struct PIDState {
        float targetSpeed = 0;
        float currentSpeed = 0;
        float error = 0;
        float lastError = 0;
        float integral = 0;
        int pwmOutput = 0;
        long lastTicks = 0;
        unsigned long lastUpdate = 0;
    };

    static const uint8_t motorCount = 4;
    PIDState pidStates[motorCount];

    MotorController& motors;
    EncoderReader& encoders;

    // const float Kp = 1.2;
    // const float Ki = 0.0;
    // const float Kd = 0.05;

    const float Kp = 0.8;
    const float Ki = 0.02;
    const float Kd = 0.05;
};

#endif