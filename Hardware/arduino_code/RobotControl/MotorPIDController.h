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
    void resetDistanceTracking();                         // call this when starting a new move
    float getAverageDistanceMeters();                     // useful for stopping when distance met
    void getTicks(int* outTicks);                         // helpful function that gets the ticks from the encoder reader
    float getMotorDistanceMeters(uint8_t motorIndex);     // optional, if needed per motor
    void setTicksPerMeter(float tpm);                     // let external code define scale
    void setTargetSpeed(uint8_t motorIndex, float ticksPerSecond);
    void resetEncoders();
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
        long startTicks = 0;            // ← NEW: for distance tracking
        unsigned long lastUpdate = 0;
    };

    float ticksPerMeter = 1000.0;  // default — will be set properly from MotionController
    static const uint8_t motorCount = 4;
    PIDState pidStates[motorCount];
    unsigned long lastUpdateTime = 0;  // global update timestamp

    MotorController& motors;
    EncoderReader& encoders;

    // const float Kp = 1.2;
    // const float Ki = 0.0;
    // const float Kd = 0.05;

    const float Kp = 1.0;
    const float Ki = 0.02;
    const float Kd = 0.05;
};

#endif