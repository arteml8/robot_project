#include "MotorPIDController.h"

MotorPIDController::MotorPIDController(MotorController& motorCtrl, EncoderReader& encoder)
    : motors(motorCtrl), encoders(encoder) {}

void MotorPIDController::setup() {
    resetAll();
}

void MotorPIDController::setTargetSpeed(uint8_t i, float ticksPerSecond) {
    if (i < motorCount) {
        pidStates[i].targetSpeed = ticksPerSecond;
    }
}

void MotorPIDController::resetAll() {
    for (int i = 0; i < motorCount; i++) {
        pidStates[i] = PIDState();  // Reset struct to defaults
    }
}

void MotorPIDController::update() {
    unsigned long now = millis();

    for (int i = 0; i < motorCount; i++) {
        PIDState& state = pidStates[i];

        unsigned long dt_ms = now - state.lastUpdate;
        if (dt_ms < 50) continue; // Update every 50ms
        state.lastUpdate = now;
        float dt = dt_ms / 1000.0;

        long currentTicks = encoders.getTicks(i);
        long tickDiff = currentTicks - state.lastTicks;
        state.lastTicks = currentTicks;

        state.currentSpeed = tickDiff / dt;

        state.error = state.targetSpeed - state.currentSpeed;
        state.integral += state.error * dt;
        float derivative = (state.error - state.lastError) / dt;

        float output = Kp * state.error + Ki * state.integral + Kd * derivative;
        state.pwmOutput += output;
        state.pwmOutput = constrain(state.pwmOutput, -255, 255);
        state.lastError = state.error;

        motors.setMotorSpeed(i, state.pwmOutput);
    }
}