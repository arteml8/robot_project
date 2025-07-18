#include "MotorPIDController.h"

static const int PID_UPDATE_INTERVAL_MS = 50;
static const int MAX_PWM = 255;
static const int MIN_EFFECTIVE_PWM = 70;
static const float SPEED_DEADZONE_TICKS = 3.0; // small value to ignore near-zero speeds
const float MAX_INTEGRAL = 10.0;


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

void MotorPIDController::resetDistanceTracking() {
    for (int i = 0; i < motorCount; i++) {
        pidStates[i].startTicks = encoders.getTicks(i);
    }
}

float MotorPIDController::getAverageDistanceMeters() {
    float total = 0;
    for (int i = 0; i < motorCount; i++) {
        total += getMotorDistanceMeters(i);
    }
    return total / motorCount;
}

float MotorPIDController::getMotorDistanceMeters(uint8_t i) {
    if (i >= motorCount) return 0.0;
    long deltaTicks = encoders.getTicks(i) - pidStates[i].startTicks;
    return deltaTicks / ticksPerMeter;
}

void MotorPIDController::setTicksPerMeter(float tpm) {
    ticksPerMeter = tpm;
}

void MotorPIDController::getTicks(int* outTicks) {
    for (int i = 0; i < motorCount; ++i) {
        outTicks[i] = encoders.getTicks(i);
    }
}

void MotorPIDController::resetEncoders() {
    encoders.reset();
}

void MotorPIDController::update() {
    for (int i = 0; i < motorCount; i++) {
        float speed = pidStates[i].targetSpeed;
        int pwm = map(speed, -MAX_PWM, MAX_PWM, -255, 255);
        motors.setMotorSpeed(i, pwm);
    }
}

// void MotorPIDController::update() {
//     unsigned long now = millis();
//     if (now - lastUpdateTime < PID_UPDATE_INTERVAL_MS) {
//         return; // Skip until next interval
//     }

//     float dt = (now - lastUpdateTime) / 1000.0; // Convert ms to seconds
//     lastUpdateTime = now;

//     for (int i = 0; i < motorCount; i++) {
//         PIDState& state = pidStates[i];

//         long currentTicks = encoders.getTicks(i);
//         long tickDiff = currentTicks - state.lastTicks;
//         state.lastTicks = currentTicks;

//         state.currentSpeed = tickDiff / dt;

//         if (abs(state.targetSpeed) < SPEED_DEADZONE_TICKS) {
//             state.integral = 0;
//             state.error = 0;
//             state.lastError = 0;
//             state.pwmOutput = 0;
//             motors.setMotorSpeed(i, 0);
//             continue;
//         }

//         state.error = state.targetSpeed - state.currentSpeed;
//         state.integral += state.error * dt;

//         // Clamp the integral wind-up
//         const float MAX_INTEGRAL = 30.0;
//         state.integral = constrain(state.integral, -MAX_INTEGRAL, MAX_INTEGRAL);

//         float derivative = (state.error - state.lastError) / dt;

//         float output = Kp * state.error + Ki * state.integral + Kd * derivative;

//         // Optional: use feedforward if you want (simple model)
//         float feedforward = 0.2 * state.targetSpeed;
//         output += feedforward;

//         state.pwmOutput = constrain(output, -MAX_PWM, MAX_PWM);

//         if (abs(state.pwmOutput) > 0 && abs(state.pwmOutput) < MIN_EFFECTIVE_PWM) {
//             state.pwmOutput = (state.pwmOutput > 0) ? MIN_EFFECTIVE_PWM : -MIN_EFFECTIVE_PWM;
//         }

//         state.lastError = state.error;
//         motors.setMotorSpeed(i, state.pwmOutput);
//     }
// }

/*void MotorPIDController::update() {
    unsigned long now = millis();

    for (int i = 0; i < motorCount; i++) {
        PIDState& state = pidStates[i];

        unsigned long dt_ms = now - state.lastUpdate;
        if (dt_ms < UPDATE_INTERVAL) continue; // Update every XXms
        state.lastUpdate = now;
        float dt = dt_ms / 1000.0;

        long currentTicks = encoders.getTicks(i);
        long tickDiff = currentTicks - state.lastTicks;
        state.lastTicks = currentTicks;

        state.currentSpeed = tickDiff / dt;

        // If the target speed is close to zero, disable motor output completely
        if (abs(state.targetSpeed) < SPEED_DEADZONE_TICKS) {
            state.integral = 0;
            state.error = 0;
            state.lastError = 0;
            state.pwmOutput = 0;
            motors.setMotorSpeed(i, 0);  // <- This triggers your shutdown logic
            continue;
        }

        state.error = state.targetSpeed - state.currentSpeed;
        state.integral += state.error * dt;

        float derivative = (state.error - state.lastError) / dt;

        float output = Kp * state.error + Ki * state.integral + Kd * derivative;
        state.pwmOutput += output;
        state.pwmOutput = constrain(state.pwmOutput, -MAX_PWM, MAX_PWM);
        // Only apply deadzone threshold if output is *not* already zero
        if (abs(state.pwmOutput) > 0 && abs(state.pwmOutput) < MIN_EFFECTIVE_PWM) {
            state.pwmOutput = (state.pwmOutput > 0) ? MIN_EFFECTIVE_PWM : -MIN_EFFECTIVE_PWM;
        }
        state.lastError = state.error;

        motors.setMotorSpeed(i, state.pwmOutput);
    }
}*/