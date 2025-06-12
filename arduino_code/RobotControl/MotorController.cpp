#include "MotorController.h"

MotorController::MotorController() {
    // Motor 1 → pins 5 (EN), 6 (IN1), 7 (IN2)
    // Motor 2 → pins 10 (EN), 8 (IN1), 9 (IN2)
    // Motor 3 → pins 11 (EN), 12 (IN1), 13 (IN2)
    // Motor 4 → pins 46 (EN), 44 (IN1), 45 (IN2)
    setMotorPins(0, 5, 6, 7);
    setMotorPins(1, 10, 8, 9);
    setMotorPins(2, 11, 12, 13);
    setMotorPins(3, 46, 44, 45);
}

void MotorController::setMotorPins(uint8_t i, uint8_t en, uint8_t in1, uint8_t in2) {
    motors[i] = {en, in1, in2};
}

void MotorController::setup() {
    for (int i = 0; i < 4; i++) {
        pinMode(motors[i].en, OUTPUT);
        pinMode(motors[i].in1, OUTPUT);
        pinMode(motors[i].in2, OUTPUT);
    }
}

void MotorController::setMotorSpeed(uint8_t i, int speed) {
    if (i > 3) return;

    static const int MIN_EFFECTIVE_PWM = 40;
    static const int KICKSTART_PWM = 120;
    static const unsigned long KICKSTART_TIME_MS = 5;

    static unsigned long lastKickTime[4] = {0, 0, 0, 0};
    static bool kicked[4] = {false, false, false, false};

    bool forward = speed >= 0;
    int absSpeed = abs(speed);
    absSpeed = constrain(absSpeed, 0, 255);

    unsigned long now = millis();

    // Apply low outputs to all motor control pins on 0 speed
    if (speed == 0 || speed == 0.0) {
        digitalWrite(motors[i].in1, LOW);
        digitalWrite(motors[i].in2, LOW);
        digitalWrite(motors[i].en, LOW);
        return;
    }

    // Apply kickstart if speed is small and not already kicked
    if (absSpeed > 0 && absSpeed < MIN_EFFECTIVE_PWM && !kicked[i]) {
        digitalWrite(motors[i].in1, forward ? HIGH : LOW);
        digitalWrite(motors[i].in2, forward ? LOW : HIGH);
        analogWrite(motors[i].en, KICKSTART_PWM);
        lastKickTime[i] = now;
        kicked[i] = true;
        return;
    }

    // Allow time for kickstart, then resume normal control
    if (kicked[i] && now - lastKickTime[i] < KICKSTART_TIME_MS) {
        return;  // Wait until kickstart time is over
    }

    // Kickstart complete or not needed — normal operation
    kicked[i] = false;
    digitalWrite(motors[i].in1, forward ? HIGH : LOW);
    digitalWrite(motors[i].in2, forward ? LOW : HIGH);
    analogWrite(motors[i].en, absSpeed);
}

void MotorController::stopAll() {
    for (int i = 0; i < 4; i++) {
        analogWrite(motors[i].en, 0);
    }
}