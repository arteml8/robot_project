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
    bool forward = speed >= 0;
    speed = abs(speed);
    speed = constrain(speed, 0, 255);

    digitalWrite(motors[i].in1, forward ? HIGH : LOW);
    digitalWrite(motors[i].in2, forward ? LOW : HIGH);
    analogWrite(motors[i].en, speed);
}

void MotorController::stopAll() {
    for (int i = 0; i < 4; i++) {
        analogWrite(motors[i].en, 0);
    }
}