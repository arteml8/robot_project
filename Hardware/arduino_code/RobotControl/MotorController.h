#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>

class MotorController {
public:
    MotorController();
    void setup();
    void setMotorSpeed(uint8_t motorIndex, int speed); // -255 to 255
    void stopAll();

private:
    struct MotorPins {
        uint8_t en;
        uint8_t in1;
        uint8_t in2;
    };

    MotorPins motors[4];
    void setMotorPins(uint8_t motorIndex, uint8_t en, uint8_t in1, uint8_t in2);
};

#endif