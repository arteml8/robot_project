#include "MotorController.h"
#include "EncoderReader.h"
#include "BluetoothComms.h"

MotorController motors;
EncoderReader encoders;
BluetoothComms bluetooth(Serial1);  // Using Mega’s Serial1 (TX1:18, RX1:19)

void setup() {
    Serial.begin(115200);
    motors.setup();
    encoders.setup();
    bluetooth.setup();

    Serial.println("Robot system initialized.");
}

void loop() {
    encoders.update();
    bluetooth.handle();

    // Example test: run motors slowly
    static bool ran = false;
    if (!ran) {
        for (int i = 0; i < 4; i++) motors.setMotorSpeed(i, 100);
        ran = true;
    }

    // Debug encoder values
    for (int i = 0; i < 4; i++) {
        Serial.print("Encoder ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(encoders.getTicks(i));
    }

    delay(200);
}