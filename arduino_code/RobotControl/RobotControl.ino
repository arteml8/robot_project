#include "MotorController.h"
#include "EncoderReader.h"
#include "BluetoothComms.h"
#include "MotorPIDController.h"

unsigned long lastPrint = 0;

MotorController motors;
EncoderReader encoders;
BluetoothComms bluetooth(Serial1);  // Using Mega’s Serial1 (TX1:18, RX1:19)
MotorPIDController pid(motors, encoders);


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
    pid.update();

    // Example: run motors at 100 ticks/sec
    static bool started = false;
    if (!started) {
        pid.setTargetSpeed(0, 100);
        pid.setTargetSpeed(1, 100);
        pid.setTargetSpeed(2, -100);
        pid.setTargetSpeed(3, -100);
        started = true;
    }

    // Debug encoder values
    if (millis()-lastPrint > 400) {
      lastPrint = millis();
      for (int i = 0; i < 4; i++) {
          Serial.print("Encoder ");
          Serial.print(i);
          Serial.print(": ");
          Serial.println(encoders.getTicks(i));
      }
    }
    // // motors.stopAll();
    if (encoders.getTicks(2) > 2000){
      motors.stopAll();
    }
}