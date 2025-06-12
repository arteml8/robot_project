#include "MotorController.h"
#include "EncoderReader.h"
#include "MessageHandler.h"
#include "EthernetComms.h"
// #include "BluetoothComms.h"
#include "MotionController.h"

unsigned long lastRunTime = 0;
unsigned long stop_motors_time = 0;
const unsigned long interval = 500; // 500 milliseconds
bool systemReady = false;

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 177);  // use your actual IP
EthernetComms ethComms(ip, mac);

MotorController motors;
EncoderReader encoders;
// BluetoothComms bluetooth(Serial1);  // Using Mega’s Serial1 (TX1:18, RX1:19)
MotorPIDController pid(motors, encoders);
MotionController motion(pid);
MessageHandler handler(motion);


void testMotorDirections() {
    for (int i = 0; i < 4; i++) {
        motors.setMotorSpeed(i, 100);
        delay(1000);
        motors.setMotorSpeed(i, 0);
        delay(500);
    }
}

void setup() {
    for (int i = 0; i < 4; i++) {
        motors.setMotorSpeed(i, 0); // Set all motors to LOW
    }

    Serial.begin(115200);
    motors.setup();
    encoders.setup();
    pid.setup();
    motion.setup();
    // bluetooth.setup();
    motion.stop();
    Serial.println("Robot system initialized.");
    Serial.println("Ready for commands over Serial");
    ethComms.begin()
    Serial.println("Ready for commands over Ethernet");
    // testMotorDirections();
    delay(500);
    systemReady = true;
}

void loop() {
    if (!systemReady) {
      Serial.println("SYSTEM NOT READY!");
      return;
    }
    // bluetooth.update();

    // if (bluetooth.hasNewCommand()) {
    //     Serial.println("GOT A COMMAND!");
    //     MotionCommand cmd = bluetooth.getLatestCommand();
    //     motion.applyCommand(cmd);
    // }
    if (Serial.available()) {
      String rawCmd = Serial.readStringUntil('\n');
      String response = handler.processCommand(rawCmd);
      Serial.println(response);  // Echo back result (ACK or ERR)
    }
    
    ethComms.update();
    if (ethComms.hasNewMessage()) {
        String cmd = ethComms.getMessage();
        handler.handle(cmd);
    }
    
    encoders.update();
    pid.update();
    motion.update();
    
    // unsigned long now = millis();

    // Example: move forward for 1 meter at moderate speed
    // static bool testStarted = false;
    // if (!testStarted) {
    //     motion.driveFor(0.0, 0.2, 0.0, 1);
    //     // motion.driveDistance(1.0, 0.2, 0.0, 0.0);
    //     testStarted = true;
    // }

    // // Optionally: print encoder ticks
    // if (now - lastRunTime >= interval) {
    //   lastRunTime = now;
    //   for (int i = 0; i < 4; i++) {
    //       Serial.print("Motor ");
    //       Serial.print(i);
    //       Serial.print(": ");
    //       Serial.println(encoders.getTicks(i));
    //   }
    // }
}