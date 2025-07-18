#include "MotorController.h"
#include "EncoderReader.h"
#include "MessageHandler.h"
#include <Ethernet.h>
#include "MotionController.h"

unsigned long lastRunTime = 0;
unsigned long stop_motors_time = 0;
unsigned long lastBLECommandTime = millis();
const unsigned long interval = 500; // 500 milliseconds
const unsigned long BLE_TIMEOUT = 3000; // 3 seconds
bool systemReady = false;
bool seenCommand = false;

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 177);  // use your actual IP
EthernetServer server(23);

MotorController motors;
EncoderReader encoders;
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
    Serial1.begin(115200);
    motors.setup();
    encoders.setup();
    pid.setup();
    motion.setup();
    motion.stop();
    Serial.println("Robot system initialized.");
    Serial.println("Ready for commands over Serial");
    Ethernet.begin(mac, ip);
    server.begin();
    Serial.println("Ready for commands over Ethernet");
    Serial.print("My IP address: ");
    Serial.println(Ethernet.localIP());
    // testMotorDirections();
    delay(500);
    systemReady = true;
}

void loop() {
    if (!systemReady) {
        Serial.println("SYSTEM NOT READY!");
        return;
    }

    // Handle Serial (USB) commands
    if (Serial.available()) {
        String rawCmd = Serial.readStringUntil('\n');
        String response = handler.processCommand(rawCmd);
        Serial.println(response);
    }

    // Handle Serial1 (BLE/ESP32) commands
    if (Serial1.available()) {
        String msg = Serial1.readStringUntil('\n');
        msg.trim();
        if (msg.length() > 0) {
            lastBLECommandTime = millis();
            seenCommand = true;
            String response = handler.processCommand(msg);
            Serial1.println(response);
        }
    }

    // If too long since last BLE command, stop motors
    if (seenCommand && millis() - lastBLECommandTime > BLE_TIMEOUT) {
        Serial.println("⚠️ BLE TIMEOUT - No commands in 3s. Stopping motors.");
        motion.stop();
        seenCommand = false;  // Prevent repeated stop calls
    }

    // Update control systems
    encoders.update();
    pid.update();
    motion.update();
}