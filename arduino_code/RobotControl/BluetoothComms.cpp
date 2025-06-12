#include "BluetoothComms.h"

BluetoothComms::BluetoothComms(HardwareSerial& serial) : btSerial(serial) {}

void BluetoothComms::setup() {
    btSerial.begin(115200);
}

void BluetoothComms::update() {
    while (btSerial.available()) {
        char c = btSerial.read();

        Serial.print(c);  // Debug: Print incoming chars

        if (c == '\n') {
            inputBuffer.trim();  // Remove whitespace/newlines
            if (inputBuffer.length() > 0) {
                Serial.print("Received command: ");
                Serial.println(inputBuffer);

                commandAvailable = true;
                lastCommand = inputBuffer;

                // Optionally echo back to sender
                btSerial.println("ACK: " + lastCommand);
            }
            inputBuffer = "";
        } else {
            inputBuffer += c;
        }
    }
}

bool BluetoothComms::hasNewCommand() const {
    return commandAvailable;
}

MotionCommand BluetoothComms::getLatestCommand() {
    commandAvailable = false;  // Clear the flag
    return latestCommand;
}

void BluetoothComms::processCommand(const String& cmd) {
    if (cmd.startsWith("CMD:DRIVE:")) {
        // CMD:DRIVE:0.2,0.0,0.0
        Serial.println("-> DRIVE command detected.");
        String params = cmd.substring(10);
        float fwd = params.substring(0, params.indexOf(',')).toFloat();
        int idx1 = params.indexOf(',') + 1;
        int idx2 = params.indexOf(',', idx1);
        float strafe = params.substring(idx1, idx2).toFloat();
        float rotate = params.substring(idx2 + 1).toFloat();

        latestCommand = MotionCommand();
        latestCommand.type = MotionType::TIME;
        latestCommand.speedFwd = fwd;
        latestCommand.speedStrafe = strafe;
        latestCommand.speedRotate = rotate;
        latestCommand.duration = 0.5;  // Short duration drive burst
        latestCommand.startTime = millis();
        commandAvailable = true;

    } else if (cmd.startsWith("CMD:DRIVE_TIME:")) {
        // CMD:DRIVE_TIME:0.2,0.0,0.0,2.0
        Serial.println("-> DRIVE_TIME command detected.");
        String params = cmd.substring(15);
        float fwd = params.substring(0, params.indexOf(',')).toFloat();
        int i1 = params.indexOf(',') + 1;
        int i2 = params.indexOf(',', i1);
        int i3 = params.indexOf(',', i2 + 1);
        float strafe = params.substring(i1, i2).toFloat();
        float rotate = params.substring(i2 + 1, i3).toFloat();
        float duration = params.substring(i3 + 1).toFloat();

        latestCommand = MotionCommand();
        latestCommand.type = MotionType::TIME;
        latestCommand.speedFwd = fwd;
        latestCommand.speedStrafe = strafe;
        latestCommand.speedRotate = rotate;
        latestCommand.duration = duration;
        latestCommand.startTime = millis();
        commandAvailable = true;

    } else if (cmd.startsWith("CMD:DRIVE_DIST:")) {
        // CMD:DRIVE_DIST:0.2,0.0,0.0,1.0
        Serial.println("-> DRIVE_DIST command detected.");
        String params = cmd.substring(16);
        float fwd = params.substring(0, params.indexOf(',')).toFloat();
        int i1 = params.indexOf(',') + 1;
        int i2 = params.indexOf(',', i1);
        int i3 = params.indexOf(',', i2 + 1);
        float strafe = params.substring(i1, i2).toFloat();
        float rotate = params.substring(i2 + 1, i3).toFloat();
        float distance = params.substring(i3 + 1).toFloat();

        latestCommand = MotionCommand();
        latestCommand.type = MotionType::DISTANCE;
        latestCommand.speedFwd = fwd;
        latestCommand.speedStrafe = strafe;
        latestCommand.speedRotate = rotate;
        latestCommand.distance = distance;
        latestCommand.startTime = millis();
        commandAvailable = true;

    } else if (cmd.startsWith("CMD:STOP")) {
    	Serial.println("-> STOP command detected.");
        latestCommand = MotionCommand();  // type = NONE by default
        latestCommand.type = MotionType::NONE;
        commandAvailable = true;

    } else {
        Serial.println("Unknown command: " + cmd);
    }
}