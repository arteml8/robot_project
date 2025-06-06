#include "BluetoothComms.h"

BluetoothComms::BluetoothComms(HardwareSerial& serial) : btSerial(serial) {}

void BluetoothComms::setup() {
    btSerial.begin(9600);
}

void BluetoothComms::handle() {
    if (btSerial.available()) {
        String cmd = btSerial.readStringUntil('\n');
        cmd.trim();
        btSerial.println("Received: " + cmd);
        // Add simple command parsing here later
    }
}