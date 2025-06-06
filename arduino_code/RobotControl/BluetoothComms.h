#ifndef BLUETOOTH_COMMS_H
#define BLUETOOTH_COMMS_H

#include <Arduino.h>

class BluetoothComms {
public:
    BluetoothComms(HardwareSerial& serial);
    void setup();
    void handle(); // Call in loop()

private:
    HardwareSerial& btSerial;
};

#endif