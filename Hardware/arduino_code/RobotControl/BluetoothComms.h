#ifndef BLUETOOTH_COMMS_H
#define BLUETOOTH_COMMS_H

#include <Arduino.h>
#include "MotionCommand.h"

class BluetoothComms {
public:
    BluetoothComms(HardwareSerial& serial);
    void setup();
    void update();
    // void processCommand(const String& cmd);
    bool hasNewCommand() const;
    MotionCommand getLatestCommand();

private:
    HardwareSerial& btSerial;
    MotionCommand latestCommand;
    bool commandAvailable = false;
    String inputBuffer = "";
    String lastCommand;

    void processCommand(const String& cmd);
};

#endif