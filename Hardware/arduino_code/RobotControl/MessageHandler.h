#ifndef MESSAGE_HANDLER_H
#define MESSAGE_HANDLER_H

#include <Arduino.h>
#include "MotionController.h"

class MessageHandler {
public:
    MessageHandler(MotionController& motionRef);

    // Parses incoming command and returns an ACK or ERR response
    String processCommand(const String& rawCmd);

private:
    MotionController& motion;

    // Internal helpers
    bool parseDriveCommand(const String& cmd, float& fwd, float& strafe, float& rotate);
    bool parseDriveTimeCommand(const String& cmd, float& fwd, float& strafe, float& rotate, float& time);
    bool parseDriveDistCommand(const String& cmd, float& fwd, float& strafe, float& rotate, float& dist);
};

#endif