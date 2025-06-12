#include "MessageHandler.h"

MessageHandler::MessageHandler(MotionController& motionRef) : motion(motionRef) {}

String MessageHandler::processCommand(const String& rawCmd) {
    String cmd = rawCmd;
    cmd.trim();  // Remove whitespace

    if (cmd.startsWith("CMD:DRIVE:")) {
        float fwd, strafe, rotate;
        if (parseDriveCommand(cmd, fwd, strafe, rotate)) {
            motion.drive(fwd, strafe, rotate);
            return "ACK:DRIVE";
        } else {
            return "ERR:DRIVE_FORMAT";
        }
    }

    if (cmd.startsWith("CMD:DRIVE_TIME:")) {
        float fwd, strafe, rotate, time;
        if (parseDriveTimeCommand(cmd, fwd, strafe, rotate, time)) {
            motion.driveFor(fwd, strafe, rotate, time);
            return "ACK:DRIVE_TIME";
        } else {
            return "ERR:DRIVE_TIME_FORMAT";
        }
    }

    if (cmd.startsWith("CMD:DRIVE_DIST:")) {
        float fwd, strafe, rotate, dist;
        if (parseDriveDistCommand(cmd, fwd, strafe, rotate, dist)) {
            motion.driveDistance(dist, fwd, strafe, rotate);
            return "ACK:DRIVE_DIST";
        } else {
            return "ERR:DRIVE_DIST_FORMAT";
        }
    }

    if (cmd.startsWith("CMD:STOP")) {
        motion.stop();
        return "ACK:STOP";
    }

    return "ERR:UNKNOWN_CMD";
}

// Format: CMD:DRIVE:fwd,strafe,rotate
bool MessageHandler::parseDriveCommand(const String& cmd, float& fwd, float& strafe, float& rotate) {
    String params = cmd.substring(10);
    int i1 = params.indexOf(',');
    int i2 = params.indexOf(',', i1 + 1);
    if (i1 == -1 || i2 == -1) return false;
    fwd = params.substring(0, i1).toFloat();
    strafe = params.substring(i1 + 1, i2).toFloat();
    rotate = params.substring(i2 + 1).toFloat();
    return true;
}

// Format: CMD:DRIVE_TIME:fwd,strafe,rotate,time
bool MessageHandler::parseDriveTimeCommand(const String& cmd, float& fwd, float& strafe, float& rotate, float& time) {
    String params = cmd.substring(15);
    int i1 = params.indexOf(',');
    int i2 = params.indexOf(',', i1 + 1);
    int i3 = params.indexOf(',', i2 + 1);
    if (i1 == -1 || i2 == -1 || i3 == -1) return false;
    fwd = params.substring(0, i1).toFloat();
    strafe = params.substring(i1 + 1, i2).toFloat();
    rotate = params.substring(i2 + 1, i3).toFloat();
    time = params.substring(i3 + 1).toFloat();
    return true;
}

// Format: CMD:DRIVE_DIST:fwd,strafe,rotate,dist
bool MessageHandler::parseDriveDistCommand(const String& cmd, float& fwd, float& strafe, float& rotate, float& dist) {
    String params = cmd.substring(16);
    int i1 = params.indexOf(',');
    int i2 = params.indexOf(',', i1 + 1);
    int i3 = params.indexOf(',', i2 + 1);
    if (i1 == -1 || i2 == -1 || i3 == -1) return false;
    fwd = params.substring(0, i1).toFloat();
    strafe = params.substring(i1 + 1, i2).toFloat();
    rotate = params.substring(i2 + 1, i3).toFloat();
    dist = params.substring(i3 + 1).toFloat();
    return true;
}