// MotionCommand.h
#ifndef MOTION_COMMAND_H
#define MOTION_COMMAND_H

enum class MotionType { NONE, TIME, DISTANCE };

struct MotionCommand {
    MotionType type = MotionType::NONE;
    float duration = 0.0;         // For TIME mode
    float distance = 0.0;         // For DISTANCE mode
    float elapsed = 0.0;

    float speedFwd = 0.0;
    float speedStrafe = 0.0;
    float speedRotate = 0.0;

    float rampUpDist = 0.0;
    float rampDownDist = 0.0;

    unsigned long startTime = 0;
    float startTicks[4] = {0};    // Optional, for encoder tracking
};

#endif