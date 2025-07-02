#include "MotionController.h"

MotionController::MotionController(MotorPIDController& pid) : motorPID(pid) {}

void MotionController::setup() {
    stop();
}

void MotionController::drive(float forward, float strafe, float rotate) {
    forwardSpeed = forward;
    strafeSpeed = strafe;
    rotateSpeed = rotate;
}

void MotionController::driveFor(float fwd, float strafe, float rotate, float duration_s) {
    currentCommand.type = MotionType::TIME;
    currentCommand.speedFwd = fwd;
    currentCommand.speedStrafe = strafe;
    currentCommand.speedRotate = rotate;
    currentCommand.duration = duration_s;
    currentCommand.startTime = millis();
    currentCommand.elapsed = 0;
}

void MotionController::driveDistance(float meters, float speedFwd, float speedStrafe, float speedRotate) {
    currentCommand.type = MotionType::DISTANCE;
    currentCommand.distance = meters;
    currentCommand.speedFwd = speedFwd;
    currentCommand.speedStrafe = speedStrafe;
    currentCommand.speedRotate = speedRotate;
    currentCommand.startTime = millis();
    currentCommand.elapsed = 0;
    
    motorPID.resetDistanceTracking();
    drive(speedFwd, speedStrafe, speedRotate);
}

void MotionController::stop() {
    drive(0, 0, 0);
}

void MotionController::applyCommand(const MotionCommand& cmd) {
    switch (cmd.type) {
        case MotionType::TIME:
            driveFor(cmd.speedFwd, cmd.speedStrafe, cmd.speedRotate, cmd.duration);
            break;
        case MotionType::DISTANCE:
            driveDistance(cmd.distance, cmd.speedFwd, cmd.speedStrafe, cmd.speedRotate);
            break;
        default:
            stop();
            break;
    }
}

void MotionController::getTicks(int* outTicks) {
    motorPID.getTicks(outTicks);
}

void MotionController::resetEncoders(){
    return motorPID.resetEncoders();
}

void MotionController::update() {
    if (currentCommand.type == MotionType::TIME) {
        unsigned long now = millis();
        currentCommand.elapsed = (now - currentCommand.startTime) / 1000.0;

        if (currentCommand.elapsed >= currentCommand.duration) {
            stop();
            currentCommand.type = MotionType::NONE;
        } else {
            drive(currentCommand.speedFwd, currentCommand.speedStrafe, currentCommand.speedRotate);
        }
    }

    else if (currentCommand.type == MotionType::DISTANCE) {
        float distanceTravelled = motorPID.getAverageDistanceMeters();

        if (distanceTravelled >= currentCommand.distance) {
            stop();
            currentCommand.type = MotionType::NONE;
        } else {
            // Optional: implement ramp-up / ramp-down logic here
            drive(currentCommand.speedFwd, currentCommand.speedStrafe, currentCommand.speedRotate);
        }
    }

    // Standard mecanum wheel kinematics
    float v0 = forwardSpeed - strafeSpeed - rotateSpeed; // Front-left (motor 0)
    float v1 = forwardSpeed + strafeSpeed + rotateSpeed; // Front-right (motor 1)
    float v2 = forwardSpeed - strafeSpeed + rotateSpeed; // Back-right (motor 2)
    float v3 = forwardSpeed + strafeSpeed - rotateSpeed; // Back-left (motor 3)

    // Convert linear velocities (m/s) into ticks/sec
    float ticks0 = v0 * ticksPerMeter;
    float ticks1 = v1 * ticksPerMeter;
    float ticks2 = v2 * ticksPerMeter;
    float ticks3 = v3 * ticksPerMeter;

    motorPID.setTargetSpeed(0, ticks0);
    motorPID.setTargetSpeed(1, ticks1);
    motorPID.setTargetSpeed(2, ticks2);
    motorPID.setTargetSpeed(3, ticks3);
}