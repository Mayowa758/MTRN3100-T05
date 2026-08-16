#include <Wire.h>
#include <MPU6050_light.h>
#include <math.h>

#include "DualEncoder.hpp"
#include "GeneratedRoute.h"
#include "Motor.hpp"

#define LEFT_PWM_PIN 11
#define LEFT_DIR_PIN 12
#define RIGHT_PWM_PIN 9
#define RIGHT_DIR_PIN 10
#define LEFT_ENCODER_A 2
#define LEFT_ENCODER_B 7
#define RIGHT_ENCODER_A 3
#define RIGHT_ENCODER_B 8

// Values measured during the Week 8 Part 4 tests.
const float WHEEL_RADIUS_MM = 17.0f;
const float COUNTS_PER_REVOLUTION = 693.0f;

// A 195 mm encoder target produced one 180 mm cell during testing.
const float MAZE_CELL_SIZE_MM = 180.0f;
const float WEEK8_CALIBRATED_CELL_COMMAND_MM = 195.0f;
const float DISTANCE_SCALE =
    WEEK8_CALIBRATED_CELL_COMMAND_MM / MAZE_CELL_SIZE_MM;

const int LEFT_MOTOR_SIGN = 1;
const int RIGHT_MOTOR_SIGN = -1;
const int LEFT_ENCODER_SIGN = 1;
const int RIGHT_ENCODER_SIGN = -1;
const int IMU_CCW_SIGN = 1;

const int MAX_DRIVE_PWM = 85;
const int MEDIUM_DRIVE_PWM = 60;
const int SLOW_DRIVE_PWM = 38;
const int COURSE_BOOST_PWM = 200;
const float COURSE_BOOST_START_MM = 20.0f;
const float COURSE_BOOST_END_MM = 35.0f;
const int MAX_TURN_PWM = 70;
const int MEDIUM_TURN_PWM = 48;
const int SLOW_TURN_PWM = 32;

const float HEADING_KP = 2.0f;
const int MAX_HEADING_CORRECTION = 30;
const float DISTANCE_TOLERANCE_MM = 4.0f;
const float TURN_TOLERANCE_DEG = 3.0f;
const unsigned long SETTLE_MS = 150;
const unsigned long DRIVE_TIMEOUT_MS = 8000;
const unsigned long TURN_TIMEOUT_MS = 6000;

mtrn3100::Motor leftMotor(LEFT_PWM_PIN, LEFT_DIR_PIN);
mtrn3100::Motor rightMotor(RIGHT_PWM_PIN, RIGHT_DIR_PIN);
mtrn3100::DualEncoder encoders(
    LEFT_ENCODER_A, LEFT_ENCODER_B, RIGHT_ENCODER_A, RIGHT_ENCODER_B);
MPU6050 mpu(Wire);

float targetHeadingDeg = 0.0f;

float wrapAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}

float currentHeading() {
    return IMU_CCW_SIGN * mpu.getAngleZ();
}

void stopMotors() {
    leftMotor.setPWM(0);
    rightMotor.setPWM(0);
}

void setWheelPWM(int leftPWM, int rightPWM) {
    leftMotor.setPWM(LEFT_MOTOR_SIGN * constrain(leftPWM, -255, 255));
    rightMotor.setPWM(RIGHT_MOTOR_SIGN * constrain(rightPWM, -255, 255));
}

void setTurnPWM(int ccwPWM) {
    // Positive values turn the robot counter-clockwise.
    setWheelPWM(-ccwPWM, ccwPWM);
}

bool turnRelative(float deltaDeg) {
    targetHeadingDeg = wrapAngle(targetHeadingDeg + deltaDeg);
    unsigned long started = millis();
    unsigned long settledSince = 0;
    while (millis() - started < TURN_TIMEOUT_MS) {
        mpu.update();
        float error = wrapAngle(targetHeadingDeg - currentHeading());
        if (fabs(error) <= TURN_TOLERANCE_DEG) {
            stopMotors();
            if (settledSince == 0) settledSince = millis();
            if (millis() - settledSince >= SETTLE_MS) return true;
        } else {
            settledSince = 0;
            int magnitude = fabs(error) > 35.0f ? MAX_TURN_PWM
                          : fabs(error) > 15.0f ? MEDIUM_TURN_PWM
                                                : SLOW_TURN_PWM;
            setTurnPWM(error >= 0.0f ? magnitude : -magnitude);
        }
        delay(10);
    }
    stopMotors();
    return false;
}

float countDifferenceToMm(long countDifference, int encoderSign) {
    float radians = encoderSign * countDifference * 2.0f * PI / COUNTS_PER_REVOLUTION;
    return radians * WHEEL_RADIUS_MM;
}

bool driveDistance(float requestedMm, bool courseMode) {
    float targetMm = requestedMm * DISTANCE_SCALE;
    long leftStart = encoders.leftCount();
    long rightStart = encoders.rightCount();
    unsigned long started = millis();
    unsigned long settledSince = 0;
    while (millis() - started < DRIVE_TIMEOUT_MS) {
        mpu.update();
        float leftMm = countDifferenceToMm(encoders.leftCount() - leftStart, LEFT_ENCODER_SIGN);
        float rightMm = countDifferenceToMm(encoders.rightCount() - rightStart, RIGHT_ENCODER_SIGN);
        float travelledMm = (leftMm + rightMm) * 0.5f;
        float distanceError = targetMm - travelledMm;
        float headingError = wrapAngle(targetHeadingDeg - currentHeading());
        if (fabs(distanceError) <= DISTANCE_TOLERANCE_MM) {
            stopMotors();
            if (settledSince == 0) settledSince = millis();
            if (millis() - settledSince >= SETTLE_MS) return true;
        } else {
            settledSince = 0;
            bool inCourseBoostSection =
                courseMode &&
                travelledMm >= COURSE_BOOST_START_MM &&
                distanceError >= COURSE_BOOST_END_MM;
            int magnitude = inCourseBoostSection
                ? COURSE_BOOST_PWM
                : fabs(distanceError) > 120.0f ? MAX_DRIVE_PWM
                : fabs(distanceError) > 45.0f ? MEDIUM_DRIVE_PWM
                                             : SLOW_DRIVE_PWM;
            int direction = distanceError >= 0.0f ? 1 : -1;
            int correction = constrain(
                static_cast<int>(HEADING_KP * headingError),
                -MAX_HEADING_CORRECTION,
                MAX_HEADING_CORRECTION);
            setWheelPWM(direction * magnitude - correction,
                        direction * magnitude + correction);
        }
        delay(10);
    }
    stopMotors();
    return false;
}

bool runStandardCommands(const char* commands, bool courseMode) {
    for (unsigned int index = 0; commands[index] != '\0'; ++index) {
        bool success = false;
        if (commands[index] == 'f') success = driveDistance(MAZE_CELL_SIZE_MM, courseMode);
        else if (commands[index] == 'l') success = turnRelative(90.0f);
        else if (commands[index] == 'r') success = turnRelative(-90.0f);
        if (!success) return false;
    }
    return true;
}

void setup() {
    Wire.begin();
    stopMotors();
    if (mpu.begin() != 0) {
        return;
    }
    delay(1000);
    mpu.calcOffsets(true, true);
    delay(500);
    mpu.update();
    targetHeadingDeg = currentHeading();
    delay(2000);

    bool success = runStandardCommands(PRE_COMMANDS, false);
    if (success) success = runStandardCommands(COURSE_COMMANDS, true);
    if (success) success = runStandardCommands(POST_COMMANDS, false);
    stopMotors();
}

void loop() {
    stopMotors();
}
