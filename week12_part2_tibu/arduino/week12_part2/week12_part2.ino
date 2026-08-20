#include <Wire.h>
#include <MPU6050_light.h>
#include <math.h>

#include "DualEncoder.hpp"
#include "GeneratedRoute.h"
#include "Lidar.hpp"
#include "Motor.hpp"

#define LEFT_PWM_PIN 11
#define LEFT_DIR_PIN 12
#define RIGHT_PWM_PIN 9
#define RIGHT_DIR_PIN 10
#define LEFT_ENCODER_A 2
#define LEFT_ENCODER_B 7
#define RIGHT_ENCODER_A 3
#define RIGHT_ENCODER_B 8

#define LEFT_LIDAR_XSHUT A0
#define RIGHT_LIDAR_XSHUT A1
#define FRONT_LIDAR_XSHUT A2
#define LEFT_LIDAR_ADDRESS 0x30
#define RIGHT_LIDAR_ADDRESS 0x31
#define FRONT_LIDAR_ADDRESS 0x32

const float WHEEL_RADIUS_MM = 17.0f;
const float COUNTS_PER_REVOLUTION = 693.0f;

const float MAZE_CELL_SIZE_MM = 180.0f;
const float GRID_CELL_COMMAND_MM = 185.0f;
const float COURSE_DISTANCE_SCALE = GRID_CELL_COMMAND_MM / MAZE_CELL_SIZE_MM;

const int LEFT_MOTOR_SIGN = 1;
const int RIGHT_MOTOR_SIGN = -1;
const int LEFT_ENCODER_SIGN = 1;
const int RIGHT_ENCODER_SIGN = -1;
const int IMU_CCW_SIGN = 1;

const int GRID_FORWARD_PWM = 220;
const int COURSE_MAX_PWM = 100;
const int COURSE_MEDIUM_PWM = 70;
const int COURSE_SLOW_PWM = 45;
const int COURSE_BOOST_PWM = 200;
const float COURSE_BOOST_START_MM = 20.0f;
const float COURSE_BOOST_END_MM = 35.0f;
const int MAX_TURN_PWM = 150;
const int MEDIUM_TURN_PWM = 80;
const int SLOW_TURN_PWM = 50;

const float HEADING_KP = 2.0f;
const int MAX_HEADING_CORRECTION = 25;
const float DISTANCE_TOLERANCE_MM = 3.0f;
const float TURN_TOLERANCE_DEG = 3.0f;
const unsigned long SETTLE_MS = 60;
const unsigned long GRID_DRIVE_TIMEOUT_MS = 3000;
const unsigned long COURSE_DRIVE_TIMEOUT_MS = 8000;
const unsigned long TURN_TIMEOUT_MS = 6000;

const int SIDE_CENTRE_MIN_MM = 25;
const int SIDE_CENTRE_MAX_MM = 120;
const int SIDE_CENTRE_DEADBAND_MM = 12;
const int SIDE_CENTRE_CORRECTION = 4;
const int SIDE_EMERGENCY_MM = 20;
const int SIDE_EMERGENCY_CORRECTION = 5;
const int SIDE_EMERGENCY_CONFIRMATIONS = 4;
const unsigned long SIDE_NUDGE_DURATION_MS = 50;
const unsigned long SIDE_NUDGE_COOLDOWN_MS = 500;
const int FRONT_WALL_END_MM = 40;
const unsigned long LIDAR_UPDATE_MS = 45;
const bool PRINT_LIDAR_READINGS = false;

mtrn3100::Motor leftMotor(LEFT_PWM_PIN, LEFT_DIR_PIN);
mtrn3100::Motor rightMotor(RIGHT_PWM_PIN, RIGHT_DIR_PIN);
mtrn3100::DualEncoder encoders(
    LEFT_ENCODER_A, LEFT_ENCODER_B, RIGHT_ENCODER_A, RIGHT_ENCODER_B);
MPU6050 mpu(Wire);
mtrn3100::Lidar leftLidar;
mtrn3100::Lidar rightLidar;
mtrn3100::Lidar frontLidar;

float targetHeadingDeg = 0.0f;
int leftCloseCount = 0;
int rightCloseCount = 0;
int activeSideNudge = 0;
unsigned long sideNudgeEndsAt = 0;
unsigned long sideNudgeCooldownEndsAt = 0;

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

bool initialiseLidar(mtrn3100::Lidar &lidar, uint8_t xshutPin, uint8_t address) {
    digitalWrite(xshutPin, HIGH);
    delay(20);
    return lidar.init(address);
}

bool startLidars() {
    pinMode(LEFT_LIDAR_XSHUT, OUTPUT);
    pinMode(RIGHT_LIDAR_XSHUT, OUTPUT);
    pinMode(FRONT_LIDAR_XSHUT, OUTPUT);
    digitalWrite(LEFT_LIDAR_XSHUT, LOW);
    digitalWrite(RIGHT_LIDAR_XSHUT, LOW);
    digitalWrite(FRONT_LIDAR_XSHUT, LOW);
    delay(20);

    if (!initialiseLidar(leftLidar, LEFT_LIDAR_XSHUT, LEFT_LIDAR_ADDRESS)) return false;
    if (!initialiseLidar(rightLidar, RIGHT_LIDAR_XSHUT, RIGHT_LIDAR_ADDRESS)) return false;
    if (!initialiseLidar(frontLidar, FRONT_LIDAR_XSHUT, FRONT_LIDAR_ADDRESS)) return false;
    return true;
}

int readLidarMM(mtrn3100::Lidar &lidar) {
    uint16_t distance = lidar.readDistance();
    return lidar.isLastReadValid() ? distance : -1;
}

void resetLidarCorrection() {
    leftCloseCount = 0;
    rightCloseCount = 0;
    activeSideNudge = 0;
    sideNudgeEndsAt = 0;
    sideNudgeCooldownEndsAt = 0;
}

int calculateLidarCorrection(int leftMM, int rightMM) {
    const bool leftValid = leftMM >= 0;
    const bool rightValid = rightMM >= 0;
    if (leftValid && rightValid &&
        leftMM >= SIDE_CENTRE_MIN_MM && leftMM <= SIDE_CENTRE_MAX_MM &&
        rightMM >= SIDE_CENTRE_MIN_MM && rightMM <= SIDE_CENTRE_MAX_MM) {
        int sideError = leftMM - rightMM;
        if (abs(sideError) > SIDE_CENTRE_DEADBAND_MM) {
            return constrain(
                sideError / 6,
                -SIDE_CENTRE_CORRECTION,
                SIDE_CENTRE_CORRECTION);
        }
    }

    if ((long)(millis() - sideNudgeCooldownEndsAt) < 0) return 0;

    const bool leftClose = leftValid && leftMM <= SIDE_EMERGENCY_MM;
    const bool rightClose = rightValid && rightMM <= SIDE_EMERGENCY_MM;
    leftCloseCount = leftClose ? leftCloseCount + 1 : 0;
    rightCloseCount = rightClose ? rightCloseCount + 1 : 0;
    const bool leftConfirmed = leftCloseCount >= SIDE_EMERGENCY_CONFIRMATIONS;
    const bool rightConfirmed = rightCloseCount >= SIDE_EMERGENCY_CONFIRMATIONS;

    if (leftConfirmed && rightConfirmed) {
        if (leftMM > rightMM) activeSideNudge = SIDE_EMERGENCY_CORRECTION;
        else if (rightMM > leftMM) activeSideNudge = -SIDE_EMERGENCY_CORRECTION;
    } else if (leftConfirmed) {
        activeSideNudge = -SIDE_EMERGENCY_CORRECTION;
    } else if (rightConfirmed) {
        activeSideNudge = SIDE_EMERGENCY_CORRECTION;
    }

    if (activeSideNudge != 0) {
        sideNudgeEndsAt = millis() + SIDE_NUDGE_DURATION_MS;
        leftCloseCount = 0;
        rightCloseCount = 0;
        return activeSideNudge;
    }
    return 0;
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

bool driveDistance(float requestedMm, bool gridMode) {
    float targetMm = gridMode ? requestedMm : requestedMm * COURSE_DISTANCE_SCALE;
    long leftStart = encoders.leftCount();
    long rightStart = encoders.rightCount();
    unsigned long started = millis();
    unsigned long lastLidarUpdate = 0;
    int lidarCorrection = 0;
    int frontMM = -1;
    resetLidarCorrection();
    const unsigned long timeout = gridMode
        ? GRID_DRIVE_TIMEOUT_MS
        : COURSE_DRIVE_TIMEOUT_MS;

    while (millis() - started < timeout) {
        mpu.update();

        if (gridMode && activeSideNudge != 0 &&
            (long)(millis() - sideNudgeEndsAt) >= 0) {
            activeSideNudge = 0;
            lidarCorrection = 0;
            sideNudgeCooldownEndsAt = millis() + SIDE_NUDGE_COOLDOWN_MS;
        }

        if (gridMode && millis() - lastLidarUpdate >= LIDAR_UPDATE_MS) {
            int leftMM = readLidarMM(leftLidar);
            int rightMM = readLidarMM(rightLidar);
            frontMM = readLidarMM(frontLidar);
            lidarCorrection = calculateLidarCorrection(leftMM, rightMM);
            lastLidarUpdate = millis();

            if (PRINT_LIDAR_READINGS) {
                Serial.print("L: "); Serial.print(leftMM);
                Serial.print(" R: "); Serial.print(rightMM);
                Serial.print(" F: "); Serial.print(frontMM);
                Serial.print(" correction: "); Serial.println(lidarCorrection);
            }
        }

        float leftMm = countDifferenceToMm(encoders.leftCount() - leftStart, LEFT_ENCODER_SIGN);
        float rightMm = countDifferenceToMm(encoders.rightCount() - rightStart, RIGHT_ENCODER_SIGN);
        float travelledMm = (leftMm + rightMm) * 0.5f;
        float distanceError = targetMm - travelledMm;
        float headingError = wrapAngle(targetHeadingDeg - currentHeading());

        if (gridMode && frontMM >= 0 && frontMM <= FRONT_WALL_END_MM) {
            stopMotors();
            Serial.print("Front wall ended grid move at ");
            Serial.print(frontMM);
            Serial.println(" mm");
            return true;
        }

        if (distanceError <= DISTANCE_TOLERANCE_MM) {
            stopMotors();
            Serial.print(gridMode ? "Grid move complete: " : "Course move complete: ");
            Serial.print(travelledMm);
            Serial.println(" mm");
            return true;
        }

        int magnitude;
        if (gridMode) {
            magnitude = GRID_FORWARD_PWM;
        } else {
            bool inCourseBoostSection =
                travelledMm >= COURSE_BOOST_START_MM &&
                distanceError >= COURSE_BOOST_END_MM;
            magnitude = inCourseBoostSection
                ? COURSE_BOOST_PWM
                : distanceError > 120.0f ? COURSE_MAX_PWM
                : distanceError > 45.0f ? COURSE_MEDIUM_PWM
                                        : COURSE_SLOW_PWM;
        }
        int imuCorrection = constrain(
            static_cast<int>(HEADING_KP * headingError),
            -MAX_HEADING_CORRECTION,
            MAX_HEADING_CORRECTION);
        int totalCorrection = constrain(
            imuCorrection + (gridMode ? lidarCorrection : 0),
            -MAX_HEADING_CORRECTION,
            MAX_HEADING_CORRECTION);
        int leftPWM = magnitude;
        int rightPWM = magnitude;
        if (totalCorrection > 0) leftPWM -= 2 * totalCorrection;
        else if (totalCorrection < 0) rightPWM += 2 * totalCorrection;
        setWheelPWM(leftPWM, rightPWM);
        delay(5);
    }
    stopMotors();
    Serial.println(gridMode ? "Grid move timeout" : "Course move timeout");
    return false;
}

bool runStandardCommands(const char* commands) {
    for (unsigned int index = 0; commands[index] != '\0'; ++index) {
        bool success = false;
        Serial.print("Grid command: ");
        Serial.println(commands[index]);
        if (commands[index] == 'f') success = driveDistance(GRID_CELL_COMMAND_MM, true);
        else if (commands[index] == 'l') success = turnRelative(90.0f);
        else if (commands[index] == 'r') success = turnRelative(-90.0f);
        if (!success) return false;
    }
    return true;
}

bool runCourseMotions() {
    for (unsigned int index = 0; index < COURSE_MOTION_COUNT; ++index) {
        Serial.print("Course motion ");
        Serial.print(index + 1);
        Serial.print(": turn ");
        Serial.print(COURSE_MOTIONS[index].turnDeg);
        Serial.print(" deg, drive ");
        Serial.print(COURSE_MOTIONS[index].distanceMm);
        Serial.println(" mm");
        if (fabs(COURSE_MOTIONS[index].turnDeg) > 0.1f &&
            !turnRelative(COURSE_MOTIONS[index].turnDeg)) {
            return false;
        }
        if (COURSE_MOTIONS[index].distanceMm > 0.1f &&
            !driveDistance(COURSE_MOTIONS[index].distanceMm, false)) {
            return false;
        }
    }
    return true;
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    stopMotors();
    delay(1000);
    if (!startLidars()) {
        Serial.println("LiDAR initialisation failed");
        return;
    }
    if (mpu.begin() != 0) {
        Serial.println("IMU connection failed");
        return;
    }
    Serial.println("Keep robot completely still during calibration");
    delay(1000);
    mpu.calcOffsets(true, true);
    delay(500);
    mpu.update();
    targetHeadingDeg = currentHeading();
    delay(2000);

    Serial.println("Running PRE_COMMANDS with LiDAR enabled");
    bool success = runStandardCommands(PRE_COMMANDS);
    Serial.println("Running COURSE_MOTIONS with LiDAR steering disabled");
    if (success) success = runCourseMotions();
    Serial.println("Running POST_COMMANDS with LiDAR enabled");
    if (success) success = runStandardCommands(POST_COMMANDS);
    stopMotors();
    Serial.println(success ? "Route complete" : "Route failed");
}

void loop() {
    stopMotors();
}
