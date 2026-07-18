#include "Motor.hpp"
#include "DualEncoder.hpp"
#include "PIDController.hpp"
#include <MPU6050_light.h>
#define MOT1PWM 11
#define MOT1DIR 12
mtrn3100::Motor motor1(MOT1PWM, MOT1DIR);

#define MOT2PWM 9
#define MOT2DIR 10
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);

#define EN_A  2
#define EN_B  7
#define EN_A2 3
#define EN_B2 8
mtrn3100::DualEncoder encoder(EN_A, EN_B, EN_A2, EN_B2);
mtrn3100::PIDController leftDrive(30, 0, 0.3);
mtrn3100::PIDController rightDrive(30, 0, 0.3);
mtrn3100::PIDController headingHold(1.5f, 0.0f, 0.05f);

MPU6050 mpu(Wire);
const float WHEEL_RADIUS   = 0.016f;
const float CELL_SIZE      = 0.18f;
const float K_HEADING      = 27.0f;
const float K_SYNC         = 1.1f;

// 容差/TOLERANCE
const float DRIVE_TOLERANCE = 0.05f;
const float TURN_TOLERANCE  = 3.0f;

// PWM调节
const int16_t MAX_PWM       = 150;
const int16_t MIN_DRIVE_PWM = 45;
const int16_t MIN_TURN_PWM  = 50;

const unsigned long DRIVE_SETTLE_TIME_MS = 200;
const unsigned long TURN_SETTLE_TIME_MS = 250;

const int MOTOR2_TURN_SIGN = -1;

const float TURN_ANGLE_DEG = 90.0f;

const float TURN_SIGN = -1.0f;

const char* commandString = "lfrfflfr";    //change to test

float targetHeading = 0.0f;
bool ready = false;
bool sequenceFailed = false;

float applyMinimumPWM(float command, int16_t minimumPWM) {
    if (command > 0.0f && command < minimumPWM) return minimumPWM;
    if (command < 0.0f && command > -minimumPWM) return -minimumPWM;
    return command;
}

bool driveForwardCell();
bool turnRelative(float deltaDeg);

void stopMotors() {
    motor1.setPWM(0);
    motor2.setPWM(0);
}

bool commandStringIsValid() {
    if (strlen(commandString) != 8) return false;
    for (size_t i = 0; i < 8; ++i) {
        if (commandString[i] != 'f' && commandString[i] != 'l' && commandString[i] != 'r') {
            return false;
        }
    }
    return true;
}

void setup() {
    Wire.begin();

    byte imuStatus = mpu.begin();
    if (imuStatus != 0) {
        stopMotors();
        return;
    }
    mpu.calcOffsets(true, true);
    delay(500);
    mpu.update();

    if (!commandStringIsValid()) {
        stopMotors();
        return;
    }

    targetHeading = mpu.getAngleZ();
    ready = true;
}

void loop() {
    static size_t index = 0;

    if (!ready || sequenceFailed) {
        stopMotors();
        return;
    }

    if (commandString[index] == '\0') {
        stopMotors();
        return;
    }

    char cmd = commandString[index];

    bool actionSucceeded = false;
    switch (cmd) {
        case 'f':
            actionSucceeded = driveForwardCell();
            break;
        case 'l':
            actionSucceeded = turnRelative(-TURN_SIGN * TURN_ANGLE_DEG);
            break;
        case 'r':
            actionSucceeded = turnRelative(TURN_SIGN * TURN_ANGLE_DEG);
            break;
        default:
            actionSucceeded = false;
            break;
    }

    if (!actionSucceeded) {
        sequenceFailed = true;
        stopMotors();
        return;
    }

    index++;
}

bool driveForwardCell() {
    float targetAngle = CELL_SIZE / WHEEL_RADIUS;

    leftDrive.zeroAndSetTarget(encoder.getLeftRotation(), targetAngle);
    rightDrive.zeroAndSetTarget(-encoder.getRightRotation(), targetAngle);

    unsigned long lastTime = micros();
    const unsigned long interval = 10000;
    unsigned long moveStart = millis();
    const unsigned long DRIVE_TIMEOUT_MS = 3000;
    unsigned long withinToleranceSince = 0;

    while (true) {
        mpu.update();
        unsigned long now = micros();
        if (now - lastTime < interval) continue;
        lastTime = now;

        float leftPos  = encoder.getLeftRotation();
        float rightPos = -encoder.getRightRotation();

        float leftPWM  = leftDrive.compute(leftPos);
        float rightPWM = rightDrive.compute(rightPos);

        float headingError = targetHeading - mpu.getAngleZ();
        float headingCorrection = constrain(K_HEADING * headingError, -60.0f, 60.0f);

        float syncError = leftPos - rightPos;
        float syncCorrection = syncError * K_SYNC;
        float leftCmd  = constrain(leftPWM - headingCorrection - syncCorrection, (float)-MAX_PWM, (float)MAX_PWM);
        float rightCmd = constrain(-(rightPWM + headingCorrection + syncCorrection), (float)-MAX_PWM, (float)MAX_PWM);
        leftCmd = applyMinimumPWM(leftCmd, MIN_DRIVE_PWM);
        rightCmd = applyMinimumPWM(rightCmd, MIN_DRIVE_PWM);
        motor1.setPWM(leftCmd);
        motor2.setPWM(rightCmd);

        bool withinTolerance = fabs(leftDrive.getError()) < DRIVE_TOLERANCE &&
                               fabs(rightDrive.getError()) < DRIVE_TOLERANCE;
        if (withinTolerance) {
            stopMotors();
            if (withinToleranceSince == 0) withinToleranceSince = millis();
            if (millis() - withinToleranceSince >= DRIVE_SETTLE_TIME_MS) {
                return true;
            }
        } else {
            withinToleranceSince = 0;
        }

        if (millis() - moveStart >= DRIVE_TIMEOUT_MS) {
            stopMotors();
            return false;
        }
    }
}

bool turnRelative(float deltaDeg) {
    targetHeading += deltaDeg;
    headingHold.zeroAndSetTarget(0.0f, targetHeading);

    unsigned long turnStart = millis();
    const unsigned long TURN_TIMEOUT_MS = 2000;
    unsigned long withinToleranceSince = 0;
    unsigned long lastControlTime = micros();
    const unsigned long CONTROL_INTERVAL_US = 10000;

    while (true) {
        mpu.update();

        unsigned long now = micros();
        if (now - lastControlTime < CONTROL_INTERVAL_US) continue;
        lastControlTime = now;

        float headingError = targetHeading - mpu.getAngleZ();
        if (fabs(headingError) < TURN_TOLERANCE) {
            stopMotors();
            if (withinToleranceSince == 0) withinToleranceSince = millis();
            if (millis() - withinToleranceSince >= TURN_SETTLE_TIME_MS) {
                return true;
            }
        } else {
            withinToleranceSince = 0;
            float output = headingHold.compute(mpu.getAngleZ());
            output = constrain(output, (float)-MAX_PWM, (float)MAX_PWM);
            output = applyMinimumPWM(output, MIN_TURN_PWM);
            motor1.setPWM(output);
            motor2.setPWM(MOTOR2_TURN_SIGN * output);
        }

        if (millis() - turnStart >= TURN_TIMEOUT_MS) {
            stopMotors();
            return false;
        }
    }
}
