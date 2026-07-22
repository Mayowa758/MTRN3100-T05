/*
 * MTRN3100 Week 8 - Part 4: Chaining Movements
 *
 * Generative AI assistance was used in developing this code.
 *
 * Commands:
 * f = forward one 180 mm cell
 * l = turn left 90 degrees
 * r = turn right 90 degrees
 */

#include <Wire.h>
#include <MPU6050_light.h>
#include "Motor.hpp"
#include "DualEncoder.hpp"

// ---------------- PINS ----------------

#define MOT1PWM 11
#define MOT1DIR 12
#define MOT2PWM 9
#define MOT2DIR 10

#define ENC1A 2
#define ENC1B 7
#define ENC2A 3
#define ENC2B 8

// ---------------- OBJECTS ----------------

mtrn3100::Motor motor1(MOT1PWM, MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);

mtrn3100::DualEncoder encoder(
    ENC1A,
    ENC1B,
    ENC2A,
    ENC2B
);

MPU6050 mpu(Wire);

// ---------------- COMMAND STRING ----------------

// Replace this on marking day.
const char commandString[] = "fflfrflf";

// ---------------- ROBOT SETTINGS ----------------

const float CELL_DISTANCE_M = 0.195;
const float WHEEL_RADIUS_M = 0.017;

// Forward movement settings
const int FORWARD_PWM = 85;
const int MAX_FORWARD_CORRECTION = 35;
const float HEADING_KP = 2.0;

// Turning settings
const int MAX_TURN_PWM = 70;
const int MEDIUM_TURN_PWM = 48;
const int SLOW_TURN_PWM = 30;

const float TURN_TOLERANCE = 5.0;

// Distance tolerance
const float DISTANCE_TOLERANCE = 0.15;

// Time settings
const unsigned long DRIVE_TIMEOUT = 6000;
const unsigned long TURN_TIMEOUT = 6000;
const unsigned long SETTLE_TIME = 150;

// ---------------- GLOBAL VARIABLES ----------------

float targetHeading = 0;

bool systemReady = false;
bool sequenceFailed = false;

int commandIndex = 0;

// ---------------- BASIC FUNCTIONS ----------------

void stopMotors() {
    motor1.setPWM(0);
    motor2.setPWM(0);
}

float wrapAngle(float angle) {
    while (angle > 180.0) {
        angle -= 360.0;
    }

    while (angle < -180.0) {
        angle += 360.0;
    }

    return angle;
}

/*
 * Forward movement:
 *
 * Motor 2 uses the opposite sign because the two motors
 * are mounted in opposite physical directions.
 */
void setForwardPWM(int leftPWM, int rightPWM) {
    leftPWM = constrain(leftPWM, -120, 120);
    rightPWM = constrain(rightPWM, -120, 120);

    motor1.setPWM(leftPWM);
    motor2.setPWM(-rightPWM);
}

/*
 * In-place turning:
 *
 * Both motors receive the same sign because the motors
 * are mounted in opposite directions.
 */
void setTurnPWM(int pwm) {
    pwm = constrain(
        pwm,
        -MAX_TURN_PWM,
        MAX_TURN_PWM
    );

    motor1.setPWM(-pwm);
    motor2.setPWM(-pwm);
}

// ---------------- FORWARD MOVEMENT ----------------

bool driveForwardOneCell() {
    float targetRotation =
        CELL_DISTANCE_M / WHEEL_RADIUS_M;

    float leftStart =
        encoder.getLeftRotation();

    float rightStart =
        -encoder.getRightRotation();

    unsigned long startTime = millis();
    unsigned long settledSince = 0;

    while (millis() - startTime < DRIVE_TIMEOUT) {
        mpu.update();

        float leftTravel =
            encoder.getLeftRotation() - leftStart;

        float rightTravel =
            (-encoder.getRightRotation()) - rightStart;

        float averageTravel =
            (leftTravel + rightTravel) / 2.0;

        float distanceError =
            targetRotation - averageTravel;

        float headingError =
            wrapAngle(
                targetHeading - mpu.getAngleZ()
            );

        int headingCorrection =
            constrain(
                (int)(HEADING_KP * headingError),
                -MAX_FORWARD_CORRECTION,
                MAX_FORWARD_CORRECTION
            );

        if (fabs(distanceError) <= DISTANCE_TOLERANCE) {
            stopMotors();

            if (settledSince == 0) {
                settledSince = millis();
            }

            if (millis() - settledSince >= SETTLE_TIME) {
                Serial.println("Forward complete");
                return true;
            }
        } else {
            settledSince = 0;

            int leftPWM =
                FORWARD_PWM - headingCorrection;

            int rightPWM =
                FORWARD_PWM + headingCorrection;

            setForwardPWM(
                leftPWM,
                rightPWM
            );
        }

        delay(10);
    }

    stopMotors();

    Serial.println("Forward timeout");

    return false;
}

// ---------------- TURNING ----------------

bool turnToHeading(float newHeading) {
    targetHeading = newHeading;

    unsigned long startTime = millis();
    unsigned long settledSince = 0;

    while (millis() - startTime < TURN_TIMEOUT) {
        mpu.update();

        float currentHeading =
            mpu.getAngleZ();

        float error =
            wrapAngle(
                targetHeading - currentHeading
            );

        Serial.print("Heading: ");
        Serial.print(currentHeading);

        Serial.print(" Target: ");
        Serial.print(targetHeading);

        Serial.print(" Error: ");
        Serial.println(error);

        /*
         * Stop when the robot is within ±5 degrees.
         */
        if (fabs(error) <= TURN_TOLERANCE) {
            stopMotors();

            if (settledSince == 0) {
                settledSince = millis();
            }

            if (millis() - settledSince >= SETTLE_TIME) {
                Serial.println("Turn complete");

                return true;
            }
        } else {
            settledSince = 0;

            int turnPWM;

            /*
             * Fast when far away,
             * medium when closer,
             * slow near target.
             */
            if (fabs(error) > 35.0) {
                turnPWM = MAX_TURN_PWM;
            } else if (fabs(error) > 15.0) {
                turnPWM = MEDIUM_TURN_PWM;
            } else {
                turnPWM = SLOW_TURN_PWM;
            }

            /*
             * Choose direction using the error sign.
             */
            if (error < 0) {
                turnPWM = -turnPWM;
            }

            setTurnPWM(turnPWM);
        }

        delay(10);
    }

    stopMotors();

    Serial.println("Turn timeout");

    return false;
}

bool turnLeft90() {
    return turnToHeading(
        targetHeading + 90.0
    );
}

bool turnRight90() {
    return turnToHeading(
        targetHeading - 90.0
    );
}

// ---------------- SETUP ----------------

void setup() {
    Serial.begin(9600);

    Wire.begin();

    stopMotors();

    Serial.println("Starting Part 4");

    delay(1000);

    byte imuStatus =
        mpu.begin();

    Serial.print("IMU status: ");
    Serial.println(imuStatus);

    if (imuStatus != 0) {
        Serial.println("IMU connection failed");

        return;
    }

    Serial.println(
        "Keep robot completely still during calibration"
    );

    delay(1000);

    mpu.calcOffsets(true, true);

    delay(500);

    mpu.update();

    targetHeading =
        mpu.getAngleZ();

    Serial.print("Initial heading: ");
    Serial.println(targetHeading);

    Serial.print("Commands: ");
    Serial.println(commandString);

    /*
     * Time to place the robot down.
     */
    delay(2000);

    systemReady = true;
}

// ---------------- MAIN LOOP ----------------

void loop() {
    if (!systemReady || sequenceFailed) {
        stopMotors();

        return;
    }

    if (commandIndex >= 8) {
        stopMotors();

        return;
    }

    char command =
        commandString[commandIndex];

    Serial.print("Running command ");
    Serial.print(commandIndex + 1);
    Serial.print(": ");
    Serial.println(command);

    bool success = false;

    if (command == 'f') {
        success =
            driveForwardOneCell();
    } else if (command == 'l') {
        success =
            turnLeft90();
    } else if (command == 'r') {
        success =
            turnRight90();
    } else {
        Serial.println("Invalid command");

        sequenceFailed = true;

        stopMotors();

        return;
    }

    stopMotors();

    if (!success) {
        Serial.println("Sequence stopped");

        sequenceFailed = true;

        return;
    }

    commandIndex++;

    delay(200);

    if (commandIndex >= 8) {
        Serial.println("All 8 commands completed");

        stopMotors();
    }
}
