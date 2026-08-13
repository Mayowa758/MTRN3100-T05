#pragma once

#include <Arduino.h>
#include "Motor.hpp"
#include "DualEncoder.hpp"
#include <MPU6050_light.h>

class Robot {
public:

    Robot(
        mtrn3100::Motor& leftMotor,
        mtrn3100::Motor& rightMotor,
        mtrn3100::DualEncoder& encoder,
        MPU6050& mpu
    )
        : motorL(leftMotor),
          motorR(rightMotor),
          encoder(encoder),
          mpu(mpu)
    {}

    // ---------------- ROBOT SETTINGS ----------------

    const float CELL_DISTANCE_M = 0.180;
    const float WHEEL_RADIUS_M = 0.016;

    const int FORWARD_PWM = 85;
    const int MAX_FORWARD_CORRECTION = 35;
    const float HEADING_KP = 2.0;

    const int MAX_TURN_PWM = 100;
    const int MEDIUM_TURN_PWM = 60;
    const int SLOW_TURN_PWM = 30;

    const float TURN_TOLERANCE = 5.0;
    const float DISTANCE_TOLERANCE = 0.15;

    const unsigned long DRIVE_TIMEOUT = 6000;
    const unsigned long TURN_TIMEOUT = 6000;
    const unsigned long SETTLE_TIME = 150;


    // ---------------- MOVEMENT ----------------

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

                if (fabs(error) > 35.0) {

                    turnPWM = MAX_TURN_PWM;

                } else if (fabs(error) > 15.0) {

                    turnPWM = MEDIUM_TURN_PWM;

                } else {

                    turnPWM = SLOW_TURN_PWM;
                }

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


    // ---------------- HEADING ----------------

    float wrapAngle(float angle) {

        while (angle > 180.0) {
            angle -= 360.0;
        }

        while (angle < -180.0) {
            angle += 360.0;
        }

        return angle;
    }


    // ---------------- MOTOR CONTROL ----------------

    void setForwardPWM(int leftPWM, int rightPWM) {

        leftPWM = constrain(
            leftPWM,
            -120,
            120
        );

        rightPWM = constrain(
            rightPWM,
            -120,
            120
        );

        motorL.setPWM(leftPWM);
        motorR.setPWM(-rightPWM);
    }


    void setTurnPWM(int pwm) {

        pwm = constrain(
            pwm,
            -MAX_TURN_PWM,
            MAX_TURN_PWM
        );

        motorL.setPWM(-pwm);
        motorR.setPWM(-pwm);
    }


    void stopMotors() {

        motorL.setPWM(0);
        motorR.setPWM(0);
    }


    // ---------------- HEADING ACCESS ----------------

    void setTargetHeading(float heading) {
        targetHeading = heading;
    }

    float getTargetHeading() {
        return targetHeading;
    }


private:

    mtrn3100::Motor& motorL;
    mtrn3100::Motor& motorR;

    mtrn3100::DualEncoder& encoder;

    MPU6050& mpu;

    float targetHeading = 0;
};