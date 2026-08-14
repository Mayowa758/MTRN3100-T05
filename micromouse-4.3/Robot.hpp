#pragma once
#include <Arduino.h>
#include "Motor.hpp"
#include "DualEncoder.hpp"
#include "Lidar.hpp"
#include <MPU6050_light.h>

constexpr float CELL_DISTANCE_M = 0.185;
constexpr float WHEEL_RADIUS_M = 0.017;

constexpr int FORWARD_PWM = 100;
constexpr int MAX_FORWARD_CORRECTION = 35;
constexpr float HEADING_KP = 2.0;

constexpr int MAX_TURN_PWM = 70;
constexpr int MEDIUM_TURN_PWM = 48;
constexpr int SLOW_TURN_PWM = 30;

constexpr float TURN_TOLERANCE = 2.0;
constexpr float DISTANCE_TOLERANCE = 0.15;

constexpr unsigned long DRIVE_TIMEOUT = 6000;
constexpr unsigned long TURN_TIMEOUT = 6000;
constexpr unsigned long SETTLE_TIME = 50;

constexpr int SIDE_CENTRE_MIN_MM = 25;
constexpr int SIDE_CENTRE_MAX_MM = 120;
constexpr int SIDE_CENTRE_DEADBAND_MM = 12;
constexpr int SIDE_CENTRE_CORRECTION = 4;

constexpr int SIDE_EMERGENCY_MM = 15;
constexpr int SIDE_EMERGENCY_CORRECTION = 5;
constexpr int SIDE_EMERGENCY_CONFIRMATIONS = 4;

constexpr unsigned long SIDE_NUDGE_DURATION_MS = 50;
constexpr unsigned long SIDE_NUDGE_COOLDOWN_MS = 500;

constexpr int FRONT_WALL_END_MM = 15;
constexpr unsigned long LIDAR_UPDATE_MS = 45;

class Robot {
public:
    Robot(
        mtrn3100::Motor& leftMotor,
        mtrn3100::Motor& rightMotor,
        mtrn3100::DualEncoder& encoder,
        MPU6050& mpu,
        mtrn3100::Lidar& leftLidar,
        mtrn3100::Lidar& rightLidar,
        mtrn3100::Lidar& frontLidar
    )
        : motorL(leftMotor),
          motorR(rightMotor),
          encoder(encoder),
          mpu(mpu),
          leftLidar(leftLidar),
          rightLidar(rightLidar),
          frontLidar(frontLidar)
    {}

    bool driveForwardOneCell() {
        float targetRotation = CELL_DISTANCE_M / WHEEL_RADIUS_M;
        float leftStart = encoder.getLeftRotation();
        float rightStart = -encoder.getRightRotation();
        unsigned long startTime = millis();
        unsigned long settledSince = 0;
        unsigned long lastLidarUpdate = 0;
        int lidarCorrection = 0;
        int frontMM = -1;

        leftCloseCount = 0;
        rightCloseCount = 0;
        activeSideNudge = 0;
        sideNudgeEndsAt = 0;
        sideNudgeCooldownEndsAt = 0;

        while (millis() - startTime < DRIVE_TIMEOUT) {
            mpu.update();

            if (activeSideNudge != 0 && (long)(millis() - sideNudgeEndsAt) >= 0) {
                activeSideNudge = 0;
                lidarCorrection = 0;
                sideNudgeCooldownEndsAt = millis() + SIDE_NUDGE_COOLDOWN_MS;
            }

            if (millis() - lastLidarUpdate >= LIDAR_UPDATE_MS) {
                int leftMM = readLidarMM(leftLidar);
                int rightMM = readLidarMM(rightLidar);
                frontMM = readLidarMM(frontLidar);
                lidarCorrection = calculateLidarCorrection(leftMM, rightMM);
                lastLidarUpdate = millis();

                // Serial.print("L: ");
                // Serial.print(leftMM);
                // Serial.print(" R: ");
                // Serial.print(rightMM);
                // Serial.print(" F: ");
                // Serial.print(frontMM);
                // Serial.print(" side correction: ");
                // Serial.println(lidarCorrection);
            }

            float leftTravel = encoder.getLeftRotation() - leftStart;
            float rightTravel = (-encoder.getRightRotation()) - rightStart;
            float averageTravel = (leftTravel + rightTravel) / 2.0;
            float distanceError = targetRotation - averageTravel;

            float headingError = wrapAngle(targetHeading - mpu.getAngleZ());

            int imuCorrection = constrain(
                (int)(HEADING_KP * headingError),
                -MAX_FORWARD_CORRECTION,
                MAX_FORWARD_CORRECTION
            );

            int totalCorrection = constrain(
                imuCorrection + lidarCorrection,
                -MAX_FORWARD_CORRECTION,
                MAX_FORWARD_CORRECTION
            );

            if (frontMM >= 0 && frontMM <= FRONT_WALL_END_MM) {
                stopMotors();
                // //Serial.print("Front wall detected at ");
                // //Serial.print(frontMM);
                // //Serial.println(" mm; continuing to next command");
                return true;
            }

            if (averageTravel >= targetRotation - DISTANCE_TOLERANCE) {
                stopMotors();

                if (settledSince == 0) {
                    settledSince = millis();
                }

                if (millis() - settledSince >= SETTLE_TIME) {
                    //Serial.println("Forward complete");
                    return true;
                }
            } else {
                settledSince = 0;

                int leftPWM = FORWARD_PWM - totalCorrection;
                int rightPWM = FORWARD_PWM + totalCorrection;

                setForwardPWM(leftPWM, rightPWM);
            }

            delay(5);
        }

        stopMotors();
        //Serial.println("Forward timeout");
        return false;
    }

    void initialiseHeading() {
        mpu.update();
        targetHeading = mpu.getAngleZ();
    }

    bool turnToHeading(float newHeading) {
        targetHeading = wrapAngle(newHeading);
        unsigned long startTime = millis();
        unsigned long settledSince = 0;

        while (millis() - startTime < TURN_TIMEOUT) {
            mpu.update();

            float currentHeading = mpu.getAngleZ();
            float error = wrapAngle(targetHeading - currentHeading);

            if (fabs(error) <= TURN_TOLERANCE) {
                stopMotors();

                if (settledSince == 0) {
                    settledSince = millis();
                }

                if (millis() - settledSince >= SETTLE_TIME) {
                    //Serial.println("Turn complete");
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
        //Serial.println("Turn timeout");
        return false;
    }

    bool turnLeft90() {
        mpu.update();
        float currentHeading = mpu.getAngleZ();
        return turnToHeading(currentHeading + 90.0);
    }

    bool turnRight90() {
        mpu.update();
        float currentHeading = mpu.getAngleZ();
        return turnToHeading(currentHeading - 90.0);
    }

    int readLidarMM(mtrn3100::Lidar& lidar) {
        uint16_t distance = lidar.readDistance();

        if (!lidar.isLastReadValid()) {
            return -1;
        }

        return distance;
    }

    int calculateLidarCorrection(int leftMM, int rightMM) {
        const bool leftValid = leftMM >= 0;
        const bool rightValid = rightMM >= 0;

        if (
            leftValid &&
            rightValid &&
            leftMM >= SIDE_CENTRE_MIN_MM &&
            leftMM <= SIDE_CENTRE_MAX_MM &&
            rightMM >= SIDE_CENTRE_MIN_MM &&
            rightMM <= SIDE_CENTRE_MAX_MM
        ) {
            int sideError = leftMM - rightMM;

            if (abs(sideError) > SIDE_CENTRE_DEADBAND_MM) {
                return constrain(
                    sideError / 6,
                    -SIDE_CENTRE_CORRECTION,
                    SIDE_CENTRE_CORRECTION
                );
            }
        }

        if ((long)(millis() - sideNudgeCooldownEndsAt) < 0) {
            return 0;
        }

        const bool leftClose = leftValid && leftMM <= SIDE_EMERGENCY_MM;
        const bool rightClose = rightValid && rightMM <= SIDE_EMERGENCY_MM;

        leftCloseCount = leftClose ? leftCloseCount + 1 : 0;
        rightCloseCount = rightClose ? rightCloseCount + 1 : 0;

        const bool leftConfirmed =
            leftCloseCount >= SIDE_EMERGENCY_CONFIRMATIONS;

        const bool rightConfirmed =
            rightCloseCount >= SIDE_EMERGENCY_CONFIRMATIONS;

        if (leftConfirmed && rightConfirmed) {
            if (leftMM > rightMM) {
                activeSideNudge = SIDE_EMERGENCY_CORRECTION;
            } else if (rightMM > leftMM) {
                activeSideNudge = -SIDE_EMERGENCY_CORRECTION;
            }
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

    float wrapAngle(float angle) {
        while (angle > 180.0) {
            angle -= 360.0;
        }

        while (angle < -180.0) {
            angle += 360.0;
        }

        return angle;
    }

    void setForwardPWM(int leftPWM, int rightPWM) {
        leftPWM = constrain(leftPWM, -120, 120);
        rightPWM = constrain(rightPWM, -120, 120);
        motorL.setPWM(leftPWM);
        motorR.setPWM(-rightPWM);
    }

    void setTurnPWM(int pwm) {
        pwm = constrain(pwm, -MAX_TURN_PWM, MAX_TURN_PWM);
        motorL.setPWM(-pwm);
        motorR.setPWM(-pwm);
    }

    void stopMotors() {
        motorL.setPWM(0);
        motorR.setPWM(0);
    }

    void setTargetHeading(float heading) {
        targetHeading = wrapAngle(heading);
    }

    float getTargetHeading() {
        return targetHeading;
    }

private:
    mtrn3100::Motor& motorL;
    mtrn3100::Motor& motorR;
    mtrn3100::DualEncoder& encoder;
    MPU6050& mpu;
    mtrn3100::Lidar& leftLidar;
    mtrn3100::Lidar& rightLidar;
    mtrn3100::Lidar& frontLidar;

    float targetHeading = 0;

    uint8_t leftCloseCount = 0;
    uint8_t rightCloseCount = 0;
    uint8_t activeSideNudge = 0;
    unsigned long sideNudgeEndsAt = 0;
    unsigned long sideNudgeCooldownEndsAt = 0;
};