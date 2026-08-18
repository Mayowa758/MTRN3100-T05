#pragma once
#include <Arduino.h>
#include "Motor.hpp"
#include "DualEncoder.hpp"
#include "Lidar.hpp"
#include <MPU6050_light.h>

constexpr float CELL_DISTANCE_M = 0.180;
constexpr float WHEEL_RADIUS_M = 0.016;

constexpr int FORWARD_PWM = 120;
constexpr int MAX_FORWARD_CORRECTION = 60;
constexpr float HEADING_KP = 4.0;

constexpr int MAX_TURN_PWM = 80;
constexpr int MEDIUM_TURN_PWM = 55;
constexpr int SLOW_TURN_PWM = 35;

constexpr float TURN_TOLERANCE = 1.0;
constexpr float DISTANCE_TOLERANCE = 0.15;

constexpr unsigned long DRIVE_TIMEOUT = 4500;
constexpr unsigned long DRIVE_STALL_TIMEOUT = 900;
constexpr float DRIVE_PROGRESS_EPSILON_RAD = 0.08;
constexpr unsigned long TURN_TIMEOUT = 2000;
constexpr unsigned long SETTLE_TIME = 0;

constexpr int SIDE_CENTRE_MIN_MM = 35;
constexpr int SIDE_CENTRE_MAX_MM = 120;
constexpr int SIDE_CENTRE_DEADBAND_MM = 8;
constexpr int SIDE_CENTRE_CORRECTION = 12;

// Proactive side-wall protection.  The old implementation waited until 55 mm,
// pulsed only +/-10 PWM for 50 ms, then entered a cooldown.  At driving speed
// that correction arrived too late.  Start steering away much earlier and keep
// correcting until the robot has recovered a safe clearance.
constexpr int SIDE_WARNING_MM = 75;
constexpr int SIDE_DANGER_MM = 35;
constexpr int SIDE_RELEASE_MM = 85;
constexpr int SIDE_ESCAPE_MIN_CORRECTION = 14;
constexpr int SIDE_ESCAPE_MAX_CORRECTION = 35;

constexpr int FRONT_WALL_END_MM = 50;

// After every 90-degree turn, use the front wall as a local position reference
// when one is visible. This corrects accumulated forward/backward position error
// before the next maze movement.
constexpr int FRONT_CENTRE_TARGET_MM = 50;
constexpr int FRONT_CENTRE_TOLERANCE_MM = 5;
constexpr int FRONT_CENTRE_PWM = 45;
constexpr int FRONT_CENTRE_MAX_DETECT_MM = 160;
constexpr unsigned long FRONT_CENTRE_TIMEOUT_MS = 1500;
constexpr unsigned long FRONT_CENTRE_SETTLE_MS = 60;

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
        // This flag lets Maze distinguish a complete encoder-based cell move
        // from an early stop caused by the front lidar.
        lastForwardCompletedCell = false;
        float targetRotation = CELL_DISTANCE_M / WHEEL_RADIUS_M;
        float leftStart = encoder.getLeftRotation();
        float rightStart = -encoder.getRightRotation();
        unsigned long startTime = millis();
        unsigned long settledSince = 0;
        unsigned long lastLidarUpdate = 0;
        unsigned long lastProgressTime = millis();
        float lastProgressTravel = 0.0f;
        int lidarCorrection = 0;
        int frontMM = -1;

        activeSideNudge = 0;

        while (millis() - startTime < DRIVE_TIMEOUT) {
            mpu.update();

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

            // A strong side-wall correction can slow forward progress enough that
            // a fixed short timeout falsely looks like a failed drive. Track
            // encoder progress separately: only treat the robot as stalled when
            // neither wheel has advanced meaningfully for DRIVE_STALL_TIMEOUT.
            if (averageTravel > lastProgressTravel + DRIVE_PROGRESS_EPSILON_RAD) {
                lastProgressTravel = averageTravel;
                lastProgressTime = millis();
            }

            if (millis() - lastProgressTime >= DRIVE_STALL_TIMEOUT) {
                stopMotors();
                return false;
            }

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
                    lastForwardCompletedCell = true;
                    return true;
                }
            } else {
                settledSince = 0;

                int leftPWM = FORWARD_PWM - totalCorrection;
                int rightPWM = FORWARD_PWM + totalCorrection + 5;

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

            delay(5);
        }

        stopMotors();
        //Serial.println("Turn timeout");
        return false;
    }

    // bool turnLeft90() {
    //     mpu.update();
    //     float currentHeading = mpu.getAngleZ();
    //     return turnToHeading(currentHeading + 90.0);
    // }

    // bool turnRight90() {
    //     mpu.update();
    //     float currentHeading = mpu.getAngleZ();
    //     return turnToHeading(currentHeading - 90.0);
    // }

    bool turnLeft90() {
        if (!turnToHeading(targetHeading + 90.0)) {
            return false;
        }

        centreToFrontWallAfterTurn();
        return true;
    }

    bool turnRight90() {
        if (!turnToHeading(targetHeading - 90.0)) {
            return false;
        }

        centreToFrontWallAfterTurn();
        return true;
    }

    // A 90-degree turn can leave the robot slightly forward/backward of the
    // cell centre. If the front lidar can see a nearby wall, use that wall as
    // a local reference and gently correct to 50 mm from it. If no front wall
    // is visible (invalid/out-of-range reading), do nothing.
    void centreToFrontWallAfterTurn() {
        stopMotors();
        delay(40);

        int frontMM = readLidarMM(frontLidar);

        if (frontMM < 0 || frontMM > FRONT_CENTRE_MAX_DETECT_MM) {
            return;
        }

        unsigned long startTime = millis();
        unsigned long settledSince = 0;

        while (millis() - startTime < FRONT_CENTRE_TIMEOUT_MS) {
            mpu.update();
            frontMM = readLidarMM(frontLidar);

            // If the wall disappears from the lidar range while correcting,
            // stop rather than blindly continuing.
            if (frontMM < 0 || frontMM > FRONT_CENTRE_MAX_DETECT_MM) {
                stopMotors();
                return;
            }

            int distanceErrorMM = frontMM - FRONT_CENTRE_TARGET_MM;

            if (abs(distanceErrorMM) <= FRONT_CENTRE_TOLERANCE_MM) {
                stopMotors();

                if (settledSince == 0) {
                    settledSince = millis();
                }

                if (millis() - settledSince >= FRONT_CENTRE_SETTLE_MS) {
                    return;
                }
            } else {
                settledSince = 0;

                float headingError = wrapAngle(targetHeading - mpu.getAngleZ());
                int headingCorrection = constrain(
                    (int)(HEADING_KP * headingError),
                    -12,
                    12
                );

                if (distanceErrorMM > 0) {
                    // Too far from the wall: move forward gently.
                    setForwardPWM(
                        FRONT_CENTRE_PWM - headingCorrection,
                        FRONT_CENTRE_PWM + headingCorrection
                    );
                } else {
                    // Too close to the wall: reverse gently. The heading
                    // correction sign is inverted while travelling backwards.
                    setForwardPWM(
                        -FRONT_CENTRE_PWM - headingCorrection,
                        -FRONT_CENTRE_PWM + headingCorrection
                    );
                }
            }

            delay(20);
        }

        stopMotors();
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

        // If both walls are visible, steer from the difference between them.
        // This avoids using a fixed absolute threshold in a narrow corridor:
        // when centred, both side readings can legitimately be fairly small.
        if (leftValid && rightValid) {
            activeSideNudge = 0;
            const int sideError = leftMM - rightMM;

            // If one wall is critically close and clearly closer than the other,
            // immediately use the full escape correction.
            if (leftMM <= SIDE_DANGER_MM && leftMM + 8 < rightMM) {
                return -SIDE_ESCAPE_MAX_CORRECTION;
            }
            if (rightMM <= SIDE_DANGER_MM && rightMM + 8 < leftMM) {
                return SIDE_ESCAPE_MAX_CORRECTION;
            }

            if (abs(sideError) > SIDE_CENTRE_DEADBAND_MM) {
                // Continuous proportional centring.  The correction grows with
                // the side-distance mismatch instead of being a 30 ms pulse.
                const int magnitude = constrain(
                    abs(sideError) / 2,
                    SIDE_CENTRE_CORRECTION,
                    SIDE_ESCAPE_MAX_CORRECTION
                );
                return (sideError > 0) ? magnitude : -magnitude;
            }

            return 0;
        }

        // With only one side wall visible, use a hysteretic escape controller.
        // It starts before the robot is in collision range and remains active
        // until there is clearly safe clearance again.
        if (activeSideNudge < 0) {
            if (!leftValid || leftMM >= SIDE_RELEASE_MM) {
                activeSideNudge = 0;
            }
        } else if (activeSideNudge > 0) {
            if (!rightValid || rightMM >= SIDE_RELEASE_MM) {
                activeSideNudge = 0;
            }
        }

        if (activeSideNudge == 0) {
            if (leftValid && leftMM <= SIDE_WARNING_MM) {
                activeSideNudge = -1;
            } else if (rightValid && rightMM <= SIDE_WARNING_MM) {
                activeSideNudge = 1;
            }
        }

        if (activeSideNudge != 0) {
            const int distance = (activeSideNudge < 0) ? leftMM : rightMM;
            int magnitude = SIDE_ESCAPE_MIN_CORRECTION;

            if (distance <= SIDE_DANGER_MM) {
                magnitude = SIDE_ESCAPE_MAX_CORRECTION;
            } else {
                magnitude = map(
                    distance,
                    SIDE_DANGER_MM,
                    SIDE_WARNING_MM,
                    SIDE_ESCAPE_MAX_CORRECTION,
                    SIDE_ESCAPE_MIN_CORRECTION
                );
                magnitude = constrain(
                    magnitude,
                    SIDE_ESCAPE_MIN_CORRECTION,
                    SIDE_ESCAPE_MAX_CORRECTION
                );
            }

            return activeSideNudge * magnitude;
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
        leftPWM = constrain(leftPWM, -150, 150);
        rightPWM = constrain(rightPWM, -150, 150);
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

    bool didLastForwardCompleteCell() const {
        return lastForwardCompletedCell;
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
    bool lastForwardCompletedCell = false;

    // -1: escaping left wall, +1: escaping right wall, 0: normal steering.
    int activeSideNudge = 0;
};
