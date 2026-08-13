/*
 * MTRN3100 Part 4.1.2 - Chained movement with lidar correction
 *
 * A0/A1/A2 are the XSHUT pins. The sensors are assigned the I2C
 * addresses 0x30/0x31/0x32 during setup.
 */

#include <Wire.h>
#include <MPU6050_light.h>
#include "Motor.hpp"
#include "DualEncoder.hpp"
#include "Lidar.hpp"

#define MOT1PWM 11
#define MOT1DIR 12
#define MOT2PWM 9
#define MOT2DIR 10
#define ENC1A 2
#define ENC1B 7
#define ENC2A 3
#define ENC2B 8

#define LEFT_LIDAR_XSHUT A0
#define RIGHT_LIDAR_XSHUT A1
#define FRONT_LIDAR_XSHUT A2
#define LEFT_LIDAR_ADDRESS 0x30
#define RIGHT_LIDAR_ADDRESS 0x31
#define FRONT_LIDAR_ADDRESS 0x32

mtrn3100::Motor motor1(MOT1PWM, MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);
mtrn3100::DualEncoder encoder(ENC1A, ENC1B, ENC2A, ENC2B);
MPU6050 mpu(Wire);
mtrn3100::Lidar leftLidar;
mtrn3100::Lidar rightLidar;
mtrn3100::Lidar frontLidar;

const char commandString[] = "fflfrflf";
const int COMMAND_COUNT = sizeof(commandString) - 1;

const float CELL_DISTANCE_M = 0.195;
const float WHEEL_RADIUS_M = 0.017;
const int FORWARD_PWM = 85;
const int MAX_FORWARD_CORRECTION = 35;
const float HEADING_KP = 2.0;

const int MAX_TURN_PWM = 70;
const int MEDIUM_TURN_PWM = 48;
const int SLOW_TURN_PWM = 30;
const float TURN_TOLERANCE = 5.0;
const float DISTANCE_TOLERANCE = 0.15;

// Tune these four values on the actual maze.
const int SIDE_WALL_MAX_MM = 150;
const int SIDE_TARGET_MM = 75;
const int FRONT_STOP_MM = 55;

// Front-obstacle recovery.
// If the front LiDAR sees something too close, the robot briefly reverses,
// stops, then continues the SAME 'f' command instead of failing the sequence.
const int REVERSE_PWM = 65;
const unsigned long REVERSE_TIME_MS = 250;
const unsigned long REVERSE_SETTLE_MS = 120;

const float LIDAR_KP = 0.20;
const int MAX_LIDAR_CORRECTION = 22;
const unsigned long LIDAR_UPDATE_MS = 45;

const unsigned long DRIVE_TIMEOUT = 6000;
const unsigned long TURN_TIMEOUT = 6000;
const unsigned long SETTLE_TIME = 150;

float targetHeading = 0;
bool systemReady = false;
bool sequenceFailed = false;
int commandIndex = 0;

void stopMotors() {
    motor1.setPWM(0);
    motor2.setPWM(0);
}

float wrapAngle(float angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

void setForwardPWM(int leftPWM, int rightPWM) {
    leftPWM = constrain(leftPWM, -120, 120);
    rightPWM = constrain(rightPWM, -120, 120);
    motor1.setPWM(leftPWM);
    motor2.setPWM(-rightPWM);
}

void reverseBriefly() {
    setForwardPWM(-REVERSE_PWM, -REVERSE_PWM);
    delay(REVERSE_TIME_MS);
    stopMotors();
    delay(REVERSE_SETTLE_MS);
}

void setTurnPWM(int pwm) {
    pwm = constrain(pwm, -MAX_TURN_PWM, MAX_TURN_PWM);
    motor1.setPWM(-pwm);
    motor2.setPWM(-pwm);
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
    if (!lidar.isLastReadValid()) return -1;
    return distance;
}

int calculateLidarCorrection(int leftMM, int rightMM) {
    bool leftWall = leftMM > 0 && leftMM < SIDE_WALL_MAX_MM;
    bool rightWall = rightMM > 0 && rightMM < SIDE_WALL_MAX_MM;
    float wallError = 0;

    if (leftWall && rightWall) wallError = leftMM - rightMM;
    else if (leftWall) wallError = leftMM - SIDE_TARGET_MM;
    else if (rightWall) wallError = SIDE_TARGET_MM - rightMM;
    else return 0;

    return constrain((int)(LIDAR_KP * wallError), -MAX_LIDAR_CORRECTION, MAX_LIDAR_CORRECTION);
}

bool driveForwardOneCell() {
    float targetRotation = CELL_DISTANCE_M / WHEEL_RADIUS_M;
    float leftStart = encoder.getLeftRotation();
    float rightStart = -encoder.getRightRotation();
    unsigned long startTime = millis();
    unsigned long settledSince = 0;
    unsigned long lastLidarUpdate = 0;
    int lidarCorrection = 0;
    int frontMM = -1;

    while (millis() - startTime < DRIVE_TIMEOUT) {
        mpu.update();

        if (millis() - lastLidarUpdate >= LIDAR_UPDATE_MS) {
            int leftMM = readLidarMM(leftLidar);
            int rightMM = readLidarMM(rightLidar);
            frontMM = readLidarMM(frontLidar);
            lidarCorrection = calculateLidarCorrection(leftMM, rightMM);
            lastLidarUpdate = millis();

            Serial.print("L: "); Serial.print(leftMM);
            Serial.print(" R: "); Serial.print(rightMM);
            Serial.print(" F: "); Serial.print(frontMM);
            Serial.print(" correction: "); Serial.println(lidarCorrection);
        }

        if (frontMM > 0 && frontMM <= FRONT_STOP_MM) {
            stopMotors();
            Serial.print("Front obstacle detected at ");
            Serial.print(frontMM);
            Serial.println(" mm - reversing briefly");

            reverseBriefly();

            int leftMM = readLidarMM(leftLidar);
            int rightMM = readLidarMM(rightLidar);
            frontMM = readLidarMM(frontLidar);
            lidarCorrection = calculateLidarCorrection(leftMM, rightMM);
            lastLidarUpdate = millis();

            Serial.print("After reverse - L: "); Serial.print(leftMM);
            Serial.print(" R: "); Serial.print(rightMM);
            Serial.print(" F: "); Serial.println(frontMM);

            continue;
        }

        float leftTravel = encoder.getLeftRotation() - leftStart;
        float rightTravel = (-encoder.getRightRotation()) - rightStart;
        float averageTravel = (leftTravel + rightTravel) / 2.0;
        float distanceError = targetRotation - averageTravel;
        float headingError = wrapAngle(targetHeading - mpu.getAngleZ());
        int imuCorrection = constrain((int)(HEADING_KP * headingError), -MAX_FORWARD_CORRECTION, MAX_FORWARD_CORRECTION);
        int totalCorrection = constrain(imuCorrection + lidarCorrection, -MAX_FORWARD_CORRECTION, MAX_FORWARD_CORRECTION);

        if (fabs(distanceError) <= DISTANCE_TOLERANCE) {
            stopMotors();
            if (settledSince == 0) settledSince = millis();
            if (millis() - settledSince >= SETTLE_TIME) {
                Serial.println("Forward complete");
                return true;
            }
        } else {
            settledSince = 0;
            setForwardPWM(FORWARD_PWM - totalCorrection, FORWARD_PWM + totalCorrection);
        }

        delay(5);
    }

    stopMotors();
    Serial.println("Forward timeout");
    return false;
}

bool turnToHeading(float newHeading) {
    targetHeading = newHeading;
    unsigned long startTime = millis();
    unsigned long settledSince = 0;

    while (millis() - startTime < TURN_TIMEOUT) {
        mpu.update();
        float error = wrapAngle(targetHeading - mpu.getAngleZ());

        if (fabs(error) <= TURN_TOLERANCE) {
            stopMotors();
            if (settledSince == 0) settledSince = millis();
            if (millis() - settledSince >= SETTLE_TIME) {
                Serial.println("Turn complete");
                return true;
            }
        } else {
            settledSince = 0;
            int turnPWM;
            if (fabs(error) > 35.0) turnPWM = MAX_TURN_PWM;
            else if (fabs(error) > 15.0) turnPWM = MEDIUM_TURN_PWM;
            else turnPWM = SLOW_TURN_PWM;
            if (error < 0) turnPWM = -turnPWM;
            setTurnPWM(turnPWM);
        }
        delay(10);
    }

    stopMotors();
    Serial.println("Turn timeout");
    return false;
}

bool turnLeft90() { return turnToHeading(targetHeading + 90.0); }
bool turnRight90() { return turnToHeading(targetHeading - 90.0); }

void setup() {
    Serial.begin(9600);
    Wire.begin();
    stopMotors();
    delay(1000);

    if (!startLidars()) {
        Serial.println("Lidar initialisation failed");
        return;
    }

    byte imuStatus = mpu.begin();
    if (imuStatus != 0) {
        Serial.println("IMU connection failed");
        return;
    }

    Serial.println("Keep robot completely still during calibration");
    delay(1000);
    mpu.calcOffsets(true, true);
    delay(500);
    mpu.update();
    targetHeading = mpu.getAngleZ();
    delay(2000);
    systemReady = true;
}

void loop() {
    if (!systemReady || sequenceFailed) {
        stopMotors();
        return;
    }

    if (commandIndex >= COMMAND_COUNT) {
        stopMotors();
        return;
    }

    char command = commandString[commandIndex];
    bool success = false;
    if (command == 'f') success = driveForwardOneCell();
    else if (command == 'l') success = turnLeft90();
    else if (command == 'r') success = turnRight90();
    else sequenceFailed = true;

    stopMotors();
    if (!success) {
        sequenceFailed = true;
        Serial.println("Sequence stopped");
        return;
    }

    commandIndex++;
    delay(200);
    if (commandIndex >= COMMAND_COUNT) Serial.println("All commands completed");
}
