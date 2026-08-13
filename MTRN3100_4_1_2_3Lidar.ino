/*
 * MTRN3100 Micromouse - Task 4.1.2 Maze Completion
 *
 * Based on the team's Week 8 Part 4 chaining-movements code.
 *
 * Generative AI assistance was used in developing this code.
 *
 * Part 4.1.1 supplies a command string:
 *   f = forward one maze cell
 *   l = turn left 90 degrees
 *   r = turn right 90 degrees
 *
 * Sensors used during forward motion:
 *   - IMU: holds the commanded heading
 *   - Encoders: measure one-cell travel
 *   - Front LiDAR: collision protection / speed reduction near a wall
 *   - Left + Right LiDAR: corridor centring / wall following
 *
 * IMPORTANT HARDWARE NOTE:
 * Three VL6180X sensors start with the same I2C address, so their XSHUT pins
 * must be connected to three separate Arduino pins. This sketch assumes:
 *
 *   FRONT XSHUT -> D4
 *   LEFT  XSHUT -> D5
 *   RIGHT XSHUT -> D6
 *
 * Change those three #defines if your wiring is different.
 */

#include <Wire.h>
#include <MPU6050_light.h>
#include <VL6180X.h>
#include <string.h>

#include "Motor.hpp"
#include "DualEncoder.hpp"

// ---------------- MOTOR / ENCODER PINS ----------------

#define MOT1PWM 11
#define MOT1DIR 12
#define MOT2PWM 9
#define MOT2DIR 10

#define ENC1A 2
#define ENC1B 7
#define ENC2A 3
#define ENC2B 8

// ---------------- LIDAR XSHUT PINS ----------------
//
// CHANGE THESE IF YOUR REAL ROBOT USES DIFFERENT XSHUT PINS.
//
#define FRONT_LIDAR_XSHUT 4
#define LEFT_LIDAR_XSHUT  5
#define RIGHT_LIDAR_XSHUT 6

// New I2C addresses assigned after startup.
// VL6180X default address is 0x29, therefore each sensor must be
// brought out of reset one at a time and re-addressed.
#define FRONT_LIDAR_ADDR 0x30
#define LEFT_LIDAR_ADDR  0x31
#define RIGHT_LIDAR_ADDR 0x32

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

VL6180X frontLidar;
VL6180X leftLidar;
VL6180X rightLidar;

// ---------------- COMMAND STRING ----------------
//
// Paste the command string printed by Task 4.1.1 here on marking day.
// Example:
// const char commandString[] = "ffrfflfff";
//
const char commandString[] = "fflfrflf";

// ---------------- ROBOT GEOMETRY / CALIBRATION ----------------
//
// The physical maze cell is 180 mm. Your Week 8 Part 4 code used 0.195 m
// as the calibrated encoder travel for one cell. Keep this value only if it
// is still correct on the real robot.
//
const float CELL_DISTANCE_M = 0.195f;
const float WHEEL_RADIUS_M = 0.017f;

// ---------------- FORWARD CONTROL ----------------

const int FORWARD_PWM = 85;
const int SLOW_FORWARD_PWM = 60;

const int MAX_TOTAL_CORRECTION = 38;

// IMU heading correction.
const float HEADING_KP = 2.0f;

// Side-LiDAR correction.
// Increase carefully if the robot does not recentre strongly enough.
const float SIDE_KP = 0.35f;
const int MAX_SIDE_CORRECTION = 24;

// A side reading below this value is treated as a usable wall.
// Tune this to your robot and VL6180X mounting.
const uint16_t SIDE_WALL_DETECT_MM = 150;

// Desired sensor-to-wall clearance when only one side wall is visible.
// MUST be tuned on the real robot because it depends on robot width
// and exact LiDAR mounting position.
const float SIDE_TARGET_MM = 55.0f;

// Front LiDAR thresholds.
// The front sensor is NOT used to decide the path; Part 4.1.1 already
// supplied the path. It is used to avoid driving hard into an unexpected wall.
const uint16_t FRONT_SLOW_MM = 100;
const uint16_t FRONT_STOP_MM = 45;

// Encoder completion tolerance, in wheel radians.
const float DISTANCE_TOLERANCE = 0.15f;

// ---------------- TURN CONTROL ----------------

const int MAX_TURN_PWM = 70;
const int MEDIUM_TURN_PWM = 48;
const int SLOW_TURN_PWM = 30;

const float TURN_TOLERANCE = 5.0f;

// ---------------- TIMING ----------------

const unsigned long DRIVE_TIMEOUT = 6500;
const unsigned long TURN_TIMEOUT = 6000;
const unsigned long SETTLE_TIME = 150;

// LiDAR data are updated at this interval rather than on every encoder loop.
const unsigned long LIDAR_UPDATE_MS = 25;

// ---------------- GLOBAL STATE ----------------

float targetHeading = 0.0f;

bool systemReady = false;
bool sequenceFailed = false;
bool lidarReady = false;

size_t commandIndex = 0;
const size_t commandCount = strlen(commandString);

uint16_t frontDistance = 0;
uint16_t leftDistance = 0;
uint16_t rightDistance = 0;

bool frontValid = false;
bool leftValid = false;
bool rightValid = false;

unsigned long lastLidarUpdate = 0;

// ---------------- BASIC MOTOR FUNCTIONS ----------------

void stopMotors() {
    motor1.setPWM(0);
    motor2.setPWM(0);
}

float wrapAngle(float angle) {
    while (angle > 180.0f) {
        angle -= 360.0f;
    }

    while (angle < -180.0f) {
        angle += 360.0f;
    }

    return angle;
}

/*
 * Forward movement.
 *
 * Motor 2 uses the opposite sign because the two motors are physically
 * mounted in opposite directions.
 */
void setForwardPWM(int leftPWM, int rightPWM) {
    leftPWM = constrain(leftPWM, -120, 120);
    rightPWM = constrain(rightPWM, -120, 120);

    motor1.setPWM(leftPWM);
    motor2.setPWM(-rightPWM);
}

/*
 * In-place turning.
 *
 * Both motors receive the same Arduino-side sign because the motors are
 * physically mounted in opposite directions.
 */
void setTurnPWM(int pwm) {
    pwm = constrain(pwm, -MAX_TURN_PWM, MAX_TURN_PWM);

    motor1.setPWM(-pwm);
    motor2.setPWM(-pwm);
}

// ---------------- THREE-LIDAR INITIALISATION ----------------

bool setupOneLidar(VL6180X &sensor, uint8_t newAddress) {
    if (!sensor.init()) {
        return false;
    }

    sensor.configureDefault();

    // Scaling 2 extends the usable range compared with scaling 1.
    sensor.setScaling(2);
    sensor.setTimeout(100);

    sensor.setAddress(newAddress);

    // Continuous ranging avoids three long single-shot reads inside the
    // forward control loop.
    sensor.startRangeContinuous(20);

    delay(20);

    return true;
}

bool initThreeLidars() {
    pinMode(FRONT_LIDAR_XSHUT, OUTPUT);
    pinMode(LEFT_LIDAR_XSHUT, OUTPUT);
    pinMode(RIGHT_LIDAR_XSHUT, OUTPUT);

    // Shut down all three so only one device at address 0x29 is active.
    digitalWrite(FRONT_LIDAR_XSHUT, LOW);
    digitalWrite(LEFT_LIDAR_XSHUT, LOW);
    digitalWrite(RIGHT_LIDAR_XSHUT, LOW);

    delay(20);

    // Front.
    digitalWrite(FRONT_LIDAR_XSHUT, HIGH);
    delay(10);

    if (!setupOneLidar(frontLidar, FRONT_LIDAR_ADDR)) {
        Serial.println("Front LiDAR init failed");
        return false;
    }

    // Left.
    digitalWrite(LEFT_LIDAR_XSHUT, HIGH);
    delay(10);

    if (!setupOneLidar(leftLidar, LEFT_LIDAR_ADDR)) {
        Serial.println("Left LiDAR init failed");
        return false;
    }

    // Right.
    digitalWrite(RIGHT_LIDAR_XSHUT, HIGH);
    delay(10);

    if (!setupOneLidar(rightLidar, RIGHT_LIDAR_ADDR)) {
        Serial.println("Right LiDAR init failed");
        return false;
    }

    Serial.println("All 3 LiDARs initialised");
    return true;
}

bool validLidarReading(VL6180X &sensor, uint16_t distance) {
    return !sensor.timeoutOccurred()
        && sensor.readRangeStatus() == VL6180X_ERROR_NONE
        && distance > 0;
}

void updateLidars(bool forceUpdate = false) {
    if (!lidarReady) {
        return;
    }

    if (!forceUpdate && (millis() - lastLidarUpdate < LIDAR_UPDATE_MS)) {
        return;
    }

    lastLidarUpdate = millis();

    uint16_t f = frontLidar.readRangeContinuousMillimeters();
    frontValid = validLidarReading(frontLidar, f);
    if (frontValid) {
        frontDistance = f;
    }

    uint16_t l = leftLidar.readRangeContinuousMillimeters();
    leftValid = validLidarReading(leftLidar, l);
    if (leftValid) {
        leftDistance = l;
    }

    uint16_t r = rightLidar.readRangeContinuousMillimeters();
    rightValid = validLidarReading(rightLidar, r);
    if (rightValid) {
        rightDistance = r;
    }
}

// ---------------- SIDE-LIDAR CORRIDOR CORRECTION ----------------

int getSideLidarCorrection() {
    bool leftWall =
        leftValid && leftDistance < SIDE_WALL_DETECT_MM;

    bool rightWall =
        rightValid && rightDistance < SIDE_WALL_DETECT_MM;

    float sideError = 0.0f;

    if (leftWall && rightWall) {
        /*
         * Centre between two walls.
         *
         * leftDistance > rightDistance means the robot is closer to the
         * right wall, so a positive correction turns it left.
         */
        sideError =
            (float)leftDistance - (float)rightDistance;
    }
    else if (leftWall) {
        /*
         * Follow the left wall.
         * If left distance is too large, turn left toward the wall.
         */
        sideError =
            (float)leftDistance - SIDE_TARGET_MM;
    }
    else if (rightWall) {
        /*
         * Follow the right wall.
         * If right distance is too large, turn right.
         */
        sideError =
            SIDE_TARGET_MM - (float)rightDistance;
    }
    else {
        // No reliable side wall: let the IMU hold the heading.
        sideError = 0.0f;
    }

    int correction =
        (int)(SIDE_KP * sideError);

    return constrain(
        correction,
        -MAX_SIDE_CORRECTION,
        MAX_SIDE_CORRECTION
    );
}

// ---------------- FORWARD ONE CELL ----------------

bool driveForwardOneCell() {
    float targetRotation =
        CELL_DISTANCE_M / WHEEL_RADIUS_M;

    float leftStart =
        encoder.getLeftRotation();

    float rightStart =
        -encoder.getRightRotation();

    unsigned long startTime = millis();
    unsigned long settledSince = 0;

    updateLidars(true);

    while (millis() - startTime < DRIVE_TIMEOUT) {
        mpu.update();
        updateLidars();

        float leftTravel =
            encoder.getLeftRotation() - leftStart;

        float rightTravel =
            (-encoder.getRightRotation()) - rightStart;

        float averageTravel =
            (leftTravel + rightTravel) / 2.0f;

        float distanceError =
            targetRotation - averageTravel;

        // ---------- FRONT LIDAR ----------
        //
        // If something is extremely close in front, stop this run instead
        // of turning a small tracking error into a major collision.
        //
        if (frontValid && frontDistance <= FRONT_STOP_MM) {
            stopMotors();

            Serial.print("Front obstacle too close: ");
            Serial.print(frontDistance);
            Serial.println(" mm");

            return false;
        }

        // ---------- IMU HEADING ----------
        float headingError =
            wrapAngle(
                targetHeading - mpu.getAngleZ()
            );

        int headingCorrection =
            (int)(HEADING_KP * headingError);

        // ---------- LEFT + RIGHT LIDAR ----------
        int sideCorrection =
            getSideLidarCorrection();

        /*
         * Both corrections have the same sign convention:
         * positive correction -> steer left
         * negative correction -> steer right
         */
        int totalCorrection =
            headingCorrection + sideCorrection;

        totalCorrection =
            constrain(
                totalCorrection,
                -MAX_TOTAL_CORRECTION,
                MAX_TOTAL_CORRECTION
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
        }
        else {
            settledSince = 0;

            int basePWM = FORWARD_PWM;

            // Slow down near a front wall, but keep encoder distance as the
            // primary cell-completion measurement.
            if (frontValid && frontDistance <= FRONT_SLOW_MM) {
                basePWM = SLOW_FORWARD_PWM;
            }

            int leftPWM =
                basePWM - totalCorrection;

            int rightPWM =
                basePWM + totalCorrection;

            setForwardPWM(
                leftPWM,
                rightPWM
            );
        }

        delay(8);
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
        updateLidars();

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
        }
        else {
            settledSince = 0;

            int turnPWM;

            if (fabs(error) > 35.0f) {
                turnPWM = MAX_TURN_PWM;
            }
            else if (fabs(error) > 15.0f) {
                turnPWM = MEDIUM_TURN_PWM;
            }
            else {
                turnPWM = SLOW_TURN_PWM;
            }

            if (error < 0.0f) {
                turnPWM = -turnPWM;
            }

            setTurnPWM(turnPWM);
        }

        delay(8);
    }

    stopMotors();

    Serial.println("Turn timeout");
    return false;
}

bool turnLeft90() {
    return turnToHeading(
        targetHeading + 90.0f
    );
}

bool turnRight90() {
    return turnToHeading(
        targetHeading - 90.0f
    );
}

// ---------------- COMMAND EXECUTION ----------------

bool executeCommand(char command) {
    // Accept either lower-case or upper-case commands.
    if (command >= 'A' && command <= 'Z') {
        command =
            command - 'A' + 'a';
    }

    if (command == 'f') {
        return driveForwardOneCell();
    }

    if (command == 'l') {
        return turnLeft90();
    }

    if (command == 'r') {
        return turnRight90();
    }

    Serial.print("Invalid command: ");
    Serial.println(command);

    return false;
}

// ---------------- SETUP ----------------

void setup() {
    Serial.begin(9600);
    Wire.begin();

    stopMotors();

    Serial.println("Starting MTRN3100 Task 4.1.2");
    Serial.println("3-LiDAR maze completion");

    delay(500);

    // ---------- THREE LIDARS ----------
    lidarReady =
        initThreeLidars();

    if (!lidarReady) {
        Serial.println("LiDAR setup failed");
        stopMotors();
        return;
    }

    // ---------- IMU ----------
    byte imuStatus =
        mpu.begin();

    Serial.print("IMU status: ");
    Serial.println(imuStatus);

    if (imuStatus != 0) {
        Serial.println("IMU connection failed");
        stopMotors();
        return;
    }

    Serial.println(
        "Keep robot completely still during IMU calibration"
    );

    delay(1000);

    mpu.calcOffsets(true, true);

    delay(500);

    mpu.update();

    targetHeading =
        mpu.getAngleZ();

    Serial.print("Initial heading: ");
    Serial.println(targetHeading);

    Serial.print("Commands (");
    Serial.print(commandCount);
    Serial.print("): ");
    Serial.println(commandString);

    updateLidars(true);

    Serial.print("Front/Left/Right LiDAR: ");
    Serial.print(frontDistance);
    Serial.print(" / ");
    Serial.print(leftDistance);
    Serial.print(" / ");
    Serial.print(rightDistance);
    Serial.println(" mm");

    /*
     * Small placement delay. Once systemReady becomes true, the movement
     * sequence is completely autonomous.
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

    // Unlike Week 8 Part 4, the path is NOT restricted to 8 commands.
    // Task 4.1.1 can generate whatever command-string length is required.
    if (commandIndex >= commandCount) {
        stopMotors();

        Serial.println("Maze command sequence complete");
        systemReady = false;

        return;
    }

    char command =
        commandString[commandIndex];

    Serial.print("Running command ");
    Serial.print(commandIndex + 1);
    Serial.print("/");
    Serial.print(commandCount);
    Serial.print(": ");
    Serial.println(command);

    bool success =
        executeCommand(command);

    stopMotors();

    if (!success) {
        Serial.println("Sequence stopped for safety/timeout");
        sequenceFailed = true;
        return;
    }

    commandIndex++;

    delay(120);
}
