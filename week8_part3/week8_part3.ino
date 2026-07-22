#include <Wire.h>
#include <MPU6050_light.h>

#include "Motor.hpp"
#include "PIDController.hpp"

// ---------------- MOTOR PINS ----------------

#define MOT1PWM 11
#define MOT1DIR 12

#define MOT2PWM 9
#define MOT2DIR 10

mtrn3100::Motor motor1(MOT1PWM, MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);

// ---------------- IMU ----------------

MPU6050 mpu(Wire);

// ---------------- HEADING PID ----------------

// P = 8, I = 0, D = 0
mtrn3100::PIDController headingController(8, 0, 0);

// ---------------- SETTINGS ----------------

// Change to -90 if the robot turns in the wrong direction.
const float TURN_ANGLE = 90.0;

// Aim tighter than the required +/- 5 degrees.
const float HEADING_TOLERANCE = 2.0;

// If it moves outside this after stopping, correct again.
const float MAX_ACCEPTABLE_ERROR = 4.0;

const int MAX_TURN_PWM = 55;
const int MEDIUM_TURN_PWM = 38;
const int MIN_TURN_PWM = 20;

const unsigned long PID_INTERVAL_US = 10000;  // 10 ms
const unsigned long SETTLE_TIME_MS = 250;

float targetHeading = 0.0;

unsigned long lastPIDTime = 0;
unsigned long settledStartTime = 0;
unsigned long lastPrintTime = 0;

bool systemReady = false;
bool headingSettled = false;

// ------------------------------------------------

void stopMotors() {
    motor1.setPWM(0);
    motor2.setPWM(0);
}

// Keep an angle between -180 and +180 degrees.
float wrapAngle(float angle) {
    while (angle > 180.0) {
        angle -= 360.0;
    }

    while (angle < -180.0) {
        angle += 360.0;
    }

    return angle;
}

// Both motors use the same sign for an in-place turn.
void rotateRobot(float pwm) {
    pwm = constrain(
        pwm,
        -MAX_TURN_PWM,
        MAX_TURN_PWM
    );

    motor1.setPWM((int)pwm);
    motor2.setPWM((int)pwm);
}

// ------------------------------------------------

void setup() {
    Serial.begin(9600);
    Wire.begin();

    stopMotors();

    Serial.println("Starting Part 3");

    byte status = mpu.begin();

    Serial.print("IMU status: ");
    Serial.println(status);

    if (status != 0) {
        Serial.println("IMU connection failed");
        return;
    }

    Serial.println(
        "Keep robot completely still during calibration"
    );

    delay(300);

    mpu.calcOffsets(true, true);

    delay(200);

    /*
     * Update the IMU several times before reading the
     * initial heading.
     */
    for (int i = 0; i < 20; i++) {
        mpu.update();
        delay(5);
    }

    float startHeading = mpu.getAngleZ();

    targetHeading = startHeading + TURN_ANGLE;

    headingController.zeroAndSetTarget(
        startHeading,
        targetHeading
    );

    Serial.print("Start heading: ");
    Serial.println(startHeading);

    Serial.print("Target heading: ");
    Serial.println(targetHeading);

    delay(200);

    systemReady = true;
}

// ------------------------------------------------

void loop() {
    if (!systemReady) {
        stopMotors();
        return;
    }

    mpu.update();

    unsigned long currentTime = micros();

    if (
        currentTime - lastPIDTime <
        PID_INTERVAL_US
    ) {
        return;
    }

    lastPIDTime = currentTime;

    float currentHeading = mpu.getAngleZ();

    float headingError = wrapAngle(
        targetHeading - currentHeading
    );

    float absoluteError = fabs(headingError);

    /*
     * Stop when inside +/-2 degrees.
     */
    if (absoluteError <= HEADING_TOLERANCE) {
        stopMotors();

        if (settledStartTime == 0) {
            settledStartTime = millis();
        }

        /*
         * Confirm that it remains inside the tolerance
         * for 250 ms.
         */
        if (
            millis() - settledStartTime >=
            SETTLE_TIME_MS
        ) {
            if (!headingSettled) {
                headingSettled = true;

                Serial.print("Settled. Final error: ");
                Serial.println(headingError);
            }
        }
    } else {
        /*
         * The robot has moved away from the target,
         * so resume heading correction.
         */
        settledStartTime = 0;
        headingSettled = false;

        float output =
            headingController.compute(currentHeading);

        /*
         * Reverse the PID output to match the motor
         * orientation used in the previous working code.
         */
        output = -output;

        /*
         * Slow down near the target to reduce overshoot.
         */
        if (absoluteError <= 12.0) {
            output = constrain(
                output,
                -MIN_TURN_PWM,
                MIN_TURN_PWM
            );

            if (output > 0) {
                output = MIN_TURN_PWM;
            } else {
                output = -MIN_TURN_PWM;
            }
        } else if (absoluteError <= 30.0) {
            output = constrain(
                output,
                -MEDIUM_TURN_PWM,
                MEDIUM_TURN_PWM
            );
        } else {
            output = constrain(
                output,
                -MAX_TURN_PWM,
                MAX_TURN_PWM
            );
        }

        rotateRobot(output);
    }

    /*
     * Safety check:
     * if inertia moves it more than 4 degrees away,
     * the controller automatically corrects it.
     */
    if (
        headingSettled &&
        absoluteError > MAX_ACCEPTABLE_ERROR
    ) {
        headingSettled = false;
        settledStartTime = 0;
    }

    if (millis() - lastPrintTime >= 100) {
        lastPrintTime = millis();

        Serial.print("Current: ");
        Serial.print(currentHeading);

        Serial.print("  Target: ");
        Serial.print(targetHeading);

        Serial.print("  Error: ");
        Serial.println(headingError);
    }
}
