#include "DualEncoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include <MPU6050_light.h>
#include "EncoderOdometry.hpp"

#define MOT1PWM 11
#define MOT1DIR 12
mtrn3100::Motor motor1(MOT1PWM, MOT1DIR);

#define MOT2PWM 9
#define MOT2DIR 10
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);

// Encoder pins
#define EN_A  2
#define EN_B  7
#define EN_A2 3
#define EN_B2 8

mtrn3100::DualEncoder encoder(EN_A, EN_B, EN_A2, EN_B2);

// PID Controllers
mtrn3100::PIDController controller1(14, 0, 0.3);
mtrn3100::PIDController controller2(14, 0, 0.3);

bool finishedDrive = false;
bool finishedTurn = false;

// IMU
MPU6050 mpu(Wire);

float targetHeading = 0.0; 
const float K_HEADING = 27;
const float K_SYNC = -1.1;

const float WHEEL_RADIUS = 0.017f;   // metres
const float WHEEL_BASE   = 0.145f;   // distance between wheels
float targetDistance = 1.0f; // metres
float targetAngle = targetDistance / WHEEL_RADIUS;

mtrn3100::EncoderOdometry odom(WHEEL_RADIUS, WHEEL_BASE);

unsigned long lastPIDTime = 0;
const unsigned long PID_INTERVAL = 10000;   // 10 ms

void setup() {
    Serial.begin(9600);

    Wire.begin();
    
    // Robot must be completely still during setup to calibrate the gyroscope
    Serial.println("Hello");

    mpu.begin();
    Serial.println("mpu.begin() returned");
    mpu.calcOffsets(true, true);
    delay(500);

    controller1.zeroAndSetTarget(encoder.getLeftRotation(), targetAngle);
    controller2.zeroAndSetTarget(-encoder.getRightRotation(), targetAngle);

    Serial.println("Done");

    targetHeading = mpu.getAngleZ();
    // Serial.println(targetHeading);
}

void loop() {

    unsigned long currentTime = micros();
    mpu.update();

    if (currentTime - lastPIDTime >= PID_INTERVAL) {

        lastPIDTime = currentTime;

        // Current wheel positions
        float leftPos  = encoder.getLeftRotation();
        float rightPos = -encoder.getRightRotation();

        // Independent PID outputs
        float leftPWM  = controller1.compute(leftPos);
        float rightPWM = controller2.compute(rightPos);

        float headingError = targetHeading - mpu.getAngleZ();
        float headingCorrection = K_HEADING * headingError;
        headingCorrection = constrain(headingCorrection, -60, 60);

        

        float syncError = leftPos - rightPos;
        float syncCorrection = syncError * K_SYNC;

        motor1.setPWM(leftPWM - headingCorrection - syncCorrection);
        motor2.setPWM(-(rightPWM + headingCorrection + syncCorrection));

        // Debug
        Serial.print("L: ");
        Serial.print(leftPos);
        Serial.print("  R: ");
        Serial.print(rightPos);
        Serial.print("  H: ");
        Serial.print(mpu.getAngleZ());
        Serial.print("  TH: ");
        Serial.println(targetHeading);
    }
}


    // float actual_position2 = -encoder2.getRotation(); 
    // float output2 = controller2.compute(actual_position2);
    // // output2 = constrain(output2, -MAX_SPEED, MAX_SPEED);
    // motor2.setPWM(-output2);
    // Serial.println(encoder1.count);
    // Serial.println((encoder1.getRotation()));
    // Serial.println("");
    // delay(1000);

    // float pos1 = encoder1.getRotation();
    // float pos2 = -encoder2.getRotation();

    // float output1 = controller1.compute(pos1);
    // float output2 = controller2.compute(pos2);

    // // Cross-coupling: If pos1 > pos2, the robot is veering right. 
    // // We slow down motor 1 and speed up motor 2 slightly to compensate.
    // float sync_correction = (pos1 - pos2) * 10.0; // Tune this multiplier (10.0) if needed

    // motor1.setPWM(output1 - sync_correction);
    // motor2.setPWM(-(output2 + sync_correction));

    // delayMicroseconds(2000);

// }

// void loop() {

//   Serial.println("Hello World");
  
// //   motor1.setPWM(controller1.compute(encoder.getRotation()));
// //   motor2.setPWM(controller2.compute(encoder.getRotation()));

// //   enum direction dir = left;
// //   // turn left
// //   Serial.println("We are turning left!");
// //   while (count < 4) {
// //     int curr_pos = mpu.getAngleZ();
// //     Serial.println(curr_pos);
// //     turning(left);
// //     if (curr_pos == return_position) {
// //       count++;
// //     }
// //     Serial.println(count);
// //   }

// //   delay(1000);
// //   count = 0;

// //   dir = right;
// //   // turn right
// //   Serial.println("We are turning right!");
// //   while (count < 4) {
// //     int curr_pos = mpu.getAngleZ();
// //     Serial.println(curr_pos);
// //     turning(right);
// //     if (curr_pos == return_position) {
// //       count++;
// //     }
// //     Serial.println(count);
// //   }

// // }

// // void turning(enum direction dir) {
// //   switch (dir) {
// //     case left:
// //       motor1.setPWM(-100); // need to tune
// //       motor2.setPWM(100); // need to tune
// //       break;
// //     case right:
// //       motor1.setPWM(100); // need to tune
// //       motor2.setPWM(-100); // need to tune
// //       break;
// //   }
// }

// // void setup() {
// //   Serial.begin(9600);
// //   Serial.println("hi");
// // }

// // void loop() {
// //   Serial.println(("this is working"));
// //   delay(1000);
// // }

// #include "Motor.hpp"

// #define MOT1PWM 11
// #define MOT1DIR 12
// mtrn3100::Motor motor1(MOT1PWM, MOT1DIR);

// #define MOT2PWM 9
// #define MOT2DIR 10
// mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);

// enum direction { left, right };

// const int TURN_SPEEDL = 100;
// const int TURN_SPEEDR = 100;
// const int TURN_TIME = 1000; // ms, tune experimentally

// void turning(direction dir) {
//   switch (dir) {
//     case left:
//       motor1.setPWM(-TURN_SPEEDL);
//       motor2.setPWM(TURN_SPEEDR);
//       break;

//     case right:
//       motor1.setPWM(TURN_SPEEDL);
//       motor2.setPWM(-TURN_SPEEDR);
//       break;
//   }
// }

// void stopMotors() {
//   motor1.setPWM(0);
//   motor2.setPWM(0);
// }

// void setup() {
//   Serial.begin(9600);
//   Serial.println("Starting...");
// }

// void loop() {

//   Serial.println("Turning left 4 times");

//   for (int i = 0; i < 4; i++) {
//     turning(left);
//     delay(TURN_TIME);

//     stopMotors();
//     delay(500);
//   }

//   delay(1000);

//   Serial.println("Turning right 4 times");

//   for (int i = 0; i < 4; i++) {
//     turning(right);
//     delay(TURN_TIME);

//     stopMotors();
//     delay(500);
//   }

//   stopMotors();

//   while (true) {
//     // stop program
//   }
// }