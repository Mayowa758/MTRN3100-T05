// #include "Encoder.hpp"
// #include "Motor.hpp"
// #include "PIDController.hpp"
// #include "BangBangController.hpp"
// #include <MPU6050_light.h>

// #define MOT1PWM 11
// #define MOT1DIR 12
// mtrn3100::Motor motor1(MOT1PWM,MOT1DIR);

// #define MOT2PWM 9
// #define MOT2DIR 10
// mtrn3100::Motor motor2(MOT2PWM,MOT2DIR);

// #define EN_A 2
// #define EN_B 7
// mtrn3100::Encoder encoder(EN_A, EN_B);

// #define EN_A2 3 
// #define EN_B2 8
// mtrn3100::Encoder encoder2(EN_A2, EN_B2);

// mtrn3100::BangBangController controller1(255,0);
// mtrn3100::BangBangController controller2(255,0);
// // mtrn3100::PIDController controller(100, 0, 0);


// // IMU Setup
// MPU6050 mpu(Wire);
// int return_position = 0;
// int count = 0;

// enum direction {left, right};

// void setup() {
//   Serial.begin(9600);
//   controller1.zeroAndSetTarget(encoder.getRotation(), 0.5); // Set the target as 2 Radians
//   controller2.zeroAndSetTarget(encoder.getRotation(), 0.5); // Set the target as 2 Radians
//   // // for imu sensor
//   Wire.begin();
//   mpu.begin();
//   mpu.calcOffsets();
//   int return_position = mpu.getAngleZ();

//   Serial.println("Done");
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

#include "Motor.hpp"

#define MOT1PWM 11
#define MOT1DIR 12
mtrn3100::Motor motor1(MOT1PWM, MOT1DIR);

#define MOT2PWM 9
#define MOT2DIR 10
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);

enum direction { left, right };

const int TURN_SPEEDL = 100;
const int TURN_SPEEDR = 100;
const int TURN_TIME = 1000; // ms, tune experimentally

void turning(direction dir) {
  switch (dir) {
    case left:
      motor1.setPWM(-TURN_SPEEDL);
      motor2.setPWM(TURN_SPEEDR);
      break;

    case right:
      motor1.setPWM(TURN_SPEEDL);
      motor2.setPWM(-TURN_SPEEDR);
      break;
  }
}

void stopMotors() {
  motor1.setPWM(0);
  motor2.setPWM(0);
}

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");
}

void loop() {

  Serial.println("Turning left 4 times");

  for (int i = 0; i < 4; i++) {
    turning(left);
    delay(TURN_TIME);

    stopMotors();
    delay(500);
  }

  delay(1000);

  Serial.println("Turning right 4 times");

  for (int i = 0; i < 4; i++) {
    turning(right);
    delay(TURN_TIME);

    stopMotors();
    delay(500);
  }

  stopMotors();

  while (true) {
    // stop program
  }
}