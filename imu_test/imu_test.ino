#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "BangBangController.hpp"
#include <MPU6050_light.h>

#define MOT1PWM 9 // PIN 9 is a PWM pin
#define MOT1DIR 10
mtrn3100::Motor motor_left(MOT1PWM,MOT1DIR);
mtrn3100::Motor motor_right(MOT1PWM,MOT1DIR);

#define EN_A 2 // PIN 2 is an interupt
#define EN_B 4
mtrn3100::Encoder encoder(EN_A, EN_B);

// mtrn3100::BangBangController controller(255,0);
mtrn3100::PIDController controller(100, 0, 0);


// IMU Setup
MPU6050 mpu(Wire);
int return_position = 0;
int count = 0;

enum direction {left, right};

void setup() {
  Serial.begin(9600);
  controller.zeroAndSetTarget(encoder.getRotation(), 0.5); // Set the target as 2 Radians

  // for imu sensor
  delay(1000);
  mpu.calculateOffsets();
  Serial.print("Done");
  int return_position = mpu.getAngleZ();
  count = 0;
}

void loop() {
  
  motor.setPWM(controller.compute(encoder.getRotation()));

  enum direction dir = left;

  while (count < 4) {
    int curr_pos = mpu.getAngleZ();
    turning(left);
    if (curr_pos == return_position) {
      count++;
    }
  }

  delay(1000);
  count = 0;
  dir = right;

  while (count < 4) {
    int curr_pos = mpu.getAngleZ();
    turning(right);
    if (curr_pos == return_position) {
      count++;
    }
  }

}

int turning(enum direction) {
  switch (direction) {
    case left:
      motor_left.setPWM(-100); // need to tune
      motor_right.setPWM(100); // need to tune
      break;
    case right:
      motor_left.setPWM(100); // need to tune
      motor_right.setPWM(-100); // need to tune
      break;
  }
}
