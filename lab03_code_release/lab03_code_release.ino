#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "BangBangController.hpp"

#define MOT1PWM 9 // PIN 9 is a PWM pin
#define MOT1DIR 10
mtrn3100::Motor motor(MOT1PWM,MOT1DIR);

#define EN_A 2 // PIN 2 is an interupt
#define EN_B 4
mtrn3100::Encoder encoder(EN_A, EN_B);

// mtrn3100::BangBangController controller(255,0);
mtrn3100::PIDController controller(100, 0, 0);


void setup() {
  Serial.begin(9600);
  controller.zeroAndSetTarget(encoder.getRotation(), 0.5); // Set the target as 2 Radians
}

void loop() {
  
  motor.setPWM(controller.compute(encoder.getRotation()));

  // Tasks 1-3
  // motor.setPWM(100);
  // Serial.print(encoder.getRotation());
  // Serial.print("\n");
  // delay(3500);
  // motor.setPWM(0);
  // delay(3000);
}
