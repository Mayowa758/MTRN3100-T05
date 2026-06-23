#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "BangBangController.hpp"

#define MOT1PWM 11
#define MOT1DIR 12
mtrn3100::Motor motor1(MOT1PWM,MOT1DIR);

#define MOT2PWM 9
#define MOT2DIR 10
mtrn3100::Motor motor2(MOT2PWM,MOT2DIR);

#define EN_A1 2 
#define EN_B1 7
mtrn3100::Encoder encoder1(EN_A1, EN_B1);

#define EN_A2 3 
#define EN_B2 8
mtrn3100::Encoder encoder2(EN_A2, EN_B2);

mtrn3100::BangBangController controller1(255,0);
mtrn3100::BangBangController controller2(255,0);
// mtrn3100::PIDController controller(-100, 0, 0);


void setup() {
  Serial.begin(9600);
  Serial.print("Starting\n");

  controller1.zeroAndSetTarget(encoder1.getRotation(), distanceToRadians(200)); // Set the target as 200mm
  controller2.zeroAndSetTarget(encoder2.getRotation(), distanceToRadians(200)); 
}

void loop() {
  forwards();
}

void forwards() {
  motor1.setPWM(controller1.compute(encoder1.getRotation()));
  motor2.setPWM(controller2.compute(encoder2.getRotation()));
  
  // debugging
  Serial.print("\nradians: ");
  Serial.print(encoder1.getRotation());
  Serial.print(" ");
  Serial.print(encoder2.getRotation());

  Serial.print("\ndistance: ");
  Serial.print(encoder1.getRotation() * 16);
  Serial.print(" ");
  Serial.print(encoder2.getRotation() * 16);
}

// give dist & radius in mm
float distanceToRadians(float dist) {
  float wheelRadius = 16; 
  return dist / wheelRadius;
}

