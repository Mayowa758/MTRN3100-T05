// setup of Motors

// pwm pin setup
// encoder setup

// setup of IMU
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu (Wire);
unsgined long timer = 0;

// timing
unsigned long timer = 0;
int counter = 0;

// positioning
float starting_angle = 0;
int revolutions = 4;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);

  while(status!=0){} // stop everything if could not connect to MPU

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);

  mpu.calcOffsets();
  Serial.println("Done!\n");
}

void loop() {
  // put your main code here, to run repeatedly:
    Serial.println("Turning Left!");
    while(rotateLeft());

    counter = 0;

    Serial.println("Turning Right!");
    while(rotateRight());

    delay(1000);
    Serial.println("Finished");

  // needs to turn stationary left 
  // needs to turn stationary right
}

bool rotateLeft() {
  mpu.update();
  completed = false;

  // print data every 10ms
  if((millis()-timer)>10){ 
  float angle = mpu.getAngleZ();

  Serial.print("\tZ : ");
	Serial.println(angle);
	timer = millis();  
  }

  if (angle == starting_angle) counter++;;
  if (counter == revolutions) completed = true;

  return completed;

}

bool rotateRight() {
  mpu.update();
  completed = false;

  if((millis()-timer)>10){ 
  float angle = mpu.getAngleZ();

  Serial.print("\tZ : ");
	Serial.println(angle);
	timer = millis();  
  }

  if (angle = starting_angle) counter++;
  if (counter == revolutions) completed = true;

  return completed;
}

// TODO code up motor movement functions
// we may need to create motor classes probs easier
