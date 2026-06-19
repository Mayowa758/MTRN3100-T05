#pragma once 
#include "math.h"
#include <MPU6050_light.h>

// May need to use in the future but I think right now it is quite irrelevant

// MPU5050 mpu(Wire);

// namespace mtrn3100 {

//   class IMU {


//     public:
//       IMU() {
//         mpu.calculateOffsets();
//         Serial.print("Done");
//       }

//       float getAngleZ() {
//         return mpu.getAngleZ();
//       }

//       float getAngleX() {
//         return mpu.getAngleX();
//       }

//       float getAngleY() {
//         return mpu.getAngleY();
//       }

//       void printAngleX() {
//         Serial.print("This is the the X-position: ");
//         Serial.println(getAngle(X));
//       }

//       void printAngleY() {
//         Serial.print("This is the the Y-position: ");
//         Serial.println(getAngle(Y));
//       }

//       void printAngleZ() {
//         Serial.print("This is the the Z-position: ");
//         Serial.println(getAngle(Z));
//       }

//   }

// }