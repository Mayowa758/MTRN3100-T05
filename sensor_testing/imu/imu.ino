#include <Wire.h>
#include <MPU6050_light.h>

// This is the imu test file


MPU6050 mpu(Wire);
void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    Wire.begin();
    Serial.println("hi");
    byte status = mpu.begin();
    if (status != 0) {
      Serial.println("Could not find a valid MPU sensors");
    }

    delay(1000);
    mpu.calcOffsets();
    Serial.println("Done");
}

void loop() {
  // put your main code here, to run repeatedly:
  mpu.update();

  Serial.print("Yaw: ");
  Serial.print(mpu.getAngleZ());
  Serial.println(" deg");
  delay(500);
}

// Standard Code

// #include <Wire.h>

// void setup() {
//   Wire.begin();
//   Serial.begin(9600);
//   while (!Serial); // Wait for serial monitor
//   Serial.println("\nI2C Scanner");
// }

// void loop() {
//   byte error, address;
//   int nDevices = 0;

//   Serial.println("Scanning...");

//   for (address = 1; address < 127; address++ ) {
//     Wire.beginTransmission(address);
//     error = Wire.endTransmission();

//     if (error == 0) {
//       Serial.print("I2C device found at address 0x");
//       if (address < 16) Serial.print("0");
//       Serial.print(address, HEX);
//       Serial.println("  !");
//       nDevices++;
//     }
//     else if (error == 4) {
//       Serial.print("Unknown error at address 0x");
//       if (address < 16) Serial.print("0");
//       Serial.println(address, HEX);
//     }    
//   }
//   if (nDevices == 0) Serial.println("No I2C devices found\n");
//   else Serial.println("done\n");

//   delay(5000);           
// }