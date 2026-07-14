#include <Wire.h>
#include <VL6180X.h>

VL6180X sensor;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  sensor.init();
  Serial.println("VL6180X Single Lidar Test");
  sensor.configureDefault();
  sensor.setTimeout(500); // Prevents the code from locking up if the sensor stops responding

  
}

void loop() {
  // Use uint16_t to safely handle all possible return values
  uint16_t distance = sensor.readRangeSingleMillimeters();

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" mm");

  // Check if a hardware timeout occurred
  if (sensor.timeoutOccurred()) {
    Serial.print(" (TIMEOUT!)");
  }
  
  Serial.println(); // Prints a newline so each reading starts on a fresh line

  delay(500);
}

// scanner test code
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