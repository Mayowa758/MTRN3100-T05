#include "Motor.hpp"
#include "Lidar.hpp"
#include "DualEncoder.hpp"
#include "Robot.hpp"
#include "Maze-small.hpp"
#include <MPU6050_light.h>
#include <Wire.h>
#include <U8g2lib.h>

// motors
#define MOT1PWM 11
#define MOT1DIR 12
mtrn3100::Motor motorL(MOT1PWM, MOT1DIR);

#define MOT2PWM 9
#define MOT2DIR 10
mtrn3100::Motor motorR(MOT2PWM, MOT2DIR);

// Lidar
mtrn3100::Lidar lidarL(A0, 0x30);
mtrn3100::Lidar lidarR(A1, 0x31);
mtrn3100::Lidar lidarF(A2, 0x32);

// Encoder 
#define EN_A  2
#define EN_B  7
#define EN_A2 3
#define EN_B2 8

mtrn3100::DualEncoder encoder(EN_A, EN_B, EN_A2, EN_B2);

// IMU
MPU6050 mpu(Wire);

// robot
Robot robot(motorL, motorR, encoder, mpu, lidarL, lidarR, lidarF);

// maze
mtrn3100::Maze maze(lidarL, lidarR, lidarF, mtrn3100::Maze::SOUTH, 0, 6, 1, 6);

// display
U8G2_SSD1306_128X64_NONAME_1_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

// ---------------- MAIN ----------------
void setup() {
    Wire.begin();
    Serial.begin(9600);

    // OLED
    display.begin();
    display.firstPage();
    do {
        display.setFont(u8g2_font_6x10_tf);
        display.drawStr(0, 10, "Robot Ready");
    } while (display.nextPage());

    // imu initialisation
    byte status = mpu.begin();

    Serial.print("MPU status: ");
    Serial.println(status);
    if (status != 0) {
        Serial.println("MPU failed");
        while (1);
    }
    delay(1000);
    mpu.calcOffsets(true, true);
    robot.initialiseHeading();

    // Shut every VL6180X down before assigning unique I2C addresses. Without
    // this, more than one sensor can respond at the default address and create
    // false wall readings.
    pinMode(A0, OUTPUT);
    pinMode(A1, OUTPUT);
    pinMode(A2, OUTPUT);
    digitalWrite(A0, LOW);
    digitalWrite(A1, LOW);
    digitalWrite(A2, LOW);
    delay(20);

    // Bring up one sensor at a time so each receives its own I2C address.
    lidarL.init();
    lidarR.init();
    lidarF.init();

    maze.mapCurrentCell();
}

void loop() {
    static bool pathFound = false;
    static bool pathDriven = false;

    maze.displayMap(display);

    // ---------------- MAPPING ----------------

    if (!maze.mappingComplete()) {

        maze.mappingStep(robot);
    }

    // ---------------- SHORTEST PATH ----------------

    else {

        if (!pathFound) {

            Serial.println("Mapping complete");
            Serial.println("Finding shortest path...");

            pathFound = maze.findShortestPath();

            if (pathFound) {
                Serial.println("Shortest path found");
            }
            else {
                Serial.println("No path found");
                robot.stopMotors();
            }
        }

        if (pathFound && !pathDriven) {

            pathDriven = maze.driveShortestPath(robot);

            if (pathDriven) {
                Serial.println("Reached goal");
            }
        }
    }
    delay(5);
}
