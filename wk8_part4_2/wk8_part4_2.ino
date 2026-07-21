#include "Motor.hpp"
#include "DualEncoder.hpp"
#include "PIDController.hpp"
#include <MPU6050_light.h>
#define MOT1PWM 11
#define MOT1DIR 12
mtrn3100::Motor motor1(MOT1PWM, MOT1DIR);

#define MOT2PWM 9
#define MOT2DIR 10
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);

#define EN_A  2
#define EN_B  7
#define EN_A2 3
#define EN_B2 8
MPU6050 mpu(Wire);

const char* commandString = "flrfflfr";    //change to test

int index = 0;
float angleTolerance = 5;
int pwm1 = 100;
int pwm2 = 100;


void stopMotors() {
    motor1.setPWM(0);
    motor2.setPWM(0);
}

bool driveForwardCell() {
    // adjust pwm and delay to go 18cm 
    motor1.setPWM(pwm1);
    motor2.setPWM(-pwm2);
    delay(3000);


    stopMotors();
    return;
}

// turning using imu
bool turnLeft() {
    float targetAngle = mpu.getAngleZ() - 90;
    while (fabs(mpu.getAngleZ() - targetAngle) > angleTolerance ) {
        motor1.setPWM(-pwm1);
        motor2.setPWM(-pwm2);
        mpu.update();
    }

    stopMotors();
    return;
}

bool turnRight() {
    float targetAngle = mpu.getAngleZ() + 90;
    while (fabs(mpu.getAngleZ() - targetAngle) > angleTolerance ) {
        motor1.setPWM(pwm1);
        motor2.setPWM(pwm2);
        mpu.update();
    }

    stopMotors();
    return;
}

// // hard coded turning
// bool turnLeft() {
//     motor1.setPWM(-pwm1);
//     motor2.setPWM(-pwm2);
//     delay(3000);

//     stopMotors();
//     return;
// }

// bool turnRight() {
//     motor1.setPWM(pwm1);
//     motor2.setPWM(pwm1);
//     delay(3000);

//     stopMotors();
//     return;
// }

void setup() {
    Serial.begin(9600);
    Wire.begin();
    Serial.println("starting");

    byte imuStatus = mpu.begin();
    if (imuStatus != 0) {
        Serial.print("imu failed");
        stopMotors();
        return;
    }
    mpu.calcOffsets(true, true);
    delay(500);
    mpu.update();
} 

void loop() {
    if (index > 8) {
        return;
    }

    if (commandString[index] == '\0') {
        stopMotors();
        Serial.print("sequence end reached");
        return;
    }

    mpu.update();

    char cmd = commandString[index];

    switch (cmd) {
        case 'f':
            Serial.println("forward");
            driveForwardCell();
            break;
        case 'l':
            Serial.println("left");
            turnLeft();
            break;
        case 'r':
            Serial.println("right");
            turnRight();
            break;
        default:
            stopMotors();
            break;
    }

    index++;
}
