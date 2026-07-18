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

// 若小车前进时原地旋转，说明某个电机方向相反；优先修改下方
// If a forward command makes the robot spin, one motor direction is reversed;
#define EN_A  2
#define EN_B  7
#define EN_A2 3
#define EN_B2 8
mtrn3100::DualEncoder encoder(EN_A, EN_B, EN_A2, EN_B2);

//前两个参数为左右轮位置P增益，第三个为D增益。
// 到终点太慢可增大P；终点附近振荡可减小P或增大D。
// The first value is position Kp and the third is Kd for each wheel.
// Increase Kp if approach is too slow; reduce Kp or increase Kd if it oscillates.
mtrn3100::PIDController controller1(30, 0, 0.3);
mtrn3100::PIDController controller2(30, 0, 0.3);

MPU6050 mpu(Wire);
float targetHeading = 0.0f;

const float WHEEL_RADIUS    = 0.016f;  //轮半径单位为m  / Wheel radius is in metres
const float TARGET_DISTANCE = 1.0f;
const float K_HEADING       = 27.0f;  // 航向调节/ heading adjustment 
const float K_SYNC          = 1.1f;  // 双轮同步/WHEEL SYNC: 编码器显示一侧持续领先时增大；左右命令互相打架时减小。/ Increase if one encoder consistently leads; decrease if wheel-sync fights heading control.
const float STOP_TOLERANCE  = 0.05f; // 停车阈值单位为轮子弧度。/Stop tolerance is in wheel radians.
// PWM调节/PWM: MAX控制速度；MIN必须略高于电机刚好能够启动的PWM。
// MAX limits speed; MIN should be just above the measured motor breakaway PWM.
const int16_t MAX_PWM        = 150;
const int16_t MIN_EFFECTIVE_PWM = 45;
// 稳定时间/SETTLING: 到位后仍滑动就增大；动作确认太慢可小幅减小。
// Increase if the robot still coasts after arrival; reduce slightly if confirmation is too slow.
const unsigned long SETTLE_TIME_MS = 250;
const unsigned long DRIVE_TIMEOUT_MS = 30000;//编码器故障时强制停车，不建议删除。/Stops the robot on encoder/control failure
const float targetAngle = TARGET_DISTANCE / WHEEL_RADIUS;

unsigned long lastPIDTime = 0;
const unsigned long PID_INTERVAL = 10000;  

bool finished = false;  
bool failed = false;
unsigned long driveStartTime = 0;
unsigned long withinToleranceSince = 0;

float applyMinimumPWM(float command) {
    if (command > 0.0f && command < MIN_EFFECTIVE_PWM) return MIN_EFFECTIVE_PWM;
    if (command < 0.0f && command > -MIN_EFFECTIVE_PWM) return -MIN_EFFECTIVE_PWM;
    return command;
}

void setup() {
    Wire.begin();
    byte imuStatus = mpu.begin();
    if (imuStatus != 0) {
        failed = true;
        return;
    }
    mpu.calcOffsets(true, true);
    delay(500);
    mpu.update();

    controller1.zeroAndSetTarget(encoder.getLeftRotation(), targetAngle);
    controller2.zeroAndSetTarget(-encoder.getRightRotation(), targetAngle);

    targetHeading = mpu.getAngleZ();  
    driveStartTime = millis();
}

void loop() {
    if (finished || failed) {
        motor1.setPWM(0);
        motor2.setPWM(0);
        return;
    }

    unsigned long currentTime = micros();
    mpu.update();

    if (currentTime - lastPIDTime >= PID_INTERVAL) {
        lastPIDTime = currentTime;

        float leftPos  = encoder.getLeftRotation();
        float rightPos = -encoder.getRightRotation();

        float leftPWM  = controller1.compute(leftPos);
        float rightPWM = controller2.compute(rightPos);

        float headingError = targetHeading - mpu.getAngleZ();
        float headingCorrection = K_HEADING * headingError;
        headingCorrection = constrain(headingCorrection, -60.0f, 60.0f);

        float syncError = leftPos - rightPos;
        float syncCorrection = syncError * K_SYNC;

        float leftCmd  = constrain(leftPWM - headingCorrection - syncCorrection, (float)-MAX_PWM, (float)MAX_PWM);
        float rightCmd = constrain(-(rightPWM + headingCorrection + syncCorrection), (float)-MAX_PWM, (float)MAX_PWM);
        leftCmd = applyMinimumPWM(leftCmd);
        rightCmd = applyMinimumPWM(rightCmd);
        motor1.setPWM(leftCmd);
        motor2.setPWM(rightCmd);

        bool withinTolerance = fabs(controller1.getError()) < STOP_TOLERANCE &&
                               fabs(controller2.getError()) < STOP_TOLERANCE;
        if (withinTolerance) {
            motor1.setPWM(0);
            motor2.setPWM(0);
            if (withinToleranceSince == 0) withinToleranceSince = millis();
            if (millis() - withinToleranceSince >= SETTLE_TIME_MS) {
                finished = true;
            }
        } else {
            withinToleranceSince = 0;
        }

        if (!finished && millis() - driveStartTime >= DRIVE_TIMEOUT_MS) {
            failed = true;
            motor1.setPWM(0);
            motor2.setPWM(0);
        }

    }
}
