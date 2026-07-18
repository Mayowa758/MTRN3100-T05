#include "Motor.hpp"
#include "PIDController.hpp"
#include <MPU6050_light.h>

#define MOT1PWM 11
#define MOT1DIR 12
mtrn3100::Motor motor1(MOT1PWM, MOT1DIR);

#define MOT2PWM 9
#define MOT2DIR 10
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);

MPU6050 mpu(Wire);

// 转向PID/TURN PID。转得慢可增大Kp；来回摆动可减小Kp或增大Kd。/ Increase Kp for a slow turn; reduce Kp or increase Kd for overshoot.
mtrn3100::PIDController headingController(1.5f, 0.0f, 0.05f);
const float TURN_ANGLE_DEG = 90.0f;
// PWM调节。MAX控制最高转速；MIN应为刚好能原地转动的最低PWM。/ MAX limits turn speed; MIN should be the lowest PWM 
const int16_t MAX_PWM = 150;
const int16_t MIN_TURN_PWM = 50;
// 停车容差/TOLERANCE。抖动可略增但必须小于5。/ increase slightly for noise, but stay below 5.
const float STOP_TOLERANCE_DEG = 3.0f;
// 控制周期/CONTROL PERIOD: 噪声大可增大；响应迟缓可小幅减小。/ Increase for noisy control; reduce slightly if response is sluggish.
const unsigned long CONTROL_INTERVAL_US = 10000;

// 方向调试/DIRECTION。 开机必须顺时针90度；若变成逆时针，只改-1为+1。 / change +1 if it turns anticlockwise.
const float TURN_SIGN = -1.0f;
const int MOTOR2_TURN_SIGN = -1;  // 原地转向符号/MOTOR SIGN

float targetHeading = 0.0f;
bool imuReady = false;
unsigned long lastControlTime = 0;

void rotate(float pwm) {
    motor1.setPWM(pwm);
    motor2.setPWM(MOTOR2_TURN_SIGN * pwm);
}

void setup() {
    Wire.begin();

    byte imuStatus = mpu.begin();
    if (imuStatus != 0) {
        rotate(0);
        return;
    }
    mpu.calcOffsets(true, true);
    delay(500);
    mpu.update();

    float startHeading = mpu.getAngleZ();
    targetHeading = startHeading + TURN_SIGN * TURN_ANGLE_DEG;

    headingController.zeroAndSetTarget(0.0f, targetHeading);

    imuReady = true;
}

void loop() {
    mpu.update();

    if (!imuReady) {
        rotate(0);
        return;
    }

    unsigned long now = micros();
    if (now - lastControlTime < CONTROL_INTERVAL_US) return;
    lastControlTime = now;

    float error = targetHeading - mpu.getAngleZ();
    if (fabs(error) <= STOP_TOLERANCE_DEG) {
        rotate(0);
        return;
    }

    float output = headingController.compute(mpu.getAngleZ());
    output = constrain(output, (float)-MAX_PWM, (float)MAX_PWM);

    if (fabs(output) < MIN_TURN_PWM) {
        output = output < 0.0f ? -MIN_TURN_PWM : MIN_TURN_PWM;
    }

    rotate(output);

}
