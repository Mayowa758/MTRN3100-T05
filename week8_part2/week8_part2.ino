#include "Motor.hpp"
#include "PIDController.hpp"
#include "Lidar.hpp"

#define MOT1PWM 11
#define MOT1DIR 12
mtrn3100::Motor motor1(MOT1PWM, MOT1DIR);

#define MOT2PWM 9
#define MOT2DIR 10
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);

mtrn3100::Lidar lidar;

//跟墙太慢可增大Kp；冲过目标/振荡可减小Kp或增大Kd。
// Increase Kp if wall following is slow; reduce Kp or increase Kd for overshoot.
mtrn3100::PIDController distanceController(0.6f, 0.0f, 0.05f);

// 目标为小车前端到墙的距离 / Target is front-of-robot to wall
const float TARGET_DISTANCE_MM = 100.0f;

// 撞墙风险高先降低MAX；无法克服静摩擦则逐步提高MIN。
// Lower MAX first if approach is unsafe; raise MIN gradually if the robot cannot break friction.
const int16_t MAX_PWM = 150;
const int16_t MIN_EFFECTIVE_PWM = 45;

// 停车死区/DEADBAND。抖动可增大但不要超过5。
// Increase for noise, but never above 5mm.
const float DISTANCE_TOLERANCE_MM = 4.0f;

// 控制周期/CONTROL PERIOD: 过小会放大噪声，过大会响应迟缓。
// Too short amplifies noise; too long makes wall following sluggish.
const unsigned long CONTROL_INTERVAL_MS = 20;

bool lidarReady = false;
unsigned long lastControlTime = 0;

void stopMotors() {
    motor1.setPWM(0);
    motor2.setPWM(0);
}

void setup() {
    Wire.begin();

    lidarReady = lidar.init();
    if (!lidarReady) {
        stopMotors();
        return;
    }

    distanceController.zeroAndSetTarget(0.0f, TARGET_DISTANCE_MM);

}

void loop() {
    if (!lidarReady) {
        stopMotors();
        return;
    }

    if (millis() - lastControlTime < CONTROL_INTERVAL_MS) return;
    lastControlTime = millis();

    uint16_t distance = lidar.readDistance();

    if (!lidar.isLastReadValid()) {
        stopMotors();
        return;
    }

    float distanceError = TARGET_DISTANCE_MM - (float)distance;
    if (fabs(distanceError) <= DISTANCE_TOLERANCE_MM) {
        stopMotors();
        return;
    }

    float output = -distanceController.compute((float)distance);
    output = constrain(output, (float)-MAX_PWM, (float)MAX_PWM);

    if (fabs(output) < MIN_EFFECTIVE_PWM) {
        output = output < 0.0f ? -MIN_EFFECTIVE_PWM : MIN_EFFECTIVE_PWM;
    }

    motor1.setPWM(output);
    motor2.setPWM(-output);


}
