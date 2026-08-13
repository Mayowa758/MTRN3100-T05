#pragma once
#include <math.h>

namespace mtrn3100 {

class PIDController {
public:
    PIDController(float kp, float ki, float kd)
        : prev_time(0), curr_time(0), dt(0), kp(kp), ki(ki), kd(kd),
          error(0), derivative(0), integral(0), output(0), prev_error(0),
          setpoint(0), zero_ref(0), derivative_ready(false) {}

    float compute(float input) {
        curr_time = micros();
        dt = static_cast<float>(curr_time - prev_time) / 1e6f;
        prev_time = curr_time;

        if (dt <= 0.0f) dt = 1.0e-6f;

        error = setpoint - (input - zero_ref);

        float proportional = error;
        integral = integral + error * dt;
        if (derivative_ready) {
            derivative = (error - prev_error) / dt;
        } else {
            derivative = 0.0f;
            derivative_ready = true;
        }
        output = kp * proportional + ki * integral + kd * derivative;

        prev_error = error;

        return output;
    }

    float getError() {
        return error;
    }

    void zeroAndSetTarget(float zero, float target) {
        prev_time = micros();
        zero_ref = zero;
        setpoint = target;
        error = target;
        prev_error = error;
        integral = 0.0f;
        derivative = 0.0f;
        output = 0.0f;
        derivative_ready = false;
    }

public:
    uint32_t prev_time, curr_time;
    float dt;

private:
    float kp, ki, kd;
    float error, derivative, integral, output;
    float prev_error;
    float setpoint;
    float zero_ref;
    bool derivative_ready;
};

}
