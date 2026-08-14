#pragma once

#include <Arduino.h>

namespace mtrn3100 {

class Motor {
public:
    Motor(uint8_t pwmPin, uint8_t directionPin)
        : pwmPin_(pwmPin), directionPin_(directionPin) {
        pinMode(pwmPin_, OUTPUT);
        pinMode(directionPin_, OUTPUT);
    }

    void setPWM(int16_t pwm) {
        // Keep the PWM value in the range accepted by analogWrite.
        pwm = constrain(pwm, -255, 255);
        if (pwm < 0) {
            digitalWrite(directionPin_, HIGH);
            analogWrite(pwmPin_, -pwm);
        } else {
            digitalWrite(directionPin_, LOW);
            analogWrite(pwmPin_, pwm);
        }
    }

private:
    const uint8_t pwmPin_;
    const uint8_t directionPin_;
};

}
