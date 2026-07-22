#pragma once

#include <Arduino.h>
#include <util/atomic.h>

namespace mtrn3100 {

class DualEncoder {
public:
    DualEncoder(uint8_t enc1, uint8_t enc2, uint8_t enc3, uint8_t enc4)
        : mot1_int(enc1), mot1_dir(enc2), mot2_int(enc3), mot2_dir(enc4) {
        instance = this;
        pinMode(mot1_int, INPUT_PULLUP);
        pinMode(mot1_dir, INPUT_PULLUP);
        pinMode(mot2_int, INPUT_PULLUP);
        pinMode(mot2_dir, INPUT_PULLUP);

        attachInterrupt(digitalPinToInterrupt(mot1_int), DualEncoder::readLeftEncoderISR, RISING);
        attachInterrupt(digitalPinToInterrupt(mot2_int), DualEncoder::readRightEncoderISR, RISING);
    }

    void readLeftEncoder() {
        int8_t direction = digitalRead(mot1_dir) ? 1 : -1;
        l_count += direction;
    }

    void readRightEncoder() {
        int8_t direction = digitalRead(mot2_dir) ? 1 : -1;
        r_count += direction;
    }

    float getLeftRotation() {
        long count;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            count = l_count;
        }
        return (static_cast<float>(count) / counts_per_revolution) * 2 * PI;
    }

    float getRightRotation() {
        long count;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            count = r_count;
        }
        return (static_cast<float>(count) / counts_per_revolution) * 2 * PI;
    }

private:
    static void readLeftEncoderISR() {
        if (instance != nullptr) {
            instance->readLeftEncoder();
        }
    }

    static void readRightEncoderISR() {
        if (instance != nullptr) {
            instance->readRightEncoder();
        }
    }

public:
    const uint8_t mot1_int, mot1_dir, mot2_int, mot2_dir;
    // 实车标定/CALIBRATION: 每圈编码器计数。距离按固定比例错误时修改此值。
    // Encoder counts per wheel revolution. Change this if distance has a consistent scale error.
    uint16_t counts_per_revolution = 693;  
    volatile long l_count = 0;
    volatile long r_count = 0;

private:
    static DualEncoder* instance;
};

DualEncoder* DualEncoder::instance = nullptr;

}  
