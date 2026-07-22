#pragma once

#include <Arduino.h>
#include "math.h"

namespace mtrn3100 {

class Encoder {
public:
    Encoder(uint8_t enc1, uint8_t enc2) : encoder1_pin(enc1), encoder2_pin(enc2) {
        // Assign this instance to the correct static pointer based on the interrupt pin
        if (encoder1_pin == 2) {
            instancePin2 = this;
            attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoderISRPin2, RISING);
        } else if (encoder1_pin == 3) {
            instancePin3 = this;
            attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoderISRPin3, RISING);
        }
        
        pinMode(encoder1_pin, INPUT_PULLUP);
        pinMode(encoder2_pin, INPUT_PULLUP);
    }

    // Encoder function used to update the encoder
    void readEncoder() {
        // Note: No need to call noInterrupts() inside the ISR, 
        // AVR microcontrollers disable interrupts automatically while running an ISR.
        if (digitalRead(encoder2_pin) == HIGH) {
            count++;
        } else {
            count--;
        }
    }

    // Helper function to convert encoder count to radians
    float getRotation() {
        // Protect reading the multi-byte volatile long variable from being corrupted mid-read
        noInterrupts();
        long currentCount = count;
        interrupts();

        // Ensure floating point math by using 1500.0 instead of 1500
        return (static_cast<float>(currentCount) / 730) * 2.0 * PI;
    }

private:
    // Separate ISRs for each hardware interrupt pin
    static void readEncoderISRPin2() {
        if (instancePin2 != nullptr) {
            instancePin2->readEncoder();
        }
    }

    static void readEncoderISRPin3() {
        if (instancePin3 != nullptr) {
            instancePin3->readEncoder();
        }
    }

public:
    const uint8_t encoder1_pin;
    const uint8_t encoder2_pin;
    volatile int8_t direction;
    float position = 0;
    uint16_t counts_per_revolution = 693; 
    volatile long count = 0;
    uint32_t prev_time;
    bool read = false;

private:
    // Unique static pointers for the two hardware interrupt pins on the Nano
    static Encoder* instancePin2;
    static Encoder* instancePin3;
};

// Initialize static members
Encoder* Encoder::instancePin2 = nullptr;
Encoder* Encoder::instancePin3 = nullptr;

}  // namespace mtrn3100