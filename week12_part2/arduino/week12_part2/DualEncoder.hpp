#pragma once

#include <Arduino.h>
#include <util/atomic.h>

namespace mtrn3100 {

class DualEncoder {
public:
    DualEncoder(uint8_t leftInterrupt, uint8_t leftDirection,
                uint8_t rightInterrupt, uint8_t rightDirection)
        : leftInterrupt_(leftInterrupt), leftDirection_(leftDirection),
          rightInterrupt_(rightInterrupt), rightDirection_(rightDirection) {
        instance_ = this;
        pinMode(leftInterrupt_, INPUT_PULLUP);
        pinMode(leftDirection_, INPUT_PULLUP);
        pinMode(rightInterrupt_, INPUT_PULLUP);
        pinMode(rightDirection_, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(leftInterrupt_), leftISR, RISING);
        attachInterrupt(digitalPinToInterrupt(rightInterrupt_), rightISR, RISING);
    }

    long leftCount() const {
        long value;
        // Prevent the interrupt from changing a multi-byte value mid-read.
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { value = leftCount_; }
        return value;
    }

    long rightCount() const {
        long value;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { value = rightCount_; }
        return value;
    }

private:
    static void leftISR() {
        if (instance_) {
            instance_->leftCount_ += digitalRead(instance_->leftDirection_) ? 1 : -1;
        }
    }

    static void rightISR() {
        if (instance_) {
            instance_->rightCount_ += digitalRead(instance_->rightDirection_) ? 1 : -1;
        }
    }

    const uint8_t leftInterrupt_;
    const uint8_t leftDirection_;
    const uint8_t rightInterrupt_;
    const uint8_t rightDirection_;
    volatile long leftCount_ = 0;
    volatile long rightCount_ = 0;
    static DualEncoder* instance_;
};

DualEncoder* DualEncoder::instance_ = nullptr;

}
