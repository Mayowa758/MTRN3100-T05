#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <VL6180X.h>

namespace mtrn3100 {

class Lidar {
public:
    Lidar() {}

    bool init() {
        sensor.init();
        sensor.configureDefault();
        sensor.setScaling(2);
        sensor.setTimeout(500);
        initialized = true;
        return true;
    }

    uint16_t readDistance() {
        uint16_t d = sensor.readRangeSingleMillimeters();
        lastReadValid = initialized && !sensor.timeoutOccurred() && d > 0 &&
                        sensor.readRangeStatus() == VL6180X_ERROR_NONE;
        if (lastReadValid) {
            lastValid = d;
        }
        return lastValid;
    }

    bool isLastReadValid() const {
        return lastReadValid;
    }

private:
    VL6180X sensor;
    uint16_t lastValid = 0;
    bool initialized = false;
    bool lastReadValid = false;
};

}
