#pragma once

#include <Arduino.h>
#include <VL6180X.h>

namespace mtrn3100 {

class Lidar {
public:
    Lidar() : lastReadValid(false) {}

    bool init(uint8_t newAddress) {
        sensor.init();
        sensor.configureDefault();
        sensor.setScaling(2);
        sensor.setTimeout(100);
        sensor.setAddress(newAddress);
        sensor.startRangeContinuous(20);
        delay(20);
        lastReadValid = true;
        return true;
    }

    uint16_t readDistance() {
        uint16_t distance = sensor.readRangeContinuousMillimeters();
        lastReadValid =
            !sensor.timeoutOccurred() &&
            sensor.readRangeStatus() == VL6180X_ERROR_NONE &&
            distance > 0;
        return distance;
    }

    bool isLastReadValid() const {
        return lastReadValid;
    }

private:
    VL6180X sensor;
    bool lastReadValid;
};

}