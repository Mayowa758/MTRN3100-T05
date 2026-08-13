#pragma once

#include <Arduino.h>
#include <VL6180X.h>

namespace mtrn3100 {

class Lidar {
public:
    Lidar() : lastReadValid(false) {}

    bool init(uint8_t newAddress) {
        // This matches the Pololu VL6180X library version where init() returns void.
        sensor.init();
        sensor.configureDefault();

        // Scaling 2 gives a longer usable range than the default scaling.
        sensor.setScaling(2);
        sensor.setTimeout(100);

        // Each sensor is brought out of XSHUT individually before this call,
        // so it is safe to change its address from the default 0x29.
        sensor.setAddress(newAddress);

        // Continuous ranging keeps the reads short inside the movement loop.
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

} // namespace mtrn3100
