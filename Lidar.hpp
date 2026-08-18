#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <VL6180X.h>

namespace mtrn3100 {

class Lidar {
public:

    Lidar(uint8_t xshutPin, uint8_t address)
        : xshutPin(xshutPin), address(address) {}

    bool init() {

        // Set XSHUT pin as an output
        pinMode(xshutPin, OUTPUT);

        // Turn this sensor on
        digitalWrite(xshutPin, HIGH);

        delay(10);

        // Initialise sensor
        sensor.init();
        sensor.configureDefault();

        // Give this sensor a unique I2C address
        sensor.setAddress(address);

        // Configure sensor
        sensor.setScaling(2);
        sensor.setTimeout(500);

        initialized = true;

        return true;
    }

    // returns distance reading in mm
    uint16_t readDistance() {

        uint16_t d = sensor.readRangeSingleMillimeters();

        lastReadValid =
            initialized &&
            !sensor.timeoutOccurred() &&
            d > 0 &&
            sensor.readRangeStatus() == VL6180X_ERROR_NONE;

        if (lastReadValid) {
            lastValid = d;
            return d;
        }

        // Do not return a stale close-wall distance when the new reading is
        // invalid or out of range. Callers should treat this as no detection.
        return 0;
    }

    bool isLastReadValid() const {
        return lastReadValid;
    }

private:

    VL6180X sensor;

    uint8_t xshutPin;
    uint8_t address;

    uint16_t lastValid = 0;

    bool initialized = false;
    bool lastReadValid = false;
};

}
