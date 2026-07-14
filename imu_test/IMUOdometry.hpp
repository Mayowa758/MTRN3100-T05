#ifndef IMU_ODOMETRY_HPP
#define IMU_ODOMETRY_HPP

#include <Arduino.h>

namespace mtrn3100 {
    class IMUOdometry {
    public:
        IMUOdometry() : x(0), y(0), vx(0), vy(0), lastUpdateTime(millis()) {}

        void update(float accel_x, float accel_y) {
            unsigned long currentTime = millis();
            float dt = (currentTime - lastUpdateTime) / 1000.0f;
            lastUpdateTime = currentTime;

            vx += accel_x * dt;
            vy += accel_y * dt;

            x += vx * dt;
            y += vy * dt;
        }

        float getX() const { return x; }
        float getY() const { return y; }

    private:
        float x, y;
        float vx, vy;
        unsigned long lastUpdateTime;
    };
}

#endif // IMU_ODOMETRY_HPP
