#pragma once

#include <Arduino.h>

namespace mtrn3100 
{
    class EncoderOdometry
    {
    public:
        EncoderOdometry(float radius, float wheelBase) : x(0), y(0), h(0), R(radius), B(wheelBase), lastLPos(0), lastRPos(0) {}

        // TODO: COMPLETE THIS FUNCTION
        void update(float leftValue, float rightValue)
        {

            // Change in wheel angle since last update
            float deltaLeft = leftValue - lastLPos;
            float deltaRight = rightValue - lastRPos;

            // Store for next iteration
            lastLPos = leftValue;
            lastRPos = rightValue;

            // Wheel distances travelled
            float dL = deltaLeft * R;
            float dR = deltaRight * R;

            // Robot motion
            float dS = (dL + dR) * 0.5f;
            float dTheta = (dR - dL) / B;

            // Update pose
            float midHeading = h + dTheta * 0.5f;

            x += dS * cos(midHeading);
            y += dS * sin(midHeading);
            h += dTheta;
        }

        float getX() const { return x; }
        float getY() const { return y; }
        float getH() const { return h; }

    private:
        float x, y, h;
        const float R, B;
        float lastLPos, lastRPos;
    };

}
