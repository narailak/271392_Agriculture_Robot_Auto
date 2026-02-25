#ifndef UTILIZE_H
#define UTILIZE_H

#include <Arduino.h>



float ToDegrees(float radians) {
    return radians / 2.0 / PI * 360.0;
}

float ToRadians(float degrees) {
    return degrees / 360.0 * 2.0 * PI;
}

static boolean AtTargetRange(float number, float target, float range) {
        return target - range < number && number < target + range;
    }

static double WrapRads(float rads) {
    if (rads >  PI) return rads - (2.0 * PI);
    if (rads < -PI) return rads + (2.0 * PI);
return rads;
}

static float WrapDegs(float degs) {
    if (degs >  180) return degs - 360;
    if (degs < -180) return degs + 360;
return degs;
}

bool AtTargetAngle(float current_angle, float target_angle, float tolerance) {
    float diff = abs(WrapDegs(target_angle - current_angle));
    return diff <= tolerance;
}


static float NormalizeRads(float rads) {
    rads = fmod(rads, 2.0 * PI);
    if (rads == (2.0 * PI)) return 0;
    return rads;
}

static float NormalizeDegs(float degs) {
    degs = fmod(degs, 360);
    if (degs == 360) return 0;
    return degs;
}

static int SigNum(float number) {
    return (int) (number == 0 ? 0 : (number < 0 ? -1 : 1));
}

float MPSToRPM(float mps, float wheel_diameter) {
    return (mps * 60.0) / (PI * wheel_diameter);

}

#endif