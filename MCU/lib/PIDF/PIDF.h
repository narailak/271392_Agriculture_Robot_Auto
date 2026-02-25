#ifndef PIDF_H
#define PIDF_H

#include <Arduino.h>
// #include <Utilize.h>

class PIDF {
  private:
    // unsigned long long LastTime;
  public:
    float Kp, Ki, Kd, Kf, Setpoint, Error, LastError, error_tolerance;
    float Dt, Integral, i_min, i_max, min_val, max_val;
    float compute(float, float);
    float compute_with_error(float);
    // float Calculate(float);
    static byte SigNum(float);
    static boolean AtTargetRange(float number, float target, float range) {
        return target - range < number && number < target + range;
    }
    PIDF(float, float, float, float, float, float, float, float, float);
    void setPIDF(float, float, float, float, float);
    void reset();
};

#endif