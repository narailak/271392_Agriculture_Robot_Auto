#include "PIDF.h"

PIDF::PIDF(float min_val, float max_val, float Kp = 0, float Ki = 0, float i_min = 0, float i_max = 0, float Kd = 0, float Kf = 0, float error_tolerance = 0) {
  this->setPIDF(Kp, Ki, Kd, Kf, error_tolerance);
  this->i_min = i_min;
  this->i_max = i_max;
  this->min_val = min_val;
  this->max_val = max_val;
  // Setpoint = Dt = Error = Integral = LastTime = LastError = 0;
  Setpoint = Dt = Error = Integral = LastError = 0;
}

void PIDF::setPIDF(float Kp, float Ki, float Kd, float Kf, float error_tolerance) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->Kf = Kf;
  this->error_tolerance = error_tolerance;
}

void PIDF::reset(){
  // this->LastTime = 0;
  this->Integral = 0;
  this->LastError = 0;
  // this->LastTime = 0;
}

float PIDF::compute(float setpoint, float measure){
  Setpoint = setpoint;
  // unsigned long CurrentTime = millis();
  // Dt = (CurrentTime - LastTime) / 1000.0;
  // if (Dt <  1E-6) return;
  // Serial.println(Dt);
  Error = setpoint - measure;
  if (setpoint == 0 && Kf > 0) return 0;
  return compute_with_error(Error);
}

byte PIDF::SigNum(float number) {
    return (byte) (number == 0 ? 0 : (number < 0 ? -1 : 1));
}

float PIDF::compute_with_error(float error) {
  if (AtTargetRange(error, 0, this->error_tolerance)) {
    Integral = 0;
    LastError = 0;
    return 0; // If the error is within the tolerance, return 0
  }
  // Integral += Error * Dt;
  Integral += error;
  if(i_min != -1 && i_max != -1) Integral = constrain(Integral, this->i_min, this->i_max);
  // float Derivative = (Error - LastError) / Dt;
  float Derivative = error - LastError;
  if (Setpoint == 0) {
    if (Kf > 0 || error == 0) {
      Integral = 0;
      Derivative = 0;
    }
  } 
  LastError = error;
  // Serial.println("Error: " + String(Error, 2) + " | Integral: " + String(Integral, 2) + " | Derivative: " + String(Derivative, 2));
  // LastTime = CurrentTime;
  // return constrain(Kp * Error + Ki * Integral + Kd * Derivative + Kf * SigNum(Error), min_val, max_val);
  return constrain(Kp * error + Ki * Integral + Kd * Derivative + Kf * Setpoint, min_val, max_val);
}