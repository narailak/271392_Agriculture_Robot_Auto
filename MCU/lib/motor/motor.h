#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

#if defined(ESP32)
    #include <ESP32Servo.h>
#else
    #include <Servo.h>
#endif

class Controller
{
    public:
        enum driver {Drive2pin, Drive3pin, SERVO};
        Controller(driver motor_driver = Drive2pin, float pwm_frequency = 20000, int pwm_bits = 10,
                    bool invert = false, bool brakemotor = true, int pwm_pin = -1,
                    int motor_pinA = -1, int motor_pinB = -1, int servo_min = 500, int servo_max = 2500):   
                        
                        motor_driver_(motor_driver), 
                        pwm_bits_(pwm_bits), 
                        brakemotor_(brakemotor), 
                        pwm_pin_(pwm_pin), 
                        motor_pinA_(invert ? motor_pinB : motor_pinA),
                        motor_pinB_(invert ? motor_pinA : motor_pinB)
        {
            if (motor_driver_ != SERVO){
                if (pwm_frequency > 0) {
                    #if defined(ESP32)
                        analogWriteFrequency(pwm_frequency);
                    #else
                        analogWriteFrequency(pwm_pin, pwm_frequency);
                    #endif
                }   
                
                analogWriteResolution(pwm_bits);
            }

            switch (motor_driver_) {
                case Drive2pin:
                    pinMode(motor_pinA_, OUTPUT);
                    pinMode(motor_pinB_, OUTPUT);
                    
                    //ensure that the motor is in neutral state during bootup
                    analogWrite(motor_pinB_, 0);
                    analogWrite(motor_pinA_, 0);
                    
                    break;
                    
                case Drive3pin: 
                    pinMode(pwm_pin_, OUTPUT);
                    pinMode(motor_pinA_, OUTPUT);
                    pinMode(motor_pinB_, OUTPUT);

                    //ensure that the motor is in neutral state during bootup
                    analogWrite(pwm_pin_, abs(0));

                    break;

                case SERVO:
                    motor_.attach(pwm_pin_, servo_min, servo_max);

                    //ensure that the motor is in neutral state during bootup
                    motor_.writeMicroseconds(1500);

                    break;
            }
        }

        void spin(int pwm) {
            switch (motor_driver_) {
                case Drive2pin:
                    if (pwm == 0) {
                        brake();
                        break;
                    }     

                    if (pwm > 0) {
                        analogWrite(motor_pinA_, abs(pwm));
                        analogWrite(motor_pinB_, 0);
                    }
                    else if (pwm < 0) {
                        analogWrite(motor_pinA_, 0);
                        analogWrite(motor_pinB_, abs(pwm));
                    }
                    break;

                case Drive3pin:
                    if (pwm == 0) {
                        brake();
                        break;
                    }
                    
                    if(pwm > 0) {
                        digitalWrite(motor_pinA_, HIGH);
                        digitalWrite(motor_pinB_, LOW);
                    }
                    else if(pwm < 0) {
                        digitalWrite(motor_pinA_, LOW);
                        digitalWrite(motor_pinB_, HIGH);
                    }
                    analogWrite(pwm_pin_, abs(pwm));

                    break;

                
                case SERVO:
                    motor_.writeMicroseconds(1500 + pwm);

                    break;
            }
        }

        void brake()
        {
            switch (motor_driver_)
            {
                case Drive2pin:
                    analogWrite(motor_pinA_, brakemotor_ ? HIGH : LOW);
                    analogWrite(motor_pinB_, brakemotor_ ? HIGH : LOW);
                    break;

                case Drive3pin:
                    digitalWrite(motor_pinA_, brakemotor_ ? HIGH : LOW);
                    digitalWrite(motor_pinB_, brakemotor_ ? HIGH : LOW);
                    analogWrite(pwm_pin_, 0);
                    break;

                case SERVO:
                    motor_.writeMicroseconds(1500);
                    break;
            }
        }

    private:
        Servo motor_;
        driver motor_driver_;
        int pwm_pin_, motor_pinA_, motor_pinB_, pwm_bits_;
        bool brakemotor_;

};

#endif