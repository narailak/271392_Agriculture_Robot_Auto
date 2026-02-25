#ifndef ESP32_ENCODER_H
#define ESP32_ENCODER_H

#include <Arduino.h>
#include <ESP32Encoder.h>    

class esp32_Encoder {
    public:
        esp32_Encoder(int encoder_A, int encoder_B, int counts_per_rev, bool invert = false, float gear_ratio = 1.0f, float wheel_diameter = 1.0f) :
            counts_per_rev_(counts_per_rev),
            encoder_A_(invert ? encoder_B : encoder_A),
            encoder_B_(invert ? encoder_A : encoder_B),
            gear_ratio_(gear_ratio), wheel_diameter_(wheel_diameter)
            
            {
            ESP32Encoder::useInternalWeakPullResistors = puType::up;    
            Encoder.attachFullQuad(encoder_A_, encoder_B_);
            Encoder.setCount(0);
        }

        int64_t read(){
            return Encoder.getCount();
        }

        void reset(){
            Encoder.setCount(0);
        }

        int64_t read_and_reset(){
            int64_t ticks = read();
            reset();
            return ticks;
        }   

        float getRPM(){
            int64_t encoder_ticks = read();
            //this function calculates the motor's RPM based on encoder ticks and delta time
            unsigned long current_time = micros();
            unsigned long dt = current_time - prev_update_time_;
            if (dt <= 0) return 0;

            //convert the time from microseconds to minutes
            double dtm = (double)dt / 60000000.0;
            double delta_ticks = encoder_ticks - prev_encoder_ticks_;

            //calculate wheel's speaed (RPM)
            prev_update_time_ = current_time;
            prev_encoder_ticks_ = encoder_ticks;

            return ((delta_ticks / float(counts_per_rev_)) / dtm) / gear_ratio_;
        }

        float getLinearSpeedMps(){
            float wheel_rpm = getRPM();                     // จากฟังก์ชันด้านบน
            double r = wheel_diameter_ * 0.5;               // เมตร
            double v_mps = (wheel_rpm * 2.0 * M_PI * r) / 60.0;
            return (float)v_mps;
        }


    private:
        ESP32Encoder Encoder;
        int counts_per_rev_, encoder_A_, encoder_B_;
        float gear_ratio_, wheel_diameter_;
        unsigned long prev_update_time_;
        long prev_encoder_ticks_;

};

#endif