#ifndef ESP32_HARDWARE_H
#define ESP32_HARDWARE_H

#if !defined(Drive_Control) && !defined(Actuator_Control)
#error "Please define Drive_Control or Actuator_Control in build_flags"
#endif

// =====================================================
// ================== Drive_Control ====================
// =====================================================
#if defined(Drive_Control)

// ================= Robot Specs =================
    #define MOTOR_MAX_RPM 500                               // motor's max RPM          
    #define MAX_RPM_RATIO 0.85                              // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO          
    #define MOTOR_OPERATING_VOLTAGE 12                      // motor's operating voltage (used to calculate max RPM)
    #define MOTOR_POWER_MAX_VOLTAGE 12                      // max voltage of the motor's power source (used to calculate max RPM)
    #define MOTOR_POWER_MEASURED_VOLTAGE 12                 // current voltage reading of the power connected to the motor (used for calibration)

    // ================= PWM Settings =================
    #define PWM_BITS 10                                     // PWM Resolution of the microcontroller
    #define PWM_FREQUENCY 20000                             // PWM Frequency
    #define PWM_Max 1023
    #define PWM_Min (PWM_Max * -1)
    
    #define WHEEL_DIAMETER 0.13                             // wheel's diameter in meters 

    // ================= Motor Config =================
    // Invert Motor Directions  
    #define MOTOR1_INV false
    #define MOTOR2_INV true
    #define MOTOR3_INV false
    #define MOTOR4_INV true

    // Motor Brake
    #define MOTOR1_BRAKE true
    #define MOTOR2_BRAKE true
    #define MOTOR3_BRAKE true
    #define MOTOR4_BRAKE true

    // Motor 1 Parameters (Front Left)
    #define MOTOR1_PWM  -1
    #define MOTOR1_IN_A 4
    #define MOTOR1_IN_B 5 

    // Motor 2 Parameters (Front Right)
    #define MOTOR2_PWM  -1
    #define MOTOR2_IN_A 36
    #define MOTOR2_IN_B 35 

    // Motor 3 Parameters (Back Left)
    #define MOTOR3_PWM  -1
    #define MOTOR3_IN_A 1 
    #define MOTOR3_IN_B 2 

    // Motor 4 Parameters (Back Right)
    #define MOTOR4_PWM  -1
    #define MOTOR4_IN_A 16
    #define MOTOR4_IN_B 15

    // ================= Encoder Config =================
    #define ENCODER_TICKS 11
    #define GEAR_RATIO 534.0168 
    #define COUNTS_PER_REV (ENCODER_TICKS * GEAR_RATIO * 4) // encoder resolution (23496.7392 counts per rev)

    // Encoder Pins (Front Left)
    #define Encoder_LF_A 41
    #define Encoder_LF_B 42
    #define ENCODER_INV_LF true

    // Encoder Pins (Front Right)
    #define Encoder_RF_A 13
    #define Encoder_RF_B 14
    #define ENCODER_INV_RF true

    // Encoder Pins (Back Left)
    #define Encoder_LB_A 39
    #define Encoder_LB_B 40
    #define ENCODER_INV_LB false

    // Encoder Pins (Back Right)
    #define Encoder_RB_A 38
    #define Encoder_RB_B 37
    #define ENCODER_INV_RB false

#endif



// =====================================================
// ================= Actuator_Control ==================
// =====================================================
#if defined(Actuator_Control)

// ================= SERVO PINS =================
#define PIN_SERVO_GRIPPER        16
#define PIN_SERVO_DRIL           23
#define PIN_SERVO_SWITCH180      17

// ================= STEPPER (TB6600) =================
#define PIN_STEPPER_PUL          25
#define PIN_STEPPER_DIR          26
#define PIN_STEPPER_ENA          27

// ================= LIMIT SWITCH =================
#define PIN_LIMIT_LEFT           13  // ซ้าย/-1
#define PIN_LIMIT_RIGHT          12  // ขวา/+1

// ================= TB6612FNG =================
#define PIN_TB_AIN1              19
#define PIN_TB_AIN2              21
#define PIN_TB_PWMA              22
#define PIN_TB_STBY              5

// ================= PWM CONFIG =================
#define TB_PWM_CHANNEL           15  // ยึดตามต้นฉบับ ใช้ 15 เพื่อเลี่ยงชนกับ ESP32Servo
#define TB_PWM_FREQ              20000
#define TB_PWM_RES               8

#endif
#endif

