#ifndef ESP32_CONFIG_H
#define ESP32_CONFIG_H

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
    #define MOTOR_OPERATING_VOLTAGE 12                      // motor's operating voltage
    #define MOTOR_POWER_MAX_VOLTAGE 12                      // max voltage of the motor's power source
    #define MOTOR_POWER_MEASURED_VOLTAGE 12                 // current voltage reading of the power connected to the motor

    // ================= PWM Settings =================
    #define PWM_BITS 10                                     // PWM Resolution of the microcontroller
    #define PWM_FREQUENCY 20000                             // PWM Frequency
    #define PWM_Max 1023
    #define PWM_Min (PWM_Max * -1)
    
    #define WHEEL_DIAMETER 0.13                             // wheel's diameter in meters 

    // ================= Motor Config =================

    // Motor Settings (PWM + DIR)
    // ==========================================

    // --- Motor 1 (Front Left / หน้าซ้าย) ---
    #define MOTOR1_INV false
    #define MOTOR1_BRAKE true
    #define L_DIR1  32
    #define L_PWM1  33

    // --- Motor 2 (Front Right / หน้าขวา) ---
    #define MOTOR2_INV true
    #define MOTOR2_BRAKE true
    #define R_DIR1  22
    #define R_PWM1  23

    // --- Motor 3 (Back Left / หลังซ้าย) ---
    #define MOTOR3_INV false
    #define MOTOR3_BRAKE true
    #define L_DIR2  26
    #define L_PWM2  25

    // --- Motor 4 (Back Right / หลังขวา) ---
    #define MOTOR4_INV true
    #define MOTOR4_BRAKE true
    #define R_DIR2  19
    #define R_PWM2  21

    // ================= Encoder Config =================
    #define ENCODER_TICKS 11
    #define GEAR_RATIO 534.0168 
    #define COUNTS_PER_REV (ENCODER_TICKS * GEAR_RATIO * 4)

    #define Encoder_LF_A 41
    #define Encoder_LF_B 42
    #define ENCODER_INV_LF true

    #define Encoder_RF_A 11
    #define Encoder_RF_B 12
    #define ENCODER_INV_RF true

    #define Encoder_LB_A 2
    #define Encoder_LB_B 1
    #define ENCODER_INV_LB false

    #define Encoder_RB_A 10
    #define Encoder_RB_B 9
    #define ENCODER_INV_RB false

    // ================= Stepper Config (TB6600) =================
    #define PIN_PUL 18      
    #define PIN_DIR 17      
    #define PIN_ENA 16     
    #define ENA_ACTIVE_LOW true

    #define BASE_STEPS_PER_REV 200    // 1.8° per step
    #define MICROSTEP 16              // DIP switch setting
    #define STEPPER_GEAR_RATIO 1.0f
    #define STEPS_PER_REV (long)(BASE_STEPS_PER_REV * MICROSTEP * STEPPER_GEAR_RATIO)

    #define HOLD_TORQUE false         // true = ค้างแรงบิด, false = ปลดแรงบิดเพื่อลดความร้อน
    #define STP_CMD_DEBOUNCE_MS 120
    #define STP_CMD_MIN_DELTA_DEG 2
    #define STP_CMD_DEADBAND_TO_TARGET_DEG 1
    // ================= Servo Config =================
    
    #define PIN_SERVO_CAM 15  // TODO: เว้นไว้ก่อนรอเปลี่ยนเลขพิน


#endif



// =====================================================
// ================= Actuator_Control ==================
// =====================================================
#if defined(Actuator_Control)

// ================= SERVO PINS =================
#define PIN_SERVO_GRIPPER        16
#define PIN_SERVO_DRIL           23
#define PIN_SERVO_SWITCH180      17
#define PIN_SERVO_CAM            32   // <<< NEW SERVO

// ================= STEPPER (TB6600) =================
#define PIN_STEPPER_PUL          25
#define PIN_STEPPER_DIR          26
#define PIN_STEPPER_ENA          27

// ================= LIMIT SWITCH =================
#define PIN_LIMIT_LEFT           13
#define PIN_LIMIT_RIGHT          12

// ================= TB6612FNG =================
#define PIN_TB_AIN1              19
#define PIN_TB_AIN2              21
#define PIN_TB_PWMA              22
#define PIN_TB_STBY              5

// ================= PWM CONFIG =================
#define TB_PWM_CHANNEL           15
#define TB_PWM_FREQ              20000
#define TB_PWM_RES               8

#endif
#endif

