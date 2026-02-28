#ifndef ESP32_HARDWARE_H
#define ESP32_HARDWARE_H

#if !defined(Drive_Control) && !defined(Actuator_Control)
#error "Please define Drive_Control or Actuator_Control in build_flags"
#endif

// =====================================================
// ================== Drive_Control ====================
// =====================================================
#if defined(Drive_Control)

// MOTOR PIN CONFIG
#define L_DIR1  32
#define L_PWM1  33

#define L_DIR2  26
#define L_PWM2  25

#define R_DIR1  22
#define R_PWM1  23

#define R_DIR2  19
#define R_PWM2  21


//  PWM CONFIG
#define PWM_FREQ       20000
#define PWM_RESOLUTION 8

#define PWM_CH_M1 0
#define PWM_CH_M2 1
#define PWM_CH_M3 2
#define PWM_CH_M4 3


//  ENCODER PIN CONFIG
#define ENC1_A 35
#define ENC1_B 34

#define ENC2_A 27
#define ENC2_B 14

#define ENC3_A 4
#define ENC3_B 16

#define ENC4_A 5
#define ENC4_B 17


// STEPPER TB6600
#define PIN_PUL 13
#define PIN_DIR 18
#define PIN_ENA 15

//   ROBOT PARAMETERS
static const float WHEEL_RADIUS = 0.045f;   // meter
static const float TRACK_WIDTH  = 0.30f;    // meter
static const float MAX_RPM      = 60.0f;


//  CONTROL PARAMETERS
static const float DEADBAND = 0.02f;
static const uint32_t CMD_TIMEOUT_MS = 300;

#endif



// =====================================================
// ================= Actuator_Control ==================
// =====================================================
#if defined(Actuator_Control)

// SERVO PINS 
#define PIN_SERVO_GRIPPER        16
#define PIN_SERVO_DRIL           23
#define PIN_SERVO_SWITCH180      17

// STEPPER (TB6600) 
#define PIN_STEPPER_PUL          25
#define PIN_STEPPER_DIR          26
#define PIN_STEPPER_ENA          27

// LIMIT SWITCH 
#define PIN_LIMIT_LEFT           13
#define PIN_LIMIT_RIGHT          12

// ================= TB6612FNG =================
#define PIN_TB_AIN1              19
#define PIN_TB_AIN2              21
#define PIN_TB_PWMA              22
#define PIN_TB_STBY              5

// PWM CONFIG
#define TB_PWM_CHANNEL  0
#define TB_PWM_FREQ     20000
#define TB_PWM_RES      8

#endif
#endif

