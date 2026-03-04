// =========================== main.cpp (ESP32-S3 + micro-ROS)
// - Differential drive (4 DC motors + 4x encoders) via /tao/cmd_vel
// - Stepper TB6600 absolute angle via /man/cmd_tao (Int16 0..359)
// - ROS_DOMAIN_ID = 96
// ============================================================================

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int16.h>
#include <geometry_msgs/msg/twist.h>

#include "soc/gpio_reg.h"

// ---------------- Utils/Macros ----------
void rclErrorLoop();
#define RCCHECK(fn) { rcl_ret_t rc=(fn); if(rc!=RCL_RET_OK){ rclErrorLoop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = (fn); (void)temp_rc; }
#define EXECUTE_EVERY_N_MS(MS, X) do{ static int64_t t__=-1; if(t__==-1)t__=uxr_millis(); if(uxr_millis()-t__>(MS)){ X; t__=uxr_millis(); } }while(0)

// ============================================================================
//                          micro-ROS CORE VARIABLES
// ============================================================================
rclc_executor_t   executor;
rclc_support_t    support;
rcl_allocator_t   allocator;
rcl_node_t        node;
rcl_init_options_t init_options;

enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
static AgentState state = WAITING_AGENT;

// ============================================================================
//                             DC MOTORS + ENCODERS
// ============================================================================
#define R_DIR1  21
#define R_PWM1  20
#define R_DIR2  36
#define R_PWM2  35
#define L_DIR1  13
#define L_PWM1  14
#define L_DIR2  8
#define L_PWM2  7

#define ENC1_A  41
#define ENC1_B  42
#define ENC2_A  11
#define ENC2_B  12
#define ENC3_A  2
#define ENC3_B  1
#define ENC4_A  10
#define ENC4_B  9

#define PWM_FREQ        20000
#define PWM_RESOLUTION  8
#define PWM_CH_M1 0
#define PWM_CH_M2 1
#define PWM_CH_M3 2
#define PWM_CH_M4 3

const float WHEEL_RADIUS = 0.045f;
const float TRACK_WIDTH  = 0.300f;
const float MAX_RPM      = 60.0f;
const float DEADBAND     = 0.02f;
const float SLEW_RPM_PER_SEC = 200.0f; 
const uint32_t CMD_TIMEOUT_MS = 300;

volatile long encoder1_count=0, encoder2_count=0, encoder3_count=0, encoder4_count=0;

// ROS Entities (DC)
rcl_publisher_t debug_motor_pub;
std_msgs__msg__Float32MultiArray debug_motor_msg;
rcl_publisher_t encoder_pub;
std_msgs__msg__Int16MultiArray encoder_msg;
rcl_subscription_t cmd_sub;
geometry_msgs__msg__Twist cmd_msg;
rcl_timer_t control_timer;

volatile float cmd_vx = 0.0f;
volatile float cmd_wz = 0.0f;
volatile uint32_t drive_last_cmd_ms = 0;
float m1_rpm=0, m2_rpm=0, m3_rpm=0, m4_rpm=0;
float tgt_m1_rpm=0, tgt_m2_rpm=0, tgt_m3_rpm=0, tgt_m4_rpm=0;

// ISR Encoders
void IRAM_ATTR encoder1_ISR(){ (digitalRead(ENC1_A)==digitalRead(ENC1_B))?encoder1_count++:encoder1_count--; }
void IRAM_ATTR encoder2_ISR(){ (digitalRead(ENC2_A)==digitalRead(ENC2_B))?encoder2_count++:encoder2_count--; }
void IRAM_ATTR encoder3_ISR(){ (digitalRead(ENC3_A)==digitalRead(ENC3_B))?encoder3_count++:encoder3_count--; }
void IRAM_ATTR encoder4_ISR(){ (digitalRead(ENC4_A)==digitalRead(ENC4_B))?encoder4_count++:encoder4_count--; }

// ============================================================================
//                                STEPPER (TB6600)
// ============================================================================
#define PIN_PUL 18
#define PIN_DIR 17
#define PIN_ENA 16
static const int BASE_STEPS_PER_REV = 200;
static const int MICROSTEP = 16;
static const long STEPS_PER_REV = (long)(BASE_STEPS_PER_REV * MICROSTEP);

static volatile uint32_t HALF_PERIOD_US = 400;
static volatile int8_t  STP_RUN_DIR = 0;
static volatile long    stp_steps_remaining = 0;
static volatile long    stp_current_steps   = 0;
static volatile long    stp_target_steps_abs = LONG_MIN;

hw_timer_t* stp_tmr = nullptr;
portMUX_TYPE stp_spinlock = portMUX_INITIALIZER_UNLOCKED;

rcl_subscription_t   stp_sub_cmd;
std_msgs__msg__Int16 stp_cmd_msg;
rcl_publisher_t      stp_pub_fb;
std_msgs__msg__Int16 stp_fb_msg;


// ============================================================================
//                          CONTROL LOGIC & CALLBACKS
// ============================================================================

void controlStep(float dt){
    // Safety Stop
    if (millis() - drive_last_cmd_ms > CMD_TIMEOUT_MS) {
        tgt_m1_rpm = tgt_m2_rpm = tgt_m3_rpm = tgt_m4_rpm = 0.0f;
    } else {
        float v_left  = cmd_vx - cmd_wz * (TRACK_WIDTH * 0.5f);
        float v_right = cmd_vx + cmd_wz * (TRACK_WIDTH * 0.5f);
        auto v_to_rpm = [&](float v){ return (v / WHEEL_RADIUS) * 60.0f / (2.0f * PI); };
        tgt_m1_rpm = tgt_m3_rpm = v_to_rpm(v_left);
        tgt_m2_rpm = tgt_m4_rpm = v_to_rpm(v_right);
    }

    // Slew rate
    auto slew = [&](float cur, float tgt) {
        float max_step = SLEW_RPM_PER_SEC * dt;
        float diff = constrain(tgt, -MAX_RPM, MAX_RPM) - cur;
        return cur + constrain(diff, -max_step, max_step);
    };

    m1_rpm = slew(m1_rpm, tgt_m1_rpm); m2_rpm = slew(m2_rpm, tgt_m2_rpm);
    m3_rpm = slew(m3_rpm, tgt_m3_rpm); m4_rpm = slew(m4_rpm, tgt_m4_rpm);

    auto setMotor = [](int dirPin, int pwmCh, float rpm) {
        digitalWrite(dirPin, (rpm >= 0) ? HIGH : LOW);
        ledcWrite(pwmCh, (uint8_t)((fabs(rpm)/MAX_RPM) * 255.0f));
    };
    setMotor(L_DIR1, PWM_CH_M1, m1_rpm); setMotor(R_DIR1, PWM_CH_M2, m2_rpm);
    setMotor(L_DIR2, PWM_CH_M3, m3_rpm); setMotor(R_DIR2, PWM_CH_M4, m4_rpm);
}

void controlTimerCb(rcl_timer_t* timer, int64_t last_call_time) {
    static uint32_t last_ms = millis();
    float dt = (millis() - last_ms) / 1000.0f;
    last_ms = millis();
    controlStep(dt);

    // Publish Feedback
    debug_motor_msg.data.data[0] = m1_rpm; debug_motor_msg.data.data[1] = m2_rpm;
    RCSOFTCHECK(rcl_publish(&debug_motor_pub, &debug_motor_msg, NULL));

    encoder_msg.data.data[0] = (int16_t)(encoder1_count & 0xFFFF);
    encoder_msg.data.data[1] = (int16_t)(encoder2_count & 0xFFFF);
    RCSOFTCHECK(rcl_publish(&encoder_pub, &encoder_msg, NULL));
}

void twistCb(const void *msgin) {
    const auto *m = (const geometry_msgs__msg__Twist*)msgin;
    cmd_vx = (fabs(m->linear.x) > DEADBAND) ? m->linear.x : 0;
    cmd_wz = (fabs(m->angular.z) > DEADBAND) ? m->angular.z : 0;
    drive_last_cmd_ms = millis();
}

// Stepper Timer ISR
void IRAM_ATTR stp_onTimer(){
    if (STP_RUN_DIR == 0 || stp_steps_remaining <= 0) return;
    static bool high = false;
    if (!high) { REG_WRITE(GPIO_OUT_W1TS_REG, (1U << PIN_PUL)); high = true; }
    else { 
        REG_WRITE(GPIO_OUT_W1TC_REG, (1U << PIN_PUL)); high = false; 
        stp_steps_remaining--;
        if (STP_RUN_DIR == 1) stp_current_steps++; else stp_current_steps--;
    }
}

void stp_cmd_cb(const void* msgin){
    const auto* m = (const std_msgs__msg__Int16*)msgin;
    long target_steps = (m->data * STEPS_PER_REV) / 360;
    long current_mod = stp_current_steps % STEPS_PER_REV;
    if(current_mod < 0) current_mod += STEPS_PER_REV;
    long delta = target_steps - current_mod;
    if (delta > (STEPS_PER_REV/2)) delta -= STEPS_PER_REV;
    if (delta < -(STEPS_PER_REV/2)) delta += STEPS_PER_REV;

    if(delta == 0) return;
    digitalWrite(PIN_DIR, (delta > 0) ? LOW : HIGH);
    digitalWrite(PIN_ENA, LOW); 
    portENTER_CRITICAL(&stp_spinlock);
    STP_RUN_DIR = (delta > 0) ? 1 : -1;
    stp_steps_remaining = abs(delta);
    portEXIT_CRITICAL(&stp_spinlock);
}

// ============================================================================
//                          micro-ROS ENTITIES
// ============================================================================
bool createEntities(){
    allocator = rcl_get_default_allocator();
    init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 96));
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_robot_node", "", &support));

    // Malloc dynamic arrays
    debug_motor_msg.data.capacity = 4; debug_motor_msg.data.size = 4;
    debug_motor_msg.data.data = (float*)malloc(4*sizeof(float));
    encoder_msg.data.capacity = 4; encoder_msg.data.size = 4;
    encoder_msg.data.data = (int16_t*)malloc(4*sizeof(int16_t));

    // Publishers
    RCCHECK(rclc_publisher_init_best_effort(&debug_motor_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/motor_debug/duty"));
    RCCHECK(rclc_publisher_init_best_effort(&encoder_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray), "/motor_feedback/encoders"));
    RCCHECK(rclc_publisher_init_best_effort(&stp_pub_fb, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "/tao/cmd_step_load/fb"));

    // Subscriptions
    RCCHECK(rclc_subscription_init_default(&cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/tao/cmd_vel"));
    RCCHECK(rclc_subscription_init_default(&stp_sub_cmd, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "/tao/cmd_step_load"));

    // Timer
    RCCHECK(rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS(20), controlTimerCb));

    // Executor (3 entities: 2 subs, 1 timer)
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &twistCb, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &stp_sub_cmd, &stp_cmd_msg, &stp_cmd_cb, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    return true;
}

// ============================================================================
//                                SETUP & LOOP
// ============================================================================
void setup(){
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    // Pins Setup
    pinMode(L_DIR1, OUTPUT); pinMode(R_DIR1, OUTPUT); pinMode(L_DIR2, OUTPUT); pinMode(R_DIR2, OUTPUT);
    ledcSetup(PWM_CH_M1, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(L_PWM1, PWM_CH_M1);
    ledcSetup(PWM_CH_M2, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(R_PWM1, PWM_CH_M2);
    ledcSetup(PWM_CH_M3, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(L_PWM2, PWM_CH_M3);
    ledcSetup(PWM_CH_M4, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(R_PWM2, PWM_CH_M4);

    pinMode(ENC1_A, INPUT_PULLUP); attachInterrupt(ENC1_A, encoder1_ISR, CHANGE);
    pinMode(ENC2_A, INPUT_PULLUP); attachInterrupt(ENC2_A, encoder2_ISR, CHANGE);
    pinMode(ENC3_A, INPUT_PULLUP); attachInterrupt(ENC3_A, encoder3_ISR, CHANGE);
    pinMode(ENC4_A, INPUT_PULLUP); attachInterrupt(ENC4_A, encoder4_ISR, CHANGE);

    pinMode(PIN_PUL, OUTPUT); pinMode(PIN_DIR, OUTPUT); pinMode(PIN_ENA, OUTPUT);
    digitalWrite(PIN_ENA, HIGH); 
    stp_tmr = timerBegin(0, 80, true);
    timerAttachInterrupt(stp_tmr, &stp_onTimer, true);
    timerAlarmWrite(stp_tmr, HALF_PERIOD_US, true);
    timerAlarmEnable(stp_tmr);
}

void loop(){
    switch(state){
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK==rmw_uros_ping_agent(100,1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK==rmw_uros_ping_agent(100,1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if(state == AGENT_CONNECTED) rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            break;
        case AGENT_DISCONNECTED:
            state = WAITING_AGENT; // ระบบจะทำลาย Entity อัตโนมัติในรอบถัดไป
            break;
    }
}

void rclErrorLoop(){ while(1){ delay(100); } }