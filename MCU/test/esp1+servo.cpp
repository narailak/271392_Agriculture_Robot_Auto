// =========================== main.cpp (ESP32-S3 + micro-ROS)
// - Differential drive (4 DC motors) via /tao/cmd_vel
// - Stepper TB6600 absolute angle via /tao/cmd_step_load (Int16 0..359)
// - Cam Servo via /tao/cmd_servo_cam (Int16 0..180)
// - ROS_DOMAIN_ID = 96
// - Included: 4 Wheel Encoders logic
// ============================================================================

#include <Arduino.h>
#include <ESP32Servo.h>                         
#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int16.h>
#include <geometry_msgs/msg/twist.h>

#include "soc/gpio_reg.h"

// --- Include Encoder Library ---
#include <esp32_Encoder.h>

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
//                               DC MOTORS
// ============================================================================
#define R_DIR1  20
#define R_PWM1  19
#define R_DIR2  36
#define R_PWM2  37
#define L_DIR1  13
#define L_PWM1  14
#define L_DIR2  8
#define L_PWM2  7

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

// ROS Entities (DC)
rcl_publisher_t debug_motor_pub;
std_msgs__msg__Float32MultiArray debug_motor_msg;
rcl_subscription_t cmd_sub;
geometry_msgs__msg__Twist cmd_msg;
rcl_timer_t control_timer;

volatile float cmd_vx = 0.0f;
volatile float cmd_wz = 0.0f;
volatile uint32_t drive_last_cmd_ms = 0;
float m1_rpm=0, m2_rpm=0, m3_rpm=0, m4_rpm=0;
float tgt_m1_rpm=0, tgt_m2_rpm=0, tgt_m3_rpm=0, tgt_m4_rpm=0;

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

hw_timer_t* stp_tmr = nullptr;
portMUX_TYPE stp_spinlock = portMUX_INITIALIZER_UNLOCKED;

rcl_subscription_t   stp_sub_cmd;
std_msgs__msg__Int16 stp_cmd_msg;
rcl_publisher_t      stp_pub_fb;
std_msgs__msg__Int16 stp_fb_msg;

// ============================================================================
//                         CAM SERVO 
// ============================================================================
#define PIN_SERVO_CAM   15  
#define SERVO_MIN_US    500
#define SERVO_MAX_US    2500
#define SERVO_MAX_DEG   180

Servo cam_servo;

rcl_subscription_t   sub_cmd_cam;
std_msgs__msg__Int16 cmd_msg_cam;
rcl_publisher_t      pub_fb_cam;
std_msgs__msg__Int16 fb_msg_cam;

static int16_t  cam_last_angle  = 0;
static uint32_t cam_last_hb_ms  = 0;

static inline int cam_clamp(int angle){
  if(angle < 0)            angle = 0;
  if(angle > SERVO_MAX_DEG) angle = SERVO_MAX_DEG;
  return angle;
}
static inline int cam_angle_to_us(int angle){
  long us = (long)SERVO_MIN_US + (long)angle * (SERVO_MAX_US - SERVO_MIN_US) / SERVO_MAX_DEG;
  if(us < SERVO_MIN_US) us = SERVO_MIN_US;
  if(us > SERVO_MAX_US) us = SERVO_MAX_US;
  return (int)us;
}
static inline void cam_publish_fb(int16_t angle){
  fb_msg_cam.data = angle;
  RCSOFTCHECK(rcl_publish(&pub_fb_cam, &fb_msg_cam, NULL));
}
static void sub_cb_cam(const void* msgin){
  const auto* m = (const std_msgs__msg__Int16*)msgin;
  int a = cam_clamp((int)m->data);
  cam_servo.writeMicroseconds(cam_angle_to_us(a));
  cam_last_angle = (int16_t)a;
  cam_publish_fb(cam_last_angle);
}

// ============================================================================
//                            WHEEL ENCODERS
// ============================================================================
#define Encoder_LF_A 41
#define Encoder_LF_B 42
#define Encoder_LB_A 21
#define Encoder_LB_B 38
#define Encoder_RF_A 11
#define Encoder_RF_B 12
#define Encoder_RB_A 9
#define Encoder_RB_B 10

#define COUNTS_PER_REV 340
#define GEAR_RATIO 1
#define WHEEL_DIAMETER 0.09
#define ENCODER_INV_LF false
#define ENCODER_INV_LB false
#define ENCODER_INV_RF false
#define ENCODER_INV_RB false

esp32_Encoder encLF(Encoder_LF_A, Encoder_LF_B, COUNTS_PER_REV, ENCODER_INV_LF, GEAR_RATIO, WHEEL_DIAMETER);
esp32_Encoder encLB(Encoder_LB_A, Encoder_LB_B, COUNTS_PER_REV, ENCODER_INV_LB, GEAR_RATIO, WHEEL_DIAMETER);
esp32_Encoder encRF(Encoder_RF_A, Encoder_RF_B, COUNTS_PER_REV, ENCODER_INV_RF, GEAR_RATIO, WHEEL_DIAMETER);
esp32_Encoder encRB(Encoder_RB_A, Encoder_RB_B, COUNTS_PER_REV, ENCODER_INV_RB, GEAR_RATIO, WHEEL_DIAMETER);

long offsetLF = 0, offsetLB = 0, offsetRF = 0, offsetRB = 0;

rcl_publisher_t debug_encoder_wheels_publisher;
std_msgs__msg__Float32MultiArray debug_encoder_wheels_msg;

rcl_subscription_t cmd_resetencoder_subscriber;
geometry_msgs__msg__Twist cmd_resetencoder_msg;

void getEncoderWheelsTick() {
    debug_encoder_wheels_msg.data.data[0] = encLF.read() - offsetLF;
    debug_encoder_wheels_msg.data.data[1] = encLB.read() - offsetLB;
    debug_encoder_wheels_msg.data.data[2] = encRF.read() - offsetRF;
    debug_encoder_wheels_msg.data.data[3] = encRB.read() - offsetRB;
}

void resetEncoderOffset() {
    offsetLF = encLF.read(); offsetLB = encLB.read();
    offsetRF = encRF.read(); offsetRB = encRB.read();
    debug_encoder_wheels_msg.data.data[0] = 0;
    debug_encoder_wheels_msg.data.data[1] = 0;
    debug_encoder_wheels_msg.data.data[2] = 0;
    debug_encoder_wheels_msg.data.data[3] = 0;
}

// ============================================================================
//                          CONTROL LOGIC & CALLBACKS
// ============================================================================
void cmd_reset_encoder_callback(const void *msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    if (msg->linear.x > 0.5) resetEncoderOffset();
}

void controlStep(float dt){
    if (millis() - drive_last_cmd_ms > CMD_TIMEOUT_MS) {
        tgt_m1_rpm = tgt_m2_rpm = tgt_m3_rpm = tgt_m4_rpm = 0.0f;
    } else {
        float v_left  = cmd_vx - cmd_wz * (TRACK_WIDTH * 0.5f);
        float v_right = cmd_vx + cmd_wz * (TRACK_WIDTH * 0.5f);
        auto v_to_rpm = [&](float v){ return (v / WHEEL_RADIUS) * 60.0f / (2.0f * PI); };
        tgt_m1_rpm = tgt_m3_rpm = v_to_rpm(v_left);
        tgt_m2_rpm = tgt_m4_rpm = v_to_rpm(v_right);
    }

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
    getEncoderWheelsTick();

    debug_motor_msg.data.data[0] = m1_rpm; debug_motor_msg.data.data[1] = m2_rpm;
    RCSOFTCHECK(rcl_publish(&debug_motor_pub, &debug_motor_msg, NULL));
    RCSOFTCHECK(rcl_publish(&debug_encoder_wheels_publisher, &debug_encoder_wheels_msg, NULL));

    // Cam Servo Heartbeat (ทุก 300ms)
    uint32_t now = millis();
    if((now - cam_last_hb_ms) >= 300){
        cam_publish_fb(cam_last_angle);
        cam_last_hb_ms = now;
    }
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
    debug_encoder_wheels_msg.data.capacity = 4; debug_encoder_wheels_msg.data.size = 4;
    debug_encoder_wheels_msg.data.data = (float*)malloc(4*sizeof(float));

    // --- Publishers ---
    RCCHECK(rclc_publisher_init_best_effort(&debug_motor_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/motor_debug/duty"));
    RCCHECK(rclc_publisher_init_best_effort(&stp_pub_fb, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "/tao/cmd_step_load/fb"));
    RCCHECK(rclc_publisher_init_default(&debug_encoder_wheels_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "teelek/debug/encoder_wheels"));
    RCCHECK(rclc_publisher_init_best_effort(&pub_fb_cam, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "/tao/cmd_servo_cam/rpm"));

    // --- Subscribers ---
    RCCHECK(rclc_subscription_init_default(&cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/tao/cmd_vel"));
    RCCHECK(rclc_subscription_init_default(&stp_sub_cmd, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "/tao/cmd_step_load"));
    RCCHECK(rclc_subscription_init_default(&cmd_resetencoder_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/teelek/cmd_resetencoder"));
    RCCHECK(rclc_subscription_init_default(&sub_cmd_cam, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "/tao/cmd_servo_cam"));

    // --- Timer ---
    RCCHECK(rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS(20), controlTimerCb));

    // --- Executor (5 entities: 4 subs + 1 timer) ---
    RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub,                    &cmd_msg,              &twistCb,                      ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &stp_sub_cmd,               &stp_cmd_msg,          &stp_cmd_cb,                   ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_resetencoder_subscriber,&cmd_resetencoder_msg, &cmd_reset_encoder_callback,   ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_cam,               &cmd_msg_cam,          &sub_cb_cam,                   ON_NEW_DATA)); 
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    // --- Init cam servo state ---
    cam_last_angle = 0;
    cam_servo.writeMicroseconds(cam_angle_to_us(0));
    cam_publish_fb(0);

    return true;
}

// ============================================================================
//                                SETUP & LOOP
// ============================================================================
void setup(){
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    // ==========================================
    // 1. จัดสรร Hardware Timer 2 ให้ Servo แยกออกมา
    // ==========================================
    ESP32PWM::allocateTimer(2);
    cam_servo.setPeriodHertz(50); // เซ็ตความถี่เป็น 50Hz ตามสเปก Servo ทั่วไป

    // ==========================================
    // 2. DC Motor Pins (ledc จะจัดการ Timer 0 และ 1 เอง)
    // ==========================================
    pinMode(L_DIR1, OUTPUT); pinMode(R_DIR1, OUTPUT);
    pinMode(L_DIR2, OUTPUT); pinMode(R_DIR2, OUTPUT);
    ledcSetup(PWM_CH_M1, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(L_PWM1, PWM_CH_M1);
    ledcSetup(PWM_CH_M2, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(R_PWM1, PWM_CH_M2);
    ledcSetup(PWM_CH_M3, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(L_PWM2, PWM_CH_M3);
    ledcSetup(PWM_CH_M4, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(R_PWM2, PWM_CH_M4);

    // ==========================================
    // 3. Stepper Pins (เปลี่ยนไปใช้ Timer 3)
    // ==========================================
    pinMode(PIN_PUL, OUTPUT); pinMode(PIN_DIR, OUTPUT); pinMode(PIN_ENA, OUTPUT);
    digitalWrite(PIN_ENA, HIGH);
    
    stp_tmr = timerBegin(3, 80, true); 
    timerAttachInterrupt(stp_tmr, &stp_onTimer, true);
    timerAlarmWrite(stp_tmr, HALF_PERIOD_US, true);
    timerAlarmEnable(stp_tmr);

    // ==========================================
    // 4. เริ่มทำงาน Servo
    // ==========================================
    cam_servo.attach(PIN_SERVO_CAM, SERVO_MIN_US, SERVO_MAX_US);
    cam_servo.writeMicroseconds(cam_angle_to_us(0));
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
            state = WAITING_AGENT;
            break;
    }
}

void rclErrorLoop(){ while(1){ delay(100); } }