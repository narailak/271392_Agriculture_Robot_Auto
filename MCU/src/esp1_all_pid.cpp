// =========================== main.cpp (ESP32-S3 + micro-ROS)
// - Differential drive (4 DC motors) via /tao/motor_pwm (Int32MultiArray)
// - Encoder Feedback (Velocity m/s) via /tao/encoder_feedback (Float32MultiArray)
// - Encoder Sum (Total Ticks) via /tao/encoder_sum (Int32MultiArray)
// - Reset Encoder via /tao/reset_encoder (Twist, linear.x > 0.5)
// - Stepper TB6600 absolute angle via /tao/cmd_step_load (Int16 0..359)
// - Cam Servo via /tao/cmd_servo_cam (Int16 0..180)
// - ROS_DOMAIN_ID = 96
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
#include <std_msgs/msg/int32_multi_array.h>
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
#define L_DIR1  13 // LF
#define L_PWM1  14
#define R_DIR1  20 // RF
#define R_PWM1  19
#define L_DIR2  8  // LB
#define L_PWM2  7
#define R_DIR2  36 // RB
#define R_PWM2  37

#define PWM_FREQ        20000
#define PWM_RESOLUTION  8
#define PWM_CH_M1 0 // LF
#define PWM_CH_M2 1 // RF
#define PWM_CH_M3 2 // LB
#define PWM_CH_M4 3 // RB

const float WHEEL_DIAMETER = 0.09f;
const uint32_t CMD_TIMEOUT_MS = 300;

// ROS Entities (DC)
rcl_subscription_t pwm_sub;
std_msgs__msg__Int32MultiArray pwm_msg;
rcl_timer_t control_timer;

volatile int32_t target_pwm[4] = {0, 0, 0, 0};
volatile uint32_t last_pwm_ms = 0;

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
//                          CAM SERVO 
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
#define Encoder_LB_A 38
#define Encoder_LB_B 21
#define Encoder_RF_A 12
#define Encoder_RF_B 11
#define Encoder_RB_A 10
#define Encoder_RB_B 9

#define COUNTS_PER_REV 340
#define GEAR_RATIO 1
#define ENCODER_INV_LF false
#define ENCODER_INV_LB false
#define ENCODER_INV_RF false
#define ENCODER_INV_RB false

esp32_Encoder encLF(Encoder_LF_A, Encoder_LF_B, COUNTS_PER_REV, ENCODER_INV_LF, GEAR_RATIO, WHEEL_DIAMETER);
esp32_Encoder encRF(Encoder_RF_A, Encoder_RF_B, COUNTS_PER_REV, ENCODER_INV_RF, GEAR_RATIO, WHEEL_DIAMETER);
esp32_Encoder encLB(Encoder_LB_A, Encoder_LB_B, COUNTS_PER_REV, ENCODER_INV_LB, GEAR_RATIO, WHEEL_DIAMETER);
esp32_Encoder encRB(Encoder_RB_A, Encoder_RB_B, COUNTS_PER_REV, ENCODER_INV_RB, GEAR_RATIO, WHEEL_DIAMETER);

// ตัวแปรสำหรับคำนวณ m/s (PID)
long last_tick[4] = {0, 0, 0, 0};

// ตัวแปรสำหรับคำนวณ Ticks รวม (Odometry)
long offsetLF = 0, offsetRF = 0, offsetLB = 0, offsetRB = 0;

// ROS Entities (Encoder Wheels)
rcl_publisher_t debug_encoder_wheels_publisher; 
std_msgs__msg__Float32MultiArray debug_encoder_wheels_msg;

rcl_publisher_t pub_encoder_sum; 
std_msgs__msg__Int32MultiArray msg_encoder_sum;

rcl_subscription_t cmd_resetencoder_subscriber;
geometry_msgs__msg__Twist cmd_resetencoder_msg;

// ============================================================================
//                          CONTROL LOGIC & CALLBACKS
// ============================================================================

// 🌟 Callback สำหรับสั่งรีเซ็ตค่า Encoder ทั้ง 2 ระบบ 🌟
void cmd_reset_encoder_callback(const void *msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    if (msg->linear.x > 0.5) {
        offsetLF = encLF.read();
        offsetRF = encRF.read();
        offsetLB = encLB.read();
        offsetRB = encRB.read();
        
        last_tick[0] = offsetLF;
        last_tick[1] = offsetRF;
        last_tick[2] = offsetLB;
        last_tick[3] = offsetRB;
    }
}

// 1. รับค่าจาก PC
void pwm_cb(const void *msgin) {
    const auto *m = (const std_msgs__msg__Int32MultiArray*)msgin;
    if (m->data.size >= 4) {
        target_pwm[0] = m->data.data[0]; 
        target_pwm[1] = m->data.data[1]; 
        target_pwm[2] = m->data.data[2]; 
        target_pwm[3] = m->data.data[3]; 
        last_pwm_ms = millis();
    }
}

// 2. สั่งมอเตอร์
void applyMotorPWM() {
    if (millis() - last_pwm_ms > CMD_TIMEOUT_MS) {
        target_pwm[0] = target_pwm[1] = target_pwm[2] = target_pwm[3] = 0;
    }

    auto setMotor = [](int dirPin, int pwmCh, int32_t pwm) {
        digitalWrite(dirPin, (pwm >= 0) ? HIGH : LOW);
        ledcWrite(pwmCh, (uint8_t)constrain(abs(pwm), 0, 255));
    };

    setMotor(L_DIR1, PWM_CH_M1, target_pwm[0]); 
    setMotor(R_DIR1, PWM_CH_M2, target_pwm[1]); 
    setMotor(L_DIR2, PWM_CH_M3, target_pwm[2]); 
    setMotor(R_DIR2, PWM_CH_M4, target_pwm[3]); 
}

// 3. อ่าน Encoder แปลงเป็น m/s (สำหรับ PID)
void getEncoderWheelsVelocity(float dt) {
    long curLF = encLF.read();
    long curRF = encRF.read();
    long curLB = encLB.read();
    long curRB = encRB.read();

    if (dt > 0.0f) {
        float dist_per_tick = (PI * WHEEL_DIAMETER) / COUNTS_PER_REV;
        
        auto calcV = [&](long cur, long &last) {
            float v = ((float)(cur - last) * dist_per_tick) / dt;
            last = cur; 
            return v;
        };

        debug_encoder_wheels_msg.data.data[0] = calcV(curLF, last_tick[0]);
        debug_encoder_wheels_msg.data.data[1] = calcV(curRF, last_tick[1]); 
        debug_encoder_wheels_msg.data.data[2] = calcV(curLB, last_tick[2]);
        debug_encoder_wheels_msg.data.data[3] = calcV(curRB, last_tick[3]);
    }
}

// 4. อ่าน Ticks รวมหักลบ Offset (สำหรับ Odometry)
void getEncoderWheelsSum() {
    msg_encoder_sum.data.data[0] = encLF.read() - offsetLF;
    msg_encoder_sum.data.data[1] = encRF.read() - offsetRF;
    msg_encoder_sum.data.data[2] = encLB.read() - offsetLB;
    msg_encoder_sum.data.data[3] = encRB.read() - offsetRB;
}

void controlTimerCb(rcl_timer_t* timer, int64_t last_call_time) {
    static uint32_t last_ms = millis();
    float dt = (millis() - last_ms) / 1000.0f;
    last_ms = millis();

    applyMotorPWM();

    getEncoderWheelsVelocity(dt);
    RCSOFTCHECK(rcl_publish(&debug_encoder_wheels_publisher, &debug_encoder_wheels_msg, NULL));

    getEncoderWheelsSum();
    RCSOFTCHECK(rcl_publish(&pub_encoder_sum, &msg_encoder_sum, NULL));

    uint32_t now = millis();
    if((now - cam_last_hb_ms) >= 300){
        cam_publish_fb(cam_last_angle);
        cam_last_hb_ms = now;
    }
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

    pwm_msg.data.capacity = 4;
    pwm_msg.data.size = 0; 
    pwm_msg.data.data = (int32_t*)malloc(pwm_msg.data.capacity * sizeof(int32_t));
    
    debug_encoder_wheels_msg.data.capacity = 4; 
    debug_encoder_wheels_msg.data.size = 4;
    debug_encoder_wheels_msg.data.data = (float*)malloc(debug_encoder_wheels_msg.data.capacity * sizeof(float));

    msg_encoder_sum.data.capacity = 4;
    msg_encoder_sum.data.size = 4;
    msg_encoder_sum.data.data = (int32_t*)malloc(msg_encoder_sum.data.capacity * sizeof(int32_t));

    // --- Publishers ---
    RCCHECK(rclc_publisher_init_best_effort(&stp_pub_fb, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "/tao/cmd_step_load/fb"));
    RCCHECK(rclc_publisher_init_default(&debug_encoder_wheels_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/tao/encoder_feedback"));
    RCCHECK(rclc_publisher_init_default(&pub_encoder_sum, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/tao/encoder_sum"));
    RCCHECK(rclc_publisher_init_best_effort(&pub_fb_cam, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "/tao/cmd_servo_cam/rpm"));

    // --- Subscribers ---
    RCCHECK(rclc_subscription_init_default(&pwm_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/tao/motor_pwm"));
    RCCHECK(rclc_subscription_init_default(&stp_sub_cmd, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "/tao/cmd_step_load"));
    
    // 🌟 Topic สำหรับคำสั่ง Reset 🌟
    RCCHECK(rclc_subscription_init_default(&cmd_resetencoder_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/tao/reset_encoder"));
        
    RCCHECK(rclc_subscription_init_default(&sub_cmd_cam, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "/tao/cmd_servo_cam"));

    // --- Timer ---
    RCCHECK(rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS(20), controlTimerCb));

    // --- Executor ---
    RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &pwm_sub,                    &pwm_msg,              &pwm_cb,                     ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &stp_sub_cmd,                &stp_cmd_msg,          &stp_cmd_cb,                 ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_resetencoder_subscriber,&cmd_resetencoder_msg, &cmd_reset_encoder_callback,   ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_cam,                &cmd_msg_cam,          &sub_cb_cam,                 ON_NEW_DATA)); 
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

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

    ESP32PWM::allocateTimer(2);
    cam_servo.setPeriodHertz(50); 

    pinMode(L_DIR1, OUTPUT); pinMode(R_DIR1, OUTPUT);
    pinMode(L_DIR2, OUTPUT); pinMode(R_DIR2, OUTPUT);
    ledcSetup(PWM_CH_M1, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(L_PWM1, PWM_CH_M1);
    ledcSetup(PWM_CH_M2, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(R_PWM1, PWM_CH_M2);
    ledcSetup(PWM_CH_M3, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(L_PWM2, PWM_CH_M3);
    ledcSetup(PWM_CH_M4, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(R_PWM2, PWM_CH_M4);

    pinMode(PIN_PUL, OUTPUT); pinMode(PIN_DIR, OUTPUT); pinMode(PIN_ENA, OUTPUT);
    digitalWrite(PIN_ENA, HIGH);
    
    stp_tmr = timerBegin(3, 80, true); 
    timerAttachInterrupt(stp_tmr, &stp_onTimer, true);
    timerAlarmWrite(stp_tmr, HALF_PERIOD_US, true);
    timerAlarmEnable(stp_tmr);

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