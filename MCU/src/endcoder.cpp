#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <vector>
#include <cmath>
#include <utility>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

#include <esp32_Encoder.h>

// ================= PIN CONFIG =================

// ===== Encoder Left Front =====
#define Encoder_LF_A  32
#define Encoder_LF_B  33

// ===== Encoder Left Back =====
#define Encoder_LB_A  
#define Encoder_LB_B  

// ===== Encoder Right Front =====
#define Encoder_RF_A  
#define Encoder_RF_B  

// ===== Encoder Right Back =====
#define Encoder_RB_A  
#define Encoder_RB_B  

// ===== Encoder Parameters =====
#define COUNTS_PER_REV   2048
#define GEAR_RATIO       1.0
#define WHEEL_DIAMETER   0.1   // meter

#define ENCODER_INV_LF   false
#define ENCODER_INV_LB   false
#define ENCODER_INV_RF   true
#define ENCODER_INV_RB   true

// ================= MACROS =================

#define RCCHECK(fn) if((fn) != RCL_RET_OK){ESP.restart();}
#define EXECUTE_EVERY_N_MS(MS, X) \
  do { static uint64_t t = -1; \
  if(t == -1) t = millis(); \
  if(millis() - t > MS){ X; t = millis(); }} while(0)

// ================= ROS VARIABLES =================

rcl_publisher_t encoder_pub;
std_msgs__msg__Float32MultiArray encoder_msg;

rcl_subscription_t reset_sub;
geometry_msgs__msg__Twist reset_msg;

rcl_timer_t control_timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_init_options_t init_options;

// ================= ENCODERS =================

esp32_Encoder encLF(Encoder_LF_A, Encoder_LF_B, COUNTS_PER_REV, ENCODER_INV_LF, GEAR_RATIO, WHEEL_DIAMETER);
esp32_Encoder encLB(Encoder_LB_A, Encoder_LB_B, COUNTS_PER_REV, ENCODER_INV_LB, GEAR_RATIO, WHEEL_DIAMETER);
esp32_Encoder encRF(Encoder_RF_A, Encoder_RF_B, COUNTS_PER_REV, ENCODER_INV_RF, GEAR_RATIO, WHEEL_DIAMETER);
esp32_Encoder encRB(Encoder_RB_A, Encoder_RB_B, COUNTS_PER_REV, ENCODER_INV_RB, GEAR_RATIO, WHEEL_DIAMETER);

long offsetLF = 0, offsetLB = 0, offsetRF = 0, offsetRB = 0;

// ================= STATE MACHINE =================

enum states {WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED} state;
static unsigned long last_sync = 0;

// ================= FUNCTIONS =================

void syncTime(){
  RCCHECK(rmw_uros_sync_session(10));
}

void resetEncoderOffset(){
  offsetLF = encLF.read();
  offsetLB = encLB.read();
  offsetRF = encRF.read();
  offsetRB = encRB.read();
}

void reset_callback(const void *msgin){
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  if(msg->linear.x > 0.5){
    resetEncoderOffset();
  }
}

void control_callback(rcl_timer_t * timer, int64_t last_call_time){
  RCLC_UNUSED(last_call_time);
  if(timer != NULL){
    encoder_msg.data.data[0] = encLF.read() - offsetLF;
    encoder_msg.data.data[1] = encLB.read() - offsetLB;
    encoder_msg.data.data[2] = encRF.read() - offsetRF;
    encoder_msg.data.data[3] = encRB.read() - offsetRB;
    rcl_publish(&encoder_pub, &encoder_msg, NULL);
  }
}

bool create_entities(){

  allocator = rcl_get_default_allocator();

  init_options = rcl_get_zero_initialized_init_options();
  if(rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) return false;
  rcl_init_options_set_domain_id(&init_options, 96);

  if(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) return false;
  if(rclc_node_init_default(&node, "wheels_encoder_node", "", &support) != RCL_RET_OK) return false;

  if(rclc_publisher_init_default(
      &encoder_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "teelek/debug/encoder_wheels") != RCL_RET_OK) return false;

  rosidl_runtime_c__float__Sequence__init(&encoder_msg.data, 4);

  if(rclc_subscription_init_default(
      &reset_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/teelek/cmd_resetencoder") != RCL_RET_OK) return false;

  if(rclc_timer_init_default(
      &control_timer,
      &support,
      RCL_MS_TO_NS(50),
      control_callback) != RCL_RET_OK) return false;

  executor = rclc_executor_get_zero_initialized_executor();
  if(rclc_executor_init(&executor, &support.context, 3, &allocator) != RCL_RET_OK) return false;

  if(rclc_executor_add_subscription(&executor, &reset_sub, &reset_msg, &reset_callback, ON_NEW_DATA) != RCL_RET_OK) return false;
  if(rclc_executor_add_timer(&executor, &control_timer) != RCL_RET_OK) return false;

  syncTime();
  return true;
}

void destroy_entities(){
  rcl_subscription_fini(&reset_sub, &node);
  rcl_publisher_fini(&encoder_pub, &node);
  rcl_timer_fini(&control_timer);
  rcl_node_fini(&node);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);
}

// ================= SETUP =================

void setup(){
  Serial.begin(115200);

  set_microros_serial_transports(Serial);

  pinMode(Encoder_LF_A, INPUT_PULLUP);
  pinMode(Encoder_LF_B, INPUT_PULLUP);
  pinMode(Encoder_LB_A, INPUT_PULLUP);
  pinMode(Encoder_LB_B, INPUT_PULLUP);
  pinMode(Encoder_RF_A, INPUT_PULLUP);
  pinMode(Encoder_RF_B, INPUT_PULLUP);
  pinMode(Encoder_RB_A, INPUT_PULLUP);
  pinMode(Encoder_RB_B, INPUT_PULLUP);

  state = WAITING_AGENT;
}

// ================= LOOP =================

void loop(){

  switch(state){

    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(500, 4)) ? AGENT_AVAILABLE : WAITING_AGENT;
      );
      break;

    case AGENT_AVAILABLE:
      state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(1000,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );

      if(state == AGENT_CONNECTED){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }
}