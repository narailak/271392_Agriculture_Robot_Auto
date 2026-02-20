// ================= ESP32 + micro-ROS: 3x Servo + Stepper + TB6612 =================
#include <Arduino.h>
#include <ESP32Servo.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16.h>

#include "soc/gpio_reg.h"
#include "driver/gpio.h"

#define RCCHECK(fn) do { rcl_ret_t rc=(fn); if(rc!=RCL_RET_OK){ while(1){ delay(100); } } } while(0)
#define RCSOFTCHECK(fn) (void)(fn)

enum ConnState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
static ConnState state = WAITING_AGENT;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_init_options_t init_options;

// ================= SERVO =================
struct ServoChan {
  Servo servo;
  int pin;
  int min_us;
  int max_us;
  int max_deg;
  const char* sub_topic;
  const char* pub_topic;
  rcl_subscription_t sub;
  std_msgs__msg__Int16 sub_msg;
  rcl_publisher_t pub;
  std_msgs__msg__Int16 pub_msg;
  int16_t last_angle = -1;
};

static ServoChan make_gripper(){
  ServoChan ch{};
  ch.pin=16; ch.min_us=500; ch.max_us=2500; ch.max_deg=180;
  ch.sub_topic="/cmd_gripper";
  ch.pub_topic="/cmd_gripper/fb";
  return ch;
}

static ServoChan make_dril_servo(){
  ServoChan ch{};
  ch.pin=23; ch.min_us=500; ch.max_us=2500; ch.max_deg=180;
  ch.sub_topic="/cmd_servo_dril";
  ch.pub_topic="/cmd_servo_dril/fb";
  return ch;
}

static ServoChan make_sw180(){
  ServoChan ch{};
  ch.pin=17; ch.min_us=500; ch.max_us=2500; ch.max_deg=270;
  ch.sub_topic="/cmd_servo_switch180";
  ch.pub_topic="/cmd_servo_switch180/fb";
  return ch;
}

static ServoChan CH_GRIPPER = make_gripper();
static ServoChan CH_DRIL_SERVO = make_dril_servo();
static ServoChan CH_SW180 = make_sw180();

int clamp_angle(const ServoChan& ch, int angle){
  if(angle < 0) angle = 0;
  if(angle > ch.max_deg) angle = ch.max_deg;
  return angle;
}

int angle_to_us(const ServoChan& ch, int angle){
  long span = ch.max_us - ch.min_us;
  long us = ch.min_us + (long)angle * span / ch.max_deg;
  return (int)us;
}

void move_servo(ServoChan& ch, int angle){
  ch.servo.writeMicroseconds(angle_to_us(ch, angle));
}

void servo_cb_generic(ServoChan& ch, const void* msgin){
  const auto* m = (const std_msgs__msg__Int16*)msgin;
  int a = clamp_angle(ch, m->data);
  move_servo(ch, a);
  ch.last_angle = a;
  ch.pub_msg.data = a;
  rcl_publish(&ch.pub, &ch.pub_msg, NULL);
}

void sub_cb_gripper(const void* msgin){ servo_cb_generic(CH_GRIPPER, msgin); }
void sub_cb_dril(const void* msgin){ servo_cb_generic(CH_DRIL_SERVO, msgin); }
void sub_cb_sw180(const void* msgin){ servo_cb_generic(CH_SW180, msgin); }

// ================= STEPPER =================
static const int PIN_PUL=25;
static const int PIN_DIR=26;
static const int PIN_ENA=27;

rcl_subscription_t sub_cmd_linear;
std_msgs__msg__Int16 cmd_msg_linear;

void cmd_cb_linear(const void* msgin){
  const auto* m=(const std_msgs__msg__Int16*)msgin;
  Serial.print("Linear cmd: ");
  Serial.println(m->data);
}

// ================= TB6612 =================
#define TB_AIN1 19
#define TB_AIN2 21
#define TB_PWMA 22
#define TB_STBY 5

#define PWM_CHANNEL 0
#define PWM_FREQ 20000
#define PWM_RES 8

rcl_subscription_t sub_cmd_step_load;
std_msgs__msg__Int16 cmd_msg_step_load;

void cmd_cb_step_load(const void* msgin){
  const auto* m=(const std_msgs__msg__Int16*)msgin;
  int val=m->data;
  if(val<=0){
    ledcWrite(PWM_CHANNEL,0);
    digitalWrite(TB_AIN1,LOW);
    digitalWrite(TB_AIN2,LOW);
  }else{
    if(val>100) val=100;
    int duty=map(val,0,100,0,255);
    digitalWrite(TB_AIN1,HIGH);
    digitalWrite(TB_AIN2,LOW);
    ledcWrite(PWM_CHANNEL,duty);
  }
}

// ================= CREATE ENTITIES =================
bool createEntities(){
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support,0,NULL,&allocator));
  RCCHECK(rclc_node_init_default(&node,"esp32_node","",&support));

  // Servo pubs/subs
  RCCHECK(rclc_publisher_init_default(&CH_GRIPPER.pub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),CH_GRIPPER.pub_topic));
  RCCHECK(rclc_subscription_init_default(&CH_GRIPPER.sub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),CH_GRIPPER.sub_topic));

  RCCHECK(rclc_publisher_init_default(&CH_DRIL_SERVO.pub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),CH_DRIL_SERVO.pub_topic));
  RCCHECK(rclc_subscription_init_default(&CH_DRIL_SERVO.sub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),CH_DRIL_SERVO.sub_topic));

  RCCHECK(rclc_publisher_init_default(&CH_SW180.pub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),CH_SW180.pub_topic));
  RCCHECK(rclc_subscription_init_default(&CH_SW180.sub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),CH_SW180.sub_topic));

  RCCHECK(rclc_subscription_init_default(&sub_cmd_linear,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),"/cmd_linear"));
  RCCHECK(rclc_subscription_init_default(&sub_cmd_step_load,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),"/cmd_step_load"));

  RCCHECK(rclc_executor_init(&executor,&support.context,5,&allocator));

  RCCHECK(rclc_executor_add_subscription(&executor,&CH_GRIPPER.sub,&CH_GRIPPER.sub_msg,&sub_cb_gripper,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&CH_DRIL_SERVO.sub,&CH_DRIL_SERVO.sub_msg,&sub_cb_dril,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&CH_SW180.sub,&CH_SW180.sub_msg,&sub_cb_sw180,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&sub_cmd_linear,&cmd_msg_linear,&cmd_cb_linear,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&sub_cmd_step_load,&cmd_msg_step_load,&cmd_cb_step_load,ON_NEW_DATA));

  return true;
}

// ================= SETUP =================
void setup(){
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  CH_GRIPPER.servo.attach(CH_GRIPPER.pin,CH_GRIPPER.min_us,CH_GRIPPER.max_us);
  CH_DRIL_SERVO.servo.attach(CH_DRIL_SERVO.pin,CH_DRIL_SERVO.min_us,CH_DRIL_SERVO.max_us);
  CH_SW180.servo.attach(CH_SW180.pin,CH_SW180.min_us,CH_SW180.max_us);

  pinMode(TB_AIN1,OUTPUT);
  pinMode(TB_AIN2,OUTPUT);
  pinMode(TB_STBY,OUTPUT);
  digitalWrite(TB_STBY,HIGH);

  ledcSetup(PWM_CHANNEL,PWM_FREQ,PWM_RES);
  ledcAttachPin(TB_PWMA,PWM_CHANNEL);

  createEntities();
}

// ================= LOOP =================
void loop(){
  rclc_executor_spin_some(&executor,RCL_MS_TO_NS(100));
}