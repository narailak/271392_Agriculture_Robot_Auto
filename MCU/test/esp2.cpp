#include <Arduino.h>
#include <ESP32Servo.h>
#include <micro_ros_platformio.h>

#include "esp32_config.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16.h>

#include "soc/gpio_reg.h"
#include "driver/gpio.h"

// ================= MACRO =================
#define RCCHECK(fn) {rcl_ret_t rc=fn; if(rc!=RCL_RET_OK){while(1){delay(100);}}}
#define RCSOFTCHECK(fn) (void)(fn)

#define EXECUTE_EVERY_MS(MS,X) \
do{static uint32_t t=0;if(millis()-t>MS){X;t=millis();}}while(0)

// ================= CONNECTION =================
enum ConnState{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};

ConnState state = WAITING_AGENT;

// ================= ROS =================
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_init_options_t init_options;

// =================================================
// LIMIT PUB
// =================================================
rcl_publisher_t pub_limit_left;
rcl_publisher_t pub_limit_right;

std_msgs__msg__Int16 msg_limit_left;
std_msgs__msg__Int16 msg_limit_right;

inline int limit_left(){
  return digitalRead(PIN_LIMIT_LEFT)==LOW;
}

inline int limit_right(){
  return digitalRead(PIN_LIMIT_RIGHT)==LOW;
}

// =================================================
// SERVO
// =================================================
Servo servo_gripper;
Servo servo_dril;
Servo servo_sw;

rcl_subscription_t sub_gripper;
rcl_subscription_t sub_servo_dril;
rcl_subscription_t sub_servo_sw;

std_msgs__msg__Int16 msg_gripper;
std_msgs__msg__Int16 msg_servo_dril;
std_msgs__msg__Int16 msg_servo_sw;

void cb_gripper(const void * msg){
  servo_gripper.write(((std_msgs__msg__Int16*)msg)->data);
}

void cb_servo_dril(const void * msg){
  servo_dril.write(((std_msgs__msg__Int16*)msg)->data);
}

void cb_servo_sw(const void * msg){
  servo_sw.write(((std_msgs__msg__Int16*)msg)->data);
}

// =================================================
// STEPPER
// =================================================
volatile int RUN_DIR=0;
volatile bool pul=false;
hw_timer_t* timer0=NULL;

void IRAM_ATTR onTimer(){

  if(RUN_DIR==0) return;

  uint32_t mask=(1U<<PIN_STEPPER_PUL);

  if(!pul){
    REG_WRITE(GPIO_OUT_W1TS_REG,mask);
    pul=true;
  }else{
    REG_WRITE(GPIO_OUT_W1TC_REG,mask);
    pul=false;
  }
}

rcl_subscription_t sub_linear;
rcl_publisher_t pub_linear;

std_msgs__msg__Int16 msg_linear;
std_msgs__msg__Int16 fb_linear;

void cb_linear(const void * msg){

  int cmd=((std_msgs__msg__Int16*)msg)->data;

  digitalWrite(PIN_STEPPER_DIR,(cmd>0));
  RUN_DIR=cmd;

  fb_linear.data=cmd;
  RCSOFTCHECK(rcl_publish(&pub_linear,&fb_linear,NULL));
}

// =================================================
// TB6612 MOTOR
// =================================================
rcl_subscription_t sub_motor;
rcl_publisher_t pub_motor;

std_msgs__msg__Int16 msg_motor;
std_msgs__msg__Int16 fb_motor;

void motor_stop(){
  ledcWrite(TB_PWM_CHANNEL,0);
  digitalWrite(PIN_TB_AIN1,LOW);
  digitalWrite(PIN_TB_AIN2,LOW);
}

void motor_forward(uint8_t pwm){
  digitalWrite(PIN_TB_AIN1,HIGH);
  digitalWrite(PIN_TB_AIN2,LOW);
  ledcWrite(TB_PWM_CHANNEL,pwm);
}

void cb_motor(const void * msg){

  int v=((std_msgs__msg__Int16*)msg)->data;

  if(v<=0){motor_stop();return;}

  if(v>100)v=100;

  uint8_t pwm=map(v,1,100,0,255);
  motor_forward(pwm);

  fb_motor.data=v;
  RCSOFTCHECK(rcl_publish(&pub_motor,&fb_motor,NULL));
}

// =================================================
// CREATE ENTITIES
// =================================================
bool createEntities(){

  allocator=rcl_get_default_allocator();

  RCCHECK(rcl_init_options_init(&init_options,allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options,ROS_DOMAIN_ID));

  RCCHECK(rclc_support_init_with_options(
      &support,0,NULL,&init_options,&allocator));

  RCCHECK(rclc_node_init_default(
      &node,"esp32_actuator","",&support));

  // LIMIT PUB
  RCCHECK(rclc_publisher_init_best_effort(
      &pub_limit_left,&node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),
      "/tao/limit_left"));

  RCCHECK(rclc_publisher_init_best_effort(
      &pub_limit_right,&node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),
      "/tao/limit_right"));

  // SERVO SUB
  RCCHECK(rclc_subscription_init_default(
      &sub_gripper,&node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),
      "/tao/cmd_gripper"));

  RCCHECK(rclc_subscription_init_default(
      &sub_servo_dril,&node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),
      "/tao/cmd_servo_dril"));

  RCCHECK(rclc_subscription_init_default(
      &sub_servo_sw,&node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),
      "/tao/cmd_servo_switch180"));

  // LINEAR
  RCCHECK(rclc_subscription_init_default(
      &sub_linear,&node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),
      "/tao/cmd_linear"));

  RCCHECK(rclc_publisher_init_best_effort(
      &pub_linear,&node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),
      "/tao/cmd_linear/fb"));

  // MOTOR
  RCCHECK(rclc_subscription_init_default(
      &sub_motor,&node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),
      "/tao/moter_dril"));

  RCCHECK(rclc_publisher_init_best_effort(
      &pub_motor,&node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),
      "/tao/moter_dril/rpm"));

  RCCHECK(rclc_executor_init(&executor,&support.context,5,&allocator));

  RCCHECK(rclc_executor_add_subscription(&executor,&sub_gripper,&msg_gripper,cb_gripper,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&sub_servo_dril,&msg_servo_dril,cb_servo_dril,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&sub_servo_sw,&msg_servo_sw,cb_servo_sw,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&sub_linear,&msg_linear,cb_linear,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&sub_motor,&msg_motor,cb_motor,ON_NEW_DATA));

  return true;
}

// =================================================
// SETUP
// =================================================
void setup(){

  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  pinMode(PIN_STEPPER_PUL,OUTPUT);
  pinMode(PIN_STEPPER_DIR,OUTPUT);
  pinMode(PIN_STEPPER_ENA,OUTPUT);

  pinMode(PIN_LIMIT_LEFT,INPUT_PULLUP);
  pinMode(PIN_LIMIT_RIGHT,INPUT_PULLUP);

  pinMode(PIN_TB_AIN1,OUTPUT);
  pinMode(PIN_TB_AIN2,OUTPUT);
  pinMode(PIN_TB_STBY,OUTPUT);

  digitalWrite(PIN_TB_STBY,HIGH);

  ledcSetup(TB_PWM_CHANNEL,TB_PWM_FREQ,TB_PWM_RES);
  ledcAttachPin(PIN_TB_PWM,TB_PWM_CHANNEL);

  timer0=timerBegin(0,80,true);
  timerAttachInterrupt(timer0,&onTimer,true);
  timerAlarmWrite(timer0,20,true);
  timerAlarmEnable(timer0);

  servo_gripper.attach(PIN_SERVO_GRIPPER);
  servo_dril.attach(PIN_SERVO_DRIL);
  servo_sw.attach(PIN_SERVO_SWITCH180);
}

// =================================================
// LOOP
// =================================================
void loop(){

  switch(state){

    case WAITING_AGENT:
      if(rmw_uros_ping_agent(100,1)==RMW_RET_OK)
        state=AGENT_AVAILABLE;
      break;

    case AGENT_AVAILABLE:
      state=createEntities()?AGENT_CONNECTED:WAITING_AGENT;
      break;

    case AGENT_CONNECTED:

      rclc_executor_spin_some(&executor,RCL_MS_TO_NS(50));

      EXECUTE_EVERY_MS(100,{
        msg_limit_left.data=limit_left();
        msg_limit_right.data=limit_right();

        RCSOFTCHECK(rcl_publish(&pub_limit_left,&msg_limit_left,NULL));
        RCSOFTCHECK(rcl_publish(&pub_limit_right,&msg_limit_right,NULL));
      });

      break;

    case AGENT_DISCONNECTED:
      state=WAITING_AGENT;
      break;
  }
}