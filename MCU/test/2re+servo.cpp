#include <Arduino.h>
#include <ESP32Servo.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/bool.h>

#include "esp32_config.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ while(1){delay(100);} } }
#define RCSOFTCHECK(fn) (void)(fn)

// ================= ROS =================
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_init_options_t init_options;

// ================= SERVO STRUCT =================
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
  int16_t last_angle;
};

ServoChan CH[4];

// ================= UTIL =================
int clamp_angle(ServoChan& ch,int a){
  if(a<0)a=0;
  if(a>ch.max_deg)a=ch.max_deg;
  return a;
}
int angle_to_us(ServoChan& ch,int a){
  long span=ch.max_us-ch.min_us;
  return ch.min_us+(long)a*span/ch.max_deg;
}
void move_servo(ServoChan& ch,int a){
  ch.servo.writeMicroseconds(angle_to_us(ch,a));
}
void publish_servo(ServoChan& ch,int a){
  ch.pub_msg.data=a;
  RCSOFTCHECK(rcl_publish(&ch.pub,&ch.pub_msg,NULL));
}

// ================= CALLBACK =================
void servo_callback_0(const void* msg){
  int a=((std_msgs__msg__Int16*)msg)->data;
  a=clamp_angle(CH[0],a);
  move_servo(CH[0],a);
  publish_servo(CH[0],a);
}
void servo_callback_1(const void* msg){
  int a=((std_msgs__msg__Int16*)msg)->data;
  a=clamp_angle(CH[1],a);
  move_servo(CH[1],a);
  publish_servo(CH[1],a);
}
void servo_callback_2(const void* msg){
  int a=((std_msgs__msg__Int16*)msg)->data;
  a=clamp_angle(CH[2],a);
  move_servo(CH[2],a);
  publish_servo(CH[2],a);
}
void servo_callback_3(const void* msg){
  int a=((std_msgs__msg__Int16*)msg)->data;
  a=clamp_angle(CH[3],a);
  move_servo(CH[3],a);
  publish_servo(CH[3],a);
}

// ================= SETUP =================
void setup(){

  set_microros_serial_transports(Serial);
  Serial.begin(115200);
  delay(2000);

  // ==== Servo Init ====
  CH[0]={Servo(),PIN_SERVO_GRIPPER,500,2500,180,"/tao/cmd_gripper","/tao/cmd_gripper/rpm"};
  CH[1]={Servo(),PIN_SERVO_DRIL,500,2500,180,"/tao/cmd_servo_dril","/tao/cmd_servo_dril/rpm"};
  CH[2]={Servo(),PIN_SERVO_SWITCH180,500,2500,270,"/tao/cmd_servo_switch180","/tao/cmd_servo_switch180/rpm"};
  CH[3]={Servo(),PIN_SERVO_CAM,500,2500,180,"/tao/cmd_servo_cam","/tao/cmd_servo_cam/rpm"};

  for(int i=0;i<4;i++){
    CH[i].servo.attach(CH[i].pin,CH[i].min_us,CH[i].max_us);
    CH[i].last_angle=0;
    move_servo(CH[i],0);
  }

  allocator=rcl_get_default_allocator();
  init_options=rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options,allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options,96));
  RCCHECK(rclc_support_init_with_options(&support,0,NULL,&init_options,&allocator));
  RCCHECK(rclc_node_init_default(&node,"esp32_node","",&support));

  for(int i=0;i<4;i++){
    RCCHECK(rclc_publisher_init_best_effort(
      &CH[i].pub,&node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),
      CH[i].pub_topic));

    RCCHECK(rclc_subscription_init_default(
      &CH[i].sub,&node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),
      CH[i].sub_topic));
  }

  RCCHECK(rclc_executor_init(&executor,&support.context,4,&allocator));

  RCCHECK(rclc_executor_add_subscription(&executor,&CH[0].sub,&CH[0].sub_msg,&servo_callback_0,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&CH[1].sub,&CH[1].sub_msg,&servo_callback_1,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&CH[2].sub,&CH[2].sub_msg,&servo_callback_2,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&CH[3].sub,&CH[3].sub_msg,&servo_callback_3,ON_NEW_DATA));
}

void loop(){
  rclc_executor_spin_some(&executor,RCL_MS_TO_NS(10));
}