// ================= ESP32 + micro-ROS: 4 Servo + Stepper + TB6612 =================
#include <Arduino.h>
#include <ESP32Servo.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16.h>
#include "esp32_config.h"

#define RCCHECK(fn) do{ rcl_ret_t rc=(fn); if(rc!=RCL_RET_OK){ while(1){delay(100);} } }while(0)
#define RCSOFTCHECK(fn) (void)(fn)

// ================= ROS =================
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_init_options_t init_options;

// ================= SERVO STRUCT =================
struct ServoChan{
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

// ===== Declare (ห้ามใช้ {}) =====
ServoChan CH_GRIPPER;
ServoChan CH_DRIL_SERVO;
ServoChan CH_SW180;
ServoChan CH_CAM;

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

// ================= SERVO CALLBACK =================
void cb_gripper(const void* msg){
  int a=clamp_angle(CH_GRIPPER,((std_msgs__msg__Int16*)msg)->data);
  move_servo(CH_GRIPPER,a); publish_servo(CH_GRIPPER,a);
}
void cb_dril_servo(const void* msg){
  int a=clamp_angle(CH_DRIL_SERVO,((std_msgs__msg__Int16*)msg)->data);
  move_servo(CH_DRIL_SERVO,a); publish_servo(CH_DRIL_SERVO,a);
}
void cb_sw180(const void* msg){
  int a=clamp_angle(CH_SW180,((std_msgs__msg__Int16*)msg)->data);
  move_servo(CH_SW180,a); publish_servo(CH_SW180,a);
}
void cb_cam(const void* msg){
  int a=clamp_angle(CH_CAM,((std_msgs__msg__Int16*)msg)->data);
  move_servo(CH_CAM,a); publish_servo(CH_CAM,a);
}

// ================= TB6612 MOTOR =================
rcl_subscription_t sub_motor;
std_msgs__msg__Int16 msg_motor;
rcl_publisher_t pub_motor;
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
void cb_motor(const void* msg){
  int val=((std_msgs__msg__Int16*)msg)->data;
  if(val<=0){ motor_stop(); }
  else{
    if(val>100)val=100;
    uint8_t duty=map(val,1,100,0,255);
    motor_forward(duty);
  }
  fb_motor.data=val;
  RCSOFTCHECK(rcl_publish(&pub_motor,&fb_motor,NULL));
}

// ================= STEPPER ENABLE/DIR =================
rcl_subscription_t sub_linear;
std_msgs__msg__Int16 msg_linear;
rcl_publisher_t pub_linear;
std_msgs__msg__Int16 fb_linear;

void cb_linear(const void* msg){
  int val=((std_msgs__msg__Int16*)msg)->data;
  if(val==1){
    digitalWrite(PIN_STEPPER_DIR,HIGH);
    digitalWrite(PIN_STEPPER_ENA,LOW);
  }
  else if(val==-1){
    digitalWrite(PIN_STEPPER_DIR,LOW);
    digitalWrite(PIN_STEPPER_ENA,LOW);
  }
  else{
    digitalWrite(PIN_STEPPER_ENA,HIGH);
  }
  fb_linear.data=val;
  RCSOFTCHECK(rcl_publish(&pub_linear,&fb_linear,NULL));
}

// ================= SETUP =================
void setup(){

  Serial.begin(115200);
  delay(2000);
  set_microros_serial_transports(Serial);

  // ===== Allocate PWM Timers (กัน servo ทำงานได้ตัวเดียว) =====
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // ===== Servo Config =====
  CH_GRIPPER.pin=PIN_SERVO_GRIPPER;
  CH_GRIPPER.min_us=500; CH_GRIPPER.max_us=2500; CH_GRIPPER.max_deg=180;
  CH_GRIPPER.sub_topic="/tao/cmd_gripper";
  CH_GRIPPER.pub_topic="/tao/cmd_gripper/rpm";

  CH_DRIL_SERVO.pin=PIN_SERVO_DRIL;
  CH_DRIL_SERVO.min_us=500; CH_DRIL_SERVO.max_us=2500; CH_DRIL_SERVO.max_deg=180;
  CH_DRIL_SERVO.sub_topic="/tao/cmd_servo_dril";
  CH_DRIL_SERVO.pub_topic="/tao/cmd_servo_dril/rpm";

  CH_SW180.pin=PIN_SERVO_SWITCH180;
  CH_SW180.min_us=500; CH_SW180.max_us=2500; CH_SW180.max_deg=270;
  CH_SW180.sub_topic="/tao/cmd_servo_switch180";
  CH_SW180.pub_topic="/tao/cmd_servo_switch180/rpm";

  CH_CAM.pin=PIN_SERVO_CAM;
  CH_CAM.min_us=500; CH_CAM.max_us=2500; CH_CAM.max_deg=180;
  CH_CAM.sub_topic="/tao/cmd_servo_cam";
  CH_CAM.pub_topic="/tao/cmd_servo_cam/rpm";

  // ===== Attach Servos =====
  CH_GRIPPER.servo.attach(CH_GRIPPER.pin,500,2500);
  CH_DRIL_SERVO.servo.attach(CH_DRIL_SERVO.pin,500,2500);
  CH_SW180.servo.attach(CH_SW180.pin,500,2500);
  CH_CAM.servo.attach(CH_CAM.pin,500,2500);

  move_servo(CH_GRIPPER,0);
  move_servo(CH_DRIL_SERVO,0);
  move_servo(CH_SW180,0);
  move_servo(CH_CAM,0);

  // ===== TB6612 =====
  pinMode(PIN_TB_AIN1,OUTPUT);
  pinMode(PIN_TB_AIN2,OUTPUT);
  pinMode(PIN_TB_PWMA,OUTPUT);
  pinMode(PIN_TB_STBY,OUTPUT);
  digitalWrite(PIN_TB_STBY,HIGH);

  ledcSetup(TB_PWM_CHANNEL,TB_PWM_FREQ,TB_PWM_RES);
  ledcAttachPin(PIN_TB_PWMA,TB_PWM_CHANNEL);
  motor_stop();

  // ===== Stepper =====
  pinMode(PIN_STEPPER_DIR,OUTPUT);
  pinMode(PIN_STEPPER_ENA,OUTPUT);
  digitalWrite(PIN_STEPPER_ENA,HIGH);

  // ===== micro-ROS Init =====
  allocator=rcl_get_default_allocator();
  init_options=rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options,allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options,96));
  RCCHECK(rclc_support_init_with_options(&support,0,NULL,&init_options,&allocator));
  RCCHECK(rclc_node_init_default(&node,"esp32_node","",&support));

  // Publishers
  RCCHECK(rclc_publisher_init_best_effort(&CH_GRIPPER.pub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),CH_GRIPPER.pub_topic));
  RCCHECK(rclc_publisher_init_best_effort(&CH_DRIL_SERVO.pub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),CH_DRIL_SERVO.pub_topic));
  RCCHECK(rclc_publisher_init_best_effort(&CH_SW180.pub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),CH_SW180.pub_topic));
  RCCHECK(rclc_publisher_init_best_effort(&CH_CAM.pub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),CH_CAM.pub_topic));
  RCCHECK(rclc_publisher_init_best_effort(&pub_motor,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),"/tao/cmd_motor_dril/fb"));
  RCCHECK(rclc_publisher_init_best_effort(&pub_linear,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),"/tao/cmd_linear/fb"));

  // Subscribers
  RCCHECK(rclc_subscription_init_default(&CH_GRIPPER.sub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),CH_GRIPPER.sub_topic));
  RCCHECK(rclc_subscription_init_default(&CH_DRIL_SERVO.sub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),CH_DRIL_SERVO.sub_topic));
  RCCHECK(rclc_subscription_init_default(&CH_SW180.sub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),CH_SW180.sub_topic));
  RCCHECK(rclc_subscription_init_default(&CH_CAM.sub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),CH_CAM.sub_topic));
  RCCHECK(rclc_subscription_init_default(&sub_motor,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),"/tao/cmd_motor_dril"));
  RCCHECK(rclc_subscription_init_default(&sub_linear,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),"/tao/cmd_linear"));

  RCCHECK(rclc_executor_init(&executor,&support.context,6,&allocator));
  RCCHECK(rclc_executor_add_subscription(&executor,&CH_GRIPPER.sub,&CH_GRIPPER.sub_msg,&cb_gripper,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&CH_DRIL_SERVO.sub,&CH_DRIL_SERVO.sub_msg,&cb_dril_servo,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&CH_SW180.sub,&CH_SW180.sub_msg,&cb_sw180,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&CH_CAM.sub,&CH_CAM.sub_msg,&cb_cam,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&sub_motor,&msg_motor,&cb_motor,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&sub_linear,&msg_linear,&cb_linear,ON_NEW_DATA));
}

void loop(){
  rclc_executor_spin_some(&executor,RCL_MS_TO_NS(10));
}