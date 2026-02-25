#include <Arduino.h>
#include <micro_ros_platformio.h>
#include "esp32_config.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ while(1){ delay(100); } } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }

// ===================== micro-ROS =====================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

// ===================== Publishers =====================
rcl_publisher_t pub_gripper;
rcl_publisher_t pub_servo_dril;
rcl_publisher_t pub_servo_switch;
rcl_publisher_t pub_linear_fb;
rcl_publisher_t pub_motor_dril;
rcl_publisher_t pub_limit_up;
rcl_publisher_t pub_limit_down;

std_msgs__msg__Int16 msg_gripper;
std_msgs__msg__Int16 msg_servo_dril;
std_msgs__msg__Int16 msg_servo_switch;
std_msgs__msg__Int16 msg_linear_fb;
std_msgs__msg__Int16 msg_motor_dril;
std_msgs__msg__Int16 msg_limit_up;
std_msgs__msg__Int16 msg_limit_down;

// ===================== Subscribers =====================
rcl_subscription_t sub_gripper;
rcl_subscription_t sub_servo_dril;
rcl_subscription_t sub_servo_switch;
rcl_subscription_t sub_linear;
rcl_subscription_t sub_motor_dril;

std_msgs__msg__Int16 sub_msg_gripper;
std_msgs__msg__Int16 sub_msg_servo_dril;
std_msgs__msg__Int16 sub_msg_servo_switch;
std_msgs__msg__Int16 sub_msg_linear;
std_msgs__msg__Int16 sub_msg_motor_dril;

// ===================== Limit Pins =====================
static const int LIMIT_UP_PIN   = PIN_LIMIT_LEFT;
static const int LIMIT_DOWN_PIN = PIN_LIMIT_RIGHT;

bool last_limit_up_state = false;
bool last_limit_down_state = false;

// ===================== Callbacks =====================
void cb_gripper(const void * msgin)
{
  const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;
  msg_gripper.data = msg->data;
  RCSOFTCHECK(rcl_publish(&pub_gripper, &msg_gripper, NULL));
}

void cb_servo_dril(const void * msgin)
{
  const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;
  msg_servo_dril.data = msg->data;
  RCSOFTCHECK(rcl_publish(&pub_servo_dril, &msg_servo_dril, NULL));
}

void cb_servo_switch(const void * msgin)
{
  const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;
  msg_servo_switch.data = msg->data;
  RCSOFTCHECK(rcl_publish(&pub_servo_switch, &msg_servo_switch, NULL));
}

void cb_linear(const void * msgin)
{
  const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;
  msg_linear_fb.data = msg->data;
  RCSOFTCHECK(rcl_publish(&pub_linear_fb, &msg_linear_fb, NULL));
}

void cb_motor_dril(const void * msgin)
{
  const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;
  msg_motor_dril.data = msg->data;
  RCSOFTCHECK(rcl_publish(&pub_motor_dril, &msg_motor_dril, NULL));
}

// ===================== Setup =====================
void setup()
{
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  pinMode(LIMIT_UP_PIN, INPUT_PULLUP);
  pinMode(LIMIT_DOWN_PIN, INPUT_PULLUP);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));

  // Publishers
  RCCHECK(rclc_publisher_init_best_effort(&pub_gripper, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/tao/cmd_gripper/rpm"));

  RCCHECK(rclc_publisher_init_best_effort(&pub_servo_dril, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/tao/cmd_servo_dril/rpm"));

  RCCHECK(rclc_publisher_init_best_effort(&pub_servo_switch, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/tao/cmd_servo_switch180/rpm"));

  RCCHECK(rclc_publisher_init_best_effort(&pub_linear_fb, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/tao/cmd_linear/fb"));

  RCCHECK(rclc_publisher_init_best_effort(&pub_motor_dril, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/tao/moter_dril/rpm"));

  RCCHECK(rclc_publisher_init_best_effort(&pub_limit_up, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/tao/limit_up"));

  RCCHECK(rclc_publisher_init_best_effort(&pub_limit_down, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/tao/limit_down"));

  // Subscribers
  RCCHECK(rclc_subscription_init_best_effort(&sub_gripper, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/tao/cmd_gripper"));

  RCCHECK(rclc_subscription_init_best_effort(&sub_servo_dril, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/tao/cmd_servo_dril"));

  RCCHECK(rclc_subscription_init_best_effort(&sub_servo_switch, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/tao/cmd_servo_switch180"));

  RCCHECK(rclc_subscription_init_best_effort(&sub_linear, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/tao/cmd_linear"));

  RCCHECK(rclc_subscription_init_best_effort(&sub_motor_dril, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/tao/moter_dril"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));

  RCCHECK(rclc_executor_add_subscription(&executor, &sub_gripper, &sub_msg_gripper, &cb_gripper, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_servo_dril, &sub_msg_servo_dril, &cb_servo_dril, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_servo_switch, &sub_msg_servo_switch, &cb_servo_switch, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_linear, &sub_msg_linear, &cb_linear, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_motor_dril, &sub_msg_motor_dril, &cb_motor_dril, ON_NEW_DATA));
}

// ===================== Loop =====================
void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  bool limit_up_state = (digitalRead(LIMIT_UP_PIN) == LOW);
  bool limit_down_state = (digitalRead(LIMIT_DOWN_PIN) == LOW);

  if(limit_up_state != last_limit_up_state)
  {
    msg_limit_up.data = limit_up_state ? 1 : 0;
    RCSOFTCHECK(rcl_publish(&pub_limit_up, &msg_limit_up, NULL));
    last_limit_up_state = limit_up_state;
  }

  if(limit_down_state != last_limit_down_state)
  {
    msg_limit_down.data = limit_down_state ? 1 : 0;
    RCSOFTCHECK(rcl_publish(&pub_limit_down, &msg_limit_down, NULL));
    last_limit_down_state = limit_down_state;
  }
}