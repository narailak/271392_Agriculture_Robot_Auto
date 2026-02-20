// ========= ESP32-S3 micro-ROS ROBOT (STABLE VERSION) =========

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>

// ---------------- Pins ----------------
#define L_DIR 4
#define L_PWM 5
#define R_DIR 15
#define R_PWM 16

#define ENC1 9
#define ENC2 10
#define ENC3 11
#define ENC4 12

// ---------------- Encoder ----------------
volatile long enc1=0, enc2=0, enc3=0, enc4=0;
void IRAM_ATTR isr1(){ enc1++; }
void IRAM_ATTR isr2(){ enc2++; }
void IRAM_ATTR isr3(){ enc3++; }
void IRAM_ATTR isr4(){ enc4++; }

// ---------------- micro-ROS ----------------
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_init_options_t init_options;
rclc_executor_t executor;

rcl_subscription_t sub_cmd;
geometry_msgs__msg__Twist msg_cmd;

rcl_publisher_t pub_enc;
std_msgs__msg__Int32MultiArray msg_enc;

rcl_publisher_t pub_counter;
std_msgs__msg__Int32 msg_counter;

// ---------------- State ----------------
enum AgentState {WAITING, AVAILABLE, CONNECTED, DISCONNECTED};
AgentState state = WAITING;

int32_t counter = 0;
float cmd_v = 0;
float cmd_w = 0;

// ===========================================================
void cmdVelCb(const void *msgin)
{
  const auto *m = (const geometry_msgs__msg__Twist*)msgin;
  cmd_v = m->linear.x;
  cmd_w = m->angular.z;
}

// ===========================================================
bool create_entities()
{
  allocator = rcl_get_default_allocator();

  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 96);

  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  rclc_node_init_default(&node, "esp32_robot_node", "", &support);

  // Subscriber
  rclc_subscription_init_default(
    &sub_cmd,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),
    "/cmd_vel");

  // Encoder publisher
  rclc_publisher_init_default(
    &pub_enc,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int32MultiArray),
    "/motor_feedback/encoders");

  msg_enc.data.capacity = 4;
  msg_enc.data.size = 4;
  msg_enc.data.data = (int32_t*)malloc(4*sizeof(int32_t));

  // Counter publisher (debug)
  rclc_publisher_init_default(
    &pub_counter,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int32),
    "/esp32_counter");

  // Executor
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, 1, &allocator);

  rclc_executor_add_subscription(
    &executor,
    &sub_cmd,
    &msg_cmd,
    &cmdVelCb,
    ON_NEW_DATA);

  return true;
}

// ===========================================================
void destroy_entities()
{
  rcl_subscription_fini(&sub_cmd,&node);
  rcl_publisher_fini(&pub_enc,&node);
  rcl_publisher_fini(&pub_counter,&node);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ===========================================================
void setup()
{
  Serial.begin(115200);
  delay(2000);
  set_microros_serial_transports(Serial);

  pinMode(ENC1,INPUT_PULLUP);
  pinMode(ENC2,INPUT_PULLUP);
  pinMode(ENC3,INPUT_PULLUP);
  pinMode(ENC4,INPUT_PULLUP);

  attachInterrupt(ENC1,isr1,RISING);
  attachInterrupt(ENC2,isr2,RISING);
  attachInterrupt(ENC3,isr3,RISING);
  attachInterrupt(ENC4,isr4,RISING);
}

// ===========================================================
void loop()
{
  switch(state)
  {
    case WAITING:
      if(RMW_RET_OK == rmw_uros_ping_agent(100,1))
        state = AVAILABLE;
      break;

    case AVAILABLE:
      if(create_entities())
        state = CONNECTED;
      else
        state = WAITING;
      break;

    case CONNECTED:

      if(RMW_RET_OK != rmw_uros_ping_agent(100,1))
      {
        state = DISCONNECTED;
        break;
      }

      rclc_executor_spin_some(&executor,0);

      // Publish encoder
      msg_enc.data.data[0] = enc1;
      msg_enc.data.data[1] = enc2;
      msg_enc.data.data[2] = enc3;
      msg_enc.data.data[3] = enc4;
      rcl_publish(&pub_enc,&msg_enc,NULL);

      // Publish counter (debug)
      msg_counter.data = counter++;
      rcl_publish(&pub_counter,&msg_counter,NULL);

      delay(100);
      break;

    case DISCONNECTED:
      destroy_entities();
      state = WAITING;
      break;
  }
}