#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

#include <config.h>
#include <motor.h>
#include <PIDF.h>
#include <Utilize.h>

#include <esp32_Encoder.h>
#include <esp32_hardware.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { ESP.restart(); } }
#define EXECUTE_EVERY_N_MS(MS, X) \
do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis(); } \
    if (uxr_millis() - init > MS) { X; init = uxr_millis(); } \
} while (0)

// ---------------- ROS ----------------
rcl_publisher_t debug_encoder_wheels_publisher;
std_msgs__msg__Float32MultiArray debug_encoder_wheels_msg;

rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

// ---------------- Motor ----------------
Controller motor1(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_BRAKE, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Controller motor2(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_BRAKE, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Controller motor3(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_BRAKE, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Controller motor4(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_BRAKE, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

// ---------------- 4 Wheel Encoder (A/B) ----------------
esp32_Encoder encLF(Encoder_LF_A, Encoder_LF_B, COUNTS_PER_REV, ENCODER_INV_LF, GEAR_RATIO, WHEEL_DIAMETER);
esp32_Encoder encLB(Encoder_LB_A, Encoder_LB_B, COUNTS_PER_REV, ENCODER_INV_LB, GEAR_RATIO, WHEEL_DIAMETER);
esp32_Encoder encRF(Encoder_RF_A, Encoder_RF_B, COUNTS_PER_REV, ENCODER_INV_RF, GEAR_RATIO, WHEEL_DIAMETER);
esp32_Encoder encRB(Encoder_RB_A, Encoder_RB_B, COUNTS_PER_REV, ENCODER_INV_RB, GEAR_RATIO, WHEEL_DIAMETER);

// ---------------- Motor Control ----------------
void MovePower(int fl, int fr, int bl, int br)
{
    motor1.spin(fl);
    motor2.spin(fr);
    motor3.spin(bl);
    motor4.spin(br);
}

void cmd_vel_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

    float V_x = msg->linear.x;
    float W_z = msg->angular.z;

    float d = max(abs(V_x) + abs(W_z), (float) PWM_Max);

    int fl = (V_x - W_z)/d * PWM_Max;
    int fr = (V_x + W_z)/d * PWM_Max;
    int bl = (V_x - W_z)/d * PWM_Max;
    int br = (V_x + W_z)/d * PWM_Max;

    MovePower(fl, fr, bl, br);
}

// ---------------- Encoder Publish ----------------
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);

    debug_encoder_wheels_msg.data.data[0] = encLF.read();
    debug_encoder_wheels_msg.data.data[1] = encLB.read();
    debug_encoder_wheels_msg.data.data[2] = encRF.read();
    debug_encoder_wheels_msg.data.data[3] = encRB.read();

    rcl_publish(&debug_encoder_wheels_publisher, &debug_encoder_wheels_msg, NULL);
}

// ---------------- Setup ----------------
void setup()
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "teelek_drive", "", &support);

    rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/teelek/cmd_move");

    rclc_publisher_init_default(
        &debug_encoder_wheels_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/teelek/debug/encoder_wheels");

    rosidl_runtime_c__float__Sequence__init(&debug_encoder_wheels_msg.data, 4);

    rclc_timer_init_default(&control_timer, &support,
        RCL_MS_TO_NS(50), timer_callback);

    rclc_executor_init(&executor, &support.context, 2, &allocator);

    rclc_executor_add_subscription(
        &executor, &cmd_vel_subscriber,
        &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);

    rclc_executor_add_timer(&executor, &control_timer);
}

void loop()
{
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}