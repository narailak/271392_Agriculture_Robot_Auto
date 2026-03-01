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
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

// User specific libraries (must be in your lib/ or src/ folder)
#include <config.h>
#include <motor.h>
#include <PIDF.h>
#include <Utilize.h>
#include <esp32_Encoder.h>     // Encoder 4 wheels
#include <esp32_hardware.h>

#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            rclErrorLoop();          \
        }                            \
    }
#define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }
#define EXECUTE_EVERY_N_MS(MS, X)          \
    do                                     \
    {                                      \
        static volatile int64_t init = -1; \
        if (init == -1)                    \
        {                                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS)      \
        {                                  \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)

//------------------------------ < Define > -------------------------------------//

// ROS 2 Entities
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

rcl_subscription_t cmd_resetencoder_subscriber;
geometry_msgs__msg__Twist cmd_resetencoder_msg;

rcl_publisher_t debug_encoder_wheels_publisher;
std_msgs__msg__Float32MultiArray debug_encoder_wheels_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;
static unsigned long last_sync = 0;

// Encoder wheels variables
long offsetLF = 0, offsetLB = 0, offsetRF = 0, offsetRB = 0;

enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

// Motor Controllers
Controller motor1(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_BRAKE, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Controller motor2(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_BRAKE, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Controller motor3(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_BRAKE, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Controller motor4(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_BRAKE, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

// Encoder 4 wheels instances
esp32_Encoder encLF(Encoder_LF_A, Encoder_LF_B, COUNTS_PER_REV, ENCODER_INV_LF, GEAR_RATIO, WHEEL_DIAMETER);
esp32_Encoder encLB(Encoder_LB_A, Encoder_LB_B, COUNTS_PER_REV, ENCODER_INV_LB, GEAR_RATIO, WHEEL_DIAMETER);
esp32_Encoder encRF(Encoder_RF_A, Encoder_RF_B, COUNTS_PER_REV, ENCODER_INV_RF, GEAR_RATIO, WHEEL_DIAMETER);
esp32_Encoder encRB(Encoder_RB_A, Encoder_RB_B, COUNTS_PER_REV, ENCODER_INV_RB, GEAR_RATIO, WHEEL_DIAMETER);

//------------------------------ < Function Prototype > ------------------------------//
void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
struct timespec getTime();

void cmd_vel_callback(const void *msgin);
void cmd_reset_encoder_callback(const void *msgin);
void controlCallback(rcl_timer_t *timer, int64_t last_call_time);

void publishData();
void getEncoderWheelsTick(); 
void resetEncoderOffset();
void MovePower(int Motor1Speed, int Motor2Speed, int Motor3Speed, int Motor4Speed);

//------------------------------ < Setup & Loop > -------------------------------------//

void setup()
{
    Serial.begin(115200);

#ifdef MICROROS_WIFI
    IPAddress agent_ip(AGENT_IP);
    uint16_t agent_port = AGENT_PORT;
    set_microros_wifi_transports((char*)SSID, (char*)SSID_PW, agent_ip, agent_port);
#else
    set_microros_serial_transports(Serial);
#endif

    state = WAITING_AGENT;
    last_sync = millis();
}

void loop()
{   
    if (millis() - last_sync > 10000) {
        syncTime();
        last_sync = millis();
    }

    switch (state)
    {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 4)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
    case AGENT_AVAILABLE:
        state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT)
        {
            destroyEntities();
        }
        break;
    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(250, state = (RMW_RET_OK == rmw_uros_ping_agent(300, 3)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED)
        {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(150));
        }
        break;
    case AGENT_DISCONNECTED:
        MovePower(0, 0, 0, 0); // Stop motors when disconnected
        destroyEntities();
        state = WAITING_AGENT;
        break;
    default:
        break;
    }
}

//------------------------------ < Functions > -------------------------------------//

void MovePower(int Motor1Speed, int Motor2Speed, int Motor3Speed, int Motor4Speed)
{
    Motor1Speed = constrain(Motor1Speed, PWM_Min, PWM_Max);
    Motor2Speed = constrain(Motor2Speed, PWM_Min, PWM_Max);
    Motor3Speed = constrain(Motor3Speed, PWM_Min, PWM_Max);
    Motor4Speed = constrain(Motor4Speed, PWM_Min, PWM_Max);

    motor1.spin(Motor1Speed);
    motor2.spin(Motor2Speed);
    motor3.spin(Motor3Speed);
    motor4.spin(Motor4Speed);
}

void cmd_vel_callback(const void *msgin) 
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    float V_x = msg->linear.x;
    float W_z = msg->angular.z;
    float d = max(abs(V_x) + abs(W_z), (float) PWM_Max);
    
    // Differential Drive Kinematics
    int fl = (V_x - W_z)/d * (float) PWM_Max;
    int fr = (V_x + W_z)/d * (float) PWM_Max;
    int bl = (V_x - W_z)/d * (float) PWM_Max;
    int br = (V_x + W_z)/d * (float) PWM_Max;
    
    MovePower(fl, fr, bl, br);
}

void cmd_reset_encoder_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    if (msg->linear.x > 0.5)
    {
        resetEncoderOffset();
    }
}

void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) 
    {
        getEncoderWheelsTick();
        publishData(); 
    }
}

void getEncoderWheelsTick() 
{
    long curLF = encLF.read();
    long curLB = encLB.read();
    long curRF = encRF.read();
    long curRB = encRB.read();

    debug_encoder_wheels_msg.data.data[0] = curLF - offsetLF;
    debug_encoder_wheels_msg.data.data[1] = curLB - offsetLB;
    debug_encoder_wheels_msg.data.data[2] = curRF - offsetRF;
    debug_encoder_wheels_msg.data.data[3] = curRB - offsetRB;
}

void resetEncoderOffset()
{
    offsetLF = encLF.read();
    offsetLB = encLB.read();
    offsetRF = encRF.read();
    offsetRB = encRB.read();

    debug_encoder_wheels_msg.data.data[0] = 0;
    debug_encoder_wheels_msg.data.data[1] = 0;
    debug_encoder_wheels_msg.data.data[2] = 0;
    debug_encoder_wheels_msg.data.data[3] = 0;
}

void publishData()
{
    rcl_publish(&debug_encoder_wheels_publisher, &debug_encoder_wheels_msg, NULL);
}

bool createEntities()
{
    allocator = rcl_get_default_allocator();

    // init options
    init_options = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) return false;
    rcl_init_options_set_domain_id(&init_options, 10);

    // support
    if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) return false;

    // executor init
    executor = rclc_executor_get_zero_initialized_executor();
    if (rclc_executor_init(&executor, &support.context, 10, &allocator) != RCL_RET_OK) return false;

    // create node
    if (rclc_node_init_default(&node, "teelek_karake", "", &support) != RCL_RET_OK) return false;

    // -------------------- Publishers --------------------
    if (rclc_publisher_init_default(
            &debug_encoder_wheels_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "teelek/debug/encoder_wheels") != RCL_RET_OK) return false;

    // init float sequence 4 elements for Float32MultiArray
    rosidl_runtime_c__float__Sequence__init(&debug_encoder_wheels_msg.data, 4);

    // -------------------- Subscriptions --------------------
    if (rclc_subscription_init_default(
            &cmd_vel_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/teelek/cmd_move") != RCL_RET_OK) return false;

    if (rclc_subscription_init_default(
            &cmd_resetencoder_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/teelek/cmd_resetencoder") != RCL_RET_OK) return false;

    // add subscriptions to executor
    if (rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg,
        &cmd_vel_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

    if (rclc_executor_add_subscription(&executor, &cmd_resetencoder_subscriber, &cmd_resetencoder_msg,
        &cmd_reset_encoder_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

    // -------------------- Timer --------------------
    const unsigned int control_timeout = 50; // Publish every 50ms (20Hz)
    if (rclc_timer_init_default(&control_timer, &support,
        RCL_MS_TO_NS(control_timeout), &controlCallback) != RCL_RET_OK) return false;

    if (rclc_executor_add_timer(&executor, &control_timer) != RCL_RET_OK) return false;

    // sync time with agent
    syncTime();

    return true;
}

bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_subscription_fini(&cmd_vel_subscriber, &node);
    rcl_subscription_fini(&cmd_resetencoder_subscriber, &node);
    rcl_publisher_fini(&debug_encoder_wheels_publisher, &node);

    rcl_timer_fini(&control_timer);
    rcl_node_fini(&node);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    return true;
}

void syncTime()
{
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

void rclErrorLoop()
{
    ESP.restart();
}