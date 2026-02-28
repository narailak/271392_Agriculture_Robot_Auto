#include <Arduino.h>
#include <math.h>

#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/float32_multi_array.h>

#include <esp32_Encoder.h>
#include "soc/gpio_reg.h"

#include "esp32_config.h"

// =====================================================
// ROS
// =====================================================
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_subscription_t cmd_move_sub;
rcl_subscription_t step_sub;
rcl_subscription_t reset_sub;

geometry_msgs__msg__Twist cmd_move_msg;
geometry_msgs__msg__Twist reset_msg;
std_msgs__msg__Int16 step_msg;

rcl_publisher_t motor_pub;
rcl_publisher_t encoder_pub;
rcl_publisher_t step_fb_pub;

std_msgs__msg__Float32MultiArray motor_msg;
std_msgs__msg__Float32MultiArray encoder_msg;
std_msgs__msg__Int16 step_fb;

rcl_timer_t control_timer;

// =====================================================
// MOTOR STATE
// =====================================================
float cmd_vx = 0.0f;
float cmd_wz = 0.0f;

float m1=0,m2=0,m3=0,m4=0;

uint8_t rpm_to_duty(float rpm)
{
    float a=fabs(rpm)/MAX_RPM;
    if(a>1)a=1;
    return a*255;
}

float v_to_rpm(float v)
{
    return (v/WHEEL_RADIUS)*60.0f/(2.0f*M_PI);
}

void setMotor(int dir,int ch,float rpm)
{
    digitalWrite(dir,(rpm>=0));
    ledcWrite(ch,rpm_to_duty(rpm));
}

// =====================================================
// ENCODER WHEELS
// =====================================================
esp32_Encoder encLF(ENC1_A,ENC1_B,2048,false,1,0.09);
esp32_Encoder encLB(ENC3_A,ENC3_B,2048,false,1,0.09);
esp32_Encoder encRF(ENC2_A,ENC2_B,2048,false,1,0.09);
esp32_Encoder encRB(ENC4_A,ENC4_B,2048,false,1,0.09);

long offsetLF=0;
long offsetLB=0;
long offsetRF=0;
long offsetRB=0;

// =====================================================
// STEPPER
// =====================================================
static volatile long step_current=0;
static volatile long step_remaining=0;
static volatile int8_t step_dir=0;

hw_timer_t *step_timer=NULL;
bool pulse_state=false;

void IRAM_ATTR stepISR()
{
    if(step_remaining<=0) return;

    uint32_t mask=(1<<PIN_PUL);

    if(!pulse_state)
    {
        REG_WRITE(GPIO_OUT_W1TS_REG,mask);
        pulse_state=true;
    }
    else
    {
        REG_WRITE(GPIO_OUT_W1TC_REG,mask);
        pulse_state=false;

        step_remaining--;
        step_current+=step_dir;
    }
}

// =====================================================
// CALLBACKS
// =====================================================
void cmdMoveCallback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg =
        (const geometry_msgs__msg__Twist*)msgin;

    cmd_vx = msg->linear.x;
    cmd_wz = msg->angular.z;
}

void resetEncoderCallback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg =
        (const geometry_msgs__msg__Twist*)msgin;

    if(msg->linear.x > 0.5)
    {
        offsetLF = encLF.read();
        offsetLB = encLB.read();
        offsetRF = encRF.read();
        offsetRB = encRB.read();
    }
}

void stepCmdCallback(const void *msgin)
{
    const std_msgs__msg__Int16 *msg =
        (const std_msgs__msg__Int16*)msgin;

    int deg = msg->data % 360;

    long target = (deg * 3200) / 360;
    long delta  = target - step_current;

    step_dir = (delta>0)?1:-1;
    step_remaining = labs(delta);
}

// =====================================================
// CONTROL LOOP
// =====================================================
void controlLoop(rcl_timer_t*, int64_t)
{
    float v_left  = cmd_vx - cmd_wz*(TRACK_WIDTH*0.5f);
    float v_right = cmd_vx + cmd_wz*(TRACK_WIDTH*0.5f);

    float rpm_left  = v_to_rpm(v_left);
    float rpm_right = v_to_rpm(v_right);

    m1=rpm_left;
    m3=rpm_left;
    m2=rpm_right;
    m4=rpm_right;

    setMotor(L_DIR1,PWM_CH_M1,m1);
    setMotor(R_DIR1,PWM_CH_M2,m2);
    setMotor(L_DIR2,PWM_CH_M3,m3);
    setMotor(R_DIR2,PWM_CH_M4,m4);

    motor_msg.data.data[0]=rpm_to_duty(m1);
    motor_msg.data.data[1]=rpm_to_duty(m2);
    motor_msg.data.data[2]=rpm_to_duty(m3);
    motor_msg.data.data[3]=rpm_to_duty(m4);

    rcl_publish(&motor_pub,&motor_msg,NULL);

    encoder_msg.data.data[0]=encLF.read()-offsetLF;
    encoder_msg.data.data[1]=encLB.read()-offsetLB;
    encoder_msg.data.data[2]=encRF.read()-offsetRF;
    encoder_msg.data.data[3]=encRB.read()-offsetRB;

    rcl_publish(&encoder_pub,&encoder_msg,NULL);

    step_fb.data=(step_current*360)/3200;
    rcl_publish(&step_fb_pub,&step_fb,NULL);
}

// =====================================================
// SETUP
// =====================================================
void setup()
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    pinMode(L_DIR1,OUTPUT);
    pinMode(L_DIR2,OUTPUT);
    pinMode(R_DIR1,OUTPUT);
    pinMode(R_DIR2,OUTPUT);

    ledcSetup(PWM_CH_M1,PWM_FREQ,PWM_RESOLUTION);
    ledcSetup(PWM_CH_M2,PWM_FREQ,PWM_RESOLUTION);
    ledcSetup(PWM_CH_M3,PWM_FREQ,PWM_RESOLUTION);
    ledcSetup(PWM_CH_M4,PWM_FREQ,PWM_RESOLUTION);

    ledcAttachPin(L_PWM1,PWM_CH_M1);
    ledcAttachPin(R_PWM1,PWM_CH_M2);
    ledcAttachPin(L_PWM2,PWM_CH_M3);
    ledcAttachPin(R_PWM2,PWM_CH_M4);

    pinMode(PIN_PUL,OUTPUT);
    pinMode(PIN_DIR,OUTPUT);
    pinMode(PIN_ENA,OUTPUT);

    step_timer = timerBegin(0,80,true);
    timerAttachInterrupt(step_timer,&stepISR,true);
    timerAlarmWrite(step_timer,400,true);
    timerAlarmEnable(step_timer);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support,0,NULL,&allocator);
    rclc_node_init_default(&node,"esp32_robot","",&support);

    motor_msg.data.data=(float*)malloc(4*sizeof(float));
    motor_msg.data.size=4;
    motor_msg.data.capacity=4;

    encoder_msg.data.data=(float*)malloc(4*sizeof(float));
    encoder_msg.data.size=4;
    encoder_msg.data.capacity=4;

    rclc_publisher_init_best_effort(
        &motor_pub,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32MultiArray),
        "/motor_debug/duty");

    rclc_publisher_init_best_effort(
        &encoder_pub,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32MultiArray),
        "/tao/debug/encoder_wheels");

    rclc_publisher_init_best_effort(
        &step_fb_pub,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),
        "/tao/cmd_step_load/fb");

    rclc_subscription_init_default(
        &cmd_move_sub,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),
        "/tao/cmd_move");

    rclc_subscription_init_default(
        &step_sub,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),
        "/tao/cmd_step_load");

    rclc_subscription_init_default(
        &reset_sub,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),
        "/tao/cmd_resetencoder");

    rclc_timer_init_default(
        &control_timer,&support,
        RCL_MS_TO_NS(20),
        controlLoop);

    executor=rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor,&support.context,4,&allocator);

    rclc_executor_add_subscription(
        &executor,&cmd_move_sub,&cmd_move_msg,
        &cmdMoveCallback,ON_NEW_DATA);

    rclc_executor_add_subscription(
        &executor,&step_sub,&step_msg,
        &stepCmdCallback,ON_NEW_DATA);

    rclc_executor_add_subscription(
        &executor,&reset_sub,&reset_msg,
        &resetEncoderCallback,ON_NEW_DATA);

    rclc_executor_add_timer(&executor,&control_timer);
}

// =====================================================
void loop()
{
    rclc_executor_spin_some(&executor,0);
}