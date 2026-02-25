#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

#include <esp32_config.h>

#define RCCHECK(fn) do { rcl_ret_t rc=(fn); if(rc!=RCL_RET_OK){ ESP.restart(); } } while(0)

// ===== Common ROS =====
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#if defined(Drive_Control)

#include <motor.h>
#include <esp32_Encoder.h>

// Motors
Controller motor1(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_BRAKE, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Controller motor2(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_BRAKE, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Controller motor3(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_BRAKE, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Controller motor4(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_BRAKE, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

rcl_subscription_t sub_cmd_vel;
geometry_msgs__msg__Twist msg_cmd_vel;

void MovePower(int fl,int fr,int bl,int br){
    motor1.spin(fl);
    motor2.spin(fr);
    motor3.spin(bl);
    motor4.spin(br);
}

void cmd_vel_cb(const void* msgin){
    const auto* msg=(const geometry_msgs__msg__Twist*)msgin;
    float Vx=msg->linear.x;
    float Wz=msg->angular.z;

    float d=max(abs(Vx)+abs(Wz),(float)PWM_Max);

    int fl=(Vx-Wz)/d*PWM_Max;
    int fr=(Vx+Wz)/d*PWM_Max;
    int bl=(Vx-Wz)/d*PWM_Max;
    int br=(Vx+Wz)/d*PWM_Max;

    MovePower(fl,fr,bl,br);
}

#endif



#if defined(Actuator_Control)

#include <ESP32Servo.h>

struct ServoChan{
    Servo servo;
    int pin;
    int max_deg;
    const char* topic;
    rcl_subscription_t sub;
    std_msgs__msg__Int16 msg;
};

ServoChan GRIP{Servo(),SERVO_GRIPPER_PIN,SERVO_GRIPPER_MAX_DEG,"/cmd_gripper"};
ServoChan DRIL{Servo(),SERVO_DRIL_PIN,SERVO_DRIL_MAX_DEG,"/cmd_servo_dril"};
ServoChan SW{Servo(),SERVO_SWITCH180_PIN,SERVO_SWITCH180_MAX_DEG,"/cmd_servo_switch180"};

int clamp(int v,int maxv){
    if(v<0) return 0;
    if(v>maxv) return maxv;
    return v;
}

void servo_cb_generic(ServoChan& ch,const void* msgin){
    const auto* m=(const std_msgs__msg__Int16*)msgin;
    int angle=clamp(m->data,ch.max_deg);
    ch.servo.write(angle);
}

void cb_grip(const void* msgin){ servo_cb_generic(GRIP,msgin); }
void cb_dril(const void* msgin){ servo_cb_generic(DRIL,msgin); }
void cb_sw(const void* msgin){ servo_cb_generic(SW,msgin); }

rcl_subscription_t sub_step;
std_msgs__msg__Int16 msg_step;

void step_cb(const void* msgin){
    const auto* m=(const std_msgs__msg__Int16*)msgin;
    int val=m->data;

    if(val<=0){
        ledcWrite(TB_PWM_CHANNEL,0);
        digitalWrite(TB_AIN1,LOW);
        digitalWrite(TB_AIN2,LOW);
    }else{
        if(val>100) val=100;
        int duty=map(val,0,100,0,255);
        digitalWrite(TB_AIN1,HIGH);
        digitalWrite(TB_AIN2,LOW);
        ledcWrite(TB_PWM_CHANNEL,duty);
    }
}

#endif



void setup(){
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    allocator=rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support,0,NULL,&allocator));
    RCCHECK(rclc_node_init_default(&node,"esp32_node","",&support));

#if defined(Drive_Control)

    RCCHECK(rclc_subscription_init_default(
        &sub_cmd_vel,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),
        "/cmd_vel"));

    RCCHECK(rclc_executor_init(&executor,&support.context,1,&allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor,&sub_cmd_vel,&msg_cmd_vel,&cmd_vel_cb,ON_NEW_DATA));

#endif


#if defined(Actuator_Control)

    GRIP.servo.attach(GRIP.pin,SERVO_MIN_US,SERVO_MAX_US);
    DRIL.servo.attach(DRIL.pin,SERVO_MIN_US,SERVO_MAX_US);
    SW.servo.attach(SW.pin,SERVO_MIN_US,SERVO_MAX_US);

    pinMode(TB_AIN1,OUTPUT);
    pinMode(TB_AIN2,OUTPUT);
    pinMode(TB_STBY,OUTPUT);
    digitalWrite(TB_STBY,HIGH);

    ledcSetup(TB_PWM_CHANNEL,TB_PWM_FREQ,TB_PWM_RES);
    ledcAttachPin(TB_PWMA,TB_PWM_CHANNEL);

    RCCHECK(rclc_subscription_init_default(
        &sub_step,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),
        "/cmd_step_load"));

    RCCHECK(rclc_executor_init(&executor,&support.context,1,&allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor,&sub_step,&msg_step,&step_cb,ON_NEW_DATA));

#endif
}

void loop(){
    rclc_executor_spin_some(&executor,RCL_MS_TO_NS(100));
}