import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, String
from geometry_msgs.msg import Twist


class ModeMux(Node):

    def __init__(self):
        super().__init__('mode_mux')

        self.mode = "manual"

        # ---- subscribe mode ----
        self.create_subscription(String, '/mode', self.mode_cb, 10)

        # ---- subscribe MANUAL ----
        self.create_subscription(Int16, '/joy_cmd_moter_dril', self.joy_motor_cb, 10)
        self.create_subscription(Int16, '/joy_cmd_gripper', self.joy_gripper_cb, 10)
        self.create_subscription(Int16, '/joy_cmd_linear', self.joy_linear_cb, 10)
        self.create_subscription(Int16, '/joy_cmd_servo_dril', self.joy_servo_cb, 10)
        self.create_subscription(Int16, '/joy_cmd_servo_switch180', self.joy_sw_cb, 10)
        self.create_subscription(Int16, '/joy_cmd_step_load', self.joy_step_cb, 10)
        self.create_subscription(Twist, '/joy_cmd_vel', self.joy_vel_cb, 10)


        # ---- subscribe AUTO ----
        self.create_subscription(Int16, '/auto_cmd_moter_dril', self.auto_motor_cb, 10)
        self.create_subscription(Int16, '/auto_cmd_gripper', self.auto_gripper_cb, 10)
        self.create_subscription(Int16, '/auto_cmd_linear', self.auto_linear_cb, 10)
        self.create_subscription(Int16, '/auto_cmd_servo_dril', self.auto_servo_cb, 10)
        self.create_subscription(Int16, '/auto_cmd_servo_switch180', self.auto_sw_cb, 10)
        self.create_subscription(Int16, '/auto_cmd_step_load', self.auto_step_cb, 10)
        self.create_subscription(Twist, '/auto_cmd_vel', self.auto_vel_cb, 10)


        # ---- publishers (final command) ----
        self.pub_motor = self.create_publisher(Int16, '/cmd_moter_dril', 10)
        self.pub_gripper = self.create_publisher(Int16, '/cmd_gripper', 10)
        self.pub_linear = self.create_publisher(Int16, '/cmd_linear', 10)
        self.pub_servo = self.create_publisher(Int16, '/cmd_servo_dril', 10)
        self.pub_sw = self.create_publisher(Int16, '/cmd_servo_switch180', 10)
        self.pub_step = self.create_publisher(Int16, '/cmd_step_load', 10)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)


    # ---------- mode ----------
    def mode_cb(self, msg):
        self.mode = msg.data
        self.get_logger().info(f"Mode: {self.mode}")

    # ---------- MANUAL ----------
    def joy_motor_cb(self, msg):
        if self.mode == "manual":
            self.pub_motor.publish(msg)

    def joy_gripper_cb(self, msg):
        if self.mode == "manual":
            self.pub_gripper.publish(msg)

    def joy_linear_cb(self, msg):
        if self.mode == "manual":
            self.pub_linear.publish(msg)

    def joy_servo_cb(self, msg):
        if self.mode == "manual":
            self.pub_servo.publish(msg)

    def joy_sw_cb(self, msg):
        if self.mode == "manual":
            self.pub_sw.publish(msg)

    def joy_step_cb(self, msg):
        if self.mode == "manual":
            self.pub_step.publish(msg)

    def joy_vel_cb(self, msg):
        if self.mode == "manual":
            self.pub_vel.publish(msg)

    # ---------- AUTO ----------
    def auto_motor_cb(self, msg):
        if self.mode == "auto":
            self.pub_motor.publish(msg)

    def auto_gripper_cb(self, msg):
        if self.mode == "auto":
            self.pub_gripper.publish(msg)

    def auto_linear_cb(self, msg):
        if self.mode == "auto":
            self.pub_linear.publish(msg)

    def auto_servo_cb(self, msg):
        if self.mode == "auto":
            self.pub_servo.publish(msg)

    def auto_sw_cb(self, msg):
        if self.mode == "auto":
            self.pub_sw.publish(msg)

    def auto_step_cb(self, msg):
        if self.mode == "auto":
            self.pub_step.publish(msg)

    def auto_vel_cb(self, msg):
        if self.mode == "auto":
            self.pub_vel.publish(msg)

def main():
    rclpy.init()
    node = ModeMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
