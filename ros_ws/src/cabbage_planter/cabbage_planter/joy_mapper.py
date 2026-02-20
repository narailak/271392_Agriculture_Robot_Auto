import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
import time
import math
from geometry_msgs.msg import Twist



class JoyMapper(Node):

    def __init__(self):
        super().__init__('joy_mapper')

        self.sub = self.create_subscription(Joy, '/joy', self.cb, 10)

        # publish MANUAL command topics
        self.pub_motor = self.create_publisher(Int16, '/joy_cmd_moter_dril', 10)
        self.pub_gripper = self.create_publisher(Int16, '/joy_cmd_gripper', 10)
        self.pub_linear = self.create_publisher(Int16, '/joy_cmd_linear', 10)
        self.pub_servo_dril = self.create_publisher(Int16, '/joy_cmd_servo_dril', 10)
        self.pub_servo_sw = self.create_publisher(Int16, '/joy_cmd_servo_switch180', 10)
        self.pub_step = self.create_publisher(Int16, '/joy_cmd_step_load', 10)
        self.pub_vel = self.create_publisher(Twist, '/joy_cmd_vel', 10)


        self.debounce_time = 0.3
        self.last_press = {}

        self.motor_on = False
        self.gripper_on = False
        self.servo_dril_on = False
        self.servo_sw_on = False

        self.n = 8
        self.step_index = 0

    def debounce(self, key):
        now = time.time()
        if key not in self.last_press or now - self.last_press[key] > self.debounce_time:
            self.last_press[key] = now
            return True
        return False

    def cb(self, msg):

        # ===== MOTOR DRILL (A toggle) =====
        if msg.buttons[0] == 1 and self.debounce("A"):
            self.motor_on = not self.motor_on
            val = 100 if self.motor_on else 0
            self.pub_motor.publish(Int16(data=val))

        # ===== GRIPPER (X toggle) =====
        if msg.buttons[2] == 1 and self.debounce("X"):
            self.gripper_on = not self.gripper_on
            val = 80 if self.gripper_on else 25
            self.pub_gripper.publish(Int16(data=val))

        # ===== LINEAR (D-pad UD) =====
        lin = Int16()
        if msg.axes[7] < -0.5:
            lin.data = -1
        elif msg.axes[7] > 0.5:
            lin.data = 1
        else:
            lin.data = 0
        self.pub_linear.publish(lin)

        # ===== SERVO DRIL (D-pad LEFT toggle) =====
        if msg.axes[6] < -0.5 and self.debounce("DL"):
            self.servo_dril_on = not self.servo_dril_on
            val = 125 if self.servo_dril_on else 25
            self.pub_servo_dril.publish(Int16(data=val))

        # ===== SERVO SWITCH 180 (D-pad RIGHT toggle) =====
        if msg.axes[6] > 0.5 and self.debounce("DR"):
            self.servo_sw_on = not self.servo_sw_on
            val = 220 if self.servo_sw_on else 40
            self.pub_servo_sw.publish(Int16(data=val))

        # ===== STEP LOAD (Y step rotate) =====
        if msg.buttons[3] == 1 and self.debounce("Y"):
            angle = round((360 / self.n) * self.step_index)
            self.pub_step.publish(Int16(data=angle))
            self.step_index = (self.step_index + 1) % self.n

        # ===== DRIVE =====
        tw = Twist()

        tw.linear.x = -msg.axes[1]    # forward/back
        tw.angular.z = msg.axes[0]    # turn

        self.pub_vel.publish(tw)

def main():
    rclpy.init()
    node = JoyMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
