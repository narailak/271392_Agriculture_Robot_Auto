import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

import sys
import termios
import tty
import select
import time


# ================= KEY READER (รองรับ Arrow keys) =================
def get_key(timeout):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)

        if r:
            c1 = sys.stdin.read(1)

            # detect arrow keys
            if c1 == '\x1b':
                c2 = sys.stdin.read(1)
                c3 = sys.stdin.read(1)
                return c1 + c2 + c3

            return c1

        return None

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ================= TELEOP NODE =================
class TaoKeyboardJoy(Node):

    def __init__(self):
        super().__init__('tao_keyboard_teleop')

        # ===== Publishers =====
        self.pub_vel = self.create_publisher(Twist, '/tao/cmd_vel', 10)
        self.pub_motor = self.create_publisher(Int16, '/tao/cmd_motor_dril', 10)
        self.pub_gripper = self.create_publisher(Int16, '/tao/cmd_gripper', 10)
        self.pub_linear = self.create_publisher(Int16, '/tao/cmd_linear', 10)
        self.pub_servo_dril = self.create_publisher(Int16, '/tao/cmd_servo_dril', 10)
        self.pub_servo_sw = self.create_publisher(Int16, '/tao/cmd_servo_switch180', 10)
        self.pub_step = self.create_publisher(Int16, '/tao/cmd_step_load', 10)

        # ===== Speed =====
        self.linear_speed = 0.6
        self.angular_speed = 1.2

        # ===== Toggle states =====
        self.motor_on = False
        self.gripper_on = False
        self.servo_dril_on = False
        self.servo_sw_on = False

        self.n = 8
        self.step_index = 0

        # debounce
        self.last_press = {}
        self.debounce_time = 0.25

        # key hold system
        self.last_key = None
        self.last_time = time.time()

        self.rate = 30.0

        self.get_logger().info("""
=========== TAO KEYBOARD TELEOP ===========
MOVE
 w/s : forward/back
 a/d : rotate

LINEAR ACTUATOR
 ↑ : extend
 ↓ : retract

TOGGLE
 m : drill motor
 g : gripper
 j : servo dril
 l : servo switch180
 y : step rotate

SPACE : STOP
CTRL+C : EXIT
===========================================
""")

        self.run()

    # ================= debounce =================
    def debounce(self, key):
        now = time.time()
        if key not in self.last_press or now - self.last_press[key] > self.debounce_time:
            self.last_press[key] = now
            return True
        return False

    # ================= MAIN LOOP =================
    def run(self):

        period = 1.0 / self.rate

        while rclpy.ok():

            key = get_key(period)

            if key:
                self.last_key = key
                self.last_time = time.time()

            # auto release like joystick
            if time.time() - self.last_time > 0.15:
                self.last_key = None

            k = self.last_key

            # ========= DRIVE =========
            tw = Twist()

            if k == 'w':
                tw.linear.x = self.linear_speed
            elif k == 's':
                tw.linear.x = -self.linear_speed

            if k == 'a':
                tw.angular.z = self.angular_speed
            elif k == 'd':
                tw.angular.z = -self.angular_speed

            self.pub_vel.publish(tw)

            # ========= LINEAR ACTUATOR =========
            lin = Int16()

            if k == '\x1b[A':      # Arrow UP
                lin.data = 1
            elif k == '\x1b[B':    # Arrow DOWN
                lin.data = -1
            else:
                lin.data = 0

            self.pub_linear.publish(lin)

            # ========= MOTOR DRIL =========
            if k == 'm' and self.debounce("motor"):
                self.motor_on = not self.motor_on
                val = 100 if self.motor_on else 0
                self.pub_motor.publish(Int16(data=val))

            # ========= GRIPPER =========
            if k == 'g' and self.debounce("gripper"):
                self.gripper_on = not self.gripper_on
                val = 80 if self.gripper_on else 25
                self.pub_gripper.publish(Int16(data=val))

            # ========= SERVO DRIL =========
            if k == 'j' and self.debounce("servo_dril"):
                self.servo_dril_on = not self.servo_dril_on
                val = 125 if self.servo_dril_on else 25
                self.pub_servo_dril.publish(Int16(data=val))

            # ========= SERVO SWITCH =========
            if k == 'l' and self.debounce("servo_sw"):
                self.servo_sw_on = not self.servo_sw_on
                val = 220 if self.servo_sw_on else 40
                self.pub_servo_sw.publish(Int16(data=val))

            # ========= STEP LOAD =========
            if k == 'y' and self.debounce("step"):
                angle = round((360 / self.n) * self.step_index)
                self.pub_step.publish(Int16(data=angle))
                self.step_index = (self.step_index + 1) % self.n

            # ========= STOP =========
            if k == ' ':
                self.pub_vel.publish(Twist())
                self.pub_linear.publish(Int16(data=0))
                self.last_key = None

            if k == '\x03':
                break


# ================= MAIN =================
def main():
    rclpy.init()
    TaoKeyboardJoy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()