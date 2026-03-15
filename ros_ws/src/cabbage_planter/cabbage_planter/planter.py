import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Bool

import sys
import termios
import tty
import select
import time


# ================= KEY READER =================
def get_key(timeout):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if r:
            c1 = sys.stdin.read(1)
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
        self.pub_servo_cam = self.create_publisher(Int16, '/tao/cmd_servo_cam', 10)

        # ===== Subscribers =====
        self.sub_limit_up = self.create_subscription(
            Bool, '/tao/fb/limit_up', self.limit_up_cb, qos_profile_sensor_data)
        self.sub_limit_down = self.create_subscription(
            Bool, '/tao/fb/limit_down', self.limit_down_cb, qos_profile_sensor_data)

        # ===== Speed =====
        self.linear_speed = 0.6
        self.angular_speed = 1.2

        # ===== Toggle & State variables =====
        self.motor_on = False
        self.gripper_on = False
        self.servo_dril_on = False
        self.servo_sw_on = False
        
        # ===== Auto Mode variables =====
        self.auto_active = False
        self.auto_paused = False       
        self.pause_start_time = 0.0    
        self.auto_state = 0         
        self.auto_timer = 0.0       
        self.auto_step_index = 1       # ตัวนับรอบการหมุน Step Motor สำหรับโหมด Auto
        
        self.limit_up_state = False
        self.limit_down_state = False

        # จำนวน ขั้นตอนการหมุนของ Step Motor ในโหมด Auto (เช่น 8 รอบ = 360 องศา / 8 = 45 องศาต่อรอบ)
        self.n = 8
        self.step_index = 0
        self.servo_cam_angle = 90

        # debounce
        self.last_press = {}
        self.debounce_time = 0.25 

        # key hold
        self.last_key = None
        self.last_time = time.time()

        self.rate = 30.0

        self.get_logger().info("""
=========== TAO KEYBOARD TELEOP READY ===========
MOVE: w/s (forward/back), a/d (rotate)
LINEAR: ↑ (extend), ↓ (retract)
CAMERA: q/e (pan left/right)

TOGGLE:
 m : drill motor
 g : gripper
 j : servo dril
 l : servo switch180
 y : step rotate

AUTO SEQUENCE:
 z : START Auto Sequence
 p : PAUSE / RESUME Auto Sequence
 x : EMERGENCY STOP Auto (Linear=0, Motor=0)
 SPACE : STOP ALL

CTRL+C : EXIT
=================================================
""")
        self.run()

    # ================= Callbacks =================
    def limit_up_cb(self, msg):
        self.limit_up_state = msg.data
        
    def limit_down_cb(self, msg):
        self.limit_down_state = msg.data

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
            rclpy.spin_once(self, timeout_sec=0)
            key = get_key(period)

            if key:
                self.last_key = key
                self.last_time = time.time()

            if time.time() - self.last_time > 0.15:
                self.last_key = None

            k = self.last_key

            # ========= AUTO MODE TRIGGER (z) =========
            if k == 'z' and self.debounce("auto_z") and not self.auto_active:
                self.auto_active = True
                self.auto_paused = False
                self.auto_state = 1
                self.get_logger().info(f"AUTO MODE: STARTED (Target Step Index: {self.auto_step_index})")

            # ========= PAUSE / RESUME AUTO (p) =========
            if k == 'p' and self.debounce("auto_p"):
                if self.auto_active:
                    self.auto_paused = not self.auto_paused
                    if self.auto_paused:
                        self.pause_start_time = time.time()
                        self.get_logger().info("AUTO MODE: PAUSED (Press 'p' to Resume)")
                        self.pub_linear.publish(Int16(data=0))
                    else:
                        self.auto_timer += (time.time() - self.pause_start_time)
                        self.get_logger().info("AUTO MODE: RESUMED")

            # ========= EMERGENCY STOP / CANCEL (x หรือ Spacebar) =========
            if k == 'x' or k == ' ':
                if self.auto_active or self.auto_paused:
                    self.auto_active = False
                    self.auto_paused = False
                    self.auto_state = 0      
                    self.pub_motor.publish(Int16(data=0))
                    self.motor_on = False
                    self.pub_linear.publish(Int16(data=0))
                    self.get_logger().warn(f"!!! EMERGENCY STOP TRIGGERED (Key: {k}) !!! Motor & Linear STOPPED.")
                    
                self.pub_vel.publish(Twist())
                self.last_key = None

            if k == '\x03':
                break

            # ========= DRIVE =========
            tw = Twist()
            if k == 'w': tw.linear.x = self.linear_speed
            elif k == 's': tw.linear.x = -self.linear_speed
            if k == 'a': tw.angular.z = self.angular_speed
            elif k == 'd': tw.angular.z = -self.angular_speed
            self.pub_vel.publish(tw)

            # ========= LINEAR ACTUATOR & AUTO SEQUENCE =========
            lin = Int16()
            
            if self.auto_active:
                if self.auto_paused:
                    lin.data = 0
                else:
                    current_time = time.time()
                    
                    # --- Step 1: ยืด Linear ขึ้นจนกว่า Limit UP = True ---
                    if self.auto_state == 1:
                        if not self.limit_up_state:
                            lin.data = 1
                        else:
                            lin.data = 0
                            # เซ็ต Servo Dril, Gripper และสั่ง Step Motor ไปที่ 0 องศา
                            self.pub_servo_dril.publish(Int16(data=7))
                            self.pub_gripper.publish(Int16(data=25))
                            self.pub_step.publish(Int16(data=0))
                            
                            self.auto_timer = current_time 
                            self.auto_state = 2
                            self.get_logger().info("AUTO: Limit UP reached. Set Drill, Gripper & Step Motor(0). Waiting 2s...")
                            
                    # --- Step 2: รอ 2s แล้วสั่ง Servo SW ---
                    elif self.auto_state == 2:
                        if current_time - self.auto_timer >= 2.0:
                            self.pub_servo_sw.publish(Int16(data=220))
                            self.auto_timer = current_time
                            self.auto_state = 3
                            self.get_logger().info("AUTO: Set Servo switch. Waiting 2s...")

                    # --- Step 3: รอ 2s แล้วสั่ง Step Motor ตามรอบของ Auto ---
                    elif self.auto_state == 3:
                        if current_time - self.auto_timer >= 2.0:
                            angle = int(round((360 / self.n) * self.auto_step_index))
                            self.pub_step.publish(Int16(data=angle))
                            
                            self.auto_timer = current_time
                            self.auto_state = 4
                            self.get_logger().info(f"AUTO: Step Motor Moved to Auto Index {self.auto_step_index} (Angle: {angle}). Waiting 2s...")

                    # --- Step 4: รอ 2 วินาที ก่อนเปิดสว่าน ---
                    elif self.auto_state == 4:
                        if current_time - self.auto_timer >= 2.0:
                            self.auto_state = 5
                            
                    # --- Step 5: เปิด Motor Drill ---
                    elif self.auto_state == 5:
                        self.pub_motor.publish(Int16(data=100))
                        self.motor_on = True
                        self.auto_timer = current_time
                        self.auto_state = 6
                        self.get_logger().info("AUTO: Drill Motor ON. Waiting 2s...")

                    # --- Step 6: รอ 2 วินาที แล้วเริ่มดันลง ---
                    elif self.auto_state == 6:
                        if current_time - self.auto_timer >= 2.0:
                            self.auto_state = 7
                            self.get_logger().info("AUTO: Moving Linear DOWN...")

                    # --- Step 7: ดันลงจนเจอ Limit DOWN ---
                    elif self.auto_state == 7:
                        if not self.limit_down_state:
                            lin.data = -1
                        else:
                            lin.data = 0
                            self.auto_timer = current_time 
                            self.auto_state = 8
                            self.get_logger().info("AUTO: Limit DOWN reached. Waiting 1s...")
                            
                    # --- Step 8: รอ 1 วินาที ---
                    elif self.auto_state == 8:
                        if current_time - self.auto_timer >= 1.0:
                            self.auto_timer = current_time
                            self.auto_state = 9
                            self.get_logger().info("AUTO: Moving Linear UP for 7 seconds...")
                            
                    # --- Step 9: ดันขึ้น 7 วินาที ---
                    elif self.auto_state == 9:
                        if current_time - self.auto_timer < 7.0:
                            lin.data = 1
                        else:
                            lin.data = 0
                            self.auto_state = 10
                            self.get_logger().info("AUTO: Stopping drill...")
                            
                    # --- Step 10: ดับสว่าน ---
                    elif self.auto_state == 10:
                        self.pub_motor.publish(Int16(data=0))
                        self.motor_on = False
                        self.auto_timer = current_time
                        self.auto_state = 11
                        self.get_logger().info("AUTO: Motor STOP. Waiting 2s...")

                    # --- Step 11: รอ 2 วินาที สั่ง Servo Drill ---
                    elif self.auto_state == 11:
                        if current_time - self.auto_timer >= 2.0:
                            self.pub_servo_dril.publish(Int16(data=90))
                            self.auto_timer = current_time
                            self.auto_state = 12
                            self.get_logger().info("AUTO: Set Servo Drill 90. Waiting 2s...")

                    # --- Step 12: รอ 2 วินาที สั่ง Servo SW ---
                    elif self.auto_state == 12:
                        if current_time - self.auto_timer >= 2.0:
                            self.pub_servo_sw.publish(Int16(data=40))
                            self.auto_timer = current_time
                            self.auto_state = 13
                            self.get_logger().info("AUTO: Set Servo SW 40. Waiting 2s...")

                    # --- Step 13: รอ 2 วินาที เริ่มดันลง ---
                    elif self.auto_state == 13:
                        if current_time - self.auto_timer >= 2.0:
                            self.auto_timer = current_time
                            self.auto_state = 14
                            self.get_logger().info("AUTO: Moving Linear DOWN for 5 seconds...")

                    # --- Step 14: ดันลง 5 วินาที ---
                    elif self.auto_state == 14:
                        if current_time - self.auto_timer < 5.0:
                            lin.data = -1
                        else:
                            lin.data = 0
                            self.pub_gripper.publish(Int16(data=60))
                            self.gripper_on = True
                            self.auto_timer = current_time 
                            self.auto_state = 15
                            self.get_logger().info("AUTO: Gripper OPENED 60. Moving Linear DOWN for another 2s...")

                    # --- Step 15: สั่ง Gripper ปิดครั้งที่ 1 แล้วดันลงอีก 2 วินาที ---
                    elif self.auto_state == 15:
                        if current_time - self.auto_timer < 2.0:
                            lin.data = -1
                        else:
                            lin.data = 0
                            self.pub_gripper.publish(Int16(data=25)) # ปิดครั้งที่ 1
                            self.gripper_on = False
                            self.auto_timer = current_time
                            self.auto_state = 16
                            self.get_logger().info("AUTO: Gripper CLOSED 25 (Loop 1). Waiting 1.5s...")

                    # --- Step 16: เปิดครั้งที่ 1 ---
                    elif self.auto_state == 16:
                        if current_time - self.auto_timer >= 1.5:
                            self.pub_gripper.publish(Int16(data=60)) # เปิดครั้งที่ 1
                            self.gripper_on = True
                            self.auto_timer = current_time
                            self.auto_state = 17
                            self.get_logger().info("AUTO: Gripper OPENED 60 (Loop 1). Waiting 1.5s...")

                    # --- Step 17: ปิดครั้งที่ 2 ---
                    elif self.auto_state == 17:
                        if current_time - self.auto_timer >= 1.5:
                            self.pub_gripper.publish(Int16(data=25)) # ปิดครั้งที่ 2
                            self.gripper_on = False
                            self.auto_timer = current_time
                            self.auto_state = 18
                            self.get_logger().info("AUTO: Gripper CLOSED 25 (Loop 2). Waiting 1.5s...")

                    # --- Step 18: เปิดครั้งที่ 2 ---
                    elif self.auto_state == 18:
                        if current_time - self.auto_timer >= 1.5:
                            self.pub_gripper.publish(Int16(data=60)) # เปิดครั้งที่ 2
                            self.gripper_on = True
                            self.auto_timer = current_time
                            self.auto_state = 19
                            self.get_logger().info("AUTO: Gripper OPENED 60 (Loop 2). Waiting 1.5s...")

                    # --- Step 19: ปิดครั้งที่ 3 ---
                    elif self.auto_state == 19:
                        if current_time - self.auto_timer >= 1.5:
                            self.pub_gripper.publish(Int16(data=25)) # ปิดครั้งที่ 3
                            self.gripper_on = False
                            self.auto_timer = current_time
                            self.auto_state = 20
                            self.get_logger().info("AUTO: Gripper CLOSED 25 (Loop 3). Waiting 1.5s...")

                    # --- Step 20: เปิดครั้งที่ 3 ---
                    elif self.auto_state == 20:
                        if current_time - self.auto_timer >= 1.5:
                            self.pub_gripper.publish(Int16(data=60)) # เปิดครั้งที่ 3
                            self.gripper_on = True
                            self.auto_timer = current_time
                            self.auto_state = 21
                            self.get_logger().info("AUTO: Gripper OPENED 60 (Loop 3). Executing FINAL STEP...")

                    # --- Step 21: FINAL STEP ดันขึ้นจนเจอ Limit UP ---
                    elif self.auto_state == 21:
                        if not self.limit_up_state:
                            lin.data = 1
                        else:
                            lin.data = 0
                            # เมื่อสุดลิมิต ให้สั่ง Servo Dril และทำการ "ปิด" Gripper
                            self.pub_servo_dril.publish(Int16(data=7))
                            self.pub_gripper.publish(Int16(data=25)) # สั่งปิด Gripper ตามที่ขอ
                            self.gripper_on = False
                            
                            self.auto_timer = current_time
                            self.auto_state = 22
                            self.get_logger().info("AUTO: Limit UP reached. Set Servo Drill 7 & Gripper CLOSED 25. Waiting 2s...")
                            
                    # --- Step 22: รอ 2 วินาที สั่ง Servo SW ---
                    elif self.auto_state == 22:
                        if current_time - self.auto_timer >= 2.0:
                            self.pub_servo_sw.publish(Int16(data=220))
                            self.auto_timer = current_time
                            self.auto_state = 23
                            self.get_logger().info("AUTO: Set Servo switch 180 (220). Waiting 2s...")

                    # --- Step 23: จบการทำงาน Auto และนับรอบเพิ่ม ---
                    elif self.auto_state == 23:
                        if current_time - self.auto_timer >= 2.0:
                            self.auto_active = False 
                            self.auto_state = 0      
                            
                            # เพิ่มตัวนับรอบ (บวกทีละ 1)
                            self.auto_step_index += 1
                            
                            # เช็คว่าถ้าเกินจำนวน n (เช่น 8) ให้วนกลับไปเริ่มที่ 1 ใหม่
                            if self.auto_step_index > self.n:
                                self.auto_step_index = 1
                                
                            self.get_logger().info(f"AUTO SEQUENCE: FULLY COMPLETED. Next Auto Step Index will be {self.auto_step_index}")

            else:
                # การควบคุม Manual ปกติ
                if k == '\x1b[A':      # Arrow UP
                    lin.data = 1
                elif k == '\x1b[B':    # Arrow DOWN
                    lin.data = -1
                else:
                    lin.data = 0

            self.pub_linear.publish(lin)

            # ========= CAMERA SERVO (q/e) =========
            if k == 'q' and self.debounce("cam_q"):
                self.servo_cam_angle = max(0, self.servo_cam_angle - 5)
                self.pub_servo_cam.publish(Int16(data=self.servo_cam_angle))
            elif k == 'e' and self.debounce("cam_e"):
                self.servo_cam_angle = min(180, self.servo_cam_angle + 5)
                self.pub_servo_cam.publish(Int16(data=self.servo_cam_angle))

            # ========= OTHERS =========
            if k == 'm' and self.debounce("motor"):
                self.motor_on = not self.motor_on
                val = 100 if self.motor_on else 0
                self.pub_motor.publish(Int16(data=val))
            if k == 'g' and self.debounce("gripper"):
                self.gripper_on = not self.gripper_on
                val = 60 if self.gripper_on else 25
                self.pub_gripper.publish(Int16(data=val))
            if k == 'j' and self.debounce("servo_dril"):
                self.servo_dril_on = not self.servo_dril_on
                val = 90 if self.servo_dril_on else 7
                self.pub_servo_dril.publish(Int16(data=val))
            if k == 'l' and self.debounce("servo_sw"):
                self.servo_sw_on = not self.servo_sw_on
                val = 220 if self.servo_sw_on else 40
                self.pub_servo_sw.publish(Int16(data=val))
            if k == 'y' and self.debounce("step"):
                angle = int(round((360 / self.n) * self.step_index))
                self.pub_step.publish(Int16(data=angle))
                self.step_index = (self.step_index + 1) % self.n

# ================= MAIN =================
def main():
    rclpy.init()
    TaoKeyboardJoy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()