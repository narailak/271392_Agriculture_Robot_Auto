import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int16, Bool
from std_srvs.srv import Trigger
import time

class AutoPlanterServiceNode(Node):
    def __init__(self):
        super().__init__('auto_planter_node')

        # ===== Publishers =====
        self.pub_motor = self.create_publisher(Int16, '/tao/cmd_motor_dril', 10)
        self.pub_gripper = self.create_publisher(Int16, '/tao/cmd_gripper', 10)
        self.pub_linear = self.create_publisher(Int16, '/tao/cmd_linear', 10)
        self.pub_servo_dril = self.create_publisher(Int16, '/tao/cmd_servo_dril', 10)
        self.pub_servo_sw = self.create_publisher(Int16, '/tao/cmd_servo_switch180', 10)
        self.pub_step = self.create_publisher(Int16, '/tao/cmd_step_load', 10)
        
        self.pub_done = self.create_publisher(Bool, '/tao/planting_done', 10)

        # ===== Subscribers =====
        self.sub_limit_up = self.create_subscription(Bool, '/tao/fb/limit_up', self.limit_up_cb, qos_profile_sensor_data)
        self.sub_limit_down = self.create_subscription(Bool, '/tao/fb/limit_down', self.limit_down_cb, qos_profile_sensor_data)

        # ===== Service Server =====
        self.srv_start = self.create_service(Trigger, '/tao/start_planting', self.start_planting_cb)

        # ===== State Variables =====
        self.auto_active = False
        self.auto_state = 0         
        self.auto_timer = 0.0       
        
        # 🌟 ตัวนับช่องของโม่ (เริ่มที่ช่อง 1 สูงสุด 8)
        self.auto_step_index = 1    
        self.n = 8                  
        
        self.limit_up_state = False
        self.limit_down_state = False

        self.timer = self.create_timer(0.05, self.planter_loop)
        self.get_logger().info("=== Auto Planter Service Ready ===")

    def limit_up_cb(self, msg):
        self.limit_up_state = msg.data
        
    def limit_down_cb(self, msg):
        self.limit_down_state = msg.data

    def start_planting_cb(self, request, response):
        if self.auto_active:
            response.success = False
            response.message = "กำลังปลูกอยู่! สั่งซ้ำไม่ได้"
        else:
            self.auto_active = True
            self.auto_state = 1
            response.success = True
            response.message = f"เริ่มการปลูก! โม่ช่องที่ {self.auto_step_index}"
            self.get_logger().info(response.message)
        return response

    def planter_loop(self):
        if not self.auto_active:
            return

        current_time = time.time()
        lin = Int16()
        
        # --- Step 1: ยืด Linear ขึ้นจนกว่า Limit UP = True ---
        if self.auto_state == 1:
            if not self.limit_up_state:
                lin.data = 1
            else:
                lin.data = 0
                self.pub_servo_dril.publish(Int16(data=7))
                self.pub_gripper.publish(Int16(data=20))
                
                # 🌟 ส่ง 0 องศา "เฉพาะตอนที่มันคือช่องที่ 1" เท่านั้น! 🌟
                if self.auto_step_index == 1:
                    self.pub_step.publish(Int16(data=0))
                    self.get_logger().info("Step 1: โม่ช่องที่ 1 -> ส่ง 0 องศาเพื่อตั้งศูนย์")
                    
                self.auto_timer = current_time 
                self.auto_state = 2
                
        # --- Step 2: รอ 2s แล้วสั่ง Servo SW ---
        elif self.auto_state == 2:
            if current_time - self.auto_timer >= 2.0:
                self.pub_servo_sw.publish(Int16(data=220))
                self.auto_timer = current_time
                self.auto_state = 3

        # --- Step 3: หมุน Step Motor โม่ต้นกล้า ---
        elif self.auto_state == 3:
            if current_time - self.auto_timer >= 2.0:
                # 🌟 คำนวณองศาจาก Index ปัจจุบัน (1=45, 2=90, 3=135 ... 8=360)
                target_angle = int(round((360 / self.n) * self.auto_step_index))
                
                self.pub_step.publish(Int16(data=target_angle))
                
                self.auto_timer = current_time
                self.auto_state = 4
                self.get_logger().info(f"Step 3: หมุนโม่ต้นกล้าไปที่ {target_angle} องศา. Wait 2s.")

        # --- Step 4: รอ 2s ก่อนเปิดสว่าน ---
        elif self.auto_state == 4:
            if current_time - self.auto_timer >= 2.0:
                self.auto_state = 5
                
        # --- Step 5: เปิดสว่าน ---
        elif self.auto_state == 5:
            self.pub_motor.publish(Int16(data=100))
            self.auto_timer = current_time
            self.auto_state = 6

        # --- Step 6: รอ 2s ---
        elif self.auto_state == 6:
            if current_time - self.auto_timer >= 2.0:
                self.auto_state = 7
                
        # --- Step 7: ดันลงจนเจอ Limit DOWN ---
        elif self.auto_state == 7:
            if not self.limit_down_state:
                lin.data = -1
            else:
                lin.data = 0
                self.auto_timer = current_time 
                self.auto_state = 8
                
        # --- Step 8: รอ 1s ---
        elif self.auto_state == 8:
            if current_time - self.auto_timer >= 1.0:
                self.auto_timer = current_time
                self.auto_state = 9
                
        # --- Step 9: ดันขึ้น 10 วินาที ---
        elif self.auto_state == 9:
            if current_time - self.auto_timer < 10.0:
                lin.data = 1
            else:
                lin.data = 0
                self.auto_state = 10
                
        # --- Step 10: ดับสว่าน ---
        elif self.auto_state == 10:
            self.pub_motor.publish(Int16(data=0))
            self.auto_timer = current_time
            self.auto_state = 11

        # --- Step 11: Servo Drill ---
        elif self.auto_state == 11:
            if current_time - self.auto_timer >= 2.0:
                self.pub_servo_dril.publish(Int16(data=90))
                self.auto_timer = current_time
                self.auto_state = 12

        # --- Step 12: Servo SW ---
        elif self.auto_state == 12:
            if current_time - self.auto_timer >= 2.0:
                self.pub_servo_sw.publish(Int16(data=40))
                self.auto_timer = current_time
                self.auto_state = 13

        # --- Step 13-14: ดันลง 5 วินาที ---
        elif self.auto_state == 13:
            if current_time - self.auto_timer >= 2.0:
                self.auto_timer = current_time
                self.auto_state = 14
        elif self.auto_state == 14:
            if current_time - self.auto_timer < .0:
                lin.data = -1
            else:
                lin.data = 0
                self.pub_gripper.publish(Int16(data=60))
                self.auto_timer = current_time 
                self.auto_state = 15

        # --- Step 15-20: บีบปล่อย Gripper 3 รอบ ---
        elif self.auto_state == 15: 
            if current_time - self.auto_timer < 2.0:
                lin.data = -1
            else:
                lin.data = 0
                self.pub_gripper.publish(Int16(data=20)) 
                self.auto_timer = current_time
                self.auto_state = 16
        elif self.auto_state == 16:
            if current_time - self.auto_timer >= 1.5:
                self.pub_gripper.publish(Int16(data=60)) 
                self.auto_timer = current_time
                self.auto_state = 17
        elif self.auto_state == 17:
            if current_time - self.auto_timer >= 1.5:
                self.pub_gripper.publish(Int16(data=20)) 
                self.auto_timer = current_time
                self.auto_state = 18
        elif self.auto_state == 18:
            if current_time - self.auto_timer >= 1.5:
                self.pub_gripper.publish(Int16(data=60)) 
                self.auto_timer = current_time
                self.auto_state = 19
        elif self.auto_state == 19:
            if current_time - self.auto_timer >= 1.5:
                self.pub_gripper.publish(Int16(data=20)) 
                self.auto_timer = current_time
                self.auto_state = 20
        elif self.auto_state == 20:
            if current_time - self.auto_timer >= 1.5:
                self.pub_gripper.publish(Int16(data=60)) 
                self.auto_timer = current_time
                self.auto_state = 21

        # --- Step 21: ดันขึ้นสุด Limit UP ---
        elif self.auto_state == 21:
            if not self.limit_up_state:
                lin.data = 1
            else:
                lin.data = 0
                self.pub_servo_dril.publish(Int16(data=7))
                self.pub_gripper.publish(Int16(data=20)) 
                self.auto_timer = current_time
                self.auto_state = 22
                
        # --- Step 22: Servo SW ---
        elif self.auto_state == 22:
            if current_time - self.auto_timer >= 2.0:
                self.pub_servo_sw.publish(Int16(data=220))
                self.auto_timer = current_time
                self.auto_state = 23

        # --- Step 23: จบงาน ส่งสัญญาณแจ้งเตือน State Machine ---
        elif self.auto_state == 23:
            if current_time - self.auto_timer >= 2.0:
                self.auto_active = False 
                self.auto_state = 0      
                
                # 🌟 บวกช่องโม่เพิ่ม 1 ช่อง
                self.auto_step_index += 1
                
                # 🌟 ถ้าหมุนครบ 8 ช่องแล้ว ให้วนกลับไปช่องที่ 1 ใหม่
                if self.auto_step_index > self.n:
                    self.auto_step_index = 1
                    
                self.get_logger().info("=== ปลูกเสร็จสิ้น! ส่งสัญญาณให้ State Machine ===")
                self.pub_done.publish(Bool(data=True))

        self.pub_linear.publish(lin)

def main(args=None):
    rclpy.init(args=args)
    node = AutoPlanterServiceNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()