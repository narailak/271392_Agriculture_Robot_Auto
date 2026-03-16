import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float32, Bool
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from agri_interfaces.srv import ReadTag
import os
import csv
import threading

class AgriStateMachineNode(Node):
    def __init__(self):
        super().__init__('agri_state_machine_node')
        
        self.pub_servo_cam = self.create_publisher(Int16, '/tao/cmd_servo_cam', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/tao/cmd_vel', 10)
        self.pub_enable_measure = self.create_publisher(Bool, '/tao/enable_measurement', 10)
        self.pub_servo_dril = self.create_publisher(Int16, '/tao/cmd_servo_dril', 10)
        
        self.sub_distance = self.create_subscription(Float32, '/tao/distance', self.distance_callback, 10)
        self.sub_planting_done = self.create_subscription(Bool, '/tao/planting_done', self.planting_done_callback, 10)
        self.sub_cabbage_centered = self.create_subscription(Bool, '/tao/cabbage_centered', self.cabbage_centered_callback, 10)
        
        self.is_planting_done = False
        self.is_cabbage_centered = False 
        
        self.tag_client = self.create_client(ReadTag, 'read_apriltag')
        self.pid_save_client = self.create_client(Trigger, '/tao/save_pid_plot') 
        self.planter_client = self.create_client(Trigger, '/tao/start_planting')
        self.cabbage_log_client = self.create_client(Trigger, '/tao/log_cabbage')
        
        # ==========================================
        # STATE DEFINITIONS
        # ==========================================
        self.STATE_WAIT_FOR_START = 0
        self.STATE_WAIT_FOR_NEXT = 100  # 🌟 สถานะใหม่สำหรับรอผู้ใช้กด 'n' ยืนยันหลังจบ Step 1
        
        self.STATE_1_PUB_SERVO = 1
        self.STATE_WAIT_SERVO = 2
        self.STATE_2_CALL_APRILTAG = 3
        self.STATE_WAIT_APRILTAG = 4
        self.STATE_3_READ_LOG = 5
        self.STATE_4_MOVING = 6         
        self.STATE_6_CALL_PLANTER = 9   
        self.STATE_WAIT_PLANTER = 10    
        self.STATE_POST_PLANT_DELAY = 11
        
        self.STATE_PRE_REVERSE = 19
        self.STATE_MOVING_BACKWARD = 20
        
        self.STATE_PREPARE_MEASUREMENT = 12
        self.STATE_MOVE_UNTIL_CABBAGE = 13
        
        self.STATE_STOP_STABILIZE = 16
        self.STATE_CALL_LOG_SERVICE = 17
        self.STATE_WAIT_LOG_SERVICE = 18
        
        self.STATE_SAVE_PID_FINAL = 14
        self.STATE_WAIT_PID_SAVE_FINAL = 15
        
        self.current_state = self.STATE_WAIT_FOR_START
        
        # สัญญาณสั่งงานจาก Keyboard
        self.start_requested = False
        self.next_requested = False     # 🌟 สัญญาณไป Step 2
        self.reset_requested = False    # 🌟 สัญญาณฉุกเฉิน / Reset
        
        self.start_time = None
        self.apriltag_start_time = None 
        self.delay_start_time = None 
        self.future = None
        
        self.current_distance = 0.0     
        self.target_distance = 0.0      
        self.plant_dist_cm = 0.0        
        self.planted_count = 0          
        self.last_cabbage_dist = 0.0
        
        self.start_reverse_dist = 0.0

        # ระยะชดเชยของหัวเจาะที่อยู่ในตัวหุ่น
        self.drill_offset_cm = 13.5 
        
        self.timer = self.create_timer(0.1, self.state_machine_loop)
        self.input_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.input_thread.start()

        self.get_logger().info("=== สมองกลพร้อมทำงาน ===")
        self.get_logger().info("พิมพ์ 's' เพื่อเริ่ม (ถ้านี่คือจุดเริ่มต้นที่จอดหน้าร่อง)")
        self.get_logger().info("พิมพ์ 'n' เพื่อลุยต่อ (ถ้าวิ่งเข้าป้ายเสร็จแล้ว / Mission Complete)")
        self.get_logger().info("พิมพ์ 'r' เพื่อหยุดฉุกเฉินและรีเซ็ตระบบ")

    def distance_callback(self, msg):
        self.current_distance = msg.data

    def planting_done_callback(self, msg):
        if msg.data == True:
            self.is_planting_done = True

    def cabbage_centered_callback(self, msg):
        if msg.data == True:
            self.is_cabbage_centered = True

    def keyboard_listener(self):
        while rclpy.ok():
            try:
                cmd = input()
                cmd = cmd.strip().lower()
                
                if cmd == 's':
                    self.start_requested = True
                elif cmd == 'n':
                    self.next_requested = True
                elif cmd == 'r':
                    self.reset_requested = True
                    
            except EOFError: break

    def stop_robot(self):
        """ส่งคำสั่งหยุดรถทันที"""
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = 0.0
        self.pub_cmd_vel.publish(cmd_msg)

    def reset_state_machine(self):
        """หยุดรถและล้างค่ากลับไปรอจุดเริ่มต้น"""
        self.stop_robot()
        self.pub_enable_measure.publish(Bool(data=False))
        self.current_state = self.STATE_WAIT_FOR_START
        self.start_requested = False
        self.next_requested = False
        self.reset_requested = False
        self.planted_count = 0
        self.get_logger().warn("⚠️ รีเซ็ตระบบ! รถหยุดนิ่งและกลับไปรอคำสั่ง 's' หรือ 'n'")

    def state_machine_loop(self):
        # 🌟 ลำดับความสำคัญสูงสุด: เช็คปุ่ม Reset (r) ก่อนเลย
        if self.reset_requested:
            self.reset_state_machine()
            return

        # ========================================================
        # State Machine Logic
        # ========================================================
        if self.current_state == self.STATE_WAIT_FOR_START:
            # ผู้ใช้กด 's' (เริ่มจากศูนย์) หรือ 'n' (ข้ามมา Step 2 เลยหลังจบ Mission 1)
            if self.start_requested or self.next_requested:
                self.start_requested = False
                self.next_requested = False
                self.planted_count = 0 
                self.get_logger().info("▶️ เริ่มเข้าสู่กระบวนการเตรียมอ่านป้าย AprilTag")
                self.current_state = self.STATE_1_PUB_SERVO

        elif self.current_state == self.STATE_1_PUB_SERVO:
            self.start_time = self.get_clock().now()
            self.current_state = self.STATE_WAIT_SERVO

        elif self.current_state == self.STATE_WAIT_SERVO:
            self.pub_servo_cam.publish(Int16(data=160))
            if (self.get_clock().now() - self.start_time).nanoseconds / 1e9 >= 1.5:
                self.current_state = self.STATE_2_CALL_APRILTAG

        elif self.current_state == self.STATE_2_CALL_APRILTAG:
            if not self.tag_client.service_is_ready(): 
                self.get_logger().info("รอ Service AprilTag...")
                return
            self.apriltag_start_time = self.get_clock().now()
            self.future = self.tag_client.call_async(ReadTag.Request())
            self.current_state = self.STATE_WAIT_APRILTAG

        elif self.current_state == self.STATE_WAIT_APRILTAG:
            if self.future.done():
                if self.future.result().success:
                    self.get_logger().info("✅ อ่านป้ายสำเร็จ! กำลังดึงข้อมูลจากไฟล์")
                    self.current_state = self.STATE_3_READ_LOG
                else:
                    if (self.get_clock().now() - self.apriltag_start_time).nanoseconds / 1e9 < 10.0:
                        self.future = self.tag_client.call_async(ReadTag.Request())
                    else:
                        self.get_logger().error("❌ อ่านป้ายไม่สำเร็จเกิน 10 วินาที กลับไปรอคำสั่งใหม่")
                        self.current_state = self.STATE_WAIT_FOR_START 

        elif self.current_state == self.STATE_3_READ_LOG:
            csv_path = '/home/aorus-ubun/CMU/P.3/P.3-T.2/271392_Agriculture_Robot_Auto/agri_log/farming_parameters.csv'
            try:
                with open(csv_path, mode='r') as f:
                    lines = list(csv.reader(f))
                    if len(lines) > 1:
                        self.plant_dist_cm = float(lines[-1][2])
                        # บวกระยะ offset 13.5 cm สำหรับต้นแรก
                        self.target_distance = self.current_distance + self.plant_dist_cm + self.drill_offset_cm
                        self.get_logger().info(f"🌱 เริ่มปลูกต้นแรก ระยะเป้าหมาย: {self.target_distance:.2f} (รวม offset {self.drill_offset_cm} cm)")
                        self.current_state = self.STATE_4_MOVING
                    else: 
                        self.current_state = self.STATE_WAIT_FOR_START
            except FileNotFoundError:
                self.get_logger().error(f"หาไฟล์ไม่เจอที่: {csv_path}")
                self.current_state = self.STATE_WAIT_FOR_START
            except Exception as e: 
                self.get_logger().error(f"เกิดข้อผิดพลาดในการอ่านไฟล์: {e}")
                self.current_state = self.STATE_WAIT_FOR_START

        elif self.current_state == self.STATE_4_MOVING:
            cmd_msg = Twist()
            if self.current_distance < self.target_distance:
                cmd_msg.linear.x = 0.4
                self.pub_cmd_vel.publish(cmd_msg)
            else:
                cmd_msg.linear.x = 0.0
                self.pub_cmd_vel.publish(cmd_msg)
                self.current_state = self.STATE_6_CALL_PLANTER

        elif self.current_state == self.STATE_6_CALL_PLANTER:
            if not self.planter_client.service_is_ready(): return
            self.is_planting_done = False 
            self.future = self.planter_client.call_async(Trigger.Request())
            self.current_state = self.STATE_WAIT_PLANTER

        elif self.current_state == self.STATE_WAIT_PLANTER:
            if self.is_planting_done:
                self.is_planting_done = False
                self.planted_count += 1 

                if self.planted_count < 2:
                    # สำหรับต้นที่สอง เดินแค่ระยะปลูกปกติ (ไม่บวก offset แล้ว)
                    self.target_distance = self.current_distance + self.plant_dist_cm
                    self.get_logger().info(f"🌱 ปลูกต้นที่ 2 ระยะเป้าหมายถัดไป: {self.target_distance:.2f}")
                    self.current_state = self.STATE_4_MOVING
                else:
                    self.delay_start_time = self.get_clock().now()
                    self.current_state = self.STATE_POST_PLANT_DELAY

        elif self.current_state == self.STATE_POST_PLANT_DELAY:
            if (self.get_clock().now() - self.delay_start_time).nanoseconds / 1e9 >= 1.5:
                self.current_state = self.STATE_PRE_REVERSE

        elif self.current_state == self.STATE_PRE_REVERSE:
            self.get_logger().info("ยกสว่าน (Servo Drill 90) และกำลังถอยหลัง 15 cm...")
            self.pub_servo_dril.publish(Int16(data=90))
            self.start_reverse_dist = self.current_distance
            self.current_state = self.STATE_MOVING_BACKWARD

        elif self.current_state == self.STATE_MOVING_BACKWARD:
            cmd_msg = Twist()
            walked_dist = abs(self.current_distance - self.start_reverse_dist)
            
            if walked_dist < 15.0:
                cmd_msg.linear.x = -0.3
                self.pub_cmd_vel.publish(cmd_msg)
            else:
                cmd_msg.linear.x = 0.0
                self.pub_cmd_vel.publish(cmd_msg)
                self.get_logger().info("ถอยหลังเสร็จสิ้น! เตรียมตัวเปิดกล้องเดินหน้าตรวจกะหล่ำ...")
                self.current_state = self.STATE_PREPARE_MEASUREMENT

        elif self.current_state == self.STATE_PREPARE_MEASUREMENT:
            self.last_cabbage_dist = self.current_distance 
            self.is_cabbage_centered = False
            self.pub_enable_measure.publish(Bool(data=True))
            self.current_state = self.STATE_MOVE_UNTIL_CABBAGE

        elif self.current_state == self.STATE_MOVE_UNTIL_CABBAGE:
            cmd_msg = Twist()
            walked_dist_empty = abs(self.current_distance - self.last_cabbage_dist)
            limit_dist = max(self.plant_dist_cm * 1.5, 80.0) 
            
            if self.is_cabbage_centered:
                cmd_msg.linear.x = 0.0
                self.pub_cmd_vel.publish(cmd_msg)
                self.get_logger().info("🎯 เป้าหมายเข้ากลางจอ! เบรกรถและรอ 1.5 วินาทีให้กล้องนิ่ง...")
                self.delay_start_time = self.get_clock().now()
                self.current_state = self.STATE_STOP_STABILIZE
                
            elif walked_dist_empty < limit_dist:
                cmd_msg.linear.x = 0.4
                self.pub_cmd_vel.publish(cmd_msg)
            else:
                cmd_msg.linear.x = 0.0
                self.pub_cmd_vel.publish(cmd_msg)
                self.get_logger().info(f"🚫 ไม่เจอกะหล่ำ ถือว่าจบแปลง สั่งเบรกและบันทึกข้อมูล PID รวบยอด!")
                self.pub_enable_measure.publish(Bool(data=False))
                self.current_state = self.STATE_SAVE_PID_FINAL

        elif self.current_state == self.STATE_STOP_STABILIZE:
            cmd_msg = Twist()
            cmd_msg.linear.x = 0.0
            self.pub_cmd_vel.publish(cmd_msg) 
            
            if (self.get_clock().now() - self.delay_start_time).nanoseconds / 1e9 >= 1.5:
                self.current_state = self.STATE_CALL_LOG_SERVICE

        elif self.current_state == self.STATE_CALL_LOG_SERVICE:
            if not self.cabbage_log_client.service_is_ready(): return
            self.get_logger().info("📸 สั่งกล้องแชะภาพและเซฟข้อมูล...")
            self.future = self.cabbage_log_client.call_async(Trigger.Request())
            self.current_state = self.STATE_WAIT_LOG_SERVICE

        elif self.current_state == self.STATE_WAIT_LOG_SERVICE:
            if self.future.done():
                self.get_logger().info("✅ บันทึกเสร็จแล้ว! เดินหน้าหากะหล่ำหัวถัดไป...")
                self.is_cabbage_centered = False
                self.last_cabbage_dist = self.current_distance
                self.current_state = self.STATE_MOVE_UNTIL_CABBAGE

        elif self.current_state == self.STATE_SAVE_PID_FINAL:
            if not self.pid_save_client.service_is_ready(): 
                self.get_logger().warning("รอ Service PID...", throttle_duration_sec=2.0)
                return
            self.future = self.pid_save_client.call_async(Trigger.Request())
            self.current_state = self.STATE_WAIT_PID_SAVE_FINAL

        elif self.current_state == self.STATE_WAIT_PID_SAVE_FINAL:
            if self.future.done():
                self.current_state = self.STATE_WAIT_FOR_START
                self.get_logger().info("\n====================================")
                self.get_logger().info("🏁 จบการทำงาน 1 แปลงสมบูรณ์! พร้อมสำหรับป้ายถัดไป!")
                self.get_logger().info("พิมพ์ 's' หรือ 'n' เพื่อลุยต่อ หรือ 'r' เพื่อรีเซ็ต")

def main(args=None):
    rclpy.init(args=args)
    node = AgriStateMachineNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()