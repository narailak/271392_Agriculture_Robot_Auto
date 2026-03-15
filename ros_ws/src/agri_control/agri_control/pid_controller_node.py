#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from std_srvs.srv import Trigger  
import time
import csv
import os
import threading
from datetime import datetime

class PidControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # ================= 1. Scaling & Physical Parameters =================
        self.target_multiplier = 6    
        self.angular_multiplier = 10.0   
        self.max_acceleration = 1.0     
        self.wheel_separation = 0.3     
        self.max_velocity = 7.8       
        self.min_pwm = 15               
        self.kf = (255 - self.min_pwm) / self.max_velocity 
        
        # PID Gains
        self.kp = 30                
        self.ki = 10                   
        self.kd = 0.1                
        
        # Low-Pass Filter
        self.lpf_alpha = 0.6            
        self.pwm_limits = 255

        # ================= 2. Grouped State Variables =================
        self.target_vels = [0.0, 0.0]   
        self.ramped_vels = [0.0, 0.0]   
        self.current_vels = [0.0, 0.0]  
        self.filtered_vels = [0.0, 0.0]
        self.integral_errs = [0.0, 0.0]
        self.previous_errs = [0.0, 0.0]
        
        # ================= 3. Logging & Memory Setup =================
        self.all_time_history = []
        self.all_target_history = [[] for _ in range(2)]
        self.all_current_history = [[] for _ in range(2)]
        
        self.start_time = time.time()
        self.data_lock = threading.Lock()
        
        # 🌟 เตรียมโฟลเดอร์สำหรับเซฟ CSV (อัปเดต Path ให้รวมกันที่ agri_log) 🌟
        self.log_dir = "/home/aorus-ubun/CMU/P.3/P.3-T.2/271392_Agriculture_Robot_Auto/agri_log"
        os.makedirs(self.log_dir, exist_ok=True)

        # ================= 4. ROS Setup =================
        self.sub_cmd_vel = self.create_subscription(Twist, '/tao/cmd_vel', self.cmd_vel_callback, 10)
        self.sub_encoder = self.create_subscription(Float32MultiArray, '/tao/encoder_feedback', self.encoder_callback, 10)
        self.pub_pwm = self.create_publisher(Int32MultiArray, '/tao/motor_pwm', 10)
        
        # 🌟 สร้าง Service Server รอรับคำสั่งเซฟ CSV จาก State Machine 🌟
        self.srv_save_plot = self.create_service(Trigger, '/tao/save_pid_plot', self.save_plot_callback)

        self.last_time = time.time()
        self.timer = self.create_timer(0.05, self.control_loop) 
        self.get_logger().info('✅ PID Controller Node Started. พร้อมเก็บข้อมูลและเซฟลงโฟลเดอร์ agri_log แล้ว!')

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x * self.target_multiplier
        wz = msg.angular.z * self.angular_multiplier
        v_l = vx - (wz * self.wheel_separation / 2.0)
        v_r = vx + (wz * self.wheel_separation / 2.0)
        self.target_vels = [v_l, v_r]

    def encoder_callback(self, msg):
        if len(msg.data) >= 4:
            raw_left = (msg.data[1] + msg.data[3]) / 2.0   
            raw_right = (msg.data[0] + msg.data[2]) / 2.0  
            
            raw_data = [raw_left, raw_right]
            for i in range(2):
                self.filtered_vels[i] = (self.lpf_alpha * self.filtered_vels[i]) + \
                                        ((1.0 - self.lpf_alpha) * raw_data[i])
            self.current_vels = list(self.filtered_vels)

    def control_loop(self):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0.0: return
            
        pwm_results = [0, 0] 
        rel_time = current_time - self.start_time

        for i in range(2):
            # 1. Ramp Logic
            if abs(self.target_vels[i]) < 0.001:
                self.ramped_vels[i] = 0.0  
            else:
                diff = self.target_vels[i] - self.ramped_vels[i]
                max_step = self.max_acceleration * dt
                if abs(diff) > max_step:
                    self.ramped_vels[i] += (1 if diff > 0 else -1) * max_step
                else:
                    self.ramped_vels[i] = self.target_vels[i]

            # 2. PID Calculation
            target = self.ramped_vels[i]
            current = self.current_vels[i]
            error = target - current
            
            if abs(target) > 0.001:
                ff_term = target * self.kf
                sign = 1 if target > 0 else -1
                base_pwm = ff_term + (sign * self.min_pwm)
            else:
                base_pwm = 0.0
                self.integral_errs[i] = 0.0

            p_term = self.kp * error
            self.integral_errs[i] = max(min(self.integral_errs[i] + (error * dt), 30.0), -30.0)
            i_term = self.ki * self.integral_errs[i]
            d_term = self.kd * (error - self.previous_errs[i]) / dt
            
            output = int(base_pwm + p_term + i_term + d_term)
            pwm_results[i] = max(min(output, self.pwm_limits), -self.pwm_limits)
            self.previous_errs[i] = error

        # 3. ส่ง PWM ออกไป
        final_pwm_msg = Int32MultiArray()
        final_pwm_msg.data = [
            pwm_results[0], pwm_results[1], 
            pwm_results[0], pwm_results[1]
        ]
        self.pub_pwm.publish(final_pwm_msg)
        
        # 4. เก็บประวัติไว้ใน Memory รอคนมาสั่งเซฟ (เก็บเฉพาะตอนรถขยับ)
        with self.data_lock:
            if abs(self.target_vels[0]) > 0.01 or abs(self.target_vels[1]) > 0.01 or abs(self.current_vels[0]) > 0.01:
                self.all_time_history.append(rel_time)
                for i in range(2):
                    self.all_target_history[i].append(self.ramped_vels[i])
                    self.all_current_history[i].append(self.current_vels[i])

        self.last_time = current_time

    # ==========================================
    # 🌟 Service Callback: ทำงานเมื่อ State Machine สั่งให้เซฟ 🌟
    # ==========================================
    def save_plot_callback(self, request, response):
        self.get_logger().info('>>> ได้รับคำสั่ง Save PID CSV ผ่าน Service...')
        
        with self.data_lock:
            if not self.all_time_history:
                response.success = False
                response.message = "ไม่มีข้อมูลให้เซฟ (หุ่นยนต์ยังไม่ได้วิ่ง)"
                return response

            # ตั้งชื่อไฟล์โดยแนบ Timestamp เพื่อไม่ให้เซฟทับไฟล์เก่า
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            csv_path = os.path.join(self.log_dir, f"pid_log_{timestamp}.csv")

            try:
                # 1. เขียนไฟล์ CSV ลงใน agri_log
                with open(csv_path, mode='w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['time', 'target_left', 'curr_left', 'target_right', 'curr_right'])
                    for i in range(len(self.all_time_history)):
                        writer.writerow([
                            self.all_time_history[i],
                            self.all_target_history[0][i], self.all_current_history[0][i],
                            self.all_target_history[1][i], self.all_current_history[1][i]
                        ])

                # 2. ล้างข้อมูลเพื่อเริ่มเก็บรอบใหม่ (สำหรับแปลงถัดไป)
                self.all_time_history.clear()
                self.all_target_history = [[] for _ in range(2)]
                self.all_current_history = [[] for _ in range(2)]
                self.start_time = time.time() # รีเซ็ตเวลาเพื่อไม่ให้กราฟรอบใหม่เวลาเพี้ยน

                response.success = True
                response.message = f"เซฟไฟล์สำเร็จ! เก็บไว้ที่: {csv_path}"
                self.get_logger().info(response.message)
                
            except Exception as e:
                response.success = False
                response.message = f"เซฟไฟล์ไม่สำเร็จ: {e}"
                self.get_logger().error(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = PidControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()