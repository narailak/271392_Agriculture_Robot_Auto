import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
import cv2
import numpy as np
import time
import math
from pupil_apriltags import Detector 

class AprilTagDetector(Node):

    def __init__(self):
        super().__init__('apriltag_detector')
        
        # ===== 📷 Camera Calibration Data =====
        self.camera_params = [942.01, 945.76, 445.51, 233.71] 
        self.camera_matrix = np.array([
            [942.00959306, 0.0, 445.50898168],
            [0.0, 945.76258164, 233.70507804],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.array([
            [-0.41678417, 0.26653241, 0.00208138, -0.00701684, -0.14829814]
        ])
        self.frame_center_x = self.camera_matrix[0, 2] 

        # ===== 📏 Physical Parameters =====
        self.tag_size = 0.036      
        self.bed_offset_x = 0.042 
        
        # 🎯 ตัวแปรระยะ State 
        self.curve_distance = 0.90   
        self.stop_distance = 0.30    

        # 🎯 ตัวแปรควบคุมมุม Yaw หน้าแปลง
        self.target_yaw_deg = 0.0    
        self.Kp_yaw = 0.5          

        # ===== 🌟 State Machine & Memory =====
        self.robot_state = 'APPROACHING'  
        self.lost_frames = 0             
        self.last_twist = Twist()        
        
        # 🏷️ ตัวแปรจำ ID ของ AprilTag
        self.target_tag_id = None    

        # 🌟 ตัวแปรจับเวลาเดินหน้าตาบอด 0.2 วินาที
        self.blind_forward_start_time = 0.0

        # ===== Setup AprilTag =====
        self.detector = Detector(
            families="tagStandard52h13", 
            nthreads=4,
            quad_decimate=1.5,
            quad_sigma=0.0,
            refine_edges=True
        )

        # ===== Publishers & Subscribers =====
        self.subscription = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/tao/cmd_vel', 10)
        self.servo_pub = self.create_publisher(Int16, '/tao/cmd_servo_cam', 10)

        # ===== PID หุ่นยนต์ (สำหรับแก้ X Offset) =====
        self.Kp = 5.0    
        self.Ki = 0.0
        self.Kd = 0.1
        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_time = time.time()

        # ===== 🚀 SPEED =====
        self.max_linear_speed = 0.2     
        self.curve_linear_speed = 0.15  
        self.max_angular_speed = 3.0   
        self.min_angular_speed = 2.0    

        # ===== 🤖 SERVO CONTROL & SCAN =====
        self.servo_angle = 70.0 
        self.max_servo = 160
        self.min_servo = 50
        
        self.scan_step = 3        
        self.scan_interval = 1.0    
        self.last_scan_time = time.time()
        
        self.Kp_servo = 0.01     
        
        self.publish_servo(int(self.servo_angle))
        self.get_logger().info("✅ AprilTag Node Started (With 0.2s Blind Forward)")

    def publish_servo(self, angle):
        msg = Int16()
        msg.data = int(angle)
        self.servo_pub.publish(msg)

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None: return

        frame_undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        gray_frame = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)
        
        img_height, img_width = frame_undistorted.shape[:2]
        center_y = img_height // 2
        cv2.line(frame_undistorted, (0, center_y), (img_width, center_y), (0, 255, 255), 1)

        results = self.detector.detect(
            gray_frame, 
            estimate_tag_pose=True, 
            camera_params=self.camera_params, 
            tag_size=self.tag_size
        )
        twist_msg = Twist()

        best_tag = None
        max_area = 0
        
        # ---------------------------------------------------------
        # 🏷️ ระบบคัดกรอง ID และเลือกด้านที่ใหญ่ที่สุด (สำหรับลูกบาศก์)
        # ---------------------------------------------------------
        if self.target_tag_id is None and len(results) > 0:
            for tag in results:
                area = cv2.contourArea(tag.corners.astype(np.float32))
                if area > max_area:
                    max_area = area
                    best_tag = tag
            
            if best_tag is not None:
                self.target_tag_id = best_tag.tag_id 
                self.get_logger().info(f"🎯 Locked on to Tag ID: {self.target_tag_id}")

        elif self.target_tag_id is not None:
            for tag in results:
                if tag.tag_id == self.target_tag_id:
                    area = cv2.contourArea(tag.corners.astype(np.float32))
                    if area > max_area:
                        max_area = area
                        best_tag = tag
        # ---------------------------------------------------------

        if best_tag is not None:
            if self.lost_frames > 0:
                self.previous_time = time.time()
                self.integral = 0.0
                self.previous_error = 0.0
                
            self.lost_frames = 0  
            
            corners = best_tag.corners.astype(int)
            for i in range(4): 
                cv2.line(frame_undistorted, tuple(corners[i]), tuple(corners[(i + 1) % 4]), (0, 255, 0), 2)
                
            cv2.putText(frame_undistorted, f"ID: {best_tag.tag_id}", (corners[0][0], corners[0][1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            tag_center_x, tag_center_y = int(best_tag.center[0]), int(best_tag.center[1])
            cv2.circle(frame_undistorted, (tag_center_x, tag_center_y), 5, (0, 0, 255), -1)
            
            # 📷 SERVO AUTO-CENTERING
            error_y = tag_center_y - center_y 
            self.servo_angle += (self.Kp_servo * error_y)
            self.servo_angle = max(min(self.servo_angle, self.max_servo), self.min_servo)
            self.publish_servo(int(self.servo_angle))
            
            cv2.putText(frame_undistorted, f"Servo Ang: {int(self.servo_angle)} | Err Y: {error_y}", (img_width - 250, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # 🤖 NAVIGATION
            if best_tag.pose_t is not None and best_tag.pose_R is not None:
                current_distance = float(best_tag.pose_t[2][0])
                tx = float(best_tag.pose_t[0][0])
                error_m = float(tx - self.bed_offset_x)
                
                R = best_tag.pose_R
                sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
                if sy > 1e-6:
                    current_yaw_rad = math.atan2(-R[2,0], sy)
                else:
                    current_yaw_rad = math.atan2(-R[2,1], R[1,1])
                
                current_yaw_deg = math.degrees(current_yaw_rad)
                yaw_error_rad = math.radians(self.target_yaw_deg) - current_yaw_rad

                current_time = time.time()
                dt = current_time - self.previous_time
                if dt <= 0.0001: 
                    dt = 0.0001

                self.integral += error_m * dt
                self.integral = max(min(self.integral, 0.5), -0.5) 
                
                derivative = (error_m - self.previous_error) / dt
                output_x = float((self.Kp * error_m) + (self.Ki * self.integral) + (self.Kd * derivative))

                self.previous_error = error_m
                self.previous_time = current_time

                output_x = float(max(min(output_x, self.max_angular_speed), -self.max_angular_speed))

                if current_distance <= self.stop_distance:
                    self.robot_state = 'FINISH'
                elif current_distance <= self.curve_distance:
                    self.robot_state = 'CURVE_IN'
                else:
                    self.robot_state = 'APPROACHING'

                # 🚀 การเคลื่อนที่
                if self.robot_state == 'APPROACHING':
                    twist_msg.linear.x = float(self.max_linear_speed)
                    twist_msg.angular.z = float(-output_x) 
                    cv2.putText(frame_undistorted, "STATE 1: APPROACHING", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                elif self.robot_state == 'CURVE_IN':
                    twist_msg.linear.x = float(self.curve_linear_speed)
                    curve_angular_speed = (-output_x) + (self.Kp_yaw * yaw_error_rad * 0.8)
                    curve_angular_speed = float(max(min(curve_angular_speed, self.max_angular_speed), -self.max_angular_speed))
                    twist_msg.angular.z = curve_angular_speed
                    cv2.putText(frame_undistorted, f"STATE 2: CURVE IN (Yaw Err: {math.degrees(yaw_error_rad):.1f} deg)", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

                elif self.robot_state == 'FINISH':
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = 0.0
                    cv2.putText(frame_undistorted, "REACHED TARGET!", (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

                cv2.putText(frame_undistorted, f"State: {self.robot_state} | Tracking ID: {self.target_tag_id}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
                cv2.putText(frame_undistorted, f"Dist: {current_distance:.2f}m | Err X: {error_m:.3f}m", (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 100), 2)
                cv2.putText(frame_undistorted, f"Yaw: {current_yaw_deg:.1f} deg", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 100, 255), 2)
                
                self.last_twist = twist_msg
                self.cmd_pub.publish(twist_msg)

        else:
            # 🛡️ SEARCHING / TAG LOST
            self.lost_frames += 1
            now = time.time()
            
            if self.lost_frames < 5:
                # 5 เฟรมแรก ปล่อยค่าเดิมเผื่อป้ายแค่กระพริบ
                self.cmd_pub.publish(self.last_twist)
                cv2.putText(frame_undistorted, "BLUR BUFFER...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 165, 255), 2)
            else:
                # 🌟 ถ้าระยะก่อนหน้านี้เข้าโค้งแล้ว (STATE 2) ให้เดินหน้าตาบอด 0.2 วินาที
                if self.robot_state == 'CURVE_IN':
                    if self.lost_frames == 5:
                        self.blind_forward_start_time = now
                        self.get_logger().warn("⚠️ ป้ายหายใกล้เป้าหมาย! เดินหน้าตาบอดความเร็ว 0.2 เป็นเวลา 0.2 วินาที")
                        
                    elapsed_blind = now - self.blind_forward_start_time
                    if elapsed_blind <= 0.2:
                        twist_msg = Twist()
                        twist_msg.linear.x = 0.2   # ความเร็ว 0.2
                        twist_msg.angular.z = 0.0  # เดินหน้าตรงๆ
                        self.cmd_pub.publish(twist_msg)
                        cv2.putText(frame_undistorted, f"BLIND FORWARD {elapsed_blind:.2f}/0.2s", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    else:
                        # เกิน 0.2 วิแล้ว ให้รถหยุด และเริ่มส่ายกล้องค้นหาปกติ
                        self.cmd_pub.publish(Twist()) 
                        
                        search_text = f"SEARCHING FOR ID: {self.target_tag_id}" if self.target_tag_id is not None else "SEARCHING..."
                        cv2.putText(frame_undistorted, search_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                        
                        if now - self.last_scan_time > self.scan_interval:
                            self.servo_angle += self.scan_step
                            if self.servo_angle >= self.max_servo:
                                self.servo_angle = self.min_servo 
                            self.publish_servo(int(self.servo_angle))
                            self.last_scan_time = now
                else:
                    # ถ้าป้ายหายในระยะไกล (APPROACHING) ให้หยุดรถแล้วสแกนหาทันที
                    self.cmd_pub.publish(Twist()) 
                    
                    search_text = f"SEARCHING FOR ID: {self.target_tag_id}" if self.target_tag_id is not None else "SEARCHING..."
                    cv2.putText(frame_undistorted, search_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                    
                    if now - self.last_scan_time > self.scan_interval:
                        self.servo_angle += self.scan_step
                        if self.servo_angle >= self.max_servo:
                            self.servo_angle = self.min_servo 
                        self.publish_servo(int(self.servo_angle))
                        self.last_scan_time = now
                        
                    cv2.putText(frame_undistorted, f"Scanning Ang: {int(self.servo_angle)}", (img_width - 250, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.imshow("AprilTag Navigation", frame_undistorted)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist()) 
            node.destroy_node()
            rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()