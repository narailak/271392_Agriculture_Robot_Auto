import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

import cv2
import numpy as np
import time

from pupil_apriltags import Detector


class FarmingAprilTagNode(Node):

    def __init__(self):
        super().__init__('final_apriltag_node')

        # ==========================================
        # 1. ROS Setup
        # ==========================================
        self.sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/tao/cmd_vel', 10)
        self.servo_pub = self.create_publisher(Int16, '/tao/cmd_servo_cam', 10) 

        # ==========================================
        # 2. AprilTag Setup
        # ==========================================
        self.detector = Detector(
            families="tagStandard52h13",
            nthreads=4,
            quad_decimate=1.5,
            quad_sigma=0.0,
            refine_edges=True
        )

        self.camera_params = [942.01, 945.76, 445.51, 233.71] 
        self.camera_matrix = np.array([
            [942.00959306, 0.0, 445.50898168],
            [0.0, 945.76258164, 233.70507804],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.array([
            [-0.41678417, 0.26653241, 0.00208138, -0.00701684, -0.14829814]
        ])

        self.tag_size = 0.053

        # ==========================================
        # 3. Kinematics & DISTANCE CALIBRATION
        # ==========================================
        self.camera_to_front_dist = 0.0  
        self.front_to_bed_stop_dist = 0.40 
        
        self.target_distance = self.camera_to_front_dist + self.front_to_bed_stop_dist
        self.bed_offset_x = 0.042  

        self.kp_linear = 3.6  
        self.kp_angular = 2.0  
        self.max_linear_speed = 5.0 
        self.max_angular_speed = 2.5  

        # ==========================================
        # 4. Servo Control Parameters
        # ==========================================
        self.servo_angle = 100       
        self.sweep_step = 5         
        self.track_step = 5         
        self.servo_dir = 1          
        self.frame_counter = 0      
        self.lost_frames = 0        

        # 🌟 ตัวแปรสำหรับ Blind Drive & Reverse Logic
        self.last_tz = 999.0                
        self.is_blind_driving = False       
        self.is_reversing = False           # สถานะกำลังถอยหลัง
        self.mission_completed = False      # สถานะทำงานเสร็จสิ้นทั้งหมด
        self.blind_drive_start_time = 0.0   

        self.publish_servo(self.servo_angle)
        self.get_logger().info("Farming AprilTag Node Started...")

    def decode_tag(self, tag_id):
        tag_str = str(tag_id).zfill(5)
        AB = int(tag_str[:2])
        C = int(tag_str[2])
        DE = int(tag_str[3:])
        spacing_map = {1: 5, 2: 10, 3: 15, 4: 20, 5: 25}
        spacing_gap = spacing_map.get(C, 0)
        return AB, spacing_gap, DE

    def publish_servo(self, angle):
        msg = Int16() 
        msg.data = int(angle)
        self.servo_pub.publish(msg)

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        frame_undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)

        results = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.tag_size
        )

        cmd = Twist()
        self.frame_counter += 1

        h, w = frame_undistorted.shape[:2]
        center_x, center_y = w // 2, h // 2

        best_tag = None
        max_area = 0

        for tag in results:
            area = cv2.contourArea(tag.corners.astype(np.float32))
            if area > max_area:
                max_area = area
                best_tag = tag

        if best_tag is not None:
            self.lost_frames = 0  
            
            # 🌟 ถ้ากำลังถอยหลังอยู่แล้วเจอ Tag แปลว่าจบงาน
            if self.is_reversing:
                self.is_reversing = False
                self.mission_completed = True
                self.get_logger().info("FINISH - Tag found after reversing! Stopping.")

            tag = best_tag
            tag_id = tag.tag_id
            AB, C_gap, DE = self.decode_tag(tag_id)

            cx, cy = int(tag.center[0]), int(tag.center[1])
            error_y = cy - center_y

            # วาดกรอบสี่เหลี่ยมรอบ Tag
            corners = tag.corners.astype(int)
            for i in range(4):
                cv2.line(frame_undistorted, tuple(corners[i]), tuple(corners[(i + 1) % 4]), (0, 255, 0), 3)
            cv2.line(frame_undistorted, (0, center_y), (w, center_y), (0, 255, 255), 1)

            if tag.pose_t is not None:
                tx = tag.pose_t[0][0] 
                tz = tag.pose_t[2][0] 
                
                self.last_tz = tz 
                current_front_to_tag = tz - self.camera_to_front_dist
                
                calib_text = f"Front to Tag: {current_front_to_tag:.3f} m"
                cv2.putText(frame_undistorted, calib_text, (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

                # =========================================================
                # 🌟 ถ้าภารกิจจบแล้ว ให้จอดนิ่งๆ อย่างเดียว ข้ามคำสั่งควบคุมปกติไปเลย
                # =========================================================
                if self.mission_completed:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    cv2.putText(frame_undistorted, "MISSION COMPLETE - ROBOT STOPPED", (50, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)
                
                # =========================================================
                # 🌟 คำสั่งควบคุมปกติ (เดินหน้า/เลี้ยว/ส่ายกล้อง)
                # =========================================================
                else:
                    self.is_blind_driving = False 

                    # ควบคุม Servo ให้มองกลาง Tag
                    if self.frame_counter % 5 == 0:
                        if error_y < -40: 
                            self.servo_angle -= self.track_step  
                        elif error_y > 40: 
                            self.servo_angle += self.track_step  
                        self.servo_angle = max(50, min(160, self.servo_angle))
                        self.publish_servo(self.servo_angle)

                    is_y_centered = (abs(error_y) < 50) or (tz < 0.5)
                    distance_error = tz - self.target_distance
                    is_distance_reached = abs(distance_error) < 0.05
                    x_error = tx - self.bed_offset_x
                    is_x_aligned = abs(x_error) < 0.02

                    angular_z = -self.kp_angular * x_error
                    angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))
                    if is_x_aligned:
                        angular_z = 0.0
                    cmd.angular.z = float(angular_z)

                    if is_y_centered:
                        linear_x = self.kp_linear * distance_error
                        linear_x = max(-self.max_linear_speed, min(self.max_linear_speed, linear_x))
                        
                        if is_distance_reached:
                            linear_x = 0.0
                        cmd.linear.x = float(linear_x)
                        
                        if is_distance_reached and is_x_aligned:
                            cv2.putText(frame_undistorted, "FINISH - TARGET REACHED!", (cx - 150, cy - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 3)
                        else:
                            cv2.putText(frame_undistorted, "MOVING TO TARGET", (cx - 100, cy - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    else:
                        cmd.linear.x = 0.0 
                        cv2.putText(frame_undistorted, "ADJUSTING CAMERA...", (cx - 100, cy - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

                cv2.putText(frame_undistorted, f"Servo Angle: {self.servo_angle} deg", (20, h - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        else:
            self.lost_frames += 1
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            current_time = time.time()

            # ถ้าภารกิจจบแล้ว แล้ว Tag หลุดกล้อง ก็ให้หยุดนิ่งเหมือนเดิม
            if self.mission_completed:
                cv2.putText(frame_undistorted, "MISSION COMPLETE - ROBOT STOPPED", (50, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)
            else:
                # =========================================================
                # 🌟 Step 1: ไม่เจอ Tag, ระยะ < 0.5 -> เริ่ม Blind Drive Forward 1.5 วิ
                # =========================================================
                if self.last_tz < 0.5:
                    if not self.is_blind_driving and not self.is_reversing:
                        self.is_blind_driving = True
                        self.blind_drive_start_time = current_time
                        self.get_logger().info("Tag lost near target! Blind drive FWD for 1.5s.")

                if self.is_blind_driving:
                    elapsed_time = current_time - self.blind_drive_start_time
                    if elapsed_time < 1.5:
                        cmd.linear.x = 3.6  
                        cmd.angular.z = 0.0 
                        cv2.putText(frame_undistorted, f"BLIND DRIVE FWD: {1.5 - elapsed_time:.1f}s", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
                    else:
                        # =========================================================
                        # 🌟 Step 2: หมด 1.5 วิ -> สั่งก้มกล้อง 160 องศา แล้วเข้าสู่โหมดถอยหลัง
                        # =========================================================
                        self.is_blind_driving = False
                        self.is_reversing = True
                        
                        self.servo_angle = 160
                        self.publish_servo(self.servo_angle)
                        self.get_logger().info("Switching to REVERSE mode. Waiting for Tag.")
                        
                # =========================================================
                # 🌟 Step 3: กำลังถอยหลัง (ถอยไปเรื่อยๆ จนกว่าจะเจอ Tag ในโค้ดด้านบน)
                # =========================================================
                elif self.is_reversing:
                    cmd.linear.x = -2.0  # ถอยหลัง (-2.0)
                    cmd.angular.z = 0.0
                    cv2.putText(frame_undistorted, "REVERSING... WAITING FOR TAG", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 3)
                    
                # =========================================================
                # 🌟 แท็กหายแบบปกติ (อยู่ไกล) ให้ส่ายกล้องหา
                # =========================================================
                else:
                    if self.lost_frames < 15:
                        cv2.putText(frame_undistorted, "WAITING FOR BLUR TO CLEAR...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
                    else:
                        if self.frame_counter % 10 == 0:
                            self.servo_angle += (self.sweep_step * self.servo_dir)
                            if self.servo_angle >= 160:
                                self.servo_angle = 160
                                self.servo_dir = -1  
                            elif self.servo_angle <= 50:
                                self.servo_angle = 50
                                self.servo_dir = 1   
                            self.publish_servo(self.servo_angle)

                        cv2.putText(frame_undistorted, "SEARCHING TAG...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                
            cv2.putText(frame_undistorted, f"Servo Angle: {self.servo_angle} deg", (20, h - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        self.cmd_pub.publish(cmd)

        cv2.imshow("Farming AprilTag Navigation", frame_undistorted)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = FarmingAprilTagNode()  # 🌟 แก้ไขตรงนี้ให้ตรงกับชื่อ Class ด้านบน
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()