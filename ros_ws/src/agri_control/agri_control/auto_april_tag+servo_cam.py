import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

import cv2
import numpy as np

from pupil_apriltags import Detector


class FarmingAprilTagNode(Node):

    def __init__(self):
        super().__init__('farming_apriltag_node')

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

        self.tag_size = 0.036

        # ==========================================
        # 3. Kinematics, OFFSET & STATE MACHINE
        # ==========================================
        self.camera_to_front_dist = 0.20  
        self.front_to_bed_stop_dist = 10.0 
        self.target_distance = self.camera_to_front_dist + self.front_to_bed_stop_dist

        self.bed_offset_x = 0.042  

        self.kp_linear = 0.6   
        self.kp_angular_meters = 5.0  
        
        self.max_linear_speed = 0.8 
        self.max_angular_speed = 20.0  
        self.min_angular_speed = 10.0  

        # ---------------------------------------------------------
        # ตัวแปรสำหรับ State Machine (Stop-and-Go Logic)
        # ---------------------------------------------------------
        self.robot_state = 'ALIGNING' 
        self.move_frame_count = 0      
        self.max_move_frames = 100      
        self.drift_tolerance = 0.06    

        # ==========================================
        # 4. Servo Control Parameters
        # ==========================================
        self.servo_angle = 100       
        self.sweep_step = 5         
        self.track_step = 5         
        self.servo_dir = 1          
        self.frame_counter = 0      
        
        self.lost_frames = 0        

        self.publish_servo(self.servo_angle)
        self.get_logger().info("Farming AprilTag Node Started (With Visual Target Line)...")

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
            tag = best_tag

            cx, cy = int(tag.center[0]), int(tag.center[1])
            error_y = cy - center_y

            # --- จัดการ Servo (แกน Y) ---
            if self.frame_counter % 5 == 0:
                if error_y < -40: 
                    self.servo_angle -= self.track_step  
                elif error_y > 40: 
                    self.servo_angle += self.track_step  
                
                self.servo_angle = max(50, min(160, self.servo_angle))
                self.publish_servo(self.servo_angle)

            # วาดกรอบของ AprilTag (สีเขียว)
            corners = tag.corners.astype(int)
            for i in range(4):
                cv2.line(frame_undistorted, tuple(corners[i]), tuple(corners[(i + 1) % 4]), (0, 255, 0), 3)
            
            # วาดเส้นกึ่งกลางกล้องแท้ๆ (แกน Y แนวนอนสีเหลือง, แกน X แนวตั้งสีเหลือง)
            cv2.line(frame_undistorted, (0, center_y), (w, center_y), (0, 255, 255), 1)
            cv2.line(frame_undistorted, (center_x, 0), (center_x, h), (0, 255, 255), 1)

            if tag.pose_t is not None:
                tx = tag.pose_t[0][0] 
                tz = tag.pose_t[2][0] 

                # =========================================================
                # 🎯 วาดเส้น Target Path (สีน้ำเงิน) บนจอภาพ 🎯
                # แปลงพิกัด 0.042m ในโลก 3D กลับมาเป็นพิกเซล 2D โดยใช้ Camera Matrix
                # =========================================================
                fx = self.camera_matrix[0, 0]
                cx_cam = self.camera_matrix[0, 2]
                
                # คำนวณหาตำแหน่งพิกเซล X ที่ Tag ควรจะมาอยู่
                target_pixel_x = int((fx * (self.bed_offset_x / tz)) + cx_cam)
                
                # ตรวจสอบให้อยู่ในขอบเขตภาพ
                target_pixel_x = max(0, min(w, target_pixel_x))
                
                # วาดเส้นแนวตั้งสีน้ำเงิน (Target Line)
                cv2.line(frame_undistorted, (target_pixel_x, 0), (target_pixel_x, h), (255, 0, 0), 2)
                cv2.putText(frame_undistorted, "TARGET PATH", (target_pixel_x + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                
                # วาดวงกลมสีแดงที่จุดกึ่งกลาง Tag ปัจจุบัน เพื่อให้ดูง่ายขึ้น
                cv2.circle(frame_undistorted, (cx, cy), 5, (0, 0, 255), -1)

                current_front_to_tag = tz - self.camera_to_front_dist
                x_error = tx - self.bed_offset_x
                
                is_x_aligned = abs(x_error) < 0.015
                is_y_centered_for_move = abs(error_y) < 50
                distance_error = tz - self.target_distance
                is_distance_reached = abs(distance_error) < 0.05

                if is_distance_reached:
                    self.robot_state = 'FINISH'

                # =========================================================
                # LOGIC: STATE MACHINE (Stop-and-Go)
                # =========================================================
                if self.robot_state == 'ALIGNING':
                    cmd.linear.x = 0.0
                    angular_z = -self.kp_angular_meters * x_error
                    
                    if not is_x_aligned:
                        if angular_z > 0:
                            angular_z = max(self.min_angular_speed, angular_z)
                        elif angular_z < 0:
                            angular_z = min(-self.min_angular_speed, angular_z)
                    
                    angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))
                    cmd.angular.z = float(angular_z)

                    if is_x_aligned and is_y_centered_for_move:
                        self.robot_state = 'MOVING'
                        self.move_frame_count = 0  

                    cv2.putText(frame_undistorted, "STATE: ALIGNING (STOPPED)", (cx - 150, cy - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

                elif self.robot_state == 'MOVING':
                    cmd.angular.z = 0.0

                    linear_x = self.kp_linear * distance_error
                    cmd.linear.x = float(max(-self.max_linear_speed, min(self.max_linear_speed, linear_x)))
                    
                    self.move_frame_count += 1

                    if self.move_frame_count >= self.max_move_frames or abs(x_error) > self.drift_tolerance:
                        self.robot_state = 'ALIGNING'

                    cv2.putText(frame_undistorted, "STATE: MOVING FORWARD", (cx - 150, cy - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                elif self.robot_state == 'FINISH':
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    cv2.putText(frame_undistorted, "FINISH - ALIGNED & REACHED!", (cx - 150, cy - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 3)

                # แสดงค่า Debug บนจอ
                cv2.putText(frame_undistorted, f"State: {self.robot_state}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.putText(frame_undistorted, f"X Error: {x_error:.3f} m", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                cv2.putText(frame_undistorted, f"Front to Tag: {current_front_to_tag:.3f} m", (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                cv2.putText(frame_undistorted, f"Cmd Linear: {cmd.linear.x:.2f} | Angular: {cmd.angular.z:.2f}", (20, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

        else:
            self.lost_frames += 1
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.robot_state = 'ALIGNING' 
            
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
            
        cv2.putText(frame_undistorted, f"Servo Angle: {self.servo_angle} deg", (20, h - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        self.cmd_pub.publish(cmd)

        cv2.imshow("Farming AprilTag Navigation", frame_undistorted)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = FarmingAprilTagNode()
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