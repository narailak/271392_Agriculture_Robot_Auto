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

        self.camera_params = [896.35, 895.53, 377.15, 224.76] 
        self.camera_matrix = np.array([
            [896.34636732, 0.0, 377.14536611],
            [0.0, 895.52629524, 224.76348467],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.array([
            [-4.33837502e-01, 9.89145910e-01, 2.32584268e-03, 2.38533357e-03, -3.81535553e+00]
        ])

        self.tag_size = 0.053

        # ==========================================
        # 3. Kinematics & DISTANCE CALIBRATION
        # ==========================================
        self.camera_to_front_dist = 0.20  
        self.front_to_bed_stop_dist = 0.10 
        
        self.target_distance = self.camera_to_front_dist + self.front_to_bed_stop_dist

        # ---------------------------------------------------------
        # [เพิ่มใหม่] ตั้งค่าการเยื้องศูนย์ (X-Offset)
        # แปลงกว้าง 15cm ป้ายอยู่ขวา ดังนั้นหุ่นต้องเล็งไปทางซ้ายของป้าย 7.5cm (0.075m)
        # ทำให้เมื่อมองจากหุ่น ป้ายจะต้องอยู่ทาง "ขวา" ของกล้องที่ระยะ 0.075m
        # ---------------------------------------------------------
        self.bed_offset_x = 0.075  # ค่าบวก = ป้ายอยู่ขวาหุ่น, ค่าลบ = ป้ายอยู่ซ้ายหุ่น

        self.kp_linear = 0.2   
        self.kp_angular = 2.0  
        self.max_linear_speed = 0.3  
        self.max_angular_speed = 4.5  

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
        self.get_logger().info("Farming AprilTag Node Started (With X-Offset)...")

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
            tag_id = tag.tag_id
            AB, C_gap, DE = self.decode_tag(tag_id)

            cx, cy = int(tag.center[0]), int(tag.center[1])
            
            error_y = cy - center_y

            # --- จัดการ Servo ---
            if self.frame_counter % 5 == 0:
                if error_y < -40: 
                    self.servo_angle -= self.track_step  
                elif error_y > 40: 
                    self.servo_angle += self.track_step  
                
                self.servo_angle = max(50, min(160, self.servo_angle))
                self.publish_servo(self.servo_angle)

            corners = tag.corners.astype(int)
            for i in range(4):
                cv2.line(frame_undistorted, tuple(corners[i]), tuple(corners[(i + 1) % 4]), (0, 255, 0), 3)
            
            cv2.line(frame_undistorted, (0, center_y), (w, center_y), (0, 255, 255), 1)

            if tag.pose_t is not None:
                tx = tag.pose_t[0][0] 
                tz = tag.pose_t[2][0] 

                current_front_to_tag = tz - self.camera_to_front_dist
                
                if self.frame_counter % 10 == 0:
                    self.get_logger().info(
                        f"Camera Dist (tz): {tz:.3f} m | "
                        f"Front Bumper to Tag: {current_front_to_tag:.3f} m | "
                        f"Lateral Offset (tx): {tx:.3f} m"
                    )

                calib_text = f"Front to Tag: {current_front_to_tag:.3f} m"
                cv2.putText(frame_undistorted, calib_text, (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

                # =========================================================
                # เงื่อนไขการเข้าเป้าหมาย (ใช้ x_error แทน tx ตัวเดิม)
                # =========================================================
                is_y_centered = abs(error_y) < 50
                distance_error = tz - self.target_distance
                is_distance_reached = abs(distance_error) < 0.05
                
                # คำนวณ error แกน X โดยเอาออฟเซ็ตมาลบออก
                x_error = tx - self.bed_offset_x
                is_x_aligned = abs(x_error) < 0.02

                # --- ควบคุมการเลี้ยว (แกน X) แก้ไขใหม่ให้คำนวณจาก x_error ---
                angular_z = -self.kp_angular * x_error
                angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))
                if is_x_aligned:
                    angular_z = 0.0
                cmd.angular.z = float(angular_z)

                # --- ควบคุมการเดินหน้าและเช็คสถานะ FINISH ---
                if is_y_centered:
                    linear_x = self.kp_linear * distance_error
                    linear_x = max(-self.max_linear_speed, min(self.max_linear_speed, linear_x))
                    
                    if is_distance_reached:
                        linear_x = 0.0
                    cmd.linear.x = float(linear_x)
                    
                    if is_distance_reached and is_x_aligned:
                        cv2.putText(frame_undistorted, "FINISH - TARGET REACHED!", (cx - 150, cy - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 3)
                    else:
                        cv2.putText(frame_undistorted, "Y-CENTERED - MOVING", (cx - 100, cy - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    cmd.linear.x = 0.0 
                    cv2.putText(frame_undistorted, "ADJUSTING CAMERA...", (cx - 100, cy - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

                cv2.putText(frame_undistorted, f"Servo Angle: {self.servo_angle} deg", (20, h - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        else:
            self.lost_frames += 1
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
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