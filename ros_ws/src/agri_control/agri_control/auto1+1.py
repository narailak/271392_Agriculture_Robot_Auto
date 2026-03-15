import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

import cv2
import numpy as np

# นำเข้าไลบรารีของทั้ง 2 ระบบ
from pupil_apriltags import Detector
from ultralytics import YOLO

class HybridNavNode(Node):

    def __init__(self):
        super().__init__('hybrid_nav_node')

        # ==========================================
        # 1. State Machine Setup
        # ==========================================
        # สถานะปัจจุบัน: เริ่มต้นที่ 'APRILTAG' และจะเปลี่ยนเป็น 'YOLO' เมื่อถึงเป้าหมาย
        self.current_state = 'APRILTAG'  

        # ==========================================
        # 2. ROS Setup (ใช้ร่วมกัน)
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
        # 3. AprilTag Setup
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

        # Parameters สำหรับ AprilTag
        self.camera_to_front_dist = 0.20  
        self.front_to_bed_stop_dist = 0.20 
        self.target_distance = self.camera_to_front_dist + self.front_to_bed_stop_dist
        self.bed_offset_x = 0.075  

        self.kp_linear = 0.6   
        self.kp_angular = 2.0  
        self.max_linear_speed = 0.8 
        self.max_angular_speed = 4.5  

        # Parameters สำหรับ Servo 
        self.servo_angle = 100       
        self.sweep_step = 5         
        self.track_step = 5         
        self.servo_dir = 1          
        self.frame_counter = 0      
        self.lost_frames = 0        

        # ==========================================
        # 4. YOLO Segmentation Setup
        # ==========================================
        self.get_logger().info("Loading YOLO Model... (อาจใช้เวลาสักครู่)")
        self.model = YOLO("/home/aorus-ubun/CMU/P.3/P.3-T.2/271392_Agriculture_Robot_Auto/ros_ws/src/cabbage_detection/models/plot_seg.pt")
        
        # Parameters สำหรับ YOLO
        self.yolo_conf_threshold = 0.70  # <-- ตั้งค่า Threshold ความมั่นใจของ YOLO ตรงนี้ (0.0 ถึง 1.0)
        self.yolo_forward_speed = 0.25   # ความเร็วเดินหน้าตอนเกาะแนวแปลง
        self.yolo_turn_gain = 1.0 / 300.0 # Gain สำหรับคูณความเร็วพวงมาลัย
        self.yolo_search_speed = 0.3     # ความเร็วหมุนตัวตอนหาแปลงไม่เจอ

        self.publish_servo(self.servo_angle)
        self.get_logger().info("✅ Hybrid Node Started: เริ่มต้นโหมด [APRILTAG]")

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
        
        if frame is None:
            return

        cmd = Twist()
        self.frame_counter += 1

        # ==========================================
        # STATE 1: โหมดขับด้วย AprilTag
        # ==========================================
        if self.current_state == 'APRILTAG':
            frame_undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
            gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)

            results = self.detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=self.camera_params,
                tag_size=self.tag_size
            )

            h, w = frame_undistorted.shape[:2]
            center_x, center_y = w // 2, h // 2

            best_tag = None
            max_area = 0

            # เลือก tag ที่ขนาดใหญ่ที่สุด (ไม่ได้เช็ค ID)
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
                    calib_text = f"Front to Tag: {current_front_to_tag:.3f} m"
                    cv2.putText(frame_undistorted, calib_text, (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

                    # เงื่อนไขการเข้าเป้าหมาย
                    distance_error = tz - self.target_distance
                    is_distance_reached = abs(distance_error) < 0.05
                    
                    x_error = tx - self.bed_offset_x
                    is_x_aligned = abs(x_error) < 0.02

                    # --- ควบคุมการขับ ---
                    angular_z = -self.kp_angular * x_error
                    angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))
                    if is_x_aligned:
                        angular_z = 0.0
                    cmd.angular.z = float(angular_z)

                    linear_x = self.kp_linear * distance_error
                    linear_x = max(-self.max_linear_speed, min(self.max_linear_speed, linear_x))
                    
                    if is_distance_reached:
                        linear_x = 0.0
                    cmd.linear.x = float(linear_x)

                    # ----------------------------------------------------
                    # จุดเปลี่ยนสถานะ (SWITCH STATE TRIGGER)
                    # ----------------------------------------------------
                    if is_distance_reached and is_x_aligned:
                        cv2.putText(frame_undistorted, "SWITCHING TO YOLO...", (cx - 150, cy - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 255), 3)
                        
                        self.get_logger().info("✅ Target Reached! สลับเข้าสู่โหมด YOLO...")
                        
                        self.cmd_pub.publish(Twist()) # หยุดรถเพื่อเปลี่ยนโหมด
                        self.current_state = 'YOLO'
                        return 
            else:
                self.lost_frames += 1
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                
                if self.lost_frames > 15:
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
            
            self.cmd_pub.publish(cmd)
            cv2.imshow("Hybrid Navigation", frame_undistorted)
            cv2.waitKey(1)

        # ==========================================
        # STATE 2: โหมดขับด้วย YOLO Segmentation
        # ==========================================
        elif self.current_state == 'YOLO':
            h, w, _ = frame.shape
            image_center_x = w / 2.0

            # รันโมเดล YOLO พร้อมใส่ค่า Threshold ที่ตั้งไว้
            yolo_results = self.model(frame, conf=self.yolo_conf_threshold, verbose=False)

            annotated_frame = frame.copy()
            
            largest_side_box = None
            max_area = 0

            # วาดกรอบและ Segment ลงบนภาพ
            for r in yolo_results:
                annotated_frame = r.plot()
                boxes = r.boxes
                
                if boxes is not None:
                    for box in boxes:
                        cls_id = int(box.cls[0])
                        label = self.model.names[cls_id]

                        if label == "side":
                            # หาพื้นที่ของกล่อง
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            area = (x2 - x1) * (y2 - y1)
                            
                            if area > max_area:
                                max_area = area
                                largest_side_box = box

            if largest_side_box is not None:
                x1, y1, x2, y2 = largest_side_box.xyxy[0].cpu().numpy()
                
                # หาจุดกึ่งกลางแกน x ของกล่อง side
                box_center_x = (x1 + x2) / 2.0
                
                # เส้นอ้างอิง: สมมติว่าต้องการให้แปลงอยู่ทางซ้าย (25% ของความกว้างจอ)
                target_x = w * 0.25 
                
                # คำนวณระยะห่างเพื่อตีวงเลี้ยว
                control_error = box_center_x - target_x
                
                # สั่งรถเดินหน้า
                cmd.linear.x = self.yolo_forward_speed
                # สั่งหักพวงมาลัย (ยิ่งไกลเส้นเป้าหมาย ยิ่งหักเยอะ)
                cmd.angular.z = float(-control_error * self.yolo_turn_gain)
                
                # วาดเส้นช่วยเหลือลงบนจอ
                cv2.rectangle(annotated_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.circle(annotated_frame, (int(box_center_x), int((y1+y2)/2)), 5, (0, 0, 255), -1)
                cv2.line(annotated_frame, (int(target_x), 0), (int(target_x), h), (255, 255, 0), 2)
                cv2.line(annotated_frame, (int(box_center_x), 0), (int(box_center_x), h), (0, 0, 255), 2)
                
            else:
                # ถ้าไม่เจอแปลง side เลย ให้รถหยุดเดินหน้า และค่อยๆ หมุนตัวหา
                cmd.linear.x = 0.0
                cmd.angular.z = float(self.yolo_search_speed)
                cv2.putText(annotated_frame, "SEARCHING PLOT...", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # แสดงสถานะ Command
            cmd_text = f"[MOVING] V: {cmd.linear.x:.2f}, W: {cmd.angular.z:.2f}"
            cv2.putText(annotated_frame, f"[YOLO] Conf: {self.yolo_conf_threshold}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
            cv2.putText(annotated_frame, cmd_text, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # --- ส่งคำสั่งขับรถจริงแล้ว ---
            self.cmd_pub.publish(cmd)

            cv2.imshow("Hybrid Navigation", annotated_frame)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = HybridNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist()) # หยุดรถทันทีเมื่อปิดโปรแกรม
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()