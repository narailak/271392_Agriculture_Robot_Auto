import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

import cv2
import numpy as np
from ultralytics import YOLO

class FrontCenterNavigator(Node):
    def __init__(self):
        super().__init__('front_center_navigator')

        # ===== YOLO Setup =====
        self.model_path = "/home/aorus-ubun/CMU/P.3/P.3-T.2/271392_Agriculture_Robot_Auto/ros_ws/src/agri_vision/models/segment3class.pt"
        self.model = YOLO(self.model_path)
        self.conf_threshold = 0.60        # 🎯 ปรับค่าความมั่นใจตรงนี้ครับ (เดิม 0.45 ผมปรับเป็น 0.60 ให้มั่นใจขึ้นค่อยจับ)
        
        # ===== ROS Communications =====
        self.sub = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/tao/cmd_vel', 10)
        self.servo_pub = self.create_publisher(Int16, '/tao/cmd_servo_cam', 10)

        # ===== Vehicle Control Parameters =====
        self.base_linear_speed = 0.5    
        self.max_angular_speed = 0.8      
        self.p_gain_x = 0.005             
        self.smoothing_factor = 0.25      
        self.last_angular_z = 0.0
        
        # ===== Servo Control Parameters =====
        self.servo_angle = 100       
        self.sweep_step = 5          
        self.track_step = 3          
        self.servo_dir = 1           
        self.frame_counter = 0
        self.lost_frames = 0
        
        self.publish_servo(self.servo_angle)
        self.get_logger().info("🎯 Fast Priority Navigator (Front > Side) Started!")

    def publish_servo(self, angle):
        msg = Int16()
        msg.data = int(angle)
        self.servo_pub.publish(msg)

    def image_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None: return

        h, w = frame.shape[:2]
        center_x, center_y = w // 2, h // 2
        
        tolerance_x = 40 
        self.frame_counter += 1

        # 🎯 นำค่า self.conf_threshold มาใช้ตรงนี้
        results = self.model(frame, conf=self.conf_threshold, verbose=False)
        
        front_box = None
        side_box = None
        
        target_x = None
        target_y = None
        active_target_label = None
        twist = Twist()

        # ==========================================
        # 1. กวาดสายตาหา Object ทั้งหมดก่อน
        # ==========================================
        for r in results:
            if r.boxes is None: continue
            
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = self.model.names[cls_id]
                
                if label == "front":
                    color = (0, 255, 0)
                    if front_box is None: front_box = (x1, y1, x2, y2) 
                elif label == "side":
                    color = (0, 165, 255)
                    if side_box is None: side_box = (x1, y1, x2, y2) 
                else:
                    color = (100, 100, 100)

                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                cv2.putText(frame, f"{label} {conf:.2f}", (int(x1), int(y1) - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # ==========================================
        # 2. เลือกล็อคเป้า (Priority: Front -> Side)
        # ==========================================
        if front_box is not None:
            target_x = (front_box[0] + front_box[2]) / 2.0
            target_y = (front_box[1] + front_box[3]) / 2.0
            active_target_label = "FRONT"
        elif side_box is not None:
            target_x = (side_box[0] + side_box[2]) / 2.0
            target_y = (side_box[1] + side_box[3]) / 2.0
            active_target_label = "SIDE"

        # ==========================================
        # 3. ควบคุมหุ่นยนต์ให้วิ่งเข้าหาเป้า (Centering)
        # ==========================================
        if active_target_label is not None:
            self.lost_frames = 0
            
            cv2.circle(frame, (int(target_x), int(target_y)), 6, (0, 0, 255), -1)
            cv2.putText(frame, f"TRACKING: {active_target_label}", (w - 200, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # --- ก. คุม Servo ก้ม-เงย ---
            error_y = target_y - center_y
            if self.frame_counter % 3 == 0:
                if error_y < -40:   self.servo_angle -= self.track_step
                elif error_y > 40:  self