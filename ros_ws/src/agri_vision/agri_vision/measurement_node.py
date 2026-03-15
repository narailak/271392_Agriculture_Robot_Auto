#!/home/aorus-ubun/CMU/P.3/P.3-T.2/271392_Agriculture_Robot_Auto/venv/bin/python3
import cv2
import numpy as np
from ultralytics import YOLO
import csv
import os
import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

class MeasurementNode(Node):
    def __init__(self):
        super().__init__('measurement_node')
        
        self.get_logger().info("🚀 ระบบ Measurement เริ่มทำงานแล้ว รอรับคำสั่งเปิดจากสมองกล...")

        model_path = "/home/aorus-ubun/CMU/P.3/P.3-T.2/271392_Agriculture_Robot_Auto/ros_ws/src/agri_vision/models/cabbage.pt"
        self.model = YOLO(model_path)

        self.mtx = np.array([[942.01, 0, 445.51], [0, 945.76, 233.71], [0, 0, 1]], dtype=np.float32)
        self.dist = np.array([[-0.41678, 0.26653, 0.00208, -0.00702, -0.14830]], dtype=np.float32)

        self.F_AVG = (942.01 + 945.76) / 2 
        self.CAMERA_HEIGHT = 43.4           
        self.CENTER_THRESHOLD = 60          

        # ==========================================
        # 🌟 ระบบจัดการไฟล์และการนับจำนวนต้น
        # ==========================================
        self.base_log_dir = "/home/aorus-ubun/CMU/P.3/P.3-T.2/271392_Agriculture_Robot_Auto/agri_log"
        self.csv_file = os.path.join(self.base_log_dir, "cabbage_log_pro.csv")
        self.img_dir = os.path.join(self.base_log_dir, "captured_evidence")
        self.logged_ids = set() 
        
        # ตัวแปรนับลำดับต้นกะหล่ำ
        self.cabbage_count = 0 

        os.makedirs(self.base_log_dir, exist_ok=True)
        os.makedirs(self.img_dir, exist_ok=True)

        # 🌟 สร้างไฟล์ใหม่และเขียนหัวตาราง (ถ้ายังไม่มีไฟล์นี้)
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["Plant_No", "Track_ID", "Size_CM", "Timestamp", "Image_Path"])
                self.get_logger().info(f"✅ สร้างไฟล์ CSV ใหม่พร้อมหัวตารางที่: {self.csv_file}")

        self.is_enabled = False
        self.display_frame = None 
        
        self.last_cabbage_info = None

        self.sub_enable = self.create_subscription(Bool, '/tao/enable_measurement', self.enable_callback, 10)
        self.subscription = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, 10)
        self.pub_centered = self.create_publisher(Bool, '/tao/cabbage_centered', 10)

        self.srv_log = self.create_service(Trigger, '/tao/log_cabbage', self.log_cabbage_cb)

        self.display_timer = self.create_timer(0.05, self.display_loop)

    def enable_callback(self, msg):
        if msg.data == True and self.is_enabled == False:
            self.logged_ids.clear()
            self.cabbage_count = 0 # รีเซ็ตการนับต้นกลับเป็น 0 เมื่อเริ่มแปลงใหม่
            self.get_logger().info("🧹 ล้างความจำ ID เก่า และรีเซ็ตการนับต้น เตรียมพร้อมสำหรับแปลงใหม่!")
            
        self.is_enabled = msg.data
        status = "🟢 เปิด" if self.is_enabled else "🔴 ปิด"
        self.get_logger().info(f"ระบบ Measurement: {status} การสแกนกะหล่ำ")

    def log_cabbage_cb(self, request, response):
        if self.last_cabbage_info is not None:
            track_id = self.last_cabbage_info['id']
            diameter_cm = self.last_cabbage_info['size']
            annotated = self.last_cabbage_info['frame']

            if track_id not in self.logged_ids:
                self.cabbage_count += 1 # นับเพิ่มทีละ 1 ทุกครั้งที่แชะภาพสำเร็จ
                
                now = datetime.datetime.now()
                time_str = now.strftime("%Y-%m-%d %H:%M:%S")
                
                # เปลี่ยนชื่อไฟล์ให้ขึ้นต้นด้วยลำดับต้นกะหล่ำ (Plant_X)
                file_name = f"Plant_{self.cabbage_count}_ID_{track_id}_{now.strftime('%H%M%S')}.jpg"
                img_path = os.path.join(self.img_dir, file_name)

                # พิมพ์ข้อความบอกลำดับต้นลงในรูปภาพ (ตัวอักษรสีส้ม)
                cv2.putText(annotated, f"Plant No: {self.cabbage_count}", (20, 80), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)

                cv2.imwrite(img_path, annotated) 
                
                # บันทึกลง CSV โดยมี Plant_No อยู่หน้าสุด
                with open(self.csv_file, mode='a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([self.cabbage_count, track_id, round(diameter_cm, 2), time_str, file_name])
                
                self.logged_ids.add(track_id)
                self.get_logger().info(f"📸 [แชะภาพสำเร็จ!] ต้นที่: {self.cabbage_count} | Track ID: {track_id} | ขนาด: {diameter_cm:.2f} cm")
                
                response.success = True
                response.message = f"Saved Plant {self.cabbage_count} (ID {track_id})"
            else:
                response.success = True
        else:
            response.success = False
            
        return response

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None: return

        h, w = frame.shape[:2]
        screen_cx, screen_cy = w // 2, h // 2

        new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))
        frame = cv2.undistort(frame, self.mtx, self.dist, None, new_camera_mtx)

        annotated = frame.copy()
        cv2.circle(annotated, (screen_cx, screen_cy), 5, (255, 0, 0), -1)
        cv2.rectangle(annotated, (screen_cx-self.CENTER_THRESHOLD, screen_cy-self.CENTER_THRESHOLD), 
                      (screen_cx+self.CENTER_THRESHOLD, screen_cy+self.CENTER_THRESHOLD), (255, 255, 255), 1)

        best_dist = float('inf')
        current_best_cabbage = None

        if self.is_enabled:
            cv2.putText(annotated, "SCANNER: ON", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            results = self.model.track(frame, persist=True, verbose=False)

            if results[0].boxes is not None and results[0].boxes.id is not None:
                boxes = results[0].boxes.xywh.cpu().numpy()
                class_ids = results[0].boxes.cls.cpu().numpy()
                track_ids = results[0].boxes.id.int().cpu().tolist()

                for box, class_id, track_id in zip(boxes, class_ids, track_ids):
                    label = self.model.names[int(class_id)]
                    if label.lower() == 'cabbage': 
                        x, y, w_box, h_box = box
                        cx, cy = int(x), int(y)
                        x1, y1 = int(x - w_box/2), int(y - h_box/2)
                        x2, y2 = int(x + w_box/2), int(y + h_box/2)

                        diameter_px = (w_box + h_box) / 2
                        diameter_cm = (diameter_px * self.CAMERA_HEIGHT) / self.F_AVG
                        dist_to_center = np.sqrt((cx - screen_cx)**2 + (cy - screen_cy)**2)
                        
                        color = (0, 255, 255) if track_id not in self.logged_ids else (0, 255, 0)
                        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                        cv2.circle(annotated, (cx, cy), 5, color, -1)
                        cv2.putText(annotated, f"ID:{track_id} {diameter_cm:.2f}cm", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                        if dist_to_center < self.CENTER_THRESHOLD and track_id not in self.logged_ids:
                            if dist_to_center < best_dist:
                                best_dist = dist_to_center
                                current_best_cabbage = {'id': track_id, 'size': diameter_cm}

        if current_best_cabbage is not None:
            current_best_cabbage['frame'] = annotated.copy()
            self.last_cabbage_info = current_best_cabbage
            self.pub_centered.publish(Bool(data=True))

        else:
            if not self.is_enabled:
                cv2.putText(annotated, "SCANNER: OFF", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        self.display_frame = annotated

    def display_loop(self):
        if self.display_frame is not None:
            cv2.imshow("King Coco - Cabbage Smart Measurement", self.display_frame)
            if cv2.waitKey(1) & 0xFF == ord("q"): 
                raise SystemExit 

def main(args=None):
    rclpy.init(args=args)
    node = MeasurementNode()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit): pass
    finally: node.destroy_node(); rclpy.shutdown(); cv2.destroyAllWindows()

if __name__ == '__main__': main()