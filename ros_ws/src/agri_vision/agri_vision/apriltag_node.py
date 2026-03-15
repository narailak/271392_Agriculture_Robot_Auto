#!/home/aorus-ubun/CMU/P.3/P.3-T.2/271392_Agriculture_Robot_Auto/venv/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

# ดึง Service ที่เราสร้างไว้มาใช้งาน
from agri_interfaces.srv import ReadTag 

import cv2
import numpy as np
import os
import csv
import time
from pupil_apriltags import Detector

class AprilTagServiceNode(Node):
    def __init__(self):
        super().__init__('apriltag_service_node')
        
        # ตัวแปรเก็บภาพล่าสุด (รับมารอไว้ แต่ยังไม่ประมวลผลให้หนักเครื่อง)
        self.latest_frame = None
        self.latest_display_frame = None

        # ==========================================
        # 1. ROS Setup
        # ==========================================
        self.sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )
        
        self.srv = self.create_service(ReadTag, 'read_apriltag', self.handle_read_tag)

        # Timer สำหรับโชว์ภาพสด (รัน 20 FPS)
        self.timer = self.create_timer(0.05, self.display_loop)

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

        self.camera_matrix = np.array([
            [896.34636732, 0.0, 377.14536611],
            [0.0, 895.52629524, 224.76348467],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.array([
            [-4.33837502e-01, 9.89145910e-01, 2.32584268e-03, 2.38533357e-03, -3.81535553e+00]
        ])

        # ==========================================
        # 3. Logging Setup (อัปเดต Path ใหม่แล้ว)
        # ==========================================
        self.logged_tags = set()
        
        # 🌟 ชี้ไปที่ Path ของโฟลเดอร์ agri_log ศูนย์รวมข้อมูลของเรา
        self.log_dir = '/home/aorus-ubun/CMU/P.3/P.3-T.2/271392_Agriculture_Robot_Auto/agri_log'
        os.makedirs(self.log_dir, exist_ok=True)
        
        self.csv_file_path = os.path.join(self.log_dir, 'farming_parameters.csv')
        
        # สร้าง Header ของไฟล์ CSV หากยังไม่มีไฟล์
        if not os.path.exists(self.csv_file_path):
            with open(self.csv_file_path, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Timestamp', 'Tag_ID', 'Planting_Dist_cm', 'Spacing_Gap_cm', 'Interval_Dist_cm'])
                self.get_logger().info(f"สร้างไฟล์ Log ใหม่ที่: {self.csv_file_path}")

        self.get_logger().info("AprilTag Service Server Started. [WAITING FOR COMMANDS...]")

    def decode_tag(self, tag_id):
        tag_str = str(tag_id).zfill(5)
        AB = int(tag_str[:2])
        C = int(tag_str[2])
        DE = int(tag_str[3:])

        spacing_map = {1: 5, 2: 10, 3: 15, 4: 20, 5: 25}
        spacing_gap = spacing_map.get(C, 0)
        return AB, spacing_gap, DE

    def image_callback(self, msg):
        # รับข้อมูลเข้า Memory เฉยๆ
        self.latest_frame = msg.data

    def display_loop(self):
        # โชว์ภาพแบบประหยัดทรัพยากร
        if self.latest_frame is not None:
            np_arr = np.frombuffer(self.latest_frame, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            frame_undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
            
            self.latest_display_frame = frame_undistorted.copy()
            
            cv2.putText(self.latest_display_frame, "STANDBY - Waiting for Service Call", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow("AprilTag Camera", self.latest_display_frame)
            cv2.waitKey(1)

    # ==========================================
    # 4. ฟังก์ชันหลักที่จะทำงานเมื่อถูกเรียกจาก State Machine
    # ==========================================
    def handle_read_tag(self, request, response):
        if self.latest_frame is None:
            response.success = False
            return response

        # ถอดรหัสภาพ
        np_arr = np.frombuffer(self.latest_frame, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        frame_undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)

        # รันโมเดล Detector
        results = self.detector.detect(gray)

        best_tag = None
        max_area = 0
        for tag in results:
            area = cv2.contourArea(tag.corners.astype(np.float32))
            if area > max_area:
                max_area = area
                best_tag = tag

        # กรณีเจอ AprilTag
        if best_tag is not None:
            tag_id = best_tag.tag_id
            AB, C_gap, DE = self.decode_tag(tag_id)

            # เช็คและเซฟไฟล์ CSV
            if tag_id not in self.logged_tags:
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                with open(self.csv_file_path, mode='a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([timestamp, tag_id, AB, C_gap, DE])
                self.logged_tags.add(tag_id)
                self.get_logger().info(f"==> SAVED TO CSV: ID={tag_id} | Plant={AB}cm, Gap={C_gap}cm, Int={DE}cm")

            # เตรียม Response ส่งกลับไป
            response.success = True
            response.tag_id = int(tag_id)
            response.plant_dist = float(AB) / 100.0
            response.gap_dist = float(C_gap) / 100.0
            response.interval_dist = float(DE) / 100.0

        # กรณีหาป้ายไม่เจอ
        else:
            response.success = False

        return response


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagServiceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()