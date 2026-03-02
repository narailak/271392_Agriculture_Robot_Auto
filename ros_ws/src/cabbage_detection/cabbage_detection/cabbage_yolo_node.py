#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import time
import numpy as np
from ultralytics import YOLO

class CabbageFactoryNode(Node):
    def __init__(self):
        # ตั้งชื่อ Node
        super().__init__('cabbage_yolo_node')
        
        # --- 1. SETUP THE BRAIN ---
        #  อัปเดต PATH ให้ตรงกับเครื่องของคุณ
        model_path = "/home/aorus-ubun/CMU/P.3/P.3-T.2/271392_Agriculture_Robot_Auto/ros_ws/src/cabbage_detection/models/best.pt"
        self.model = YOLO(model_path)

        # --- 2. ENGINEERING CONSTANTS ---
        self.PIXEL_TO_CM = 0.05 
        self.COOLDOWN_SECONDS = 5
        self.ZONE_SIZE = 30 
        self.last_log_time = 0

        # เตรียมตัวแปรจุดกึ่งกลางจอ (จะคำนวณเมื่อได้ภาพเฟรมแรก)
        self.cam_center_x = None
        self.cam_center_y = None

        # --- 3. PREPARE THE LOG FILE ---
        with open("cabbage_log.csv", "a") as log_file:
            log_file.write("Date_Time,Diameter_cm,Method\n")

        self.get_logger().info("King Coco's Automated Factory is ONLINE (ROS 2)... Waiting for /image_raw/compressed")

        # --- 4. SUBSCRIBE TO CAMERA ---
        # Subscribe หัวข้อ /image_raw/compressed
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10 # Queue size
        )

    def get_cabbage_metrics(self, frame, box_coords, ratio):
        """
        Finds the exact center and diameter using a Median Filter + Hough Circle.
        Falls back to simple Box math if no perfect circle is found.
        """
        x1, y1, x2, y2 = map(int, box_coords)
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0: return None, None, None, "NONE"

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.medianBlur(gray, 7)

        circles = cv2.HoughCircles(
            blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
            param1=50, param2=35, minRadius=20, maxRadius=200
        )

        if circles is not None:
            circle = np.round(circles[0, 0]).astype("int")
            abs_cx = x1 + circle[0]
            abs_cy = y1 + circle[1]
            dia_cm = (circle[2] * 2) * ratio
            return abs_cx, abs_cy, dia_cm, "ROUND"

        fb_cx = int((x1 + x2) / 2)
        fb_cy = int((y1 + y2) / 2)
        fb_dia = (x2 - x1) * ratio
        return fb_cx, fb_cy, fb_dia, "BOX"

    def image_callback(self, msg):
        # แปลง CompressedImage ให้เป็น OpenCV Image (ROS 2)
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"Failed to decode image: {e}")
            return

        # เซ็ตค่าจุดกึ่งกลางจอในครั้งแรกที่ได้รับภาพ
        if self.cam_center_x is None:
            cam_height, cam_width = frame.shape[:2]
            self.cam_center_x = cam_width // 2
            self.cam_center_y = cam_height // 2

        # --- 5. MAIN FACTORY PROCESSING ---
        results = self.model(frame, verbose=False)
        annotated_frame = results[0].plot()

        # Draw the green targeting Killzone
        cv2.rectangle(annotated_frame, 
                      (self.cam_center_x - self.ZONE_SIZE, self.cam_center_y - self.ZONE_SIZE), 
                      (self.cam_center_x + self.ZONE_SIZE, self.cam_center_y + self.ZONE_SIZE), 
                      (0, 255, 0), 2)
        cv2.putText(annotated_frame, "SCAN ZONE", (self.cam_center_x - 40, self.cam_center_y - 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        for box in results[0].boxes:
            if int(box.cls[0]) == 0: # Cabbage detected
                coords = box.xyxy[0].tolist()
                
                # --- PULL THE MODULAR MATH BAG ---
                cx, cy, dia, method = self.get_cabbage_metrics(frame, coords, self.PIXEL_TO_CM)
                
                if cx is None: continue

                # Draw the Red Dot and Text right on the cabbage
                cv2.circle(annotated_frame, (cx, cy), 5, (0, 0, 255), -1)
                text = f"Size: {dia:.1f}cm ({method})"
                cv2.putText(annotated_frame, text, (int(coords[0]), int(coords[1]) - 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                # --- AUTOMATED LOGGING TRIGGER ---
                in_zone_x = abs(cx - self.cam_center_x) < self.ZONE_SIZE
                in_zone_y = abs(cy - self.cam_center_y) < self.ZONE_SIZE
                
                if in_zone_x and in_zone_y:
                    current_time = time.time()
                    if current_time - self.last_log_time > self.COOLDOWN_SECONDS:
                        current_clock = time.strftime('%Y-%m-%d %H:%M:%S')
                        with open("cabbage_log.csv", "a") as f:
                            f.write(f"{current_clock},{dia:.1f},{method}\n")
                        
                        self.get_logger().info(f"BOOM! Cabbage logged: {dia:.1f}cm using {method}")
                        
                        cv2.rectangle(annotated_frame, 
                                      (self.cam_center_x - self.ZONE_SIZE, self.cam_center_y - self.ZONE_SIZE), 
                                      (self.cam_center_x + self.ZONE_SIZE, self.cam_center_y + self.ZONE_SIZE), 
                                      (0, 0, 255), -1)
                        
                        self.last_log_time = current_time

        # แสดงผลภาพ
        cv2.imshow("King Coco's Cabbage Vision", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CabbageFactoryNode()
    try:
        rclpy.spin(node) # สั่งให้ Node รันแบบวนลูปรับภาพ
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()