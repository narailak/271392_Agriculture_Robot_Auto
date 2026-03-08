import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

import cv2
import numpy as np
from ultralytics import YOLO

class GardenYoloSegNode(Node):

    def __init__(self):
        super().__init__('garden_yolo_seg_node')

        # ===== โหลดโมเดล Segmentation =====
        # ข้อควรระวัง: ต้องแน่ใจว่าไฟล์โมเดลที่เทรนมาเป็นแบบ Segmentation (มักจะลงท้ายด้วย -seg.pt)
        self.model = YOLO(
            "/home/aorus-ubun/CMU/P.3/P.3-T.2/271392_Agriculture_Robot_Auto/ros_ws/src/cabbage_detection/models/cabbage_plot_segV1.pt"
        )

        self.bridge = CvBridge()

        # ===== Subscribe กล้อง =====
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )

        # ===== Publisher cmd_vel =====
        self.cmd_pub = self.create_publisher(Twist, '/tao/cmd_vel', 10)

        # ===== Parameters =====
        self.forward_speed = 0.25
        self.turn_gain = 1.0 / 300.0
        self.search_speed = 0.3

        self.get_logger().info("✅ Garden YOLOv8 Segmentation Navigation Started")


    def image_callback(self, msg: CompressedImage):

        # ===== Decode image =====
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            self.get_logger().error("❌ Failed to decode image")
            return

        h, w, _ = frame.shape
        image_center_x = w / 2.0

        # ใช้โมเดล Predict (ตั้ง conf ตามต้องการ)
        results = self.model(frame, conf=0.5)

        twist = Twist()
        detected_front = False
        detected_side = False
        error = 0.0

        annotated_frame = frame.copy()

        # ===== ตรวจจับและวิเคราะห์ =====
        for r in results:
            # วาด Mask, Box และ Label อัตโนมัติด้วย r.plot()
            annotated_frame = r.plot()

            boxes = r.boxes
            if boxes is None:
                continue

            for box in boxes:
                # ดึงพิกัด Bounding Box
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cls_id = int(box.cls[0])
                label = self.model.names[cls_id]

                if label == "front":
                    detected_front = True

                if label == "side":
                    detected_side = True
                    box_center_x = (x1 + x2) / 2.0
                    error = box_center_x - image_center_x

        # ===== CONTROL LOGIC =====
        if detected_front:
            # เดินตรงไปเรื่อย ๆ
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0

        elif detected_side:
            # เลี้ยวจัดแนว (หา side เจอ แต่ไม่เจอ front)
            twist.linear.x = 0.0
            twist.angular.z = -error * self.turn_gain

        else:
            # หมุนหา (ไม่เจออะไรเลย)
            twist.linear.x = 0.0
            twist.angular.z = self.search_speed

        # ===== Publish =====
        self.cmd_pub.publish(twist)

        # ===== แสดงค่า cmd_vel บนจอภาพ =====
        cmd_text = f"linear.x: {twist.linear.x:.2f}  angular.z: {twist.angular.z:.2f}"
        cv2.putText(
            annotated_frame,
            cmd_text,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2
        )

        # เส้นกลางภาพเพื่อเป็น Reference Point
        cv2.line(annotated_frame, (int(image_center_x), 0),
                 (int(image_center_x), h), (255, 0, 0), 2)

        # แสดงผลภาพที่ครอบ Mask และ Box มาแล้ว
        cv2.imshow("Garden YOLO-Seg Navigation", annotated_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = GardenYoloSegNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()