import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

import cv2
import numpy as np
from ultralytics import YOLO


class GardenYoloNode(Node):

    def __init__(self):
        super().__init__('garden_yolo_node')

        # ===== โหลดโมเดล =====
        self.model = YOLO(
            "/home/aorus-ubun/CMU/P.3/P.3-T.2/271392_Agriculture_Robot_Auto/ros_ws/src/cabbage_detection/models/garden_yolo11n_200.pt"
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
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ===== Parameters =====
        self.forward_speed = 0.25
        self.turn_gain = 1.0 / 300.0
        self.search_speed = 0.3

        self.get_logger().info("✅ Garden YOLO Navigation Started")


    def image_callback(self, msg: CompressedImage):

        # ===== Decode image =====
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            self.get_logger().error("❌ Failed to decode image")
            return

        h, w, _ = frame.shape
        image_center_x = w / 2

        results = self.model(frame, conf=0.5)

        twist = Twist()
        detected_front = False
        detected_side = False

        # ===== ตรวจจับ =====
        for r in results:
            boxes = r.boxes
            if boxes is None:
                continue

            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cls_id = int(box.cls[0])
                label = self.model.names[cls_id]

                # วาดกรอบ
                cv2.rectangle(
                    frame,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    frame,
                    label,
                    (int(x1), int(y1) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2
                )

                if label == "front":
                    detected_front = True

                if label == "side":
                    detected_side = True
                    box_center_x = (x1 + x2) / 2
                    error = box_center_x - image_center_x

        # ===== CONTROL LOGIC =====

        if detected_front:
            # เดินตรงไปเรื่อย ๆ
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0

        elif detected_side:
            # เลี้ยวจัดแนว
            twist.linear.x = 0.0
            twist.angular.z = -error * self.turn_gain

        else:
            # หมุนหา
            twist.linear.x = 0.0
            twist.angular.z = self.search_speed

        # ===== Publish =====
        self.cmd_pub.publish(twist)

        # ===== แสดงค่า cmd_vel บนจอ =====
        cmd_text = f"linear.x: {twist.linear.x:.2f}  angular.z: {twist.angular.z:.2f}"
        cv2.putText(
            frame,
            cmd_text,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2
        )

        # เส้นกลางภาพ
        cv2.line(frame, (int(image_center_x), 0),
                 (int(image_center_x), h), (255, 0, 0), 2)

        cv2.imshow("Garden YOLO Navigation", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = GardenYoloNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()