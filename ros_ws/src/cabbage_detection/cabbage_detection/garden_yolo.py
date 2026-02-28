import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
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

        # ===== Subscribe topic =====
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )

        self.get_logger().info("✅ Garden YOLO Subscriber Started")

    def image_callback(self, msg: CompressedImage):

        # ===== แปลง CompressedImage → OpenCV =====
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            self.get_logger().error("❌ Failed to decode image")
            return

        # ===== Run YOLO =====
        results = self.model(frame, conf=0.5)

        for r in results:
            boxes = r.boxes

            if boxes is None:
                continue

            for box in boxes:

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0])
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

                # ใส่ข้อความ
                text = f"{label} {conf:.2f}"
                cv2.putText(
                    frame,
                    text,
                    (int(x1), int(y1) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2
                )

        # แสดงผล
        cv2.imshow("Garden YOLO Detection", frame)
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