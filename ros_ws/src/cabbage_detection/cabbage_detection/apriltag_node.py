import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

from pupil_apriltags import Detector


class AprilTagNode(Node):

    def __init__(self):
        super().__init__('apriltag_node')

        # subscriber
        self.sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )

        # apriltag detector
        self.detector = Detector(
            families="tag52h13",
            nthreads=4,
            quad_decimate=1.5,
            quad_sigma=0.0,
            refine_edges=True
        )

        self.get_logger().info("AprilTag node started")

    # ===============================
    # decode farming tag logic
    # ===============================
    def decode_tag(self, tag_id):

        tag_str = str(tag_id).zfill(5)

        AB = int(tag_str[:2])
        C = int(tag_str[2])
        DE = int(tag_str[3:])

        spacing_map = {
            1: 5,
            2: 10,
            3: 15,
            4: 20,
            5: 25
        }

        spacing_gap = spacing_map.get(C, 0)

        return AB, spacing_gap, DE

    # ===============================
    def image_callback(self, msg):

        # decode compressed image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        results = self.detector.detect(gray)

        for tag in results:

            tag_id = tag.tag_id

            AB, C_gap, DE = self.decode_tag(tag_id)

            corners = tag.corners.astype(int)

            # draw box
            for i in range(4):
                cv2.line(
                    frame,
                    tuple(corners[i]),
                    tuple(corners[(i + 1) % 4]),
                    (0, 255, 0),
                    2
                )

            cx, cy = int(tag.center[0]), int(tag.center[1])

            text = f"ID:{tag_id} AB:{AB}cm C:{C_gap}cm DE:{DE}cm"

            cv2.putText(
                frame,
                text,
                (cx - 120, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2
            )

            # log output
            self.get_logger().info(
                f"Tag {tag_id} -> Plant:{AB} Gap:{C_gap} Interval:{DE}"
            )

        # show window
        cv2.imshow("AprilTag Detection", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = AprilTagNode()
    rclpy.spin(node)

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()