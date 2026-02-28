import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
import cv2


class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')

        # ===== setting =====
        self.width = 640
        self.height = 480
        self.fps = 30
        self.jpeg_quality = 70   # 0-100

        # publisher
        self.publisher_ = self.create_publisher(
            CompressedImage,
            '/image_raw/compressed',
            10
        )

        # open camera
        self.cap = cv2.VideoCapture(4, cv2.CAP_V4L2)

        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        if not self.cap.isOpened():
            self.get_logger().error("Camera not opened")

        self.timer = self.create_timer(
            1.0 / self.fps,
            self.timer_callback
        )

        self.get_logger().info(
            f"Compressed camera {self.width}x{self.height} @ {self.fps} FPS"
        )

    def timer_callback(self):

        ret, frame = self.cap.read()
        if not ret:
            return

        # resize
        frame = cv2.resize(
            frame,
            (self.width, self.height)
        )

        # ===== JPEG compression =====
        encode_param = [
            int(cv2.IMWRITE_JPEG_QUALITY),
            self.jpeg_quality
        ]

        success, encoded_image = cv2.imencode(
            '.jpg',
            frame,
            encode_param
        )

        if not success:
            return

        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = encoded_image.tobytes()

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = CameraPublisher()
    rclpy.spin(node)

    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()