import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage # 1. นำเข้า CompressedImage
import cv2
import numpy as np
import math
from pupil_apriltags import Detector 

class AprilTagRPYDetector(Node):

    def __init__(self):
        super().__init__('apriltag_rpy_detector')
        
        # ===== 📷 Camera Calibration Data =====
        self.camera_params = [942.01, 945.76, 445.51, 233.71] 
        self.camera_matrix = np.array([
            [942.00959306, 0.0, 445.50898168],
            [0.0, 945.76258164, 233.70507804],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.array([
            [-0.41678417, 0.26653241, 0.00208138, -0.00701684, -0.14829814]
        ])

        # เนื่องจากเราหา AprilTag บนภาพที่ Undistort ไปแล้ว 
        # ตอนวาดเส้น 3D กลับลงไป จึงไม่ต้องชดเชยความโค้งอีก
        self.zero_dist_coeffs = np.zeros((4, 1))

        # ===== Setup AprilTag =====
        self.tag_size = 0.036
        self.detector = Detector(
            families="tagStandard52h13", 
            nthreads=4,
            quad_decimate=1.5,
            quad_sigma=0.0,
            refine_edges=True
        )

        # ===== 📹 Subscriber (รับภาพจาก Topic) =====
        self.subscription = self.create_subscription(
            CompressedImage, 
            '/image_raw/compressed', 
            self.image_callback, 
            10
        )
        self.get_logger().info("🚀 AprilTag RPY Node Started (Topic Subscriber Mode)")

    def rotation_matrix_to_euler_angles(self, R):
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6

        if not singular:
            roll = math.atan2(R[2, 1], R[2, 2])
            pitch = math.atan2(-R[2, 0], sy)
            yaw = math.atan2(R[1, 0], R[0, 0])
        else:
            roll = math.atan2(-R[1, 2], R[1, 1])
            pitch = math.atan2(-R[2, 0], sy)
            yaw = 0

        roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        yaw = math.degrees(yaw)

        return roll, pitch, yaw

    def draw_3d_axes(self, img, R, tvec):
        """
        วาดแกน 3 มิติ (X, Y, Z) บน AprilTag
        """
        # แปลง Rotation Matrix เป็น Rotation Vector ตามที่ OpenCV ต้องการ
        rvec, _ = cv2.Rodrigues(R)
        
        # กำหนดความยาวของเส้นแกน (อ้างอิงจากขนาด tag)
        axis_length = self.tag_size 
        
        # จุดพิกัด 3 มิติ (สมมติให้จุดศูนย์กลางแท็กคือ 0,0,0)
        object_points = np.array([
            [0.0, 0.0, 0.0],               # จุดกำเนิด (กึ่งกลาง)
            [axis_length, 0.0, 0.0],       # ปลายแกน X
            [0.0, axis_length, 0.0],       # ปลายแกน Y
            [0.0, 0.0, axis_length]        # ปลายแกน Z (พุ่งออกจากแท็ก)
        ], dtype=np.float32)

        # ฉายภาพจุด 3 มิติ ลงบนภาพ 2 มิติ
        image_points, _ = cv2.projectPoints(
            object_points, rvec, tvec, self.camera_matrix, self.zero_dist_coeffs
        )
        image_points = image_points.astype(int).reshape(-1, 2)

        # พิกัดบนภาพ
        origin = tuple(image_points[0])
        pt_x = tuple(image_points[1])
        pt_y = tuple(image_points[2])
        pt_z = tuple(image_points[3])

        #
def main(args=None):
    rclpy.init(args=args)
    node = AprilTagRPYDetector()
    
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