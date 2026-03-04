import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import cv2
import numpy as np

from pupil_apriltags import Detector


class FarmingAprilTagNode(Node):

    def __init__(self):
        super().__init__('farming_apriltag_node')

        # ==========================================
        # 1. ROS Setup (Subscribers & Publishers)
        # ==========================================
        self.sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ==========================================
        # 2. AprilTag Detector Setup
        # ==========================================
        self.detector = Detector(
            families="tagStandard52h13",
            nthreads=4,
            quad_decimate=1.5,
            quad_sigma=0.0,
            refine_edges=True
        )

        # ------------------------------------------
        # พารามิเตอร์กล้องที่ได้จากการ Calibrate ล่าสุด
        # ------------------------------------------
        self.camera_params = [896.35, 895.53, 377.15, 224.76] 
        
        self.camera_matrix = np.array([
            [896.34636732, 0.0, 377.14536611],
            [0.0, 895.52629524, 224.76348467],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.array([
            [-4.33837502e-01, 9.89145910e-01, 2.32584268e-03, 2.38533357e-03, -3.81535553e+00]
        ])

        # ขนาดขอบดำของ AprilTag (หน่วยเป็นเมตร)
        self.tag_size = 0.053

        # ==========================================
        # 3. Robot Kinematics & Tuning Parameters
        # ==========================================
        self.camera_to_front_dist = 0.40  # ระยะจากกล้อง(กลางหุ่น) ถึง กันชนหน้า
        self.front_to_bed_stop_dist = 0.10 # ระยะห่างที่ต้องการให้กันชนหน้าหยุดก่อนถึงแปลง
        
        self.target_distance = self.camera_to_front_dist + self.front_to_bed_stop_dist

        self.kp_linear = 0.5   
        self.kp_angular = 2.0  

        self.max_linear_speed = 0.2   
        self.max_angular_speed = 0.5  

        self.get_logger().info("Farming AprilTag Navigation Node Started...")

    # ===============================
    # ฟังก์ชันถอดรหัสข้อมูลการปลูก
    # ===============================
    def decode_tag(self, tag_id):
        tag_str = str(tag_id).zfill(5)
        AB = int(tag_str[:2])
        C = int(tag_str[2])
        DE = int(tag_str[3:])

        spacing_map = {1: 5, 2: 10, 3: 15, 4: 20, 5: 25}
        spacing_gap = spacing_map.get(C, 0)

        return AB, spacing_gap, DE

    # ===============================
    # Callback ประมวลผลภาพหลัก
    # ===============================
    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # แก้ความเบี้ยวเลนส์
        frame_undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)

        results = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.tag_size
        )

        cmd = Twist()
        tag_detected = False

        # =========================================================
        # แก้ปัญหาลูกบาศก์ (Cubic Tag): หาป้ายที่หันหน้าเข้ากล้อง (พื้นที่ใหญ่สุด)
        # =========================================================
        best_tag = None
        max_area = 0

        for tag in results:
            area = cv2.contourArea(tag.corners.astype(np.float32))
            if area > max_area:
                max_area = area
                best_tag = tag

        # =========================================================
        # ประมวลผลเฉพาะป้ายที่ดีที่สุด (best_tag)
        # =========================================================
        if best_tag is not None:
            tag_detected = True
            tag = best_tag
            tag_id = tag.tag_id

            AB, C_gap, DE = self.decode_tag(tag_id)

            # วาดกรอบสี่เหลี่ยมรอบ Tag (วาดเฉพาะป้ายที่เลือกมาใช้งาน)
            corners = tag.corners.astype(int)
            for i in range(4):
                cv2.line(frame_undistorted, tuple(corners[i]), tuple(corners[(i + 1) % 4]), (0, 255, 0), 3)

            if tag.pose_t is not None:
                tx = tag.pose_t[0][0] 
                ty = tag.pose_t[1][0] 
                tz = tag.pose_t[2][0] 

                # คำนวณความเร็ว (Linear X & Angular Z)
                distance_error = tz - self.target_distance
                linear_x = self.kp_linear * distance_error
                linear_x = max(-self.max_linear_speed, min(self.max_linear_speed, linear_x))

                if abs(distance_error) < 0.05:
                    linear_x = 0.0

                angular_z = -self.kp_angular * tx
                angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))

                if abs(tx) < 0.02:
                    angular_z = 0.0

                cmd.linear.x = float(linear_x)
                cmd.angular.z = float(angular_z)

                # ---------------------------------------------------------
                # ส่วนแสดงผลบนหน้าต่าง OpenCV (GUI)
                # ---------------------------------------------------------
                # 1. แสดงข้อมูลป้ายที่ตัวป้าย
                cx, cy = int(tag.center[0]), int(tag.center[1])
                info_text = f"ID:{tag_id} Plant:{AB} Gap:{C_gap} Int:{DE}"
                cv2.putText(frame_undistorted, info_text, (cx - 100, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                
                dist_text = f"Dist: {tz:.2f}m (Target: {self.target_distance:.2f}m)"
                cv2.putText(frame_undistorted, dist_text, (cx - 100, cy + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                # 2. แสดงความเร็วและทิศทางมุมซ้ายล่างของจอ
                h, w = frame_undistorted.shape[:2] # ดึงขนาดความสูงกว้างของรูปภาพ
                
                speed_text = f"Speed (Lin X): {linear_x:.2f} m/s"
                turn_text  = f"Turn  (Ang Z): {angular_z:.2f} rad/s"
                
                # พิมพ์ข้อความสีเหลืองโดดเด่นที่มุมซ้ายล่าง
                cv2.putText(frame_undistorted, speed_text, (20, h - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(frame_undistorted, turn_text, (20, h - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        if not tag_detected:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            # ถ้าไม่เจอป้าย จะพิมพ์บอกที่หน้าจอด้วย
            cv2.putText(frame_undistorted, "NO TAG DETECTED - STOPPING", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # ส่งคำสั่งให้หุ่นยนต์
        self.cmd_pub.publish(cmd)

        # โชว์ภาพ
        cv2.imshow("Farming AprilTag Navigation", frame_undistorted)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = FarmingAprilTagNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
        
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()