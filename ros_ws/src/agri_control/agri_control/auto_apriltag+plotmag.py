import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

import cv2
import numpy as np
import math
import time
from pupil_apriltags import Detector

class FarmingAprilTagNode(Node):

    def __init__(self):
        super().__init__('farming_apriltag_node')

        self.sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/tao/cmd_vel', 10)
        self.servo_pub = self.create_publisher(Int16, '/tao/cmd_servo_cam', 10) 

        self.detector = Detector(
            families="tagStandard52h13",
            nthreads=4,
            quad_decimate=1.5, 
            quad_sigma=0.0,
            refine_edges=True
        )

        self.camera_params = [896.35, 895.53, 377.15, 224.76] 
        self.camera_matrix = np.array([
            [896.34636732, 0.0, 377.14536611],
            [0.0, 895.52629524, 224.76348467],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.array([
            [-4.33837502e-01, 9.89145910e-01, 2.32584268e-03, 2.38533357e-03, -3.81535553e+00]
        ])

        self.tag_size = 0.053
        self.robot_to_servo_z = 0.15 
        self.target_distance = 0.20  
        
        self.kp_linear = 0.6   
        self.kp_yaw = 2.0       
        self.max_linear_speed = 0.15   
        self.max_angular_speed = 0.8 # ปรับเพิ่มนิดหน่อยให้หักเข้าแกนได้ไวขึ้น

        self.servo_angle = 100       
        self.sweep_step = 5          
        self.track_step = 5          
        self.servo_dir = 1          
        self.frame_counter = 0      
        self.lost_frames = 0        

        # Tracking Variables
        self.path_generated = False
        self.planned_path = []      
        self.actual_path = []       
        
        self.start_rx = 0.0
        self.start_rz = 0.0
        self.start_yaw_deg = 0.0

        self.smooth_rx = None
        self.smooth_rz = None
        self.smooth_yaw = None
        self.alpha = 0.15 

        self.publish_servo(self.servo_angle)
        self.get_logger().info("Farming Node: Aggressive Axis Alignment (Straight Approach) Active.")

    def transform_to_robot_frame(self, tx, ty, tz, servo_angle_deg):
        theta = math.radians(90 - servo_angle_deg) 
        z_ground = (tz * math.cos(theta)) - (ty * math.sin(theta))
        y_ground = (tz * math.sin(theta)) + (ty * math.cos(theta))
        rz = z_ground + self.robot_to_servo_z
        return tx, y_ground, rz

    def publish_servo(self, angle):
        msg = Int16() 
        msg.data = int(angle)
        self.servo_pub.publish(msg)

    def generate_bezier_path(self, rx, rz, yaw_angle, num_points=30):
        P0 = np.array([0.0, 0.0])
        target_x = rx - self.target_distance * math.sin(yaw_angle)
        target_z = rz - self.target_distance * math.cos(yaw_angle)
        P3 = np.array([target_x, target_z])

        # ปรับ P1 และ P2 ให้ตรงกับพฤติกรรม "ตบเข้าแกนแล้วตรง"
        early_z = rz * 0.25 
        dist_from_tag_to_p2 = rz - early_z
        P2 = np.array([rx - dist_from_tag_to_p2 * math.sin(yaw_angle), early_z])
        P1 = np.array([P2[0] * 0.8, early_z * 0.5])

        path = []
        for t in np.linspace(0, 1, num_points):
            pt = (1-t)**3 * P0 + 3*(1-t)**2 * t * P1 + 3*(1-t) * t**2 * P2 + t**3 * P3
            path.append((pt[0], pt[1]))
        return path

    def save_path_map(self, rx, rz, yaw_angle):
        map_size = 500
        margin = 50
        scale = 200 
        map_img = np.ones((map_size, map_size, 3), dtype=np.uint8) * 255 

        def to_px(x_m, z_m):
            px_x = int(map_size/2 - (x_m * scale))
            px_y = int(map_size - margin - (z_m * scale))
            return (px_x, px_y)

        cv2.line(map_img, (map_size//2, 0), (map_size//2, map_size), (230,230,230), 1)
        for i in range(1, 5):
            y_line = map_size - margin - (i * 100)
            cv2.line(map_img, (0, y_line), (map_size, y_line), (230,230,230), 1)

        if len(self.planned_path) > 0:
            for i in range(len(self.planned_path)-1):
                pt1 = to_px(self.planned_path[i][0], self.planned_path[i][1])
                pt2 = to_px(self.planned_path[i+1][0], self.planned_path[i+1][1])
                cv2.line(map_img, pt1, pt2, (0, 200, 0), 3)

        robot_px = to_px(0, 0)
        cv2.circle(map_img, robot_px, 8, (0, 0, 0), -1)
        cv2.putText(map_img, "ROBOT", (robot_px[0]+10, robot_px[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)

        tag_px = to_px(rx, rz)
        cv2.drawMarker(map_img, tag_px, (0, 0, 255), cv2.MARKER_SQUARE, 20, 3)
        cv2.putText(map_img, "TARGET", (tag_px[0]+15, tag_px[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

        info_text = f"Dist: {rz:.2f}m, Offset: {rx:.2f}m"
        cv2.putText(map_img, info_text, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)

        filename = f"planned_path_{time.strftime('%Y%m%d_%H%M%S')}.png"
        cv2.imwrite(filename, map_img)

    def draw_mini_map(self, img, curr_robot_x, curr_robot_z):
        h, w = img.shape[:2]
        map_size = 200
        margin = 20
        overlay = img.copy()
        cv2.rectangle(overlay, (w - map_size - margin, margin), (w - margin, map_size + margin), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, img, 0.4, 0, img)
        cv2.putText(img, "Path Tracker", (w - map_size - margin + 5, margin + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

        def to_map_px(x_m, z_m):
            scale = 80 
            px_x = int((w - map_size/2 - margin) - (x_m * scale))
            px_y = int((margin + map_size - 20) - (z_m * scale))
            return (px_x, px_y)

        target_px = to_map_px(self.start_rx, self.start_rz)
        cv2.drawMarker(img, target_px, (0, 0, 255), cv2.MARKER_SQUARE, 10, 2)

        if len(self.planned_path) > 0:
            for i in range(len(self.planned_path) - 1):
                pt1 = to_map_px(self.planned_path[i][0], self.planned_path[i][1])
                pt2 = to_map_px(self.planned_path[i+1][0], self.planned_path[i+1][1])
                cv2.line(img, pt1, pt2, (0, 255, 0), 2)

        if len(self.actual_path) > 1:
            for i in range(len(self.actual_path) - 1):
                pt1 = to_map_px(self.actual_path[i][0], self.actual_path[i][1])
                pt2 = to_map_px(self.actual_path[i+1][0], self.actual_path[i+1][1])
                cv2.line(img, pt1, pt2, (0, 255, 255), 2)

        robot_px = to_map_px(curr_robot_x, curr_robot_z)
        cv2.circle(img, robot_px, 5, (0, 255, 255), -1)

    def draw_overlays(self, img):
        h, w = img.shape[:2]
        y_min, y_max = int(h * 0.3), int(h * 0.7)
        cv2.line(img, (0, y_min), (w, y_min), (0, 255, 255), 1)
        cv2.line(img, (0, y_max), (w, y_max), (0, 255, 255), 1)

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        frame_undist = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        
        gray = cv2.cvtColor(frame_undist, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray, estimate_tag_pose=True, 
                                     camera_params=self.camera_params, tag_size=self.tag_size)

        best_tag = max(results, key=lambda t: cv2.contourArea(t.corners.astype(np.float32)), default=None)

        self.draw_overlays(frame_undist)
        cmd = Twist()
        self.frame_counter += 1
        h, w = frame_undist.shape[:2]

        if best_tag is not None:
            self.lost_frames = 0
            cx_tag, cy_tag = int(best_tag.center[0]), int(best_tag.center[1])
            
            if self.frame_counter % 5 == 0:
                y_min_thresh, y_max_thresh = h * 0.3, h * 0.7
                if cy_tag < y_min_thresh:
                    self.servo_angle -= self.track_step
                    self.publish_servo(max(50, min(160, self.servo_angle)))
                elif cy_tag > y_max_thresh:
                    self.servo_angle += self.track_step
                    self.publish_servo(max(50, min(160, self.servo_angle)))

            if best_tag.pose_t is not None:
                raw_tx, raw_ty, raw_tz = best_tag.pose_t.flatten()
                raw_rx, raw_ry, raw_rz = self.transform_to_robot_frame(raw_tx, raw_ty, raw_tz, self.servo_angle)
                raw_yaw_deg = math.degrees(math.atan2(best_tag.pose_R[0, 2], best_tag.pose_R[2, 2]))

                if self.smooth_rx is None:
                    self.smooth_rx, self.smooth_rz, self.smooth_yaw = raw_rx, raw_rz, raw_yaw_deg
                else:
                    self.smooth_rx = self.alpha * raw_rx + (1 - self.alpha) * self.smooth_rx
                    self.smooth_rz = self.alpha * raw_rz + (1 - self.alpha) * self.smooth_rz
                    self.smooth_yaw = self.alpha * raw_yaw_deg + (1 - self.alpha) * self.smooth_yaw

                rx, rz, yaw_deg = self.smooth_rx, self.smooth_rz, self.smooth_yaw
                yaw_angle = math.radians(yaw_deg)
                
                if not self.path_generated:
                    self.start_rx, self.start_rz, self.start_yaw_deg = rx, rz, yaw_deg
                    self.planned_path = self.generate_bezier_path(rx, rz, yaw_angle)
                    self.actual_path = [(0.0, 0.0)]
                    self.path_generated = True
                    self.save_path_map(rx, rz, yaw_angle)
                    curr_robot_x, curr_robot_z = 0.0, 0.0
                else:
                    theta_r = math.radians(self.start_yaw_deg - yaw_deg)
                    rot_x = rx * math.cos(theta_r) + rz * math.sin(theta_r)
                    rot_z = -rx * math.sin(theta_r) + rz * math.cos(theta_r)
                    curr_robot_x = self.start_rx - rot_x
                    curr_robot_z = self.start_rz - rot_z
                    
                    if len(self.actual_path) > 0:
                        last_x, last_z = self.actual_path[-1]
                        if math.hypot(curr_robot_x - last_x, curr_robot_z - last_z) > 0.01: 
                            self.actual_path.append((curr_robot_x, curr_robot_z))

                self.draw_mini_map(frame_undist, curr_robot_x, curr_robot_z)

                # =========================================================
                # 🎯 NEW LOGIC: Straight Approach (ตบเข้าแกนให้ไว แล้วเดินตรง)
                # =========================================================
                z_err = rz - self.target_distance
                is_arrived = abs(z_err) < 0.05 

                if is_arrived:
                    cmd.linear.x = 0.0; cmd.angular.z = 0.0
                else:
                    cmd.linear.x = float(np.clip(self.kp_linear * z_err, -self.max_linear_speed, self.max_linear_speed))
                    
                    # 1. เล็งเป้าให้หักเข้าหาเส้นกลาง (Centerline) ด้วยระยะ Lookahead สั้นๆ
                    dynamic_lookahead = 0.25 # ยิ่งสั้น ยิ่งหักพวงมาลัยแรง
                    angle_to_center = math.atan2(-rx, dynamic_lookahead)
                    
                    # 2. ปรับน้ำหนัก: ถ้าเบี้ยวเยอะให้หักเข้าแกนก่อน ถ้าเข้าแกนแล้วให้หน้าตรง (yaw=0)
                    alignment_weight = np.clip(abs(rx) * 6.0, 0.0, 1.0) 
                    
                    target_yaw = (alignment_weight * angle_to_center) + ((1.0 - alignment_weight) * 0.0)
                    yaw_diff = target_yaw - yaw_angle
                    
                    cmd.angular.z = float(np.clip(self.kp_yaw * yaw_diff, -self.max_angular_speed, self.max_angular_speed))

                pts = best_tag.corners.astype(int)
                for i in range(4):
                    cv2.line(frame_undist, tuple(pts[i]), tuple(pts[(i+1)%4]), (0, 255, 0), 2)
        
        else:
            self.lost_frames += 1
            cmd.linear.x = 0.0; cmd.angular.z = 0.0
            
            if self.lost_frames > 30:
                self.path_generated = False
                self.planned_path = []
                self.actual_path = []
                self.smooth_rx, self.smooth_rz, self.smooth_yaw = None, None, None

            if self.lost_frames > 15:
                if self.frame_counter % 10 == 0:
                    self.servo_angle += (self.sweep_step * self.servo_dir)
                    if self.servo_angle >= 160: self.servo_dir = -1
                    elif self.servo_angle <= 50: self.servo_dir = 1
                    self.publish_servo(self.servo_angle)

        self.cmd_pub.publish(cmd)
        cv2.imshow("Farming AprilTag Navigation", frame_undist)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = FarmingAprilTagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.cmd_pub.publish(Twist())
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()