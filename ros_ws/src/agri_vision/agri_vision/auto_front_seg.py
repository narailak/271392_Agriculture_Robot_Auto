import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

import cv2
import numpy as np
from ultralytics import YOLO

class FrontCenterNavigator(Node):
    def __init__(self):
        super().__init__('front_center_navigator')

        # ===== YOLO Setup =====
        self.model_path = "/home/aorus-ubun/CMU/P.3/P.3-T.2/271392_Agriculture_Robot_Auto/ros_ws/src/agri_vision/models/segment3class.pt"
        self.model = YOLO(self.model_path)
        self.conf_threshold = 0.60        
        
        # ===== ROS Communications =====
        self.sub = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/tao/cmd_vel', 10)
        self.servo_pub = self.create_publisher(Int16, '/tao/cmd_servo_cam', 10)

        # ===== 🌟 Vehicle Control Parameters (PD + Angle Control) =====
        self.base_linear_speed = 0.35     
        self.max_angular_speed = 0.8      
        
        self.p_gain_x = 0.0035            # ดึงหุ่นเข้าหาเส้นกลาง
        self.d_gain_x = 0.0060            # เบรกกันหุ่นส่าย
        self.angle_gain = 0.60            # 🌟 พลังในการ "ตบออกเพื่อตั้งลำ" (ยิ่งเยอะยิ่งตีวงกว้าง)
        
        self.smoothing_factor = 0.30      
        self.last_angular_z = 0.0
        self.last_error_x = 0.0           
        
        # ===== Servo Control Parameters =====
        self.servo_angle = 100       
        self.sweep_step = 2          
        self.track_step = 1          
        self.servo_dir = 1           
        self.frame_counter = 0
        self.lost_frames = 0
        
        self.publish_servo(self.servo_angle)
        self.get_logger().info("🚀 Auto-Align Navigator (S-Curve Smasher) Started!")

    def publish_servo(self, angle):
        msg = Int16()
        msg.data = int(angle)
        self.servo_pub.publish(msg)

    def image_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None: return

        h, w = frame.shape[:2]
        center_x, center_y = w // 2, h // 2
        
        tolerance_x = 20 # 🌟 บีบเป้าให้แคบลง บังคับให้หุ่นต้องตั้งหน้าตรงเป๊ะถึงจะซิ่งได้
        self.frame_counter += 1

        results = self.model(frame, conf=self.conf_threshold, verbose=False)
        
        front_target = None
        side_target = None
        
        target_x = None
        target_y = None
        target_angle_rad = 0.0 # 🌟 ตัวแปรเก็บองศาแปลง
        active_target_label = None
        twist = Twist()

        for r in results:
            if r.boxes is None or r.masks is None: continue 
            
            for box, mask_pts in zip(r.boxes, r.masks.xy):
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = self.model.names[cls_id]
                
                contour = np.array(mask_pts, dtype=np.int32)
                
                if len(contour) < 10: continue # ถ้า Mask เล็กไปให้ข้าม

                # 🌟 1. หาจุดกึ่งกลางแปลง
                M = cv2.moments(contour)
                cx = int(M["m10"] / M["m00"]) if M["m00"] != 0 else int(box.xyxy[0][0])
                cy = int(M["m01"] / M["m00"]) if M["m00"] != 0 else int(box.xyxy[0][1])

                # 🌟 2. หาองศาความเอียงของร่องแปลง (เพื่อทำ S-Curve)
                # ดึงจุดบนสุดและล่างสุดของ Mask ในจอ
                top_pt = contour[contour[:, :, 1].argmin()][0]
                bot_pt = contour[contour[:, :, 1].argmax()][0]
                
                # คำนวณความชัน (มุม)
                dx = top_pt[0] - bot_pt[0]
                dy = bot_pt[1] - top_pt[1] # สลับแกน y ให้เป็นบวกเสมอ
                angle_rad = np.arctan2(dx, dy) 
                
                # วาดเส้นแกนกลางแปลงสีชมพู
                cv2.line(frame, (bot_pt[0], bot_pt[1]), (top_pt[0], top_pt[1]), (255, 0, 255), 2)
                cv2.circle(frame, tuple(top_pt), 6, (0, 255, 255), -1) # จุดเล็งบนสุด

                if label == "front":
                    color = (0, 255, 0)
                    if front_target is None: 
                        front_target = (cx, cy) 
                        target_angle_rad = angle_rad
                elif label == "side":
                    color = (0, 165, 255) 
                    if side_target is None: 
                        side_target = (cx, cy) 
                        target_angle_rad = angle_rad
                else:
                    color = (100, 100, 100) 

                cv2.drawContours(frame, [contour], -1, color, 2)
                cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)
                cv2.putText(frame, f"{label} {conf:.2f}", (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        if front_target is not None:
            target_x, target_y = front_target
            active_target_label = "FRONT"
        elif side_target is not None:
            target_x, target_y = side_target
            active_target_label = "SIDE"

        if active_target_label is not None:
            self.lost_frames = 0
            
            # --- ก. คุม Servo ก้ม-เงย ---
            error_y = target_y - center_y
            if self.frame_counter % 6 == 0:
                if error_y < -40:   self.servo_angle -= self.track_step
                elif error_y > 40:  self.servo_angle += self.track_step
                self.servo_angle = max(50, min(160, self.servo_angle))
                self.publish_servo(self.servo_angle)

            # --- 🌟 ข. คุมล้อซ้าย-ขวา ด้วย PD + Angle Controller (พระเอกของเรา) ---
            error_x = target_x - center_x
            derivative_x = error_x - self.last_error_x 
            
            if abs(error_x) < tolerance_x and abs(target_angle_rad) < 0.15: # ต้องตรงกลาง และหน้าไม่เอียง
                new_angular_z = 0.0
                twist.linear.x = self.base_linear_speed
                cv2.putText(frame, "PERFECT ALIGNMENT!", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                # 🌟 สมการนี้คือการตบซ้ายตบขวาอัตโนมัติ:
                # ถ้าระยะห่าง (error_x) สั่งเลี้ยวซ้าย แต่องศาเอียงมาก (angle_rad) สั่งว่าต้องตั้งลำก่อน
                # พลังของ Angle จะไปหักล้าง ทำให้หุ่นยนต์ "ตบขวา" นิดๆ เพื่อตีวง ก่อนที่จะพุ่งเข้าหาแปลง
                new_angular_z = -(error_x * self.p_gain_x) - (target_angle_rad * self.angle_gain) - (derivative_x * self.d_gain_x)
                new_angular_z = max(min(new_angular_z, self.max_angular_speed), -self.max_angular_speed)
                
                # เบรกเดินหน้าให้แรงขึ้นเวลาที่ยังตั้งลำไม่เสร็จ (ให้เวลาล้อหมุนตบหน้าให้ตรงก่อน)
                speed_penalty = (abs(error_x) / (w/2)) * 2.5 
                twist.linear.x = self.base_linear_speed * max(0.0, (1.0 - speed_penalty))
                twist.linear.x = max(0.05, twist.linear.x) # ให้ขยับตัวช้าๆ ห้ามหยุดนิ่ง
                
                cv2.putText(frame, "ALIGNING S-CURVE", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

            twist.angular.z = (self.smoothing_factor * new_angular_z) + ((1.0 - self.smoothing_factor) * self.last_angular_z)
            self.last_error_x = error_x 

            # โชว์ค่าให้เห็นบนจอ
            cv2.putText(frame, f"Ang_Err: {np.degrees(target_angle_rad):.1f} deg", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 255), 2)

        else:
            self.lost_frames += 1
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.last_error_x = 0.0 
            
            if self.lost_frames > 15:
                if self.frame_counter % 10 == 0:
                    self.servo_angle += (self.sweep_step * self.servo_dir)
                    if self.servo_angle >= 160: self.servo_angle = 160; self.servo_dir = -1
                    elif self.servo_angle <= 50: self.servo_angle = 50; self.servo_dir = 1
                    self.publish_servo(self.servo_angle)
                cv2.putText(frame, "SEARCHING...", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        self.last_angular_z = twist.angular.z
        self.cmd_pub.publish(twist)

        cv2.line(frame, (int(center_x - tolerance_x), 0), (int(center_x - tolerance_x), h), (255, 255, 0), 1)
        cv2.line(frame, (int(center_x + tolerance_x), 0), (int(center_x + tolerance_x), h), (255, 255, 0), 1)
        
        cv2.imshow("Pro S-Curve Navigator", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = FrontCenterNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist()) 
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()