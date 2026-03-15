import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32
from geometry_msgs.msg import Twist
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # 🌟 ใส่ตัวเลข Calibration (เมตร ต่อ 1 Tick) 🌟
        self.dist_per_tick = 0.000040326
        
        # ระยะห่างระหว่างล้อซ้าย-ขวา (Track Width)
        self.wheel_separation = 0.30

        # ตัวแปรเก็บพิกัดและระยะทาง
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.total_distance = 0.0 

        self.last_ticks = [0, 0, 0, 0]
        self.first_read = True

        # 🌟 รับค่า Ticks สะสมจาก ESP32 🌟
        self.sub_encoder_sum = self.create_subscription(
            Int32MultiArray, '/tao/encoder_sum', self.encoder_callback, 10)
            
        # 🌟 รับคำสั่ง Reset จาก Terminal หรือโปรแกรมอื่น 🌟
        self.sub_reset = self.create_subscription(
            Twist, '/tao/reset_encoder', self.reset_callback, 10)

        # 🌟 Publisher สำหรับส่งระยะทางสะสมออกไป 🌟
        self.pub_distance = self.create_publisher(Float32, '/tao/distance', 10)

        self.get_logger().info('Odometry Node Started. Waiting for encoder data...')

    def reset_callback(self, msg):
        """ รีเซ็ตพิกัดกลับเป็นศูนย์ เมื่อได้รับคำสั่ง Twist ที่มี linear.x > 0.5 """
        if msg.linear.x > 0.5:
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.total_distance = 0.0
            self.first_read = True 
            self.get_logger().info('🔄 RESET COMMAND RECEIVED: Coordinates and Distances have been zeroed.')

    def encoder_callback(self, msg):
        if len(msg.data) != 4:
            return

        current_ticks = list(msg.data)

        if self.first_read:
            self.last_ticks = current_ticks
            self.first_read = False
            return

        # 1. หาผลต่าง Ticks (Delta)
        delta_ticks = [current_ticks[i] - self.last_ticks[i] for i in range(4)]
        
        # 2. แปลง Delta Ticks เป็นระยะทาง (เมตร) ของแต่ละล้อ
        dist = [dt * self.dist_per_tick for dt in delta_ticks]

        # 3. หาค่าเฉลี่ยฝั่งซ้ายและขวา (Differential Drive Approximation)
        d_left = (dist[0] + dist[2]) / 2.0
        d_right = (dist[1] + dist[3]) / 2.0

        # 4. คำนวณระยะทางรวมและมุมที่เปลี่ยนไป (Kinematics)
        d_center = (d_right + d_left) / 2.0
        d_theta = (d_right - d_left) / self.wheel_separation

        # 5. อัปเดตพิกัด X, Y, Theta และระยะสะสมรวม (หน่วยเป็นเมตร)
        self.x += d_center * math.cos(self.theta + (d_theta / 2.0))
        self.y += d_center * math.sin(self.theta + (d_theta / 2.0))
        self.theta += d_theta
        self.total_distance += abs(d_center)

        self.last_ticks = current_ticks

        # พิมพ์โชว์ระยะทางบนจอ (เก็บไว้ดูเป็นเมตรเหมือนเดิมได้)
        self.get_logger().info(f'Distance: {self.total_distance:.3f} m | Pos(X,Y): ({self.x:.3f}, {self.y:.3f}) | Head: {math.degrees(self.theta):.1f}°')

        # 🌟 Publish ระยะทางออกไป (แปลงเป็น cm และตัดเหลือ 3 ตำแหน่ง) 🌟
        dist_msg = Float32()
        distance_cm = self.total_distance * 100.0       # แปลงเมตรเป็นเซนติเมตร
        dist_msg.data = round(distance_cm, 3)           # ปัดเศษทศนิยม 3 ตำแหน่ง
        self.pub_distance.publish(dist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()