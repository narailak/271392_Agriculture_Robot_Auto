from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. รัน PID Controller Node
        Node(
            package='agri_control',
            executable='pid_controller_node',
            name='pid_controller_node',
            output='screen',
            emulate_tty=True
        ),
        # 2. รัน Odometry Node
        Node(
            package='agri_control',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            emulate_tty=True
        ),
        # 3. รัน Auto Planter Node (ปลูกกะหล่ำปลี)
        Node(
            package='cabbage_planter',
            executable='auto_planter_node',
            name='auto_planter_node',
            output='screen',
            emulate_tty=True
        ),
        # 4. รัน AprilTag Node (ระบบ Vision)
        Node(
            package='agri_vision',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            emulate_tty=True
        )
    ])