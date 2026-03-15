import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # กำหนด Parameter พื้นฐาน
    baud_rate = '115200'
    
    return LaunchDescription([
        # Node ตัวที่ 1: สำหรับ /dev/ttyUSB0 (Micro-ROS Agent)
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent_usb0',
            output='screen',
            arguments=['serial', '--dev', '/dev/ttyUSB0', '-b', baud_rate]
        ),
        
        # Node ตัวที่ 2: สำหรับ /dev/ttyACM0 (Micro-ROS Agent)
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent_acm0',
            output='screen',
            arguments=['serial', '--dev', '/dev/ttyACM0', '-b', baud_rate]
        ),

        # Node ตัวที่ 3: Camera Publisher (ระบบ Vision)
        Node(
            package='agri_vision',
            executable='camera_publisher_node',
            name='camera_publisher_node',
            output='screen',
            emulate_tty=True
        )
    ])