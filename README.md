# 271392_Agriculture_Robot_Auto

## Setup

python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt


ros2 run v4l2_camera v4l2_camera_node \
  --ros-args \
  -p video_device:=/dev/video0 \
  -r image_raw:=/front/image_raw


# run micro-ros agent 

--- ESP 32 DEV MODULE ---

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200 

--- ESP 32 -S3  ---

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 

# เช็ค index cam
v4l2-ctl --list-devices
