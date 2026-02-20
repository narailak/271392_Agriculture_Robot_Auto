# 271392_Agriculture_Robot_Auto

## Setup

python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt


ros2 run v4l2_camera v4l2_camera_node \
  --ros-args \
  -p video_device:=/dev/video0 \
  -r image_raw:=/front/image_raw
