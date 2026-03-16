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

# internet setting 
nmcli device status

# reset endcodeer
ros2 topic pub --once /tao/cmd_resetencoder geometry_msgs/msg/Twist "{linear: {x: 1.0}}"


############################### RUN #################################
# PI 5
ros2 launch agri_bringup pi.launch.py 

# COM
cd ~/CMU/P.3/P.3-T.2/271392_Agriculture_Robot_Auto

source venv/bin/activate

ros2 run agri_vision measurement_node 


---terminal 2---

ros2 launch agri_bringup sever.launch.py 

---terminal 3 ----
ros2 run agri_state_machine state_machine_node 
