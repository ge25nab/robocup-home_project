rosservice call /pal_map_manager/change_map "input: 'luosifen" "
roslaunch wit_ros start.launch
roslaunch openpose_ros openpose_ros.launch
roslaunch darknet_ros darknet_ros.launch
rosrun wit_ros poseget.py

