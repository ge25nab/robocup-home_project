# robocup-home_project

This project is a subtask of Robocup@Home competition, it can be used to control Tiago and grasp a bottle at certain position.

## Guidence

**Steps :sunglasses: before compiling and running the code.**

1. install Darknet-ROS from [GitHub Page](https://github.com/leggedrobotics/darknet_ros) as package

2. install OpenPose-ROS from [GitHub Page](https://github.com/CMU-Perceptual-Computing-Lab/openpose) as package

3. install Tiago files from [GitHub Page](https://github.com/pal-robotics) as Tiago Workspace

## Compile

1. create a new workspace

2. create ./src folder in this workspace and put all packages from folder and Tiago, Darknet, Openpose to this folder.

3. run command for compiling

``
catkin_make

``
if compile fails, try one more time.

## Run 

after successfully compiling, run fowlling codes in seperate terminal
``
rosservice call /pal_map_manager/change_map "input: 'luosifen" "
''
''
roslaunch wit_ros start.launch
''
''
roslaunch openpose_ros openpose_ros.launch
''
''
roslaunch darknet_ros darknet_ros.launch
''
''
rosrun wit_ros poseget.py
''
