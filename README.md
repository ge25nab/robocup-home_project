# robocup@home_project

This project is a subtask of Robocup@Home competition, it can be used to control Tiago and grasp a bottle at certain position.

## Introduction :cow:

In this project, poeple could point to a specific object, and then ask Tiago to grasp it and hand it over. We detect hand directions by using pose estimation, and then use linear regression to judge which object is pointed at. 

#### Simple principle
![image](https://github.com/ge25nab/robocup-home_project/blob/master/images/out.png)

As shown in the picture, a blue line is detected from left hands points. Objects are highlited with squares and then the wanted object could be selected. 

#### Node Communication  :ant:
![image](https://github.com/ge25nab/robocup-home_project/blob/master/images/node_communucation.png)

The communication between different nodes are shown in the figure above.

## Guidence

**Steps :sunglasses: before compiling and running the code.**

1. install Darknet-ROS from [GitHub Page](https://github.com/leggedrobotics/darknet_ros) as package

2. install OpenPose-ROS from [GitHub Page](https://github.com/CMU-Perceptual-Computing-Lab/openpose) as package

3. install Tiago files from [GitHub Page](https://github.com/pal-robotics) as Tiago Workspace

## Compile  :herb:

1. create a new workspace

2. create ./src folder in this workspace and put all packages from folder and Tiago, Darknet, Openpose to this folder.

3. run command for compiling

```
catkin_make

```
if compile fails, try one more time.

## Run  :frog:

after successfully compiling, run fowlling codes in seperate terminal

#### change maps in Tigao for navigation  :fish:
```
rosservice call /pal_map_manager/change_map "input: 'your_map" "
```

#### run wit for speech recognition
```
roslaunch wit_ros start.launch
```

#### run openpose package for pose estimation
```
roslaunch openpose_ros openpose_ros.launch
```

#### run darknet package for object recognition
```
roslaunch darknet_ros darknet_ros.launch
```

#### run poseget.py for inertaction
```
rosrun wit_ros poseget.py
```
