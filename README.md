# soft_gripper_page_turning
ROS package: Page turning by soft robotics gripper with UR10 robot arm. This project is to turning paper or paper strip by 3D printed 
soft robtoics gripper. The codes has 3 major parts: soft gripper actuation by Arduino and electro-pneumatic regulator; object detection
by in palm camera with Apriltag information; robot arm control on Moveit.

## Prerequisites:
#### Hardware:
1. Universal Robot UR10
2. 3D printed soft gripper
3. In palm camera
4. Arduino
5. DAC module
6. Two SMC ITV 0030 series electro-pneumatic regulator
#### Software:
1. [ROS Kinetic](http://wiki.ros.org/kinetic)
2. [ROS UR modern driver](https://github.com/ros-industrial/ur_modern_driver)
3. [ROS universal_robot Package](http://wiki.ros.org/universal_robot)
4. [ROS usb_cam package](http://wiki.ros.org/usb_cam)
5. [ROS apriltags_ros package](http://wiki.ros.org/apriltags_ros)
6. [MoveIt!](https://moveit.ros.org/)
7. [ROS rosserial_arduino package](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup#Installing_the_Software)
## Get started:
The following steps will help to run the software step by step:
#### 1.Initiate the soft gripper control:
Run ROS core:
```
roscore
```
run rosserial_python package(serial_node may varies):
```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```
test the gripper
```
rostopic pub soft std_msgs/UInt16 1 --once
```
#### 2.Setup apriltags detection:
run usb_cam package:
```
roslaunch usb_cam usb_cam-test.launch
```
link to image_proc:
```
ROS_NAMESPACE=usb_cam rosrun image_proc image_proc
```
launch apriltags detection:
```
roslaunch apriltags_ros example.launch 
```
display tag info:
```
rostopic echo /tag_detections
```
#### 3.Setup univeral robot control:
launch robot driver(IP address maybe different):
```
roslaunch ur_modern_driver ur10_bringup.launch robot_ip:=192.168.1.10 [reverse_port:=REVERSE_PORT]
```
launch UR10 planning execution:
```
roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch limited:=true 
```
run MoveIt!:
```
roslaunch ur10_moveit_config moveit_rviz.launch config:=true  
```
subsrcibe to the gripper frame:
```
rosrun a4_paper_turning soft_gripper_frame.py
```
run python code:
```
rosrun a4_paper_turning page_turning_1D_aug_demo.py
```
