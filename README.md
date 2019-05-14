# soft_gripper_page_turning
ROS package: Page turning by soft robotics gripper with UR10 robot arm. This project is to turn paper or paper strip by 3D printed 
soft robtoics gripper. The code has 3 major parts: soft gripper actuation by Arduino and electro-pneumatic regulator; object detection
by in palm camera and Apriltags; UR10 robot arm control on Moveit.

## Prerequisites:
#### Hardware:
1. Universal Robot UR10
2. 3D printed soft gripper
3. USB camera
4. Arduino
5. DAC module
6. Two SMC ITV 0030 series electro-pneumatic regulator
#### Software:
1. [ROS Kinetic](http://wiki.ros.org/kinetic)
2. [ROS UR modern driver](https://github.com/ros-industrial/ur_modern_driver)
3. [ROS universal_robot package](http://wiki.ros.org/universal_robot)
4. [ROS usb_cam package](http://wiki.ros.org/usb_cam)
5. [ROS apriltags_ros package](http://wiki.ros.org/apriltags_ros)
6. [MoveIt!](https://moveit.ros.org/)
7. [ROS rosserial_arduino package](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup#Installing_the_Software)
## Haraware preparation:
Stl files for 3D printing is in mesh folder. Here are the descriptive drawings:
![](https://gyazo.com/eb5c5741b6a9a16c692170a41a49c858.png | width=256)
![](https://gyazo.com/eb5c5741b6a9a16c692170a41a49c858.png | width=256)
![](https://github.com/HKUST-RML/soft_gripper_page_turning/blob/master/mesh/gripper_on_ur10.jpg | width=256)
## Software Setup and tests:
The following steps will help to run the software :
#### 1.Initiate the soft gripper control:
start a ROS core:
```
roscore
```
run rosserial_python package(serial_node may vary):
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
rosrun soft_gripper_page_turning soft_gripper_frame.py
```
run python code:
```
rosrun soft_gripper_page_turning page_turning_1D_aug_demo.py
```
## Note:
A shell script paper_turning.sh is written to set up the environment. However, the last python code has to be run independently.

## Author
Chunli JIANG (cjiangab@ust.hk) Jiming REN(jrenaf@connect.ust.hk)
