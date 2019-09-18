# flexflip
**flexflip**: The project demonstrates an advanced dynamic manipulation strategy called **flex-and-flip**. The technique is targeted at flexible, thin objects and applied to obtaining secure, pinch grasps on them. Initially, the flexible object (a strip of paper) is placed on a tabletop. The robotic hand approaches and deforms the object (the **“flex”** phase of the operation), and then the deformed object is tucked into the gap between the fingers (the **“flip”** phase). Finally, a pinch grasp is obtained and the page turning task (or other secondary manipulation tasks) can be performed. The technqiue could be further extented to turning  planar deformable objects and back-to-back page turning.

<p align="center">
  <img height="160" src="https://github.com/HKUST-RML/flexflip/blob/master/pictures/Picture1_complete.jpg">
</p>

**Video**: [video1](https://drive.google.com/file/d/1a20tH0woEcDpF48ZbNs7ktDp6As-niw0/view)

**Authors**: Chunli Jiang, Abdullah Nazir, Ghasem Abbasnejad and Jungwon Seo

**Contributor**: Jiming Ren

**Related Paper**: [Dynamic Flex-and-Flip Manipulation of Deformable Linear Objects](https://drive.google.com/file/d/1-5swA3RHVHJiFSBKWW0xD8tLXGSHrgOR/view?usp=sharing)

*If you use flexflip or its sub-modules for your reasearch or application, please start this repo and cite our related paper.*

## Prerequisites:
#### Hardware:
1. Universal Robot UR10
2. 3D printed soft gripper
3. USB camera
4. Arduino
5. DAC module
6. Two SMC ITV 0030 series electro-pneumatic regulators
#### Software:
1. [ROS Kinetic](http://wiki.ros.org/kinetic)
2. [ROS UR modern driver](https://github.com/ros-industrial/ur_modern_driver)
3. [ROS universal_robot package](http://wiki.ros.org/universal_robot)
4. [ROS usb_cam package](http://wiki.ros.org/usb_cam)
5. [ROS apriltags_ros package](http://wiki.ros.org/apriltags_ros)
6. [MoveIt!](https://moveit.ros.org/)
7. [ROS rosserial_arduino package](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup#Installing_the_Software)

## Haraware preparation:
Stl files and descriptive drawings for manufacturing are in [mesh folder](https://github.com/HKUST-RML/soft_gripper_page_turning/tree/master/mesh).

<p align="center">
<img src="https://github.com/HKUST-RML/flexflip/blob/master/pictures/hardware_settings.png" height="400">
</p>


The soft pneumatic finger is 3D printed by 95A hardness TPU on Ultimaker 3 Extended. The thickness of each layer is set as 0.08mm and the printing speed as 35mm/s for high precision. Mounting parts are printed by PLA on same machine. And introduction of TPU could be found at [here](http://www.huntsman.com/polyurethanes/Media%20Library/global/files/guide_tpu.pdf).
## Software setup and tests:
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
A shell script paper_turning.sh is written to set up the environment.
## Maintainrs:
Chunli JIANG (cjiangab@ust.hk)  Abdullah NAZIR(sanazir@connect.ust.hk) 
