# flexflip
**flexflip**: The project demonstrates an advanced dynamic manipulation strategy called **flex-and-flip**. The technique is targeted at flexible, thin objects and applied to obtaining secure, pinch grasps on them. Initially, the flexible object (a strip of paper) is placed on a tabletop. The robotic hand approaches and deforms the object (the **“flex”** phase of the operation), and then the deformed object is tucked into the gap between the fingers (the **“flip”** phase). Finally, a pinch grasp is obtained and the page turning task (or other secondary manipulation tasks) can be performed. The technqiue could be further extented to turning  planar deformable objects and back-to-back page turning.

<p align="center">
  <img height="160" src="https://github.com/HKUST-RML/flexflip/blob/master/pictures/Picture1_complete.jpg">
</p>

**Video**: [video1](https://drive.google.com/file/d/1a20tH0woEcDpF48ZbNs7ktDp6As-niw0/view)

**Authors**: Chunli Jiang, Abdullah Nazir, Ghasem Abbasnejad and [Jungwon Seo ](http://junseo.people.ust.hk/) (All members are from HKUST Robotic Manipulation Lab.)

**Contributor**: Jiming Ren

**Related Paper**: [Dynamic Flex-and-Flip Manipulation of Deformable Linear Objects](https://drive.google.com/file/d/1-5swA3RHVHJiFSBKWW0xD8tLXGSHrgOR/view?usp=sharing) (To be presented in IROS2019 Macau)

*If you use flexflip or its sub-modules for your reasearch or application, please star this repo and cite our related paper.*

## Theory:

**Modeling and Manipulating a Linear Deformable Object** 
The directory `DFF-Manipulation-master/modeling` contains 1) a `MATLAB` function to generate minimum bending energy curves given the $(x,y)$-coordinates of the right end-point. The left end-point is assumed clamped at the origin. The slope at the left end is set to zero, whereas the right slope is left free. This can be modified in-code.
2) a `MATLAB` scripts that considers a set of feasible right endpoint locations and obtains the minimum bending energy curve for each. Simultaneously, the magnitude of total surface energy as well as the minimum coefficient of friction required to maintain contact \#2 is computed. The following figure was generated using this script.

<p align="center">
  <img src="https://github.com/HKUST-RML/flexflip/blob/master/DFF-Manipulation-master/media/fig3">
</p>


Knowledge of variation of bending energy and the minimum coefficient of friction can help in planning the path of contact \#2 such as shown in the following figure.

<p align="center">
  <img height="500" src="https://github.com/HKUST-RML/flexflip/blob/master/DFF-Manipulation-master/media/fig8">
</p>

## Hardware Setup:
The following figure shows the setup on sight. Our gripper is, mounted on UR10, is to be controlled to turn the paper strip, localized autonomously through AprilTag. Range of the initialconfigurations of the hand to be tested in our page-turning experiments. And here are listed iterms we adopted for experiments:
- Robot arm: Universal Robot UR10
- 3D printed soft gripper: Stl files and descriptive drawings for manufacturing are in [mesh folder](https://github.com/HKUST-RML/soft_gripper_page_turning/tree/master/mesh). The soft pneumatic finger is 3D printed by 95A hardness TPU on Ultimaker 3 Extended. Thickness of each layer is set as 0.08mm and the printing speed as 35mm/s for high precision. Mounting parts are printed by PLA on the same machine. And introduction of TPU could be found at [here](http://www.huntsman.com/polyurethanes/Media%20Library/global/files/guide_tpu.pdf).
- USB camera: Logitech C525 Webcam
- Arduino: Arduino Mega 
- DAC module
- [Two SMC ITV 0030 series electro-pneumatic regulators](https://www.smcpneumatics.com/ITV0030-3ML-Q.html)

<p align="center">
<img src="https://github.com/HKUST-RML/flexflip/blob/master/pictures/hardware_settings.png" height="300">
</p>

## Software Prerequisites:
1. **Ubuntu and ROS**:Our system is devevloped in Ubuntu 16.04, [ROS Kinetic](http://wiki.ros.org/kinetic).
2. **Robot Arm Control**: [ROS UR modern driver](https://github.com/ros-industrial/ur_modern_driver) and [ROS universal_robot package](http://wiki.ros.org/universal_robot) are used to control UR10 with the IK solvers provided by  [MoveIt!](https://moveit.ros.org/).
3. **Visual Perception**: Monocular Camera drived by [ROS package usb_cam ](http://wiki.ros.org/usb_cam) provides transform between the object and center of camera [ROS package apriltags_ros](http://wiki.ros.org/apriltags_ros). The pregrasping pose is set based on this perception information.
4. **End Effector Control**: We use [ROS package rosserial_arduino](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup#Installing_the_Software) to setup the communication between ROS and Arduino board. DAC module connect with Arduino will convert PWM signal to analog for pneumatic pressure control.

## Experiments:
The following steps will help to test and debug :
#### 1.Initiate the soft gripper control:
start a ROS core:
```
roscore
```
run rosserial_python package(serial_node may vary):
```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```
test the gripper:
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
launch robot driver:
```
roslaunch ur_modern_driver ur10_bringup.launch robot_ip:=your.robot.ip [reverse_port:=REVERSE_PORT]
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
**Note: A shell script [paper_turning.sh](https://github.com/HKUST-RML/flexflip/blob/master/paper_turning.sh) is written to setup the environment automatically at home directory.**
## Maintainers:
Chunli JIANG (cjiangab@ust.hk)  Abdullah NAZIR(sanazir@connect.ust.hk) 
