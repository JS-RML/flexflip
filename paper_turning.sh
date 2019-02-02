#!/bin/bash

cd $HOME
xterm -hold -e "roscore" &
xterm -hold -e "rosrun rosserial_python serial_node.py /dev/ttyACM0" &
xterm -hold -e "roslaunch usb_cam usb_cam-test.launch" &
xterm -hold -e "ROS_NAMESPACE=usb_cam rosrun image_proc image_proc" &
xterm -hold -e "roslaunch apriltags_ros example.launch" &
xterm -hold -e "roslaunch ur_modern_driver ur10_bringup.launch robot_ip:=192.168.1.10 [reverse_port:=REVERSE_PORT]" &
xterm -hold -e "roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch limited:=true" &
xterm -hold -e "roslaunch ur10_moveit_config moveit_rviz.launch config:=true" &
xterm -hold -e "rosrun a4_paper_turning soft_gripper_frame.py" &

exit 0
