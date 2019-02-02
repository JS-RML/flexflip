#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley

import message_filters
from std_msgs.msg import Int8
from ur_moveit_myplan.msg import qr_status
from apriltags_ros.msg import AprilTagDetection, AprilTagDetectionArray
import numpy
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import tf
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from math import sqrt, pi, acos, sin, cos
from std_msgs.msg import String, UInt16

## First initialize moveit_commander and rospy.
print "============ Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_ur10_robot',
                  anonymous=True)

## Instantiate a RobotCommander object.  This object is an interface to
## the robot as a whole.
robot = moveit_commander.RobotCommander()

## Instantiate a PlanningSceneInterface object.  This object is an interface
## to the world surrounding the robot.
scene = moveit_commander.PlanningSceneInterface()

## Instantiate a MoveGroupCommander object.  This object is an interface
## to one group of joints. 
group = moveit_commander.MoveGroupCommander("manipulator")
  
display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=20)
init_x = 0.0 
init_y = 0.0 
init_z = 0.0
is_arrive = 0
is_home = 0
start_gripper = 0
is_collision_config = 0
print "============ Waiting for RVIZ..."
rospy.sleep(5)
print "============ Starting tutorial "



##def status_callback(qr_status):
##  is_qr = qr_status.status.data

  
def scale_trajectory_speed(traj, scale):
       # Create a new trajectory object
       new_traj = RobotTrajectory()
      
       # Initialize the new trajectory to be the same as the planned trajectory
       new_traj.joint_trajectory = traj.joint_trajectory
      
       # Get the number of joints involved
       n_joints = len(traj.joint_trajectory.joint_names)
      
       # Get the number of points on the trajectory
       n_points = len(traj.joint_trajectory.points)
       
       # Store the trajectory points
       points = list(traj.joint_trajectory.points)
      
       # Cycle through all points and scale the time from start, speed and acceleration
       for i in range(n_points):
           point = JointTrajectoryPoint()
           point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale
           point.velocities = list(traj.joint_trajectory.points[i].velocities)
           point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
           point.positions = traj.joint_trajectory.points[i].positions
                        
           for j in range(n_joints):
               point.velocities[j] = point.velocities[j] * scale
               point.accelerations[j] = point.accelerations[j] * scale * scale
           
           points[i] = point

       # Assign the modified points to the new trajectory
       new_traj.joint_trajectory.points = points

       # Return the new trajecotry
       return new_traj

def add_collision_object(x_length, y_length, z_length):
  ## Add collision object
  obj_pose = geometry_msgs.msg.PoseStamped()
  obj_pose.header.frame_id = robot.get_planning_frame()
  obj_pose.pose.position.x = 0.
  obj_pose.pose.position.y = 0.
  obj_pose.pose.position.z = -0.2
  scene.add_box("table", obj_pose, (x_length, y_length, z_length)) # x_axis, y_axis, z_axis

#def add_big_box_object(box_pose, size)
#  obj_pose = geometry_msgs.msg.PoseStamped()
#  obj_pose.pose.position.x = box_pose.pose.position.x
#  obj_pose.pose.position.y = box_pose.pose.position.y
#  obj_pose.pose.position.z = box_pose.pose.position.z
#  obj_pose.pose.orientation.x = box_pose.pose.orientation.x
#  obj_pose.pose.orientation.y = box_pose.pose.orientation.y
#  obj_pose.pose.orientation.z = box_pose.pose.orientation.z
#  obj_pose.pose.orientation.w = box_pose.pose.orientation.w

def go_to_home():
  ## define home position
  group.clear_pose_targets()

  ## Then, we will get the current set of joint values for the group
  group_variable_values = group.get_current_joint_values()
  print "============ Joint values: ", group_variable_values

  ## Now, let's modify joints goal position
  ## To make the end effector vertical to the plane (for the left ur10), (theta1+180)+theta2+(theta3+90) = 180
  ##                                                (for the right ur10), -theta1-theta2-(theta3+90) = 180
  ## Following two sets of values are for left and right arm respectively
  group_variable_values[0] = 0#pi/2
  group_variable_values[1] = -pi*70/180#-pi*3/9#-pi*4/9#-pi*5/9
  group_variable_values[2] = -pi*70/180#-pi*6/9#-pi/2#pi/2
  group_variable_values[3] = -pi*3/2-(group_variable_values[1]+group_variable_values[2])#-pi/2-(group_variable_values[1]+group_variable_values[2])
  group_variable_values[4] = pi*1/2#-pi*1/2
  group_variable_values[5] = -pi/2#pi/2
  group.set_joint_value_target(group_variable_values)
  plan = group.plan()
  print "============ Waiting while RVIZ displays plan2..."
  rospy.sleep(2)
  scaled_traj2 = scale_trajectory_speed(plan, 0.2)
  group.execute(scaled_traj2)
  global init_x  
  global init_y 
  global init_z
  global is_home
  is_home = 1 
  init_x = group.get_current_pose().pose.position.x
  init_y = group.get_current_pose().pose.position.y
  init_z = group.get_current_pose().pose.position.z

def move_target(x, y, z, ox, oy, oz, ow, vel):
  pose_target = geometry_msgs.msg.Pose()
  pose_target.orientation.x = ox
  pose_target.orientation.y = oy
  pose_target.orientation.z = oz
  pose_target.orientation.w = ow
  pose_target.position.x = x
  pose_target.position.y = y
  pose_target.position.z = z
  group.set_pose_target(pose_target)
  plan = group.plan()
  scaled_traj = scale_trajectory_speed(plan, vel)
  print "============ Waiting while RVIZ displays plan1..."
  group.execute(scaled_traj)

def move_waypoints(x, y, z, vel):
  ## Planning along waypoints
  waypoints = []

  # start with the current pose
  waypoints.append(group.get_current_pose().pose)

  # first orient gripper and move forward (+x)
  #wpose = geometry_msgs.msg.Pose()
  wpose = copy.deepcopy(group.get_current_pose().pose)
  #wpose.orientation.w = 1.0
  wpose.position.x += x#0.0#0.02
  wpose.position.y += y#0.01#0.02
  wpose.position.z += z#0.03
  waypoints.append(copy.deepcopy(wpose))

  ## We want the cartesian path to be interpolated at a resolution of 1 cm
  ## which is why we will specify 0.01 as the eef_step in cartesian
  ## translation.  We will specify the jump threshold as 0.0, effectively
  ## disabling it.
  (plan, fraction) = group.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold
  print "============ Waiting while RVIZ displays plan3..."
##  rospy.sleep(2)
  scaled_traj = scale_trajectory_speed(plan, vel)
  group.execute(scaled_traj)                             
##  rospy.sleep(5)

def rotate(theta0, theta1, theta2, theta3, theta4, theta5):
  group.clear_pose_targets()
  group_variable_values = group.get_current_joint_values()
  target_joint_values = [0, 0, 0, 0, 0, 0]
  target_joint_values[0] = group_variable_values[0] + theta0
  target_joint_values[1] = group_variable_values[1] + theta1
  target_joint_values[2] = group_variable_values[2] + theta2
  target_joint_values[3] = group_variable_values[3] + theta3
  target_joint_values[4] = group_variable_values[4] + theta4
  target_joint_values[5] = group_variable_values[5] + theta5
  group.set_joint_value_target(target_joint_values)
  plan = group.plan()
  print "============ Waiting while RVIZ displays plan2..."
  scaled_traj2 = scale_trajectory_speed(plan, 0.2)
  group.execute(scaled_traj2)

def quat2eular(qx, qy, qz, qw):
  quaternion = (
      qx,
      qy,
      qz,
      qw)
  euler = tf.transformations.euler_from_quaternion(quaternion)
  return euler

#def eular2quat()


def pose_callback(cam_pose):
  curr_x = group.get_current_pose().pose.position.x
  curr_y = group.get_current_pose().pose.position.y
  curr_z = group.get_current_pose().pose.position.z
  curr_x_ori = group.get_current_pose().pose.orientation.x
  curr_y_ori = group.get_current_pose().pose.orientation.y
  curr_z_ori = group.get_current_pose().pose.orientation.z
  curr_w_ori = group.get_current_pose().pose.orientation.w
  global is_arrive
  global is_home
  global start_gripper
  global is_collision_config
  arduino_pub = rospy.Publisher('/servo', UInt16, queue_size=10)

  if is_arrive == 0 and start_gripper == 0:
    if cam_pose.detections != []:
      qr_x = cam_pose.detections[0].pose.pose.position.x #- 0.099
      qr_y = cam_pose.detections[0].pose.pose.position.y #+ 0.058
      qr_z = cam_pose.detections[0].pose.pose.position.z
      qr_x_ori = cam_pose.detections[0].pose.pose.orientation.x
      qr_y_ori = cam_pose.detections[0].pose.pose.orientation.y
      qr_z_ori = cam_pose.detections[0].pose.pose.orientation.z
      qr_w_ori = cam_pose.detections[0].pose.pose.orientation.w
      qr_euler = quat2eular(qr_x_ori, qr_y_ori, qr_z_ori, qr_w_ori)
      qr_roll = qr_euler[0]
      qr_pitch = qr_euler[1]
      qr_yaw = qr_euler[2]
      print "jiaodujiaodujiaodu", qr_roll, qr_pitch, qr_yaw
      #eef_ori = 2*acos(qr_x_ori)
      if cam_pose.detections[0].id == 1 and is_collision_config == 0:
        if abs(qr_x)>0.01 or abs(qr_y)>0.01:
          print "tttttttttttttttttttttttttttttt", qr_x, "   ", qr_y,"   ", qr_z
          move_waypoints(-qr_x, qr_y, 0, 0.5)
        elif abs(qr_x)<=0.01 and abs(qr_y)<=0.01:
          print "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx", qr_x, "   ", qr_y,"   ", qr_z
          move_waypoints(0, 0, -0.05, 0.2)
	  is_home = 0
          if qr_z>0.10 and qr_z<0.20:
            is_collision_config = 1
            rotate(0, 0, 0, 0, 0, qr_yaw)
#            scene.add_mesh("big_box", cam_pose.detections[0].pose, "../mesh/b_box.STL", (0.58, 0.58, 0.58))
            move_waypoints(-0.15*sin(qr_yaw), -0.15*cos(qr_yaw), 0, 0.5)
            move_waypoints(0, 0, -0.34, 0.2)
            curr_x = group.get_current_pose().pose.position.x
            curr_y = group.get_current_pose().pose.position.y
            curr_z = group.get_current_pose().pose.position.z
            curr_x_ori = group.get_current_pose().pose.orientation.x
            curr_y_ori = group.get_current_pose().pose.orientation.y
            curr_z_ori = group.get_current_pose().pose.orientation.z
            curr_w_ori = group.get_current_pose().pose.orientation.w
            eff_curr_euler = quat2eular(curr_x_ori, curr_y_ori, curr_z_ori, curr_w_ori)
            print "end effector roll pitch yaw", eff_curr_euler[0], eff_curr_euler[1], eff_curr_euler[2]
            eff_curr_quaternion = tf.transformations.quaternion_from_euler(eff_curr_euler[0], eff_curr_euler[1]-pi/2, eff_curr_euler[2])
            group.set_rpy_target([eff_curr_euler[0], eff_curr_euler[1]-pi/2, eff_curr_euler[2]])
            plan5 = group.plan()
            print "============ Waiting while RVIZ displays plan5..."
            scaled_traj5 = scale_trajectory_speed(plan5, 0.2)
            group.execute(scaled_traj5)
            #move_target(curr_x, curr_y, curr_z, eff_curr_quaternion[0], eff_curr_quaternion[1], eff_curr_quaternion[2], eff_curr_quaternion[3], 0.2)
            #rotate()
##      if cam_pose.detections[0].id == 0:
##        if abs(qr_x-0.005)>0.01 or abs(qr_y)>0.02:
##          print "tttttttttttttttttttttttttttttt", qr_x, "   ", qr_y,"   ", qr_z
##          move_waypoints(-qr_x, qr_y, 0, 0.5)
##        elif abs(qr_x-0.005)<=0.01 and abs(qr_y)<=0.02:
##          print "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx", qr_x, "   ", qr_y,"   ", qr_z
##          move_waypoints(0, 0, -0.05, 0.2)
##          is_home = 0
##          if qr_z>0.3 and qr_z<0.4:
##          move_waypoints(0.015, 0.06, 0, 0.5)
##            if abs(eef_ori-pi)<=0.348 or abs(eef_ori)<=0.348 or abs(eef_ori-pi/2)<=0.348 or abs(eef_ori-3*pi/2)<=0.348:
##	      move_waypoints(0, 0.05, 0, 0.5)
##              start_gripper = 45
##  	      #arduino_pub.publish(start_gripper)
##	    else:
##	      rotate(eef_ori-pi)
##              move_waypoints(0.06*cos(eef_ori-pi+pi/9), -0.05*sin(eef_ori-pi+pi/9), 0, 0.5)
##  	      #rospy.sleep(2)
##  	      start_gripper = 45
##  	      #arduino_pub.publish(start_gripper)
##	    print "angle", eef_ori 
##            is_arrive = 1
##            print "isisisisisiisisis", is_arrive
    elif cam_pose.detections == [] and is_collision_config == 0:
      print "No April Tag Detected!! Detecting..."
      if is_home == 0:
        go_to_home()
      radius = 0.3
      step = 5
      dx = radius / step
      if curr_x-init_x<0 and curr_y-init_y>0:
        print "left up"
        dy = sqrt(radius*radius-(curr_x+dx-init_x)*(curr_x+dx-init_x)) + init_y - curr_y
        move_waypoints(dx,dy,0,0.5)
        print "xxxxxxxxxxxxxxxxxxxxxxxxxx", curr_x,"   ",curr_y,"   ",init_x,"   ",init_y
      elif curr_x-init_x>0 and curr_y-init_y>0:
        print "right up"
        if radius*radius-(curr_x+dx-init_x)*(curr_x+dx-init_x)<0:
          dy = -0.3
          move_waypoints(0,dy,0,0.5)
        else:
          dy = -(sqrt(radius*radius-(curr_x+dx-init_x)*(curr_x+dx-init_x)) + init_y - curr_y)
          move_waypoints(dx,-dy,0,0.5)
        print "xxxxxxxxxxxxxxxxxxxxxxxxxx", curr_x,"   ",curr_y,"   ",init_x,"   ",init_y
      elif curr_x-init_x>0 and curr_y-init_y<0:
        print "right bottom"
        dy = sqrt(radius*radius-(curr_x-dx-init_x)*(curr_x-dx-init_x)) - init_y + curr_y
        move_waypoints(-dx,-dy,0,0.5)
        print "xxxxxxxxxxxxxxxxxxxxxxxxxx", curr_x,"   ",curr_y,"   ",init_x,"   ",init_y
      elif curr_x-init_x<0 and curr_y-init_y<0:
        if radius*radius-(curr_x-dx-init_x)*(curr_x-dx-init_x)<0:
          dy = 0.1
	  move_waypoints(0,dy,0,0.5)
        else:
          dy = -sqrt(radius*radius-(curr_x-dx-init_x)*(curr_x-dx-init_x)) + init_y - curr_y
          move_waypoints(-dx,dy,0,0.5)
  elif is_arrive == 1 and start_gripper == 45:
    print "target reached, waiting for grasping..."
    move_waypoints(0, 0, 0.2, 0.5)
    move_waypoints(0.2, 0.2, 0, 0.5)
    move_waypoints(0, 0, -0.2, 0.2) #test
    arduino_pub.publish(0)
    is_arrive = 0


def move_robot():
  
  add_collision_object(2.0, 1.5, 0.25)
  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  print "============ End effector: %s" % group.get_end_effector_link()

  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot state"
  print robot.get_current_state()

  go_to_home()
  rospy.sleep(2)
  global start_gripper
  start_gripper = 0
  arduino_pub = rospy.Publisher('/servo', UInt16, queue_size=10)
  rospy.sleep(2)
  arduino_pub.publish(start_gripper)
  ## Subscribe the camera information
##  cam_pose_subscriber = rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, pose_callback)
##  cam_qrcode_status = rospy.Subscriber('custom_qr_status', qr_status, status_callback)
  cam_pose_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, pose_callback, queue_size=1)

  ## Planning to a Pose goal
  ## ^^^^^^^^^^^^^^^^^^^^^^^
##  print "============ Generating plan 1"
##  pose_target = geometry_msgs.msg.Pose()
##  pose_target.orientation.w = 1.0
##  pose_target.position.x = -0.5#0.5#0.7
##  pose_target.position.y = 0.2#0.1#-0.05
##  pose_target.position.z = 1.1
##  group.set_pose_target(pose_target)
##  plan1 = group.plan()
##  rospy.sleep(5)
##  scaled_traj = scale_trajectory_speed(plan1, 0.5)
##  print "============ Waiting while RVIZ displays plan1..."
##  group.execute(scaled_traj)
##  rospy.sleep(5)

##  move_waypoints()

  ## When finished shut down moveit_commander.
  ##moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"


if __name__=='__main__':
  move_robot()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
moveit_commander.os._exit(0)
