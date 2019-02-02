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
import tf_conversions
import numpy as np
import message_filters
from std_msgs.msg import Int8
from ur_moveit_myplan.msg import qr_status
from apriltags_ros.msg import AprilTagDetection, AprilTagDetectionArray
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
from math import sqrt, pi, acos, sin, cos
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from math import sqrt, pi, acos, sin, cos , atan2, tan

from std_msgs.msg import String,Empty

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
gripper_stat = 0
is_collision_config = 0

step=6
getObject=0

x_box = 0
y_box = 0
init_angle=0.0 

print "============ Waiting for RVIZ..."
rospy.sleep(5)
print "============ Starting tutorial "



##def status_callback(qr_status):
##  is_qr = qr_status.status.data



###################################################################################  
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


#####################################################################################
def add_collision_object1(x_length, y_length, z_length):
  ## Add collision object
  obj_pose = geometry_msgs.msg.PoseStamped()
  obj_pose.header.frame_id = robot.get_planning_frame()
  obj_pose.pose.position.x = 0.
  obj_pose.pose.position.y = 0.
  obj_pose.pose.position.z = -0.15
  scene.add_box("table1", obj_pose, (x_length, y_length, z_length)) # x_axis, y_axis, z_axis

def add_collision_object2(x_length, y_length, z_length):
  ## Add collision object
  obj_pose = geometry_msgs.msg.PoseStamped()
  obj_pose.header.frame_id = robot.get_planning_frame()
  obj_pose.pose.position.x = -0.4
  obj_pose.pose.position.y = 0.
  obj_pose.pose.position.z = 0.5
  scene.add_box("table2", obj_pose, (x_length, y_length, z_length)) 

def add_collision_object3(x_length, y_length, z_length):
  ## Add collision object
  obj_pose = geometry_msgs.msg.PoseStamped()
  obj_pose.header.frame_id = robot.get_planning_frame()
  obj_pose.pose.position.x = 0.9
  obj_pose.pose.position.y = 0.
  obj_pose.pose.position.z = 0.5
  scene.add_box("table3", obj_pose, (x_length, y_length, z_length)) # x_axis, y_axis,

def add_collision_object4(x, y, z, q):
  ## Add collision object
  obj_pose1 = geometry_msgs.msg.PoseStamped()
  obj_pose1.header.frame_id = robot.get_planning_frame()
  obj_pose1.pose.position.x = x
  obj_pose1.pose.position.y = y
  obj_pose1.pose.position.z = z-0.3
  obj_pose1.pose.orientation.x = q[0]
  obj_pose1.pose.orientation.y = q[1]
  obj_pose1.pose.orientation.z = q[2]
  obj_pose1.pose.orientation.w = q[3]
  scene.add_box("box1", obj_pose1, (0.6,0.6, 0.1)) # x_axis, y_axis, z_axis
  
  obj_pose2 = geometry_msgs.msg.PoseStamped()
  obj_pose2.header.frame_id = robot.get_planning_frame()
  obj_pose2.pose.position.x = x
  obj_pose2.pose.position.y = y
  obj_pose2.pose.position.z = z+0.3
  obj_pose2.pose.orientation.x = q[0]
  obj_pose2.pose.orientation.y = q[1]
  obj_pose2.pose.orientation.z = q[2]
  obj_pose2.pose.orientation.w = q[3]
  scene.add_box("box2", obj_pose2, (0.6,0.6, 0.1)) # x_axis, y_axis, z_axis

  obj_pose3 = geometry_msgs.msg.PoseStamped()
  obj_pose3.header.frame_id = robot.get_planning_frame()
  obj_pose3.pose.position.x = x-0.3*cos(init_angle)
  obj_pose3.pose.position.y = y+0.3*sin(init_angle)
  obj_pose3.pose.position.z = z
  obj_pose3.pose.orientation.x = q[0]
  obj_pose3.pose.orientation.y = q[1]
  obj_pose3.pose.orientation.z = q[2]
  obj_pose3.pose.orientation.w = q[3]
  scene.add_box("box3", obj_pose3, (0.1,0.6, 0.6)) # x_axis, y_axis, z_axis

  obj_pose4 = geometry_msgs.msg.PoseStamped()
  obj_pose4.header.frame_id = robot.get_planning_frame()
  obj_pose4.pose.position.x = x+0.3*cos(init_angle)
  obj_pose4.pose.position.y = y-0.3*sin(init_angle)
  obj_pose4.pose.position.z = z
  obj_pose4.pose.orientation.x = q[0]
  obj_pose4.pose.orientation.y = q[1]
  obj_pose4.pose.orientation.z = q[2]
  obj_pose4.pose.orientation.w = q[3]
  scene.add_box("box4", obj_pose4, (0.1,0.6, 0.6)) # x_axis, y_axis, z_axis

  obj_pose5 = geometry_msgs.msg.PoseStamped()
  obj_pose5.header.frame_id = robot.get_planning_frame()
  obj_pose5.pose.position.x = x+0.3*sin(init_angle)
  obj_pose5.pose.position.y = y+0.3*cos(init_angle)
  obj_pose5.pose.position.z = z
  obj_pose5.pose.orientation.x = q[0]
  obj_pose5.pose.orientation.y = q[1]
  obj_pose5.pose.orientation.z = q[2]
  obj_pose5.pose.orientation.w = q[3]
  scene.add_box("box5", obj_pose5, (0.6,0.1, 0.6)) # x_axis, y_axis, z_axis

##############################################################################
def effector_roll():
 
  waypoints = []

  waypoints.append (group.get_current_pose().pose)

  current_quaternion = (
    group.get_current_pose().pose.orientation.x,
    group.get_current_pose().pose.orientation.y,
    group.get_current_pose().pose.orientation.z,
    group.get_current_pose().pose.orientation.w,
  )

  current_euler_eff = tf.transformations.euler_from_quaternion(current_quaternion)


  wpose = geometry_msgs.msg.Pose()
  sub_step = 0.02
  full_step = 0.2

  roll_range_positive = np.arange (current_euler_eff[1] + sub_step , current_euler_eff[1] + full_step, sub_step) 
#  roll_range_negative = np.arange (current_euler_eff[1] - sub_step, current_euler_eff[1] - full_step, sub_step*-1 )

#  roll_range_negative_complete = np.concatenate((roll_range_negative, np.flipud(roll_range_negative[:-1])))
#  roll_range_positive_complete = np.concatenate((roll_range_positive, np.flipud(roll_range_positive[:-1])))

#  roll_range = np.append(roll_range_negative_complete, roll_range_positive_complete)


  for ang in roll_range_positive:

    wpose.orientation = copy.deepcopy(geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(current_euler_eff[0],ang, current_euler_eff[2])))
    wpose.position = copy.deepcopy(group.get_current_pose().pose.position)

    waypoints.append(copy.deepcopy(wpose))


  (plan_effector_yaw, fraction) = group.compute_cartesian_path(
                              waypoints,   # waypoints to follow
                              0.01,        # eef_step
                              0.0)         # jump_threshold

  group.execute (plan_effector_yaw)

  rospy.sleep(5)
####################################################################################
def go_to_home():
  ## define home position
  group.clear_pose_targets()

  ## Then, we will get the current set of joint values for the group
  group_variable_values = group.get_current_joint_values()
  print "============ Joint values: ", group_variable_values

  ## Now, let's modify joints goal position
  ## To make the end effector vertical to the plane,theta1+theta2+theta3 = -pi/2
  ## Following two sets of values are for left and right arm respectively
  group_variable_values[0] = 0
  group_variable_values[1] = -pi*100/180
  group_variable_values[2] = pi*125/180
  group_variable_values[3] = -pi/2-(group_variable_values[1]+group_variable_values[2])
  group_variable_values[4] = -pi*1/2
  group_variable_values[5] = pi*1/2
  group.set_joint_value_target(group_variable_values)
  plan = group.plan()
  print "============ Waiting while RVIZ displays plan2..."
  rospy.sleep(2)
  scaled_traj2 = scale_trajectory_speed(plan, 0.2)
  group.execute(scaled_traj2)
  global init_x  
  global init_y 
  global init_z
  global init_angle
  global is_home 
  is_home = 1 
  init_x = group.get_current_pose().pose.position.x
  init_y = group.get_current_pose().pose.position.y
  init_z = group.get_current_pose().pose.position.z
#####################################################################

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

################################################################
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

#################################################################
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
  scaled_traj2 = scale_trajectory_speed(plan, 0.5)
  group.execute(scaled_traj2)

##################################################################
def quat2eular(qx, qy, qz, qw):
  quaternion = (
      qx,
      qy,
      qz,
      qw)
  euler = tf.transformations.euler_from_quaternion(quaternion)
  return euler
###################################################################

# def target2spatial(x,y,z,rx,ry,rz):
   
###################################################################

#def rotate_tool(raw,pitch,yaw)
#  t = tf.Transformer(True, rospy.Duration(10.0))
#  matrix = t.lookupTransform("tool0","base",rospy.Time())
#  quaternion=matrix[1]
#  euler = tf.transfromations.euler_from_quaternion(quaternion)
#  euler[0] += raw
#  euler[1] += pitch
#  euler[2] += yaw
  
#####################################################################
def pose_callback(cam_pose):
  print "^^^^^^^^^************start0"
  curr_x = group.get_current_pose().pose.position.x
  curr_y = group.get_current_pose().pose.position.y
  curr_z = group.get_current_pose().pose.position.z
  curr_x_ori = group.get_current_pose().pose.orientation.x
  curr_y_ori = group.get_current_pose().pose.orientation.y
  curr_z_ori = group.get_current_pose().pose.orientation.z
  curr_w_ori = group.get_current_pose().pose.orientation.w

  group_variable_values = group.get_current_joint_values()
  curr_joint0=group_variable_values[0]
  curr_joint1=group_variable_values[1]
  curr_joint2=group_variable_values[2]
  curr_joint3=group_variable_values[3]
  curr_joint4=group_variable_values[4]
  curr_joint5=group_variable_values[5]

  global is_arrive
  global is_home
  global gripper_stat
  global is_collision_config
  global step
  global init_angle
  global getObject
  global x_box
  global y_box
  arduino_pub = rospy.Publisher('/soft', Empty, queue_size=1)

 ##########start the control logic here ^^^^^^^^^^^^^^^^^^^^^^^^^

  if is_arrive == 0 :
    
    
    if cam_pose.detections != [] :

          
      
             
      if getObject ==0 and is_collision_config == 0:
   
        qr_x = 0
        qr_y = 0
        qr_z = 0
        qr_x_ori = 0
        qr_y_ori = 0
        qr_z_ori = 0
        qr_w_ori = 0
        qr_euler = 0
        qr_roll = 0
        qr_pitch = 0
        qr_yaw = 0
          
        if cam_pose.detections[0].id == 3:
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
          print "tag lololo", cam_pose.detections[0].id

        if is_collision_config == 0 and cam_pose.detections[0].id == 3:

          print "       Coin found!"
          print"___-------------------_____"

          if qr_yaw>0.002 or qr_yaw<-0.002:
            #rotate(0,0,0,0,0,qr_yaw)
            #init_angle=-curr_joint0+curr_joint5-pi/2
            print "^^^^^^^^^^^^^^^^^qr_z",curr_z

          if abs(qr_x)>0.005 or abs(qr_y)>0.005:
            print "tttttttttttttttttttttttttttttt", qr_x, "   ", qr_y,"   ", qr_z
            move_waypoints(qr_x, -qr_y, 0, 0.2)

          elif abs(qr_x)<=0.005 and abs(qr_y)<=0.005:
            print "ffffffffffffffffffffffffffff", qr_x, "   ", qr_y,"   ", qr_z

            if qr_z>0.15:
              rotate(0,0,0,0,0,qr_yaw)
              print "^^^^^^^^^^^^^^^^^curr_z",curr_z
              move_waypoints(0, 0, -0.04, 0.4)
              
            else:
              
              move_target(curr_x-0.001,curr_y-0.133,0.1160,curr_x_ori,curr_y_ori,curr_z_ori,curr_w_ori,0.1)
              
              
              curr_x = group.get_current_pose().pose.position.x
              curr_y = group.get_current_pose().pose.position.y
              curr_z = group.get_current_pose().pose.position.z
              curr_y_ori = group.get_current_pose().pose.orientation.y
              #if curr_z>0.12185 or curr_z<0.11975:
               # move_target(curr_x,curr_y,0.120,curr_x_ori,curr_y_ori,curr_z_ori,curr_w_ori,0.1)
              rotate(0,0,0,-pi*2/180,0,0)
              arduino_pub.publish()
              #rospy.sleep(4.5)
              #move_waypoints(0,0,0.01,0.3)
              rospy.sleep(14)
              move_waypoints(0,0,0.1,0.3)
              move_waypoints(0,0,-0.1,0.3)
              rospy.sleep(2)
              arduino_pub.publish()
              rospy.sleep(3)
              move_waypoints(0,0,0.1,0.3)
              is_arrive = 1
              
	          

            
        
  elif is_arrive == 1 and gripper_stat == 1:
    print "target reached, waiting for grasping..."
    move_waypoints(0, 0, 0.2, 0.5)
    move_waypoints(-0.2, -0.2, 0, 0.5)
    move_waypoints(0, 0, -0.2, 0.4) #test
    arduino_pub.publish()
    is_arrive = 0


#######################################################################
def start_robot():
  
  add_collision_object1(3.0, 3.0, 0.25)
  add_collision_object2(0.1, 3.0, 1)
  add_collision_object3(0.1, 0.5, 1)
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
  effector_roll()
  go_to_home()
  global gripper_stat
  gripper_stat = 0
  arduino_pub = rospy.Publisher('/soft', Empty, queue_size=1)
  rospy.sleep(3)
  arduino_pub.publish()
  arduino_pub = rospy.Publisher('/soft', Empty, queue_size=1)
  rospy.sleep(3)
  arduino_pub.publish()
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
  print "============ STOPPING"
##########################################################################

if __name__=='__main__':
  start_robot()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
moveit_commander.os._exit(0)
