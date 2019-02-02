#!/usr/bin/env python

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

listener = tf.TransformListener()
## First initialize moveit_commander and rospy.
print "============ Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur10_rotation_exercise',
                  anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")

 ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
print "============ Waiting for RVIZ..."
print "============ Starting tutorial "

  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## We can get the name of the reference frame for this robot
print "============ Reference frame: %s" % group.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
print "============ Reference frame: %s" % group.get_end_effector_link()

  ## We can get a list of all the groups in the robot
print "============ Robot Groups:"
print robot.get_group_names()

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
print "============ Printing robot state"
print robot.get_current_state()
print "============"
  
display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=20)
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
  obj_pose.pose.position.x = 0
  obj_pose.pose.position.y = 0
  obj_pose.pose.position.z = -0.2
  scene.add_box("table", obj_pose, (x_length, y_length, z_length)) # x_axis, y_axis, z_axis
  #print "add collision", obj_pose

def quat2eular(qx, qy, qz, qw):
  quaternion = (
      qx,
      qy,
      qz,
      qw)
  euler = tf.transformations.euler_from_quaternion(quaternion, axes='sxyz')
  return euler


def go_to_home():
  group.clear_pose_targets()
  group_variable_values = group.get_current_joint_values()
  print "============ Joint values: ", group_variable_values
  group_variable_values[0] = 0#pi/2
  group_variable_values[1] = -pi*120/180#-pi*5/9#-pi*70/180#-pi*3/9#-pi*4/9
  group_variable_values[2] = -pi*120/180#pi/2#-pi*70/180#-pi*6/9#-pi/2
  group_variable_values[3] = -(pi*2+group_variable_values[1]+group_variable_values[2])#-pi/2-(group_variable_values[1]+group_variable_values[2])#-pi*3/2-(group_variable_values[1]+group_variable_values[2])
  group_variable_values[4] = -pi*1/2#pi*1/2#-pi*1/2
  group_variable_values[5] = 0#-pi/2#pi/2
  group.set_joint_value_target(group_variable_values)
  plan = group.plan()
  print "============ Waiting while RVIZ displays plan2..."
  rospy.sleep(2)
  scaled_traj2 = scale_trajectory_speed(plan, 0.5)
  group.execute(scaled_traj2)

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

def move_tool(x, y, z, rx, ry, rz):
  curr_x = group.get_current_pose().pose.position.x
  curr_y = group.get_current_pose().pose.position.y
  curr_z = group.get_current_pose().pose.position.z
  curr_x_ori = group.get_current_pose().pose.orientation.x
  curr_y_ori = group.get_current_pose().pose.orientation.y
  curr_z_ori = group.get_current_pose().pose.orientation.z
  curr_w_ori = group.get_current_pose().pose.orientation.w
  eff_curr_euler = quat2eular(curr_x_ori, curr_y_ori, curr_z_ori, curr_w_ori)
  curr_rot_mat = tf.transformations.euler_matrix(eff_curr_euler[0], eff_curr_euler[1], eff_curr_euler[2], axes='sxyz')
  
  zaxis = (0, 0, 1)
  yaxis = (0, 1, 0)
  xaxis = (1, 0, 0)
  Rx = tf.transformations.rotation_matrix(rx, xaxis)
  Ry = tf.transformations.rotation_matrix(ry, yaxis)
  Rz = tf.transformations.rotation_matrix(rz, zaxis)
  desire_rot_mat = numpy.dot(curr_rot_mat, Rx)
  desire_rot_mat = numpy.dot(desire_rot_mat, Ry)
  #print "rotation y", desire_rot_mat
  desire_rot_mat = numpy.dot(desire_rot_mat, Rz)
  #print "rotation x", Rx
  desire_euler = tf.transformations.euler_from_matrix(desire_rot_mat, 'sxyz')
  desire_quaternion = tf.transformations.quaternion_from_euler(desire_euler[0], desire_euler[1], desire_euler[2], axes='sxyz')
  print "tool q", desire_quaternion

  move_tool_xyz = numpy.array([x, y, z, 1])
  desire_trans_mat = numpy.dot(curr_rot_mat, (move_tool_xyz))
  curr_rot_mat[:3,3] = numpy.array([curr_x, curr_y, curr_z]) # m[:3, 3]: the first three rows in the fourth column
  desire_trans_mat = numpy.dot(curr_rot_mat, move_tool_xyz)
  print "tool trans", desire_trans_mat

  move_target(desire_trans_mat[0], desire_trans_mat[1], desire_trans_mat[2], desire_quaternion[0], desire_quaternion[1], desire_quaternion[2], desire_quaternion[3], 0.5)

def move_gripper(x, y, z, rx, ry, rz):
  (base_g_trans,base_g_rot) = listener.lookupTransform('/base_link', '/gripper', rospy.Time(0)) #express frame arg2 in frame arg1
  base_g_rot_mat = tf.transformations.quaternion_matrix(base_g_rot)
  print "qt_matrix", base_g_rot_mat
  zaxis = (0, 0, 1)
  yaxis = (0, 1, 0)
  xaxis = (1, 0, 0)
  Rx = tf.transformations.rotation_matrix(rx, xaxis)
  Ry = tf.transformations.rotation_matrix(ry, yaxis)
  Rz = tf.transformations.rotation_matrix(rz, zaxis)
  base_g_rot_mat_new = numpy.dot(base_g_rot_mat, Rx)
  base_g_rot_mat_new = numpy.dot(base_g_rot_mat_new, Ry)
  base_g_rot_mat_new = numpy.dot(base_g_rot_mat_new, Rz)
  #print "new gri mat", base_g_rot_mat_new
  move_gripper_xyz = numpy.array([x, y, z, 1])
  base_g_rot_mat[:3,3] = numpy.array(base_g_trans)
  base_g_trans_new = numpy.dot(base_g_rot_mat, move_gripper_xyz)
  print "compare g move", base_g_trans, base_g_trans_new
  base_g_rot_mat_new[:3,3] = numpy.array([base_g_trans_new[0], base_g_trans_new[1], base_g_trans_new[2]])

  (g_ee_trans,g_ee_rot) = listener.lookupTransform('/gripper', '/ee_link', rospy.Time(0)) #express frame arg2 in frame arg1
  g_ee_rot_mat = tf.transformations.quaternion_matrix(g_ee_rot)
  g_ee_rot_mat[:3,3] = numpy.array(g_ee_trans)
  
  base_ee_homo_new = numpy.dot(base_g_rot_mat_new, g_ee_rot_mat)
  desire_ee_trans = base_ee_homo_new[:3,3]
  print "compare ee move", group.get_current_pose().pose.position, desire_ee_trans
  #base_ee_homo_new[:3,3] = numpy.array([0, 0, 0])
  desire_ee_euler = tf.transformations.euler_from_matrix(base_ee_homo_new, axes='sxyz')
  desire_ee_q = tf.transformations.quaternion_from_euler(desire_ee_euler[0], desire_ee_euler[1], desire_ee_euler[2], axes='sxyz')
  print "compare ee rot", group.get_current_pose().pose.orientation, desire_ee_q
  
  move_target(desire_ee_trans[0], desire_ee_trans[1], desire_ee_trans[2], desire_ee_q[0], desire_ee_q[1], desire_ee_q[2], desire_ee_q[3], 0.5)

def move_robot():
  add_collision_object(2.0, 1.5, 0.25)
  go_to_home()
  rospy.sleep(2)
  #move_tool(0, 0, 0, 0, -pi/8, -pi/8)
  #move_tool(0.1, 0, 0, 0, 0, 0)
  move_gripper(0, 0, -0.2, 0, 0, -pi/4)
#  curr_x = group.get_current_pose().pose.position.x
#  curr_y = group.get_current_pose().pose.position.y
#  curr_z = group.get_current_pose().pose.position.z
#  curr_x_ori = group.get_current_pose().pose.orientation.x
#  curr_y_ori = group.get_current_pose().pose.orientation.y
#  curr_z_ori = group.get_current_pose().pose.orientation.z
#  curr_w_ori = group.get_current_pose().pose.orientation.w
#  eff_curr_euler = quat2eular(curr_x_ori, curr_y_ori, curr_z_ori, curr_w_ori)
#  #print robot.get_current_state()
#  print "position", curr_x, curr_y, curr_z
#  print "eff_curr_euler", eff_curr_euler[0]*180/pi, eff_curr_euler[1]*180/pi, eff_curr_euler[2]*180/pi
#  
## Rotation about tool frame, no translation
#  zaxis = (0, 0, 1)
#  yaxis = (0, 1, 0)
#  Ry = tf.transformations.rotation_matrix(pi/4, yaxis)
#  Rz = tf.transformations.rotation_matrix(pi/4, zaxis)
#  print Rz
#  curr_rot_mat = tf.transformations.euler_matrix(eff_curr_euler[0], eff_curr_euler[1], eff_curr_euler[2], axes='sxyz')
#  desire_rot_mat = numpy.dot(curr_rot_mat, Ry)
#  print "curr_rot_mat", curr_rot_mat
#  desire_euler = tf.transformations.euler_from_matrix(desire_rot_mat, 'sxyz')
#  print desire_euler[0]*180/pi, desire_euler[1]*180/pi, desire_euler[2]*180/pi
#  desire_quaternion = tf.transformations.quaternion_from_euler(desire_euler[0], desire_euler[1], desire_euler[2], axes='sxyz')
#  #move_target(curr_x, curr_y, curr_z, desire_quaternion[0], desire_quaternion[1], desire_quaternion[2], desire_quaternion[3], 0.5)
#
## Translation along tool frame, no rotation
#  move_tool_y = numpy.array([0, 0.1, 0, 1])
#  move_tool_x = numpy.array([0.1, 0, 0, 1])
#  move_tool_xyz = numpy.array([0.1, 0.1, 0, 1])
#  curr_rot_mat[:3,3] = numpy.array([curr_x, curr_y, curr_z]) # m[:3, 3]: the first three rows in the fourth column
#  print "curr_homo", curr_rot_mat
#  desire_trans_mat = numpy.dot(curr_rot_mat, move_tool_xyz)
#  print "tanslation", move_tool_xyz
#  move_target(desire_trans_mat[0], desire_trans_mat[1], desire_trans_mat[2], curr_x_ori, curr_y_ori, curr_z_ori, curr_w_ori, 0.5)

if __name__=='__main__':
  move_robot()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
moveit_commander.os._exit(0)
