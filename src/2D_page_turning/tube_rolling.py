#!/usr/bin/env python

import tf_conversions
import numpy 
import message_filters
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
import roslib; roslib.load_manifest('ur_driver')
roslib.load_manifest('ur_moveit_myplan')
import rospy
import actionlib
import tf
from std_msgs.msg import Int8
from ur_moveit_myplan.msg import qr_status
from apriltags_ros.msg import AprilTagDetection, AprilTagDetectionArray
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import sqrt, pi, acos, sin, cos
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from math import sqrt, pi, acos, sin, cos , atan2, tan

from std_msgs.msg import String,Empty,UInt16,Int16

rospy.init_node('robot_rolling', anonymous=True)

## First initialize moveit_commander and rospy.
print "============ Starting tutorial setup"






            

#######################################################################
def start_robot():
  
  for x in range(1, 50):


    arduino_pub = rospy.Publisher('/soft', Int16, queue_size=1)
    rospy.sleep(1.2)
    arduino_pub.publish(5050)
    rospy.sleep(5)
    arduino_pub = rospy.Publisher('/soft', Int16, queue_size=1)
    rospy.sleep(1.2)
    arduino_pub.publish(0)
    rospy.sleep(3)
  
  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot state"


  print "============ STOPPING"
##########################################################################

if __name__=='__main__':
  start_robot()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
moveit_commander.os._exit(0)
