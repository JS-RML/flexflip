#!/usr/bin/env python  
import roslib

import numpy
import rospy
import tf
from math import pi

if __name__ == '__main__':
    rospy.init_node('base_cam_transform')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(20.0)
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0, axes='sxyz')

    while not rospy.is_shutdown():
        br.sendTransform((0.005,0,0.062),
                 (quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
                 rospy.Time.now(),
                 "soft_gripper","ee_link")
        rate.sleep()
