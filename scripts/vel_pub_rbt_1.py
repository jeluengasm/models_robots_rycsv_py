#!/usr/bin/python

import rospy

from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Twist

import tf2_ros
import tf2_geometry_msgs

import numpy as np
import time

from model_rbt_1 import Velocity_publisher


if __name__ == '__main__':

    rospy.init_node('vel_publisher', anonymous=True)
    rospy.loginfo("Node init")
    Velocity_publisher()
    rospy.spin()

