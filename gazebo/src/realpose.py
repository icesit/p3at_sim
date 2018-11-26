#!/usr/bin/env python

import os
import sys
import cv2
import numpy as np
import pylab as plt
import rospy
import math
from tf import TransformListener
import tf

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

robotname = 'pioneer3at_robot'

def realposecb(data):
    #rospy.loginfo('get {},{},{}'.format(data.name[-1], data.pose[-1], data.twist[-1]))
    global realposepub, realtwistpub
    i = len(data.name) - 1
    while(i >= 0):
        if(data.name[i] == robotname):
            msgpose = Pose()
            msgpose = data.pose[i]
            realposepub.publish(msgpose)
            msgtwist = Twist()
            msgtwist = data.twist[i]
            realtwistpub.publish(msgtwist)
            break
        i -= 1

if __name__ == '__main__':
    rospy.init_node('realposepub', anonymous=True)
    rospy.Subscriber("/gazebo/model_states", ModelStates, realposecb)
    global realposepub, realtwistpub
    realposepub = rospy.Publisher('/gt_pose', Pose, queue_size=1)
    realtwistpub = rospy.Publisher('/gt_twist', Twist, queue_size=1)
    rate = rospy.Rate(100)
    rospy.loginfo('real pose pub init done')
    while not rospy.is_shutdown():
        rate.sleep()