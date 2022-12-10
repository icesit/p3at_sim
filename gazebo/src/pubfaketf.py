#!/usr/bin/env python

import os
import sys,time
import rospy
import math
import tf

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


odomt_error_factor = 1.0 - 0.02

def realodomCB(data):
    # pub fake odom
    global odomwitherrpub
    odom_msg = data
    odom_msg.pose.pose.position.x *= odomt_error_factor
    odom_msg.pose.pose.position.y *= odomt_error_factor
    odom_msg.pose.pose.position.z *= odomt_error_factor
    odom_msg.twist.twist.linear.x *= odomt_error_factor
    odom_msg.twist.twist.linear.y *= odomt_error_factor
    odom_msg.twist.twist.linear.z *= odomt_error_factor
    odomwitherrpub.publish(odom_msg) 
    # pub fake tf
    global br
    tf_odom_base = TransformStamped()
    tf_odom_base.header = data.header
    tf_odom_base.child_frame_id = 'base_link'
    tf_odom_base.transform.translation.x = odom_msg.pose.pose.position.x
    tf_odom_base.transform.translation.y = odom_msg.pose.pose.position.y
    tf_odom_base.transform.translation.z = odom_msg.pose.pose.position.z
    tf_odom_base.transform.rotation.w = odom_msg.pose.pose.orientation.w
    tf_odom_base.transform.rotation.x = odom_msg.pose.pose.orientation.x
    tf_odom_base.transform.rotation.y = odom_msg.pose.pose.orientation.y
    tf_odom_base.transform.rotation.z = odom_msg.pose.pose.orientation.z
    br.sendTransformMessage(tf_odom_base)


if __name__ == '__main__':
    rospy.init_node('realposepub', anonymous=True)
    rospy.Subscriber('/sim_p3at/odom', Odometry, realodomCB, queue_size=1)
    br = tf.TransformBroadcaster()
    odomwitherrpub = rospy.Publisher('/sim_p3at/odom_with_err', Odometry, queue_size=1)
    rate = rospy.Rate(10)
    rospy.loginfo('fakepose pub start! Odom error factor is {}'.format(odomt_error_factor))
    while not rospy.is_shutdown():
        rate.sleep()