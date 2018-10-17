#!/usr/bin/env python
# coding=utf-8

import sys, os, time
import rospy
import copy

from zmq_comm import *
from zmq_cfg      import *
from brain_navi.msg import *

if __name__ == '__main__':
    rospy.init_node('location', anonymous=False)
    pub = rospy.Publisher('/brainnavi/location', array_bn_loc, queue_size=1)
    zmq_loc = zmq_comm_cli_c(name=name_location, ip=ip_location, port=port_location)
    res_msg = []
    one_msg = bn_location()
    rate2 = rospy.Rate(10)
    while(not rospy.is_shutdown()):
        res = zmq_loc.get_result()
        for a in res:
            one_msg.node_id = a[0]
            one_msg.odom_x=a[1][0]
            one_msg.odom_y=a[1][1]
            one_msg.direction=a[2]
            one_msg.blief=a[3]
            res_msg.append(copy.deepcopy(one_msg))
        pub.publish(res_msg)
        res_msg = []
        rate2.sleep()