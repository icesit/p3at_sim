#!/usr/bin/env python
# coding=utf-8

import sys, os, time
import rospy
import zmq
#from zmq_comm import *
from zmq_cfg      import *
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('semanticctrl', anonymous=False)
    pub = rospy.Publisher('/brainnavi/semanticctrl', String, queue_size=1)
    ctx=zmq.Context()
    skt=ctx.socket(zmq.SUB)
    skt.setsockopt(zmq.CONFLATE, True)
    skt.setsockopt(zmq.SUBSCRIBE,'') 
    skt.connect('tcp://%s:%d'%(ip_semanticctrl,port_semanticctrl))
    tobepub = String()
    rate2 = rospy.Rate(10)
    print('[TRK] zmq_nor_sub init')
    print('[TRK] name=%s, making ZMQ SUB socket'%name_semanticctrl)
    print('[TRK] server ip: '+ip_semanticctrl)
    print('[TRK] server port: %d'%port_semanticctrl)
    while(not rospy.is_shutdown()):
        res = skt.recv()
        #print('get message:'+res)
        tobepub.data = res
        pub.publish(tobepub)
        rate2.sleep()