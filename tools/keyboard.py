#!/usr/bin/env python
#coding: utf-8
from evdev import InputDevice
from select import select

import os
import rospy
from geometry_msgs.msg import Twist
#17-w
#31-s
#36-j
#38-l
#48-b
# 源目录
deviceFilePath = '/sys/class/input/'

def showDevice():
    os.chdir(deviceFilePath)
    for i in os.listdir(os.getcwd()):
        namePath = deviceFilePath + i + '/device/name'
        if os.path.isfile(namePath):
            print "Name: %s Device: %s" % (i, file(namePath).read())

def detectInputKey():
    os.system('sudo chmod 664 /dev/input/event3')
    dev = InputDevice('/dev/input/event3')
    rospy.init_node('keyboardtotwist', anonymous=True)
    pub = rospy.Publisher('/sim_p3at/cmd_vel', Twist, queue_size=10)
    msg = Twist()
    print("keyboard ok!")
    while True:
        select([dev], [], [])
        for event in dev.read():
            if event.code == 17:
                msg.linear.x = 0.5#event.value
            elif event.code == 31:
                msg.linear.x = -0.5#event.value
            elif event.code == 36:
                msg.angular.z = 0.3#event.value
            elif event.code == 38:
                msg.angular.z = -0.3#event.value
            elif event.code == 48:
                msg.linear.x = 0
                msg.angular.z = 0
            pub.publish(msg)

if __name__ == '__main__':
    showDevice()
    detectInputKey()
