#!/usr/bin/env python
# coding=utf-8

import sys, os, time
from myzmq.zmq_comm import *
from myzmq.zmq_cfg import *

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from PIL import Image as plimg
import numpy as np

DISPLAY_COLOR_SEG = 1
ID_CHANGE = {'0':255, '1':255, '2':255, '3':255, '4':255, '5':255, '6':255, '7':0, '8':1, '9':255, '10':255, '11':2, '12':3, '13':4, '14':255,'15':255,'16':255, '17':5, '18':255, '19':6, '20':7, '21':8, '22':9, '23':10, '24':11, '25':12, '26':13, '27':14, '28':15, '29':255, '30':255, '31':16, '32':17, '33':18, '-1':-1}
colors = [[128,64,128], [244,35,232], [70,70,70], [102,102,156], [190,153,153], [153,153,153], [250,170,30], [220,220,0], [107,142,35], [152,251,152], [255,0,0], [220,20,60], [70,130,180], [0,0,142], [0,0,70], [0,60,100], [0,80,100], [0,0,230], [119,11,32], [0,0,0]]
ALL_TYPES = {'0':'road', '1':'sidewalk', '2':'building', '3':'wall', '4':'fence', '5':'pole', '6':'trafficc light', '7':'traffic sign', '8':'vegetation', '9':'terrain', '10':'sky', '11':'person', '12':'rider', '13':'car', '14':'truck', '15':'bus', '16':'train', '17':'motorcycle', '18':'bike', '19':'unknown',}


rospy.init_node('seg_trans', anonymous=True)
segcli = zmq_comm_cli_c(name=name_semanticseg, ip=ip_semanticseg, port=port_semanticseg)
res_pub = rospy.Publisher('/seg_img', Image, queue_size=1)
bridge = CvBridge()

def color_the_typeid(type_id):
    colorimg = []
    maxid = []
    countover19 = 0
    for i in range(type_id.shape[0]):
        for j in range(type_id.shape[1]):
            if(type_id[i][j] < 34):
                realid = ID_CHANGE[str(type_id[i][j])]
            if(realid >= 19):
                countover19 += 1
                realid = 19
            colorimg.append(colors[realid])
    colorimg = np.array(colorimg,dtype='uint8')
    colorimg = colorimg.reshape((480,640,3))
    colorimg = cv2.resize(colorimg, (640,480))
    return colorimg

def inputimg_cb(data):
    rospy.loginfo('start')
    img = bridge.imgmsg_to_cv2(data, "bgr8")
    img = plimg.fromarray(cv2.cvtColor(img,cv2.COLOR_BGR2RGB))
    data_jpeg=img_rgb_to_jpeg(img)
    rospy.loginfo('1')
    res = segcli.execute(data_jpeg)
    rospy.loginfo('2')
    if(DISPLAY_COLOR_SEG):
        colorimg = color_the_typeid(res)
        cv2.imshow('segcolor', colorimg)
        cv2.waitKey(1)
    res = bridge.cv2_to_imgmsg(res, encoding="8UC1")
    res_pub.publish(res)
    rospy.loginfo('end')


if __name__ == '__main__':
    rospy.Subscriber('/camera/image_raw', Image, inputimg_cb)
    rospy.loginfo('[seg_trans]init done')
    rospy.spin()