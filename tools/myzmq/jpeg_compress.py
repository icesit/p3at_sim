#!/usr/bin/env python
# -*- coding: utf-8 -*-

## JPEG图像数据压缩解压缩工具集

import sys
import io

from PIL import Image
import numpy as np

## 将numpy的uint数组转成BytesIO
def uint8_to_bytesio(ary):
    return io.BytesIO(ary)


## 将BytesIO转成numpy的uint数组
def bytesio_to_uint8(bin_io):
    return np.fromstring(bin_io.getvalue(), dtype=np.uint8)


## 将numpy的uint数组转成PIL的图像对象
def uint8_to_pil(ary):
    return Image.fromarray(ary)


## 将PIL的图像对象转成numpy的uint数组
def pil_to_uint8(img_pil):
    return np.asarray(img_pil, dtype=np.uint8)


## 将RGB图像img_ary(uint8数组)转成JPE格式(uint8数组)
def img_rgb_to_jpeg(img_ary,quality=80):
    #img_pil = uint8_to_pil(img_ary)     # img_pil是PIL图像对象
    img_pil = img_ary
    img_io  = io.BytesIO()              # img_io_jpg是内存流对象

    img_pil.save(img_io, format="JPEG", quality=quality)
    img_io.seek(0)
    bin_jpg=np.fromstring(img_io.getvalue(), dtype=np.uint8)
    return bin_jpg


## 将jpg数据(uint8数组)转成RGB图像(uint8数组)
def jpg_to_img_rgb(bin_jpg):
    bin_io = uint8_to_bytesio(bin_jpg)
    
    # 以文件方式访问内存流对象（JPEG文件），解码并生成numpy对象表示的RGB图像
    img_pil = Image.open(bin_io).convert('RGB')
    img_rgb = np.array(img_pil, dtype=np.uint8)
    return img_rgb


####################
## 单元测试
####################
if __name__ == '__main__':
    
    sys.path.append('./')
    from global_cfg     import *
    from pygame_viewer  import *

    viewer=pygame_viewer_c()
    
    # 读取测试图像
    img_pil = Image.open("test.png")
    if not img_pil.mode == 'RGB': img_pil = img_pil.convert('RGB')
    
    # 测试图像转成uint8数组
    img_uint8=pil_to_uint8(img_pil)
    #print('img_uint8.shape:',end='')
    print(img_uint8.shape)
    
    # 测试图像进行jpeg压缩，得到压缩后的jpeg byte数组
    jpg_uint8=img_rgb_to_jpeg(img_uint8)
    #print('jpg_uint8.shape:',end='')
    print(jpg_uint8.shape)
    
    # 保存为JPEG文件
    jpg_uint8.tofile('jpg_uint8.jpg')
    
    # jpeg数据数组转成RGB图像数组(uint8)
    img_new=jpg_to_img_rgb(jpg_uint8)
    
    #viewer.show_img_u8(cv2.cvtColor(img_new,cv2.COLOR_BGR2RGB))
    viewer.show_img_u8(img_new)
    while True:
        if not viewer.poll_event():
            break
    exit()
