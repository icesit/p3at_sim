#!/usr/bin/python3
# coding=utf-8

import sys
import os
import time
import threading  

import cv2
import numpy as np

####################
##  常用的工具API
##
## V20180425
####################

## 线程类（抽象类）
# @note     通过该类，加入了信息打印，启动、安全终止和资源清除函数。\n
#           内部main_loop()和clean_up()分别是主循环和线程终止后的清理函数，需要被重载。\n
#           该类实现的线程通过成员函数run()启动\n
#           该类实现的线程只能启动一次，一旦用stop()终止，就不能再启动了。\n
#          （由于stop()会调用clean_up()清除UART_RFID线程运行所依赖的资源）
class run_thread_c(threading.Thread):
    ## 初始化
    # @param    name，线程名
    def __init__(self,name='run_thread_c'):  
        threading.Thread.__init__(self)  
        self.name=name
        
        self.running = True
        self.stopped = threading.Event()
        
        # 信息打印级别
        # 0：禁止打印书出
        # 1：仅仅输出错误信息
        # 2：输出错误和警告信息
        # 3：输出所有信息
        self.print_level=0
        
    ## 打印信息
    # @param    t，待打印内容
    # @note     打印内容中加入了线程名前缀
    def print_info(self,t):
        if self.print_level>=3:
            print(self.name+': &d'+t) 
            sys.stdout.flush()
        
    ## 打印错误信息
    # @param    t，待打印内容
    # @note     打印内容中加入了':Error!'以及线程名前缀
    def print_error_info(self,t):
        if self.print_level>=1:
            print('**** '+self.name+': Error! '+t)
            sys.stdout.flush() 

    ## 打印警告信息
    # @param    t，待打印内容
    # @note     打印内容中加入了':Warning!'以及线程名前缀
    def print_warning_info(self,t):
        if self.print_level>=2:
            print('---- '+self.name+': Warning! '+t)
            sys.stdout.flush()
    
    ## 主循环操作
    # @return    如果不需要退出线程，则返回True，否则返回False
    # @note      该函数在线程中被反复调用。\n
    #            该函数必须被重载,这里仅仅提供一个模板
    def main_loop(self):
        self.print_error_info('main_loop(), subclasses must override this function')
        return False

    ## 线程终止后，资源清除
    # @note    可以根据需要重载该函数,这里仅仅提供一个模板
    def clean_up(self):
        pass

    ## 运行线程
    # @note    不断调用main_loop直到main_loop返回False
    def run(self):  
        self.print_info('run(), thread started')

        self.running=True
        self.stopped.clear()

        while self.running:
            if not self.main_loop():
                break

        self.stopped.set()
        self.print_info('run(): thread stopped')

    ## 主动停止
    # @note    通过event标志通知run函数退出循环
    def stop(self):
        self.print_info('stopping thread...')
        self.running=False

        self.print_info('self.running=False, self.stopped.wait()')
        self.stopped.wait()
        
        self.print_info('cleanup thread...')
        self.clean_up()
        return


## 线程安全的fifo
# @note     该类用作数据顺序存储和处理
class fifo_c:
    ## 初始化    
    def __init__(self):
        #创建锁
        self.mutex = threading.Lock()
        self.data=[]
    
    ## 锁定
    # @note 独占式访问加锁
    def lock(self): self.mutex.acquire()
        
    ## 释放
    # @note 独占访问结束
    def release(self): self.mutex.release()

    ## 压入元素
    def push(self,d):
        self.lock()
        self.data.append(d)
        self.release()

    ## 检查是否为空
    def is_empty(self): return self.size()==0
    
    ## 返回最后一个物件
    # @return   返回最早压入的元素，但不删除（不是弹出），如果数据为空则返回False
    def get_last(self): return False if self.is_empty() else self.data[-1]
    
    ## 返回FIFO中的元素个数
    def size(self): return len(self.data)

    ## 列表清除
    def clear(self): self.data=[]
     
    ## 弹出最早的元素
    # @return   弹出并返回最早压入的元素，如果没有元素，则返回False
    def pop(self):
        self.lock()
        if self.is_empty(): 
            d=False
        else:
            d=self.data.pop()
        self.release()
        return d


## 保存视频
def write_avi(frame_array,fname='video.avi',fps=15,frame_hgt=240, frame_wid=320):
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    video = cv2.VideoWriter(fname,
                            fourcc,
                            fps,
                            (frame_wid,frame_hgt))
    
    num=len(frame_array)
    print('writing %d frames into video file %s'%(num,fname))
    for frame in frame_array:
        frame_RGB = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
        video.write(frame_RGB)
    
    print('end of writing')
    video.release()

## 从字典中需按出value最高的(key,value)对
def find_max_score(d): 
    return list(d.items())[np.argmax(list(d.values()))]


## 打印字典，内容为(名字，浮点数),end=''
def print_val_dict(d,fmt='%.2f'):
    fmt='%s:'+fmt+', '
    for k,v in d.items(): print(fmt%(k,v))
    print('')

'''
## matplotlib的汉字显示
from matplotlib.font_manager import FontProperties
import matplotlib.pyplot as plt
PLT_FONT_LEGEND='SimHei'
PLT_FONT_NAME=r'c:\windows\fonts\simsun.ttc'
PLT_FONT_SIZE=14
PLT_FONT=FontProperties(fname=PLT_FONT_NAME, size=PLT_FONT_SIZE) 
    
def plt_title (s): plt.title (s,fontproperties=PLT_FONT)
def plt_xlabel(s): plt.xlabel(s,fontproperties=PLT_FONT)
def plt_ylabel(s): plt.ylabel(s,fontproperties=PLT_FONT)
def plt_legend(s): plt.legend(s,prop={'family':PLT_FONT_LEGEND,'size':PLT_FONT_SIZE}) 

## 从文本文件得到文件名列表
# 各个文件名用过换行分割或者通过逗号分割
# 注释使用‘#’标识
def get_file_list(fname):
    fp=open(fname,'rt')
    flist=[]
    for line in fp:
        n=line.find('#')
        if n>=0: line=line[:n]                  # 去除注释
        line=line.strip(' ').rstrip(' \n\r ')   # 去除无用字符
        if len(line)==0: continue               # 去除空行
        
        words=line.split(',') if line.find(',')>=0 else [line]  # 一行多个文件名，用逗号分开
        for w in words:
            w=w.strip(' ').rstrip(' \n\r ')     # 去除无用字符
            if len(w)>0: flist.append(w)         # 加入文件列表
    fp.close()
    return flist

## 创建GIF动图
import imageio  

# 从文件名列表读取文件，生成GIF动图  
def create_gif_from_flist(flist, fname, duration=0.1):  
    frames = [imageio.imread(n) for n in flist]
    create_gif(frames,fname,duration)

# 从图像帧数据生成GIF动图  
def create_gif(frames,fname,duration=0.1):
    imageio.mimsave(fname, frames, 'GIF', duration = duration)  

######################
## cookbook
# 代码片段

## 保存pylab的figure
# fig = matplotlib.pyplot.gcf()
# fig.set_size_inches(18.5, 10.5)
# fig.savefig('test2png.png', dpi=100)

if __name__ == '__main__':  
    create_gif_from_flist(['./image/1.jpeg','./image/2.jpeg','./image/3.jpeg','./image/4.jpeg'], 'test.gif')  
    
'''