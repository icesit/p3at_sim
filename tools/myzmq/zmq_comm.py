#!/usr/bin/python3
# coding=utf-8

import sys
import pickle
import zmq
import numpy as np
from myzmq.jpeg_compress import *
from myzmq.misc import *

version_flag = sys.version[0]

class zmq_pub_c:
    def __init__(self, name='zmq_pub_c', ip='127.0.0.1', port=3333):
        print('[TRK] zmq_pub_c.__init__()')
        print('[TRK] name=%s, making ZMQ PUB socket'%name)
        print('[TRK] server ip: '+ip)
        print('[TRK] server port: %d'%port)
        ctx=zmq.Context()
        self.skt=ctx.socket(zmq.PUB)
        #self.skt.bind('tcp://*:%d'%port)
        self.skt.connect('tcp://%s:%d'%(ip,port))
        return

    def pub(self, msg):
        self.skt.send(msg)

class zmq_sub_c(run_thread_c):
    def __init__(self, name='zmq_sub_c', ip='127.0.0.1', port=3333):
        run_thread_c.__init__(self,name)
        print('[TRK] zmq_sub_c.__init__()')
        print('[TRK] name=%s, making ZMQ SUB socket'%name)
        print('[TRK] server ip: '+ip)
        print('[TRK] server port: %d'%port)
        ctx=zmq.Context()
        self.skt=ctx.socket(zmq.SUB)
        self.skt.setsockopt(zmq.CONFLATE, True)
        self.skt.setsockopt(zmq.SUBSCRIBE,''.encode(encoding='utf-8'))
        #self.skt.connect('tcp://%s:%d'%(ip,port))
        self.skt.bind('tcp://*:%d'%port)
        return

    def main_loop(self):
        self.zmq_sub_msg = self.skt.recv()

class zmq_sub_simple_c():
    def __init__(self, name='zmq_sub_c', ip='127.0.0.1', port=3333):
        print('[TRK] zmq_sub_c.__init__()')
        print('[TRK] name=%s, making ZMQ SUB socket'%name)
        print('[TRK] server ip: '+ip)
        print('[TRK] server port: %d'%port)
        ctx=zmq.Context()
        self.skt=ctx.socket(zmq.SUB)
        self.skt.setsockopt(zmq.CONFLATE, True)
        self.skt.setsockopt(zmq.SUBSCRIBE,''.encode(encoding='utf-8'))
        #self.skt.connect('tcp://%s:%d'%(ip,port))
        self.skt.bind('tcp://*:%d'%port)
        return

    def sub(self):
        try:
            res = self.skt.recv(flags=zmq.NOBLOCK)
        except:
            res = ''
        return res

class zmq_img_pub_c:
    def __init__(self, name='zmq_srv_pub_c', ip='127.0.0.1', port=2222):
        print('[TRK] zmq_img_pub_c.__init__()')
        print('[TRK] name=%s, making ZMQ PUB socket'%name)
        print('[TRK] server ip: '+ip)
        print('[TRK] server port: %d'%port)
        ctx=zmq.Context()
        self.skt=ctx.socket(zmq.PUB)
        self.skt.bind('tcp://*:%d'%port)
        return

    def pubimg(self, img):
        data_jpeg=img_rgb_to_jpeg(img,quality=80)
        self.skt.send(data_jpeg)
        #print('[TRK] send image')

class zmq_img_sub_c:
    def __init__(self, name='zmq_srv_pub_c', ip='127.0.0.1', port=2222):
        print('[TRK] zmq_img_sub_c.__init__()')
        print('[TRK] name=%s, making ZMQ SUB socket'%name)
        print('[TRK] server ip: '+ip)
        print('[TRK] server port: %d'%port)
        ctx=zmq.Context()
        self.skt=ctx.socket(zmq.SUB)
        self.skt.setsockopt(zmq.CONFLATE, True)
        self.skt.connect('tcp://%s:%d'%(ip,port))
        return

    def subimg(self):
        buf=self.skt.recv()
        frame=np.frombuffer(buf,dtype=np.uint8)
        img_rgb=jpg_to_img_rgb(frame)
        return img_rgb

## ZMQ远程API调用，客户端
class zmq_comm_cli_c:
    def __init__(self, name='zmq_comm_cli_c', ip='127.0.0.1', port=1111):
        print('[TRK] zmq_comm_cli_c.__init__()')

        self.name=name
        
        print('[TRK] name=%s, making ZMQ REQ socket'%name)
        print('[TRK] server ip: '+ip)
        print('[TRK] server port: %d'%port)
        
        ctx=zmq.Context()
        self.skt=ctx.socket(zmq.REQ)
        self.skt.connect('tcp://%s:%d'%(ip,port))
        return

    def api_call(self,api_name,param):
        #print('[TRK] zmq_comm_cli_c.api_call(api_name=%s)'%api_name)
        
        tx = pickle.dumps((self.name,api_name,param), protocol=0) 
        self.skt.send(tx)
        rx=self.skt.recv()
        if(version_flag == '2'):
            return pickle.loads(rx)
        else:
            return pickle.loads(rx,encoding='bytes')
    
    def reset(self,param=None): 
        return self.api_call('reset',param)
    def config(self,param=None): 
        return self.api_call('config',param)
    def query(self,param=None): 
        return self.api_call('query',param)
    def get_result(self,param=None): 
        return self.api_call('get_result',param)
    def execute(self,param=None): 
        return self.api_call('execute',param)
    def stop(self,param=None): 
        return self.api_call('stop',param)
        
    ## 等待某一状态
    def wait(self,state,delay=0.1,timeout=0.0):
        while True:
            if isinstance(state, list):
                if self.query('state') in state:
                    return state
            else:
                if self.query('state')==state:
                    return state
            time.sleep(delay)
            if timeout>0:
                timeout-=delay
                if timeout<=0: 
                    return 'timeout'    # 超时
        return 'error'


## ZMQ远程API调用，服务器端

class zmq_comm_svr_c(run_thread_c):
    def __init__(self, name='zmq_comm_svr_c', ip='127.0.0.1', port=1001):
        run_thread_c.__init__(self,name)  

        print('[TRK] name=%s, making ZMQ REP socket'%name)
        print('[TRK] server ip: '+ip)
        print('[TRK] server port: %d'%port)
        ctx=zmq.Context()
        self.skt=ctx.socket(zmq.REP)
        self.skt.bind('tcp://*:%d'%port)
        return

    def main_loop(self):
        try:
            rx=self.skt.recv()#
        except:
            time.sleep(0.001)
            return True
        
        if(version_flag == '2'):
            name,api_name,param = pickle.loads(rx)
        else:
            name,api_name,param = pickle.loads(rx,encoding='bytes')
        if name != self.name:
            print('[WRN] name mis-match (%s/%s)'%(name,self.name))
        
        if   api_name=='reset'     : self.skt.send(pickle.dumps(self.reset     (param), protocol=0))
        elif api_name=='config'    : self.skt.send(pickle.dumps(self.config    (param), protocol=0))
        elif api_name=='query'     : self.skt.send(pickle.dumps(self.query     (param), protocol=0))
        elif api_name=='get_result': self.skt.send(pickle.dumps(self.get_result(param), protocol=0))
        elif api_name=='execute'   : self.skt.send(pickle.dumps(self.execute   (param), protocol=0))
        elif api_name=='stop':
            self.skt.send(pickle.dumps(None, protocol=0))
            return False
        else:
            print('unknown api name '+api_name)
            self.skt.send(pickle.dumps(None, protocol=0))
        return True


    def reset(self,param=None): 
        print('[WRN] function reset must be overloaded!')
        return '[WRN] function reset must be overloaded!',param
        
    def config(self,param=None): 
        print('[WRN] function config must be overloaded!')
        return '[WRN] function config must be overloaded!',param
    
    def query(self,param=None): 
        print('[WRN] function query must be overloaded!')
        return '[WRN] function query must be overloaded!',param
    
    def get_result(self,param=None): 
        print('[WRN] function get_result must be overloaded!')
        return '[WRN] function get_result must be overloaded!',param
    
    def execute(self,param=None): 
        print('[WRN] function execute must be overloaded!')
        return '[WRN] function execute must be overloaded!',param

    ## 等待某一状态
    def wait(self,state,delay=0.1,timeout=0.0):
        while True:
            if isinstance(state, list):
                if self.query('state') in state:
                    return state
            else:
                if self.query('state')==state:
                    return state
            time.sleep(delay)
            if timeout>0:
                timeout-=delay
                if timeout<=0: 
                    return 'timeout'    # 超时
        return 'error'


##########
## 单元测试
##########
    
def unit_test():
    # 启动ZQM服务器
    zmq_comm_svr=zmq_comm_svr_c()
    zmq_comm_svr.start()
    
    # 启动ZMQ客户端
    zmq_comm_cli=zmq_comm_cli_c()
    print(zmq_comm_cli.reset('test'))
    time.sleep(0.3)
    print(zmq_comm_cli.config('test'))
    time.sleep(0.3)
    print(zmq_comm_cli.query('test'))
    time.sleep(0.3)
    print(zmq_comm_cli.get_result('test'))
    time.sleep(0.3)
    print(zmq_comm_cli.execute('test'))
    time.sleep(0.3)
    
    zmq_comm_svr.stop()

if __name__ == '__main__':
    unit_test()

