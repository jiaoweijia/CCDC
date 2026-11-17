from abc import ABC, abstractmethod  # 必须导入！
from enum import unique, Enum, auto
import threading
from ..common.logManager import output, outputMode,enum_Error
from .socket_rw import *
from ..common import commonTools as tools
class socket_type(Enum):
    none             = -1       #没有配置
    tcp_client       = 0        #tcp客户端
    tcp_server       = 1        #tcp服务端
    udp_client       = 2        #udp客户端(暂未实现)
    udp_server       = 3        #udp服务端(暂未实现)
    websocket_server = 4        #websocket服务端(暂未实现)
    udp_group        = 5        #组播(暂未实现)
    broadcast        = 6        #广播(暂未实现)
    serial_port      = 7        #串口通信

    def __str__(self):
        return self.value       # 打印时返回 value
'''
parameters:
    transmit_type       传输类型
    protocol_type       协议类型
    reconnect_interval  重连时间
    threadNum           线程数量
    heart_interval      心跳周期
    timeout_interval    超时周期
    exparam             扩展参数
'''
class socket_base:
    def __init__(self,name,host,port,socket_type,parameters=None):
        self._name              = name
        self._describName       = f"{name}-{host}:{port}"
        self.namespace          = name                      #
        self.host               = host
        self.port               = port
        self.socket_type        = socket_type
        self.parameters         = parameters
        self.transmit_type      = 0 
        self.protocol_type      = -1
        self.reconnect_interval = 0
        self.threadNum          = 0
        self.heartInterval      = 0
        self.timeout_interval   = 0
        self.exparam= ''
        if self.parameters and isinstance(self.parameters,dict):
            if 'transmit_type' in self.parameters:
                self.transmit_type = self.parameters.get('transmit_type',self.transmit_type)
            if 'protocol_type' in self.parameters:
                self.protocol_type = self.parameters['protocol_type']
            if 'reconnect_interval' in self.parameters:
                self.reconnect_interval = self.parameters['reconnect_interval']
            if 'threadNum' in self.parameters:
                self.threadNum = self.parameters['threadNum']
            if 'heart_interval' in self.parameters:
                self.heartInterval = self.parameters['heart_interval']
            if 'timeout_interval' in self.parameters:
                self.timeout_interval = self.parameters['timeout_interval']
            if 'exparam' in self.parameters:
                self.exparam = self.parameters['exparam']
    def send_data(self,bEncode,needCall,data, cmdCode,serialNum,proMgr,uuid=''):
        pass
    def send_data_ask(self,bEncode,needCall,data, cmdCode,serialNum,proMgr,uuid=''):
        pass

from .tcp_server import ctcp_server
from .tcp_client import ctcp_client
from .udp_client import cudp_client
from .udp_server import cupd_server
from .websocket_server import websocket_server
from ..protocol.data_protocol import enum_State_code

#分为三种情况，第一种阻塞等待回复，第二种异步轮询，第三种异步回调
class callbackCtl():
    def __init__(self):
        self.serialNum      = 0
        self.uuid           = ''
        self.callBackSigal  = threading.Event()
        self.callBackFunc   = None#异步回调
        self.data           = None
        self.protocolHead   = None
        self.parent         = None
        self.uuid           = ''
        self.status         = 1                 #状态,0成功,1失败  5执行中 6超时            

    #阻塞等待
    def wait(self,fTimeout = 5):
        bRes = self.callBackSigal.wait(fTimeout)
        self.callBackSigal.clear()
        return bRes
    #异步轮询
    def checkRes(self):
        if self.callBackSigal.isSet():
            return True
        return False
    def setRes(self,protocolHead,data):
        self.data           = data
        self.protocolHead   = protocolHead
        self.callBackSigal.set()
        if self.callBackFunc:#异步回调
            self.callBackFunc(protocolHead,data)
class data_dual_Info():
    def __init__(self,headInfo,content):
        self.headInfo   = headInfo
        self.content    = content


#数据处理
class socketDataInterface(socket_base):
    def __init__(self,socket,address,socket_type,param,name='socketDataInterface'):
        if isinstance(address,tuple) and len(address) == 2:
            super().__init__(name,address[0],address[1],socket_type,parameters=param)
        else:
            super().__init__(name,address,0,socket_type,parameters=param)
        self.socketLock             = threading.Lock()
        self.socketClient           = socket_rw(socket, address,name)
        self.heartTime              = tools.timeManager.get_timestamp_s()                           #连接时间戳
        self.cacheBuf               = bytes()
        self.protocolHead           = None
        self.headSize               = 0
        self._serialNum             = 0                                                             #连接成功后,单独维护序列号
        self.dataLock               = threading.Lock()
        self.sendList               = {}                                                            #发送列表,处理回复信息
        self._name                  = name
        self.dual_data_lock         = threading.Lock()
        self.dual_data_List         = []
        self.dual_data_RunFlag      = True
        self.dual_data_Event        = threading.Event()                                             #写信号，用于判断实时写入
        self.dual_Event_timeout_ms  = 1000
        self.hDualThread            = threading.Thread(target=self.dual_data_Thread,daemon=True)    # 设置为守护线程，主程序退出时自动终止)
        self.hDualThread.start()#启动线程
    #增加处理数据,通知处理线程进行处理,异步处理
    def add_dual_data(self,headInfo,content):
        with self.dual_data_lock:
            dual_data_member = data_dual_Info(headInfo,content)
            self.dual_data_List.append(dual_data_member)
            self.dual_data_Event.set()
    #处理协议线程
    def dual_data_Thread(self):
        while self.dual_data_RunFlag:
            self.dual_data_Event.wait(self.dual_Event_timeout_ms/1000)
            self.dual_data_Event.clear()

            while self.dual_data_RunFlag:
                dual_data_member = None
                with self.dual_data_lock:
                    if len(self.dual_data_List) > 0:
                        dual_data_member = self.dual_data_List.pop(0)
                    else:
                        break
                if dual_data_member is not None:
                    self.dual_data(dual_data_member.headInfo,dual_data_member.content)
    @property
    def serialNum(self)->int:
        nRes = 0
        #处理序列号,0-255,注意心跳不用序列号,没有必要
        with self.dataLock:
            nRes            = self._serialNum
            self._serialNum += 1
            self._serialNum %= 256
        return nRes
    #相当于c++的析构
    def __del__(self):
        self.close()
  
    @property
    def name (self):
        return self._name
    #设置心跳时间,在接收到数据都会调用,这个是虚函数,在具体中判断登录等状态使用
    def setHeartTime(self):
        pass
    @abstractmethod  # 抽象方法，子类必须实现
    def checkHeart(self):
        pass
    @abstractmethod
    def isHeart(self):
        return False
    #判断socket是否有效,如果心跳时间超时或者socket关闭,返回false
    def isValidSocket(self):
        with self.socketLock:
            #超时检测，如果超过10秒没有收到数据，就断开连接
            nTimeInterval = self.timeout_interval/1000
            if nTimeInterval >0:
                curTime = tools.timeManager.get_timestamp_s()
                if curTime - self.heartTime > nTimeInterval:
                    output(f"[socketDataInterface.isValidSocket]:[{self._describName}]心跳超时关闭连接{self.name} ",outputMode.saveLog,enum_Error.localErr)
                    return False
            if self.socketClient is not None:
                return self.socketClient.isValidSocket()
            else:
                return False
    def stop(self):
        try:
            self.close()
            if self.hDualThread is not None:
                self.dual_data_RunFlag = False
                self.dual_data_Event.set()
                self.hDualThread.join()
        except Exception as e:
            output(f"[socketDataInterface.stop]:[{self._describName}]关闭线程异常{self.name} ",outputMode.saveLog,enum_Error.localErr)
        return
    def close(self):
        with self.socketLock:
            if self.socketClient is not None:
                self.socketClient.stop()
                self.socketClient.read_callback = None
                self.socketClient=None
        return
    @abstractmethod
    def dual_data(self,protocolHead,byteContent):
        #使用socketDataInterface类时，需要实现该函数
        pass
    @abstractmethod
    def get_data(self,data):
        #使用socketDataInterface类时，需要实现该函数
        pass

    def check_call(self,serialNum,protocolHead,data):
        pass    

####socket接口,记录配置信息#######
class socketInterface(socket_base):
    def __init__(self,name,ipAddress,port,tcpType,parameters=None):
        #初始化父类
        super().__init__(name,ipAddress,port,tcpType,parameters)
        self.m_tcp_mgr              = None#具体到通信类型,tcp客户端还是tcp服务,udp客户端,udp服务器
        #websocket使用
        self.title                  = 'server_message'
        self.cors_allowed_origins   = '*'
        self.namespace              = ''
        self.secretKey              = 'HUI!WEBSOCKET'
        self.delay_time             = 1

        if self.socket_type == socket_type.websocket_server.value and parameters and isinstance(parameters,dict):
            if 'title' in parameters:                self.title                  = parameters['title']
            if 'cors_allowed_origins' in parameters: self.cors_allowed_origins   = parameters['cors_allowed_origins']
            if 'namespace' in parameters:            self.namespace              = parameters['namespace']
            if 'secretKey' in parameters:            self.secretKey              = parameters['secretKey']
            if 'delay_time' in parameters:           self.delay_time             = parameters['delay_time']
         #####
    @property
    def name (self):
        return self._name
   
    def run(self):
        try:
            if   self.socket_type == socket_type.tcp_client.value:       self.m_tcp_mgr = ctcp_client(self.name,self.host,self.port,self.parameters)
            elif self.socket_type == socket_type.tcp_server.value:       self.m_tcp_mgr = ctcp_server(self.name,self.host,self.port,self.parameters)
            elif self.socket_type == socket_type.udp_client.value:       self.m_tcp_mgr = cudp_client(self.name,self.host,self.port,self.parameters)#udp发布数据
            elif self.socket_type == socket_type.udp_server.value:       self.m_tcp_mgr = cupd_server(self.name,self.host,self.port,self.parameters)#udp发布数据
            elif self.socket_type == socket_type.websocket_server.value: self.m_tcp_mgr = websocket_server(self._name,self.host,self.port,self.title,self.cors_allowed_origins,self.namespace,self.secretKey,self.delay_time)
            else:
                output(f"[socketInterface.run]初始化失败:没有定义socket类型 {self.socket_type}",outputMode.saveLog,enum_Error.localErr)
                return
            self.m_tcp_mgr.callback =  self.callFunc#设置回调函数
            self.m_tcp_mgr.run()                    #运行
        except Exception as e:
            output(f"[异常]:[socketInterface.run]:初始化 {self.name} 时出错: {e}",outputMode.saveLog,enum_Error.localErr)
    def stop(self):
        if self.m_tcp_mgr is not None:
            self.m_tcp_mgr.stop()
            self.m_tcp_mgr = None
    def callFunc(self,socket,address):
        return socketDataInterface(socket,address,self.socket_type,self.parameters)
    def close(self):
        self.stop()
    def socket_send_data(self,data):
        if self.m_tcp_mgr is None:
            return None
        return self.m_tcp_mgr.socket_send_data(data)