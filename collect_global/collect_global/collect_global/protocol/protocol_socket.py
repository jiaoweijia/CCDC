#构建一个有关对应协议的socket管理器
from    abc import ABC, abstractmethod  # 必须导入！
#枚举定义
from    enum import unique, Enum, auto
from    typing import cast
import  sys
import  threading
#默认协议参数，如果不设置参数时，使用默认参数
from    .data_protocol import defProtocol
from    .data_protocol import *
from    ..common.logManager import output, outputMode,enum_Error
from    ..pynet.socket_define import *
from    ..common import commonTools as tools
from    ..pynet import redisMgr as redisMgr
g_MainMgr = None #全局管理类,用于保存主类
#设置管理类
def setManager(manager:'manager'):
    global g_MainMgr
    g_MainMgr = manager
#得到管理类
def getManager():
    global g_MainMgr
    return g_MainMgr

class protocolDataMgr(socketDataInterface):
    def __init__(self,socket,address,socket_type,param,dualTargetClass,name='protocolDataMgr',bLogin=True,collectDevice=None):
        #初始化父类,创建socket实例,并开始处理线程
        super().__init__(socket,address,socket_type,param,name)
        #如果是采集端,则是采集设备信息实例包含写redis及发送socket
        #如果是服务端,则是listeninfo
        self.collectDevice              = collectDevice                 #采集设备信息实例
        self.loginFlag                  = enum_State_code.fail.value    #默认登录状态是失败
        self.deviceId                   = ''                            #登录uuid与终端设备Id一致,如果是空表示没登录
        self.rdCode                     = ''                            #是否有rdcode如果有rdcode则使用t_data[rdcode]写数据
        self.dualTaget                  = dualTargetClass               #处理类,主要是服务端或者客户端处理
        if socket is not None:                                          #有socket 则启动 add by lhl 2025
            self.socketClient.read_callback = self.callback             #读取数据的回调函数,主要是解析协议完成后的完整数据
            self.socketClient.run(True,True)                            #运行读写线程
        self.nSaveDbTime                = 60000                         #单位毫秒,设置保存数据库的时间,默认一分钟保存一次
        self.nPreSaveDbTime             = 0                             #当前保存时间,临时变量用于存储上一次的保存时间,当前时间和他对比,如果大于保存时间则保存一次,此次时间更新此时间
        self.isCopyTable                = 1                             #是否复制表,默认复制 add by lhl 20251205
        self.isInput_his                = 0                             #是否写入历史表,默认不写入 add by lhl 20251205

        #如果是边平台采集端,需要发送登录信息
        if self.transmit_type == enum_transmit_type.collect.value:
            serverInfo  = loginInfo.copyServerInfo(self.collectDevice)
            serverDict  = serverInfo.to_dict()
            strData     = loadJsonNode.toJsonString(serverDict)
            self.send_data(True,False,strData.encode('utf-8'),Enum_Protocol_Cmd.login,None,None,'',True)#发送登录信息
        #add by lhl 20251223  控制的客户端和服务器端不需要登录
        elif self.transmit_type == enum_transmit_type.control_client.value: self.loginFlag = enum_State_code.success.value #如果需要处理在此处完善代码
        elif self.transmit_type == enum_transmit_type.control_server.value: self.loginFlag = enum_State_code.success.value #如果需要处理在此处完善代码
        ###########
    #得到复制表名称
    def getTable(self,tablename):
        if self.isCopyTable == 1 and not tools.is_empty(self.deviceId):#复制表时返回tablename_登录uuid
            return f"{tablename}_{self.deviceId}"
        return tablename
    def close(self):
        socketDataInterface.close(self)
        self.loginFlag = enum_State_code.fail.value
        #清除回复消息记录 add by lhl 20251224
        with self.dataLock:
            for k,v in self.sendList:
                v.setRes(None,None)
            self.sendList.clear()
        ######
        if self.transmit_type == enum_transmit_type.server.value:
            g_manger =getManager()
            if g_manger is not None:
                dbInterface = g_manger.configInfo.databaseInfo.dbInterface.getClass()#add by lhl 20251210 使用数据库接收不用具体连接
                if not tools.is_empty(self.deviceId):
                    dbInterface.singleQuery(g_manger.configInfo.databaseInfo.host,
                                           g_manger.configInfo.databaseInfo.port,
                                           g_manger.configInfo.databaseInfo.user,
                                           g_manger.configInfo.databaseInfo.password,
                                           g_manger.configInfo.databaseInfo.dbname,
                        f"update t_terminal_list set online_flag = 0 where uuid = '{self.deviceId}'",True,False)
                    g_manger.writeRedisNode(redisMgr.redis_write_info(f"{self.deviceId}",{
                                    "login_status": self.loginFlag,
                                    "login_status_tm": tools.timeManager.get_timestamp_ms(),
                                    'login_describ': '断开连接'
                                    }))
            self.deviceId=''
        return

    #接收数据后使用回调,解析具体的数据
    def callback(self, data):
        self.cacheBuf += data
        self.get_data(self.cacheBuf)

    #@abstractmethod  # 抽象方法，子类必须实现
    def checkHeart(self):
        #需要发送心跳,则发送
        if self.isHeart == True:
            strRes ='{{"L0003":"{}"}}'.format(tools.timeManager.format_time_s(tools.timeManager.get_timestamp_s()))
            serialNum = 0
            byteRes = defProtocol.encode_tcp_protocol(Enum_Protocol_Cmd.heartbeat,serialNum,strRes.encode('UTF-8'))
            with self.socketLock:
                if self.socketClient is not None:
                    self.socketClient.socket_send_data(byteRes)
    @property
    #使用属性方式
    def isHeart(self):
        with self.dataLock:
            #没有设置心跳,则不发送
            nHeartInterval = self.heartInterval/1000
            if nHeartInterval == 0:
                return False
            if self.loginFlag == enum_State_code.fail.value:
                return False
            curTime = tools.timeManager.get_timestamp_s()
            if curTime - self.heartTime >= nHeartInterval:
                return True
            else:
                return False

    #判断命令是否需要回复，这里在通过对命令判断来决定，也可以通过参数来设置
    def needCall(cmdCode:Enum_Protocol_Cmd=None):
        bRes = False
        if cmdCode is not None:
            if cmdCode in [Enum_Protocol_Cmd.reg,Enum_Protocol_Cmd.login,Enum_Protocol_Cmd.heartbeat,Enum_Protocol_Cmd.uploaddata,Enum_Protocol_Cmd.downloaddata,]:
                bRes = True
        return bRes
    def addCallback(self,callMem):
        with self.dataLock: self.sendList[callMem.serialNum] = callMem
    def removeCallback(self,serialNum):
        with self.dataLock: self.pop(serialNum)
            # if serialNum in self.sendList:
            #     self.sendList[serialNum].setRes(None,None)
            #     del self.sendList[serialNum]
    #发送数据，可以设置是否编码，如果编码需要使用协议管理类和命令码及序列号
    def send_data(self,bEncode,bNeedCall,data,cmdCode:Enum_Protocol_Cmd=None,serialNum:int=None,proMgr:protocolMgr =None,uuid='',force_sendFlag=False,deviceId=''):
        callMem = None  #回调实例
        for _ in range(1):
            #没有强制发送标识,需要判断,如果登录失败则不发送,超时后会断开连接
            #发送数据,如果是服务,如果没登录,不会下发数据,因为需要用户先发起登录
            if (force_sendFlag is None or force_sendFlag == False) and self.loginFlag == enum_State_code.fail.value: break
                    
            isCall          = bNeedCall 
            byteSendInfo    = data
            curSerialNum    = None
            if bEncode == True:
                proMgr = proMgr if proMgr else defProtocol                              #得到协议
                if proMgr is  None or cmdCode is None:  break                           #没有具体的协议,这个情况不会发生,因为有默认,只有一种情况就是默认协议实例生成失败
                curSerialNum = serialNum if serialNum else self.serialNum               #得到序列号
                if isCall is None:  isCall = self.needCall(cmdCode)                     #如果不设置回调标识，就通过命令信息判断
                byteSendInfo = proMgr.encode_tcp_protocol(cmdCode,curSerialNum,data)    #编码数据
            
            if byteSendInfo and len(byteSendInfo)>0:
                if isCall is not None and isCall is True and curSerialNum is not None:
                    callMem = callbackCtl()
                    callMem.parent = self
                    callMem.uuid = uuid
                    callMem.serialNum = curSerialNum
                    self.addCallback(callMem)
                    #with self.dataLock: self.sendList[callMem.serialNum] = callMem
            
                with self.socketLock:
                    if self.socketClient is not None:  self.socketClient.socket_send_data(byteSendInfo)#直接发送编码信息，不处理回复，一般用于ack
        return callMem
    def dual_data(self,protocolHead,byteContent):
        if self.dualTaget is None:
            return
        dualData = self.dualTaget(self,protocolHead)
        byteAck = dualData.dual_pack(byteContent)
        if byteAck is not None:
            #服务并且指令是登录
            if self.transmit_type == enum_transmit_type.server.value: 
                if dualData.cmd == Enum_Protocol_Cmd.login.value:
                    self.send_data(False,False,byteAck,None,None,None,'',True)#强制回复数据
                    if self.loginFlag == enum_State_code.fail.value:#登录失败
                        #等待3秒,如果还有数据发送,则断开连接
                        for _ in range(30):
                            with self.socketLock:
                                if self.socketClient is not None:
                                   if self.socketClient.send_count()==0:break
                            time.sleep(0.1)
                else:
                    self.send_data(False,False,byteAck,None,None,None,'')
                if self.loginFlag == enum_State_code.fail.value:#登录失败
                    self.close()#登录失败,直接断开
                  
            else:
                self.send_data(False,False,byteAck,None,None,None,'')
        return
    def setHeartTime(self):
        try:
            with self.dataLock:
                #登录成功和登录注册中的时可以设置心跳时间
                if self.loginFlag ==enum_State_code.success.value or self.loginFlag == enum_State_code.reging.value:
                    self.heartTime = tools.timeManager.get_timestamp_s()
            if self.transmit_type == enum_transmit_type.collect.value:#采集端
                self.collectDevice.writeRedisNode(redisMgr.redis_write_info(f"{self.collectDevice.uuid}",{"heart": self.heartTime*1000,"login_status":0,}))
            elif self.transmit_type == enum_transmit_type.server.value: #服务器端
                if not tools.is_empty(self.deviceId):
                    manager = getManager()
                    if manager: manager.writeRedisNode(redisMgr.redis_write_info(f"{self.deviceId}",{"heart": self.heartTime*1000,}))
        except Exception as e:
            output(f"[socketDataInterface.setHeartTime]:[{self.name}]异常{e}",outputMode.saveLog,enum_Error.localErr)
    def get_data(self,data):
        try:
            while True:
                #没有处理协议头，需要查找协议头
                if self.protocolHead==None:
                    self.protocolHead,self.headSize,self.cacheBuf = protocolMgr.get_protocol_head(self.cacheBuf)
                    
                #没有协议头，不需要继续处理，继续查找头信息
                if self.protocolHead is None:
                    break
                self.setHeartTime()#收到数据后设置心跳
                #得到数据信息
                protocolHead    = self.protocolHead
                headSize        = self.headSize
                flag,byteContent,self.protocolHead,self.headSize,self.cacheBuf = protocolMgr.get_data_info(protocolHead,self.headSize,self.cacheBuf)
                if flag == False:   break   #get_data_info得到失败,跳出循环,下次处理
                #心跳，直接回复，不影响线程中的数据处理
                if protocolHead['cmdCode'] == Enum_Protocol_Cmd.heartbeat.value:#收到心跳指令立即回复
                    strRes ='{{"L0003":"{}"}}'.format(tools.timeManager.format_time_s(tools.timeManager.get_timestamp_s()))
                    byteAck = defProtocol.encode_tcp_protocol(Enum_Protocol_Cmd.heartbeat_ack,protocolHead['serialNum'],strRes.encode())
                    self.send_data(False,False,byteAck)
                    #output(f"[protocolDataMgr.get_data]:[收到心跳]",outputMode.saveLog,enum_Error.localErr)
                    continue
                elif protocolHead['cmdCode'] == Enum_Protocol_Cmd.heartbeat_ack.value:#add by lhl 20251101 收到回复直接设置心跳
                    #output(f"[protocolDataMgr.get_data]:[收到心跳回复]:{self.loginFlag}",outputMode.saveLog,enum_Error.localErr)
                    #登录成功和注册中的时候设置心跳,主要是注册中要写,否则注册时间过长,可能会导致注册超时
                    if self.loginFlag == enum_State_code.success.value or self.loginFlag == enum_State_code.reging.value:
                        self.setHeartTime()
                    continue#不用入列表
                #除了心跳消息,其它消息增加到处理列表,通知处理线程顺序处理协议
                self.add_dual_data(protocolHead,byteContent[headSize:])
        except Exception as e:
            self.cacheBuf = bytes()
            self.protocolHead = None
            self.headSize = 0
            output(f"[protocolDataMgr.get_data]:处理数据时出错:{e}",outputMode.saveLog,enum_Error.localErr)
    #检查是否需要回复,如果需要回复则调用设置结果,通过等待
    def check_call(self,serialNum,protocolHead,data):
        bRes = False
        with self.dataLock:
            if serialNum in self.sendList:
                bRes = True
                self.sendList[serialNum].setRes(protocolHead,data)
                del self.sendList[serialNum]
        return bRes

class protocolSocket(socketInterface):
    def __init__(self,name='',nId =0,ipAddress='127.0.0.1',port=8080,tcpType=0,parameters=None,dualTargetClsss=None,bLogin=True,collectDevice=None):
       #调用父类的构造函数
       super().__init__(tools.checkName(name,nId,'protocolSocket'),ipAddress,port,tcpType,parameters)  # 如果父类构造函数需要参数，需在这里传递
       self.bLogin          = bLogin
       self.collectDevice   = collectDevice
       self.dualTaget       = dualTargetClsss
    #析构函数
    def __del__(self):
        pass
    def callFunc(self,socket,address)->protocolDataMgr:
        try:
            if self.collectDevice is not None:
                return protocolDataMgr(socket,address,self.socket_type,self.parameters,self.dualTaget,self.name,self.bLogin,self.collectDevice)
            return None
        except Exception as e:
            output(f"[异常]:[protocolSocket.callFunc]:{e}",outputMode.saveLog,enum_Error.localErr)
    def send_data(self,bEncode,bNeedCall,data,cmdCode:Enum_Protocol_Cmd=None,serialNum:int=None,proMgr:protocolMgr =None,uuid='',deviceId=''):
        resList = []
        if not self.m_tcp_mgr:
            output(f"[protocolSocket.send_data]:没有实例,不能发送数据",outputMode.saveLog,enum_Error.localErr)
            return resList
        if self.m_tcp_mgr.socket_type == socket_type.tcp_client.value:
            if self.m_tcp_mgr.socketClient is None:
                output(f"[protocolSocket.send_data]:没有连接,不能发送数据",outputMode.saveLog,enum_Error.localErr)
                return resList
            callRes = self.m_tcp_mgr.socketClient.send_data(bEncode,bNeedCall,data,cmdCode,serialNum,proMgr,uuid)
            if callRes:resList.append(callRes)
            return resList
        #发送给服务器所有连接的客户端
        if self.m_tcp_mgr.socket_type == socket_type.tcp_server.value:
            resList = []
            with self.m_tcp_mgr.listLock:
                for client in self.m_tcp_mgr.clientList:
                   if not tools.is_empty(deviceId) :
                       if client.deviceId == deviceId:
                           callRes = client.send_data(bEncode,bNeedCall,data,cmdCode,serialNum,proMgr,uuid)
                           if callRes:resList.append(callRes)
                           break
                   else:
                       callRes = client.send_data(bEncode,bNeedCall,data,cmdCode,serialNum,proMgr,uuid)
                       if callRes:resList.append(callRes)
            return resList
        #没有socket类型
        return resList
