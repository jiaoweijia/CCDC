from    __future__ import annotations  # 放在文件顶部
import  struct
import  inspect
from    enum import unique, Enum, auto
from    ..common.logManager import output, outputMode,enum_Error
from    ..common import define
#json管理
from    ..common.jsonFunc  import*
from    ..database.mysql import *

g_MainMgr = None
def setManager(manager:'manager'):
    global g_MainMgr
    g_MainMgr = manager

def getManager():
    global g_MainMgr
    return g_MainMgr

@unique#枚举类修饰符，唯一检查
class enum_State_code(Enum):
    '''
    资源信息，状态代码
    '''
    success     =   0   #成功
    fail        =   1   #失败
    logining    =   2   #登录中
    reg         =   3   #注册
    reging      =   4   #注册中
    doing       =   5   #执行中,如果指令对应返回doing说明正在执行可以继续等待
    timeout     =   6   #超时
@unique#枚举类修饰符，唯一检查
class enum_transmit_type(Enum):
    '''
    资源信息，状态代码
    '''
    none            = -1    #没有类型
    collect         = 0     #边平台采集
    server          = 1     #采集服务器
    control_client  = 2     #客户端控制端口
    control_server  = 3     #服务器端控制端口
    
@unique#枚举类修饰符，唯一检查
class Enum_Protocol_Cmd(Enum):
    '''协议命令编码'''
    defaultCmd =        -1      #默认值
    reg =               0x1001  #注册
    reg_ack=            0x1101  #注册响应
    login =             0x1002  #登录
    login_ack=          0x1102  #登录响应
    heartbeat =         0x1003  #心跳
    heartbeat_ack=      0x1103  #心跳响应
    uploaddata =        0xF001  #数据上传
    uploaddata_ack=     0xF101  #数据上传响应
    downloaddata =      0xF002  #控制
    downloaddata_ack =  0xF102  #控制响应

@unique#枚举类修饰符，唯一检查
#控制类型
class enum_uploaddata_Type(Enum):
    def_none        = -1    #没有定义
    collectVal      = 0     #采集值
    historyCollect  = 1     #历史采集数据
    nodelist        = 2     #节点列表
    #setVal          = 3 #设置值

class resInfo():
    def __init__(self,obj = None):
        self.state = enum_State_code.fail.value
        self.describe = ""
        self.paras_dict(obj)
    def to_dict():
        resDict = {}
        resDict["result"]   = self.state
        resDict["describe"]  = self.describe
        return resDict
    def paras_dict(self,dict):
        if dict is None : return
        self.state      = tools.safe_DictValue(dict,'result',enum_State_code.fail.value)
        self.describe   = tools.safe_DictValue(dict,'describe',"")

class transmitDataInfo:
    def __init__(self,uuid,transmituuid,nodeuuid):
        self.uuid           = uuid
        self.transmituuid   = transmituuid
        self.nodeuuid       = nodeuuid

class transmitInfo:
    def __init__(self,db=None,uuid="",device_uuid="",name="",serverIp="",serverport=0,socket_type=0,
                 protocolType=0,reconnect_interval=0,threadNum=0,heart_interval=0,timeout_interval=0,
                 exparam="",isenable=0,transmitType=enum_transmit_type.collect.value):
        """"""
        self.uuid               = uuid                                                  #uuid
        self.device_uuid        = device_uuid                                           #设备uuid
        self.name               = name                                                  #名称
        self.host               = serverIp                                              #服务器地址
        self.port               = serverport                                            #服务器端口
        self.socket_type        = socket_type                                           #网络类型
        self.protocoltype       = protocolType                                          #协议类型
        self.reconnect_interval = reconnect_interval if reconnect_interval else 10000   #重连间隔
        self.threadNum          = threadNum if threadNum else 0                         #线程数
        self.heart_interval     = heart_interval if heart_interval else 0               #心跳间隔
        self.timeout_interval   = timeout_interval if timeout_interval else 0           #超时时间
        self.exparam            = exparam if exparam else ""                            #扩展参数
        self.isenable           = isenable                                              #是否使用
        self.transmitType       = transmitType                                          #传输类型
        self.loadFromDb(db)
        #
        self.dataList           ={}
        self.socketInterface    = None
    
    def stop(self):
        if self.socketInterface:    self.socketInterface.stop()
    def updateDataDict(self):
        pass
    def sendcollectData(self,bEncode,bNeedCall, data,cmdCode:Enum_Protocol_Cmd=None,serialNum:int=None,proMgr:protocolMgr =None,uuid=''):
        try:
            if self.socketInterface:    return self.socketInterface.send_data(bEncode,bNeedCall,data,cmdCode,serialNum,proMgr)
        except Exception as e:
            output(f"[sendcollectData]:发送数据失败{e}",outputMode.saveLog,enum_Error.localErr)
        return False
    def to_para(self):
        dictRes ={}
        dictRes['transmit_type']        = self.transmitType
        dictRes['protocol_type']        = self.protocoltype
        dictRes['reconnect_interval']   = self.reconnect_interval
        dictRes['threadNum']            = self.threadNum
        dictRes['heart_interval']       = self.heart_interval
        dictRes['timeout_interval']     = self.timeout_interval
        dictRes["exparam"]              = self.exparam
        return dictRes
    def loadFromDb(self,db):
        if not db: return
        self.uuid               = safe_DictValue(db,'uuid'                  ,self,uuid)
        self.device_uuid        = safe_DictValue(db,'device_uuid'           ,self.device_uuid)  #设备uuid
        self.name               = safe_DictValue(db,'vch_name'              ,self.name)         #名称
        self.host               = safe_DictValue(db,'vch_server_host'       ,self.host)         #服务器地址
        self.port               = safe_DictValue(db,'n_server_port'         ,self.port)         #服务器端口
        self.socket_type        = safe_DictValue(db,'n_socket_type'         ,self.socket_type)  #网络类型
        self.protocoltype       = safe_DictValue(db,'n_protocol_type'       ,self.protocoltype) #协议类型
        self.reconnect_interval = safe_DictValue(db,'n_reconnect_interval'  ,10000)             #重连间隔
        self.threadNum          = safe_DictValue(db,'n_threadnum'           ,0)                 #线程数
        self.heart_interval     = safe_DictValue(db,'n_heart_interval'      ,0)                 #心跳间隔
        self.timeout_interval   = safe_DictValue(db,'n_timeout_interval'    ,0)                 #超时时间
        self.exparam            = safe_DictValue(db,'vch_exparam'           ,"")                #扩展参数
        self.isenable           = safe_DictValue(db,'n_isenable'            ,0)                 #是否使用

class transmitMgr:
    def __init__(self,collectDev):
        self.transmitList   = []            #传输实例列表
        self.collectDevice  = collectDev    #一个采集节点
    def sendcollectData(self,strDeviceId,nCollectTime,dataList):
        #发送传输数据
        try:
            if self.transmitList is not None:
                dataDict                = {}
                dataDict['type']        = enum_uploaddata_Type.collectVal.value
                dataDict['deviceid']    = strDeviceId
                dataDict['collecttime'] = nCollectTime
                dataDict['data']        = [val.to_dict() for val in dataList ]
                strData                 = loadJsonNode.toJsonString(dataDict)
                for transmit in self.transmitList:
                    transmit:transmitInfo
                    transmit.sendcollectData(True,False,strData.encode('utf-8'),Enum_Protocol_Cmd.uploaddata,None)
        except Exception as e:
            output(f'[{self.__class__.__name__}.{inspect.currentframe().f_code.co_name}]:{e}',outputMode.saveLog,enum_Error.localErr)
    def run(self,target_class,dataDual_class):
        if not self.transmitList: return
        nIndex = 0
        for member in self.transmitList:
            nIndex +=1
            member:transmitInfo
            member.socketInterface = target_class(member.name,nIndex,member.host,member.port,member.socket_type,member.to_para(),dataDual_class,True,self.collectDevice)
            member.socketInterface.run()
    def stop(self):
        if not self.transmitList:           return          #没有上传设置
        for member in self.transmitList:    member.stop()   #启动上传线程

#协议处理类，包括解析和解码功能
class protocolMgr:
    headFlag  = b'\xAA\xBB'                                 #类成员，属于静态
    def __init__(self,protocolVersion,codeMode,reserve):
        self.protocolVersion    = protocolVersion           #协议版本号
        self.codeMode           = codeMode                  #协议号
        self.reserve            = reserve
    @staticmethod
    def cmd_serial(protocolHead):
        #cmdCode     = protocolHead['cmdCode']
        #serialNum   = protocolHead['serialNum']
        #return cmdCode,serialNum
        return protocolHead['cmdCode'],protocolHead['serialNum']
    @staticmethod
    #struct方式解析数据格式，数据去掉了AABB
    def parse_tcp_data_head(data: bytes):
        #不增加异常处理，外面使用异常处理，主要是传入参数不全
        #!网络字节顺序,大端
        # '>'表示大端字节序
        # H二位无符号2位,h二位有符号
        # B一位无符号1位,b一位有符号 
        # I表示无符号整数32位，i表示有符号整数
        # Q表示无符号整数64位，q表示有符号整数
        # f表示32浮点数
        # d表示64浮点数
        #'10s'表示10字节字符串 
        #生成解析格式
        format_str = '>HHBBHHI'
        # 解析数据
        # 计算结构体大小
        size = struct.calcsize(format_str)
        if len(data) < size:    raise ValueError("数据长度不足") #不算错误,注意外围调用时要加上ValueError错误处理
        
        # 解包数据
        (crc, cmdCode, serialNum, protocolVersion,codeMode,reserve, contentSize) =  struct.unpack(format_str, data[:size])
        
        return {
            'crc'               : crc,              #crc16校验
            'cmdCode'           : cmdCode,          #命令码
            'serialNum'         : serialNum,        #序列码
            'protocolVersion'   : protocolVersion,  #协议版本
            'codeMode'          : codeMode,         #命令模式
            'reserve'           : reserve,          #预留
            'contentSize'       : contentSize,      #内容长度
            'headSize'          : size              #头长度
        },size

    @staticmethod
    def get_protocol_head(cacheBuf: bytes):
        headPos = cacheBuf.find(protocolMgr.headFlag)
        if headPos<0:#没有发现AABB
            #判断最后一个字符是否为AABB，防止数据丢失
            return None,0,cacheBuf[-1:] if len(cacheBuf)>0 and cacheBuf[-1] == b'\xAA' else bytes()
        else:
            try:
                #output(f"头标识位置{headPos}",outputMode.saveLog,enum_Error.localInfo)
                protocolHead,headSize = protocolMgr.parse_tcp_data_head(cacheBuf[headPos+2:])#解析协议头
                return protocolHead,headSize,cacheBuf[headPos+2:]#缓存数据去掉AABB
            except ValueError as e:
                #output(f"协议头长度不够，等待下次处理",outputMode.saveLog,enum_Error.localInfo)
                return None,0,cacheBuf
            except Exception as e:
                output(f"[protocolMgr]:得到头数据错误{e}",outputMode.saveLog,enum_Error.localErr)
                return None,0,bytes()
    @staticmethod
    def get_data_info(protocolHead,headSize,cacheBuf: bytes):
        totalSize   = protocolHead['contentSize']+headSize
        bufSize     = len(cacheBuf)
        if len(cacheBuf)<totalSize:
            #output(f"数据长度不够，等待下次处理",outputMode.saveLog,enum_Error.localInfo)
            return False,bytes(),protocolHead,headSize,cacheBuf
        crc16Value  = protocolHead['crc']
        byteContent = cacheBuf[0:totalSize]#复制出完整的协议数据
        #重置数据信息
        cacheBuf    = cacheBuf[totalSize:]#移动缓存
        crc16Val    = tools.crc_calute.calculate_crc16(byteContent[2:])
        #crc校验，在调试时取消，验证后打开
        if crc16Val != crc16Value:
            output(f"[protocolMgr]:CRC校验错误，头crc{crc16Value},计算crc{crc16Val}",outputMode.saveLog,enum_Error.localInfo)
            return False,bytes(),None,0,cacheBuf

        return True,byteContent,None,0,cacheBuf
    def setProtocolPara(self,protocolVersion,codeMode,reserve):
        self.protocolVersion    = protocolVersion
        self.codeMode           = codeMode
        self.reserve            = reserve

    def encode_tcp_protocol(self,cmdCode,serialNum,data: bytes):
        dataByte = self.encode_tcp_data(cmdCode.value,serialNum,self.protocolVersion,self.codeMode,self.reserve,data)
        return self.add_tcp_protocol_head(dataByte)

    def encode_tcp_data(self,cmdCode,serialNum,protocolVersion,codeMode,reserve,data: bytes):
        dataSize = len(data)
        return struct.pack(f'>HBBHHI{dataSize}s',cmdCode, serialNum,protocolVersion,codeMode,reserve, dataSize,data)

    def add_tcp_protocol_head(self,data: bytes):
        crc         = tools.crc_calute.calculate_crc16(data)
        header      = struct.pack('>BB', 0xAA, 0xBB)  # 帧头
        crc_bytes   = struct.pack('>H', crc)       # CRC校验值，2字节
        return header+crc_bytes+data#0x0a为了满足现在有需要增加

class loginInfo:
    def __init__(self,obj,uuid='',name='',host='',port=0,rack=0,slot=0,
                 maxmem=0,defbyteorder='',defreserved=0,defencode='',protocoltype=0,
                 ctlport=0,enablesetval=0,reconninterval=0,collectinterval=0,status=0):
        self.uuid               = uuid
        self.name               = name
        self.host               = host
        self.port               = port
        self.rack               = rack
        self.slot               = slot
        self.maxmem             = maxmem
        self.defbyteorder       = defbyteorder
        self.defreserved        = defreserved
        self.defencode          = defencode
        self.protocoltype       = protocoltype
        self.ctlport            = ctlport
        self.enablesetval       = enablesetval
        self.reconninterval     = reconninterval
        self.collectinterval    = collectinterval
        self.status             = status
        self.paras_dict(obj)
    def to_dict(self):
        resDic = {}
        resDic['uuid']              = self.uuid
        resDic['name']              = self.name
        resDic['host']              = self.host
        resDic['port']              = self.port
        resDic['rack']              = self.rack
        resDic['slot']              = self.slot
        resDic['maxmem']            = self.maxmem
        resDic['defbyteorder']      = self.defbyteorder
        resDic['defreserved']       = self.defreserved
        resDic['defencode']         = self.defencode
        resDic['protocoltype']      = self.protocoltype
        resDic['ctlport']           = self.ctlport
        resDic['enablesetval']      = self.enablesetval
        resDic['reconninterval']    = self.reconninterval
        resDic['collectinterval']   = self.collectinterval
        resDic['status']            = self.status
        return resDic
    def paras_dict(self,dict):
        if dict is None:    return
        self.uuid           = tools.safe_DictValue(dict,'uuid'              ,"")        #str
        self.name           = tools.safe_DictValue(dict,'name'              ,"")        #str
        self.host           = tools.safe_DictValue(dict,'host'              ,"")        #str
        self.port           = tools.safe_DictValue(dict,'port'              ,0)         #int
        self.rack           = tools.safe_DictValue(dict,'rack'              ,0)         #int
        self.slot           = tools.safe_DictValue(dict,'slot'              ,0)         #int
        self.maxmem         = tools.safe_DictValue(dict,'maxmem'            ,0)         #int
        self.defbyteorder   = tools.safe_DictValue(dict,'defbyteorder'      ,"big")     #str
        self.defreserved    = tools.safe_DictValue(dict,'defreserved'       ,0)         #int
        self.defencode      = tools.safe_DictValue(dict,'defencode'         ,"utf-8")   #str
        self.protocoltype   = tools.safe_DictValue(dict,'protocoltype'      ,0)         #int
        self.ctlport        = tools.safe_DictValue(dict,'ctlport'           ,0)         #int
        self.enablesetval   = tools.safe_DictValue(dict,'enablesetval'      ,0)         #int
        self.reconninterval = tools.safe_DictValue(dict,'reconninterval'    ,0)         #int
        self.collectinterval= tools.safe_DictValue(dict,'collectinterval'   ,0)         #int
        self.status         = tools.safe_DictValue(dict,'status',0)                     #int
    def copyServerInfo(serverInfo):
        return loginInfo(None,serverInfo.uuid,serverInfo.serverName,serverInfo.serverIp,serverInfo.serverPort,
                      serverInfo.rack,serverInfo.slot,serverInfo.maxMem,serverInfo.defbyteorder,serverInfo.defreserved,
                      serverInfo.defencode,serverInfo.devType,serverInfo.ctlPort,
                      serverInfo.enableSetVal,serverInfo.reconinterval,serverInfo.collectinterval,1)
class ackInfo:
    def __init__(self,obj=None):
        self.status     = 0
        self.describe   = ''
        self.paras_dict(obj)
    def to_dict(self):
        resDic = {}
        resDic['status']    = self.status
        resDic['describe']  = self.describe
        return resDic
    def paras_dict(self,dict):
        if dict is None:
            return
        self.status     = dict['status']
        self.describe   = dict['describe']

class regerInfo:
    def __init__(self,obj=None):
        self.deviceuuid = ''
        self.dataList   = []
        self.paras_dict(obj)
    def to_dict(self):
        resDic = {}
        resDic['deviceuuid']    = self.deviceuuid
        resDic['dataList']      = [val.to_dict() for val in self.dataList ]
        return resDic
    def paras_dict(self,dict):
        if dict is None:
            return
        self.deviceuuid = dict['deviceuuid']
        self.dataList.clear()
        if 'dataList' not in dict:
            return
        dataList = dict['dataList']
        for node in dataList:
            nodeVal = define.dataInfo(node)
            self.dataList.append(nodeVal)

    def copyRegerInfo(serverInfo):
        iRes            = regerInfo()
        iRes.deviceuuid = serverInfo.uuid
        for server in serverInfo.dbBlockList:
            for node in server.collectNodeList:
                iRes.dataList.append(node)
        return iRes
#处理协议的基类
class dataDualClass:
    def __init__(self,parentMgr,protocolHead):
        self.parentMgr          = parentMgr#对应socket实例
        self.protocolHead       = protocolHead
        self.cmd,self.serialnum = protocolMgr.cmd_serial(protocolHead)

    #协议处理
    def dual_pack(self,dataContent):
        res     = None
        strCmd  = ''
        try:
            #cmd,serialnum = protocolMgr.cmd_serial(self.protocolHead)
            strCmd = "{:X}".format(self.cmd)
            method_name = 'case_' + strCmd
            method = getattr(self, method_name,self.default)
            res = method(dataContent)
        except Exception as e:
            output(f"[错误]:{e},{strCmd}",outputMode.saveLog,enum_Error.protocolErr)
            res = None
        return res
    #注册处理，暂时保留，为后期扩展
    def case_1001(self,dataContent):
        output(f"[dataDualClass]:收到注册消息",outputMode.saveLog,enum_Error.localInfo)
        return None       
    #注册响应处理，不需要处理，现在模式没有去其它服务器注册
    def case_1101(self,dataContent):
        pass
    #登录处理
    def case_1002(self,dataContent):
        return None  
    #登录回复,不会有登录指令给其它系统，所以不需要处理
    def case_1102(self,dataContent):
        pass
    #心跳处理，收到其它的心跳，回复心跳信息
    def case_1003(self,dataContent):
        strRes ='{{"L0003":"{}"}}'.format(tools.timeManager.format_time_s(tools.timeManager.get_timestamp_s()))
        serialNum = self.serialnum
        return defProtocol.encode_tcp_protocol(Enum_Protocol_Cmd.heartbeat_ack,serialNum,strRes.encode())
    #心跳回复，给其它服务或者客户端发送，收到回复后设置心跳时间
    def case_1103(self,dataContent):
        if  self.parentMgr.transmit_type == enum_transmit_type.control_client.value or self.parentMgr.transmit_type == enum_transmit_type.control_server.value:
            self.parentMgr.setHeartTime()
        #更新心跳时间，保证不超时关闭，在收到数据也心跳时间
        if self.parentMgr.loginFlag == enum_State_code.success.value:
            self.parentMgr.setHeartTime()
        pass
    #收到上报数据信息
    def case_F001(self,dataContent):
        return None  
    #上报数据响应
    def case_F101(self,dataContent):
        return None
    #控制指令
    def case_F002(self,dataContent):
        return None  
    def case_F102(self,dataContent):
        output(f"收到设置指令回复",outputMode.saveLog,enum_Error.localInfo)
        self.parentMgr.check_call(self.serialnum,self.protocolHead,dataContent)
        pass
    def default(self,dataContent):
        strCmd = "{:x}".format(self.cmd)
        output(f"[dataDualClass]:[未知协议]:{strCmd},请检查协议使用",outputMode.saveLog,enum_Error.localErr)
        return None
#全局协议类型，如果在构建协议没有特殊要求时，会使用此协议
defProtocol = protocolMgr(0x40,0,0)