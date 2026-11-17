import os
for _ in range(2):
    try:
        import  redis
        break
    except ModuleNotFoundError:
        os.system('pip install redis')

##禁用redis的内部输出
import logging
logging.getLogger("redis").setLevel(logging.WARNING) 
########
from typing import Union, List, Dict, Any, Optional
from ..common.jsonFunc import *
from ..common.baseClass import  *
#日志接口
from ..common.logManager import output, outputMode,enum_Error
from ..common.baseClass import commonBase
class redis_write_info:
    def __init__(self,key=None,hashdata=None):
        self.key = key
        self.hashdata = hashdata

#redis连接接口,通用类
class RedisHelper:
    #"""Redis 操作助手类"""
    def __init__(self, host: str = 'localhost',port: int = 6379,password: Optional[str] = None,
                 db: int = 0,decode_responses: bool = True,connect_timeout=1,socket_timeout=1):
        """
        初始化 Redis 连接
        
        Args:
            host: Redis 服务器地址
            port: Redis 服务器端口
            password: 连接密码（如果有）
            db: 选择的数据库编号
            decode_responses: 是否自动解码响应
        """
        self.host               = host
        self.port               = port
        self.password           = password
        self.db                 = db
        self.decode_responses   = decode_responses
        self.connect_timeout    = connect_timeout
        self.socket_timeout     = socket_timeout
        try:
            self.redis_client = redis.Redis(host=host,port=port,password=password,
                                            db=db,decode_responses=decode_responses,
                                            socket_connect_timeout=connect_timeout,  # 连接超时 1 秒
                                            socket_timeout = socket_timeout)
            # 测试连接
            self.redis_client.ping()
            #output(f"成功连接到 Redis 服务器: {host}:{port}/{db}",outputMode.saveLog,enum_Error.localErr)
        except redis.exceptions.ConnectionError as e:
            self.redis_client = None
            output(f"[异常]:[{tools.cmn(self)}.redis.exceptions.ConnectionError]:{e}",outputMode.saveLog,enum_Error.localErr)
        except Exception as err:
            self.redis_client = None
            output(f"[异常]:[{tools.cmn(self)}.Exception]:{err}",outputMode.saveLog,enum_Error.localErr)
    def __del__(self):
        pass
    # === 字符串操作 ===
    def isconnect(self,reconnect=False):
        try:
            if self.redis_client is None:
                self.redis_client = redis.Redis(
                    host=self.host,
                    port=self.port,
                    password=self.password,
                    db=self.db,
                    decode_responses=self.decode_responses,
                    socket_connect_timeout=self.connect_timeout,  # 连接超时 1 秒
                    socket_timeout = self.socket_timeout
                )
            if self.redis_client is None:
                return False
            # 测试连接
            response = self.redis_client.ping()
            if response:
                return True
            else:
                if reconnect:
                    self.redis_client.connect()
                    return self.isconnect(False)
        except Exception as e:
            output(f"[异常]:[{tools.cmn(self)}.Exception]:{e}",outputMode.saveLog,enum_Error.localErr)
        return False
    def set(self, key: str, value: str, ex: Optional[int] = None, nx: bool = False) -> bool:
        """
        设置键值对
        
        Args:
            key: 键
            value: 值
            ex: 过期时间（秒）
            nx: 如果为 True，则只在键不存在时设置
        
        Returns:
            设置成功返回 True，否则返回 False
        """
        return self.redis_client.set(key, value, ex=ex, nx=nx)
    
    def get(self, key: str) -> Optional[str]:
        """获取键对应的值"""
        return self.redis_client.get(key)
    
    def delete(self, key: str) -> int:
        """删除键"""
        return self.redis_client.delete(key)
    
    def exists(self, key: str) -> bool:
        """检查键是否存在"""
        return self.redis_client.exists(key) == 1
    
    # === 哈希操作 ===
    
    def hset(self, name: str, key: str, value: str) -> int:
        """设置哈希字段值"""
        return self.redis_client.hset(name, key, value)
    
    def hget(self, name: str, key: str) -> Optional[str]:
        """获取哈希字段值"""
        return self.redis_client.hget(name, key)
    
    def hgetall(self, name: str) -> Dict[str, str]:
        """获取整个哈希表"""
        return self.redis_client.hgetall(name)
    
    def hdel(self, name: str, *keys) -> int:
        """删除哈希字段"""
        return self.redis_client.hdel(name, *keys)
    
    # === 列表操作 ===
    
    def lpush(self, key: str, *values) -> int:
        """向列表左侧插入值"""
        return self.redis_client.lpush(key, *values)
    
    def rpush(self, key: str, *values) -> int:
        """向列表右侧插入值"""
        return self.redis_client.rpush(key, *values)
    
    def lrange(self, key: str, start: int = 0, end: int = -1) -> List[str]:
        """获取列表范围内的元素"""
        return self.redis_client.lrange(key, start, end)
    
    def lpop(self, key: str) -> Optional[str]:
        """从列表左侧弹出元素"""
        return self.redis_client.lpop(key)
    
    def rpop(self, key: str) -> Optional[str]:
        """从列表右侧弹出元素"""
        return self.redis_client.rpop(key)
    
    # === 集合操作 ===
    
    def sadd(self, key: str, *values) -> int:
        """向集合添加元素"""
        return self.redis_client.sadd(key, *values)
    
    def smembers(self, key: str) -> List[str]:
        """获取集合所有元素"""
        return list(self.redis_client.smembers(key))
    
    def sismember(self, key: str, value: str) -> bool:
        """检查元素是否在集合中"""
        return self.redis_client.sismember(key, value)
    
    def srem(self, key: str, *values) -> int:
        """从集合中移除元素"""
        return self.redis_client.srem(key, *values)
    
    # === 有序集合操作 ===
    
    def zadd(self, name: str, mapping: Dict[str, float]) -> int:
        """向有序集合添加元素"""
        return self.redis_client.zadd(name, mapping)
    
    def zrange(self, name: str, start: int = 0, end: int = -1, withscores: bool = False) -> List[Union[str, tuple]]:
        """获取有序集合范围内的元素"""
        return self.redis_client.zrange(name, start, end, withscores=withscores)
    
    def zrem(self, name: str, *members) -> int:
        """从有序集合中移除元素"""
        return self.redis_client.zrem(name, *members)
    
    # === 通用操作 ===
    
    def expire(self, key: str, seconds: int) -> bool:
        """设置键的过期时间"""
        return self.redis_client.expire(key, seconds)
    
    def keys(self, pattern: str = '*') -> List[str]:
        """获取匹配模式的所有键"""
        return self.redis_client.keys(pattern)
    def readList(self,dataList,queryNode):
        results = []
        try:
            pip = self.redis_client.pipeline()
            for val in dataList:
                pip.hmget(val,queryNode)
            results = pip.execute()
        except Exception as e:
            output(f"[异常]:[{tools.cmn(self)}]:{e}",outputMode.saveLog,enum_Error.localErr)

        return results
    def flushdb(self) -> None:
        """清空当前数据库"""
        self.redis_client.flushdb()
    def updateList(self,dataList,groupNum=None):
        results = False
        try:
            pip = self.redis_client.pipeline()
            curNum = 0
            for val in dataList:
                curNum+=1
                pip.hmset(val.key,mapping=val.hashdata)
                if groupNum:
                    if curNum >= groupNum:
                        try:
                            results = pip.execute()
                        except Exception as e:
                            output(f"[异常]:[{tools.cmn(self)}.execute.Exception]:{str(e.args)}",outputMode.saveLog,enum_Error.localErr)
                        curNum = 0
            if curNum != 0:
                results = pip.execute()
            results = True
        except Exception as e:
            output(f"[异常]:[{tools.cmn(self)}.Exception]:{e}",outputMode.saveLog,enum_Error.localErr)
        return results
class redisThread(deviceThread):
    def __new__(cls, *args, **kwargs):
        # 控制实例创建过程,某些条件下可以返回None
        #if some_condition:  # 某些条件下,控制类创建的时候返回None
        #    return None     # 显式返回 None
        return super().__new__(cls)
    def __init__(self, name:str, redisMgrInfo:'redisMgr'):
        super().__init__(name,redisMgrInfo)
        self.uuid = redisMgrInfo.uuid
        self.name = redisMgrInfo._name
        self.host = redisMgrInfo.host
        self.port = redisMgrInfo.port
        self.password = redisMgrInfo.password
        self.db = redisMgrInfo.db
        self.decode_response = redisMgrInfo.decode_response

    def run(self,dev):
        nEventTime = self.threadWait/1000
        self.dev:redisMgr
        while self.runFlag:
            redisHelp = RedisHelper(self.dev.host,self.dev.port,
                                    self.dev.password,self.dev.db,
                                    self.dev.decode_response,
                                    self.dev.connect_timeout,
                                    self.dev.socket_timeout)
            if redisHelp is None or redisHelp.isconnect() is False:
                self.threadEvent.wait(nEventTime)
                self.threadEvent.clear()
                continue

            while self.runFlag:
                try:
                    self.threadEvent.wait(nEventTime)
                    self.threadEvent.clear()
                    writeList = dev.copyRedisData()
                    if writeList:
                        if redisHelp.updateList(writeList) is False:
                            if redisHelp.isconnect() is False:#add by lhl 20251107 判断连接是否断开
                                break
                except Exception as e:
                    output(f"[{tools.cmn(self)}]:[错误]:{e}",outputMode.saveLog,enum_Error.localErr)

class redisInfo(jsonBase,commonBase):
    def __init__(self,obj=None,bDelNode=False):
        #构建基类的初始化,尽量使用基类加__init__,
        #因为如果多基类继承,super().__init__()只针对第一个基类,以上面为例,super()是jsonBase
        commonBase.__init__(self)
        if obj is None: obj = {}    #如果是None 生成一个空的{}这样只用初始化一次
        #初始化配制信息
        jsonBase.__init__(self,obj) #生成self.JsonNode成员
        self.uuid               = self.JsonNode.stringNode( "uuid"              ,"")    #唯一标识
        self._name              = self.JsonNode.stringNode( "vch_name"          ,"")    #名称
        self.host               = self.JsonNode.stringNode( "vch_host"          ,"")    #redis服务器地址
        self.port               = self.JsonNode.intNode(    "n_port"            ,0)     #redis服务器端口
        self.password           = self.JsonNode.stringNode( "vch_pwd"           ,"")    #redis服务器密码
        self.db                 = self.JsonNode.intNode(    "n_db"              ,0)     #redis服务器db
        self.decode_response    = self.JsonNode.intNode(    "n_decode_responses",0)     #编码
        self.groupnum           = self.JsonNode.intNode(    "n_groupnum"        ,0)     #一次操作记录
        self.execnum            = self.JsonNode.intNode(    "n_execnum"         ,1)     #执行线程个数
        self.connect_timeout    = self.JsonNode.intNode(    "connect_timeout"   ,1)     #连接超时
        self.socket_timeout     = self.JsonNode.intNode(    "socket_timeout"    ,1)     #网络超时
        self.isenable           = self.JsonNode.intNode(    "isenable"          ,0)     #是否使用
        #定义自己使用的成员
        self.dataList           = []
        self.threadList         = []
        #####
        ##错误修正 
        #保证线最少一个
        if self.execnum <= 0:   self.execnum = 1
        ###

        #使用基类需要增加这个来设置是否删除dict
        self.delJsonNode(bDelNode)
    def run(self):
        for i in range(self.execnum):
            try:
                dev = redisThread(f"redisMgr-{i}", self)
                if dev is None: continue    #如果是空,继续下一操作
                self.threadList.append(dev) #增加到线程列表中,用于停止使用
                dev._run()                  #启动线程
            except Exception as e:
                output(f"[异常]:[{tools.cmn(self)}]:{e}" ,outputMode.saveLog,enum_Error.localErr)
    def stop(self):
        for dev in self.threadList:
            dev._stop()
    def copyRedisData(self,num=None):
        bList=[]
        with self.dataLock:
            size = len(self.dataList)
            if num is None or num >= size:
                bList=self.dataList[0:]
                self.dataList.clear()
            else:
                bList=self.dataList[0:num]
                self.dataList=self.dataList[num:]
        return bList
    def updateData(self,data):
        if not self.threadList:
            return
        with self.dataLock:
            self.dataList.append(data)
        for member in self.threadList:
            member._flush()

    def updateDataList(self,dataList):
        if not self.threadList:
            return
        with self.dataLock:
            self.dataList.extend(dataList)
        for member in self.threadList:
            member._flush()


class redisMgr(paraBase):
    '''
        redis管理,mgr原因无论时一个设备或者是多个设备，
        都使用mgr来管理,这样可以统一管理,也方便处理逻辑和扩展
    '''
    def __init__(self,data= None,target_class = None):
        super().__init__(data,target_class)

    def stop(self):
        for dev in self.memberList:dev.stop()
    def updateData(self,data):
        #如果配置多个redis，说明数据需要给每个实例写
        with self.interfaceLock:
            for dev in self.memberList: dev.updateData(data)
    def updateDataList(self,dataList):
        #如果配置多个redis，说明数据需要给每个实例写
        with self.interfaceLock:
            for dev in self.memberList: dev.updateDataList(dataList)
    def run(self):
        for dev in self.memberList: dev.run()


## 使用示例
#if __name__ == "__main__":
#    # 初始化 Redis 客户端
#    redis_helper = RedisHelper(host='localhost', port=6379, db=0)
    
#    # 字符串操作示例
#    redis_helper.set('name', 'Alice')
#    print(redis_helper.get('name'))  # 输出: Alice
    
#    # 哈希操作示例
#    redis_helper.hset('user:1', 'age', '30')
#    print(redis_helper.hget('user:1', 'age'))  # 输出: 30
    
#    # 列表操作示例
#    redis_helper.lpush('tasks', 'task1', 'task2')
#    print(redis_helper.lrange('tasks', 0, -1))  # 输出: ['task2', 'task1']
    
#    # 集合操作示例
#    redis_helper.sadd('fruits', 'apple', 'banana')
#    print(redis_helper.smembers('fruits'))  # 输出: ['apple', 'banana']
    
#    # 有序集合操作示例
#    redis_helper.zadd('scores', {'Alice': 95, 'Bob': 88})
#    print(redis_helper.zrange('scores', 0, -1, withscores=True))  # 输出: [('Bob', 88.0), ('Alice', 95.0)]