import socket
import threading
import time

#日志接口
from ..common.logManager import output, outputMode,enum_Error
from ..protocol.data_protocol import defProtocol
#socket基础类
from .socket_define import*
class cudp_client(socket_base):
    def __init__(self,name, host,port,param):
        self.clientThread = None
        self.is_run = False
        self.host = host
        self.port = port
        self._serialNum = 0
        self.dataLock = threading.Lock()
        self._name = f"{name}: {host}:{port}"
        self.socketLock = threading.Lock()
        self.socketClient= socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
    @property
    def serialNum(self)->int:
        nRes = 0
        with self.dataLock:
            nRes = self._serialNum
            self._serialNum += 1
            self._serialNum %= 256
        return nRes
    def send_data(self,bEncode,needCall,data, cmdCode,serialNum,proMgr,uuid='',bBlxx=False):
        callMemList = []
        try:
            proMgr = proMgr if proMgr is not None else defProtocol
            if proMgr is  None or cmdCode is None:
                return callMemList
            if serialNum is None:
                serialNum = serialNum if serialNum is not None else self.serialNum

            dataByte = proMgr.encode_tcp_protocol(cmdCode,serialNum,data)
            with self.socketLock:
                if self.socketClient is not None:
                    self.socketClient.sendto(dataByte,(self.host,self.port))
        except Exception as e:
            tools.output(f"发送数据时出错: {e}",outputMode.saveLog,enum_Error.localErr)
        return callMemList

    def stop(self):
        if self.clientThread is not None:
            self.is_run = False
            with self.socketLock:
                if self.socketClient is not None:
                    self.socketClient.close()
                    self.socketClient = None
            self.clientThread.join()

    def run(self):
        try:
            if self.clientThread is None:
                self.is_run = True  # 运行标志 - 已运行
                self.clientThread = threading.Thread(
                    target=self.run_client,
                    args=(self.host, self.port),
                    daemon=True  # 设置为守护线程，主程序退出时自动终止
                )
                # 位置参数元组)  # 定义线程
                self.clientThread.start()  # 启动线程
                tools.output(f'[{self._name}] Start [OK] ',outputMode.saveLog,enum_Error.localInfo)  # 写日志
            return True
        except Exception as err:
            self.is_run = False  # 运行标志 - 未运行
            tools.output(f'[{self._name}] Start Exception: ' + str(err.args),outputMode.saveLog,enum_Error.localErr)  # 写日志
        return False
    
    def run_client(self,host, port):
        """启动TCP服务器"""
        # 创建TCP套接字
        while self.is_run is True:
            try:
                if self.socketClient is not None:
                    
                    time.sleep(1)
                    continue

                # 创建 TCP 套接字
                with self.socketLock:
                    self.socketClient= socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                

            except socket.timeout:
                tools.output(f"{self._name}，连接超时",outputMode.saveLog,enum_Error.localErr)
            except ConnectionRefusedError:
                tools.output(f"{self._name}，连接被拒绝，服务器可能未运行或端口错误",outputMode.saveLog,enum_Error.localErr)
            except Exception as e:
                tools.output(f"{self._name}，发生错误: {e}",outputMode.saveLog,enum_Error.localErr)
            finally:
                 time.sleep(1)