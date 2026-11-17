import socket
import threading
import time
from ..common.logManager import output, outputMode,enum_Error
from .socket_define import*
#tcp客户端，主要是增加自动连接和检测在线功能，连接成功后通过调用回调函数生成具体的实例
class ctcp_client(socket_base):
    def __init__(self,name,host,port,param):
        super().__init__(name,host,port,socket_type.tcp_client.value,param)
        self.clientThread   = None #用户线程,用于断开自动重新连接
        self.is_run         = False
        self._name          = f"{name}:{host}:{port}"
        self.socketLock     = threading.Lock()
        self.socketClient   = None
        self.callback       = None#回调函数,用于生成socket接收数据类实例
    #发送数据
    def send_data(self,data):
        bRes = False
        strErr=''
        try:
            with self.socketLock:
                if self.socketClient is not None:
                    bRes,strErr =self.socketClient.socket_send_data(data)
                else:
                    strErr = "没有连接"
        except Exception as e:
            output(f"[异常]:[ctcp_client.send_data]:[{self._name}]:{e}",outputMode.saveLog,enum_Error.localErr)
        return bRes,strErr
    #停止连接线程
    def stop(self):
        if self.clientThread is not None:
            self.is_run = False
            with self.socketLock:
                if self.socketClient is not None:
                    self.socketClient.close()
                    self.socketClient = None
            self.clientThread.join()
    #启动连接线程
    def run(self):
        try:
            self.stop()
            if self.clientThread is None:
                self.is_run = True  # 运行标志 - 已运行
                self.clientThread = threading.Thread(
                    target=self.run_client,
                    args=(self.host, self.port),
                    daemon=True  # 设置为守护线程，主程序退出时自动终止
                )
                # 位置参数元组)  # 定义线程
                self.clientThread.start()  # 启动线程
            return True
        except Exception as err:
            self.is_run = False  # 运行标志 - 未运行
            output(f'[ctcp_client.run]:[{self._name}异常]:{e} ',outputMode.saveLog,enum_Error.localInfo)  # 写日志
        return False
    def run_client(self,host, port):
        """启动TCP服务器"""
        # 创建TCP套接字
        nReconnectTime = self.reconnect_interval/1000
        while self.is_run is True:
            try:
                if self.socketClient is not None and self.socketClient.isValidSocket():
                    self.socketClient.checkHeart()
                    time.sleep(1)
                    continue
                # 创建 TCP 套接字
                client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                # 设置超时时间（可选）
                client_socket.settimeout(10)#10秒连接超时
                output(f"[ctcp_client.run_client]:{self._name}，正在连接 ...",outputMode.saveLog,enum_Error.localInfo)
                # 连接服务器
                client_socket.connect((host, port))
                #设置超时没阻塞
                client_socket.settimeout(None)
                output(f"[ctcp_client.run_client]:{self._name}，连接成功",outputMode.saveLog,enum_Error.localInfo)
                if self.callback is not None:   self.socketClient = self.callback(client_socket,(self.host,self.port))  #创建实例
                continue
            except socket.timeout as e :
                output(f"[异常]:[ctcp_client.run_client.socket.timeout]:[{self._name}]:{e}",outputMode.saveLog,enum_Error.localErr)
            except ConnectionRefusedError as e:
                output(f"[异常]:[ctcp_client.run_client.ConnectionRefusedError]:[{self._name}]:{e}",outputMode.saveLog,enum_Error.localErr)
            except Exception as e:
                output(f"[异常]:[ctcp_client.run_client.Exception]:[{self._name}]:{e}",outputMode.saveLog,enum_Error.localErr)
            time.sleep(nReconnectTime)
        output(f"[ctcp_client.run_client]:{self._name}，线程退出",outputMode.saveLog,enum_Error.localInfo)