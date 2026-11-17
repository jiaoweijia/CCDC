import socket
import threading
import sys
import gc
#日志接口
from ..common.logManager import output, outputMode,enum_Error
#socket基础类
from .socket_define import*
from ..common import commonTools as tools
#tcp服务器类，用于监听端口
class cupd_server(socket_base):
    def __init__(self,name, host,port,param):
        super().__init__(name, host,port,socket_type.udp_server.value)#调用基类
        self.serverThreadHandle     = None              #服务监听线程句柄
        self.is_run                 = False             #运行标识
        self.socketLock             = threading.Lock()  #socket锁
        self.server_socket          = None              #监听的socket句柄，用于停止时强制关闭，会有一个问题，调试时会报错，但是不影响正常使用
        self.socketClient           = None              #实例,通过self.callback调用创建的具体实例
        self.callback               = None              #用于生成实例类
        
    #停止监听线程和检测线程
    def stop(self):
        #停止服务线程
        if self.serverThreadHandle is not None:
            self.is_run = False
            with self.socketLock:
                if self.server_socket is not None:
                    try:
                        #if self.server_socket.fileno() != -1:
                        #    self.server_socket.shutdown(socket.SHUT_RD)
                        self.server_socket.close()
                        #self.server_socket.shutdown(socket.SHUT_RDWR)
                    except Exception as e:
                        pass
                    self.server_socket = None
            #等待线程正确退出
            self.serverThreadHandle.join()
    def run(self):
        try:
            if self.serverThreadHandle is None:
                self.is_run = True  # 运行标志 - 已运行
                self.serverThreadHandle = threading.Thread(
                    target=self.run_server,
                    args=(self.host, self.port),
                    daemon=True  # 设置为守护线程，主程序退出时自动终止
                )
                # 位置参数元组)  # 定义线程
                self.serverThreadHandle.start()  # 启动线程
            
            output(f'[cudp_server.run.{self._name}]启动成功 ',outputMode.saveLog,enum_Error.localInfo)  # 写日志
            return True
        except Exception as e:
            self.is_run = False  # 运行标志 - 未运行
            output(f'[异常]:[cudp_server.run.{self._name}]: {e}',outputMode.saveLog,enum_Error.localErr)  # 写日志
        return False
    #监听线程
    def run_server(self,host, port):
        """启动TCP服务器"""
        # 创建TCP套接字
        while self.is_run is True:
            try:
                #with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
                
                server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                
                # 设置套接字选项，允许地址重用
                server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                # 绑定地址和端口
                server_socket.bind((host, port))
                
                with self.socketLock:   self.server_socket = server_socket #给全局,用于退出服务使用

                if self.callback is not None:   self.socketClient = self.callback(None,(self.host,self.port))  #创建实例
                output(f"[udp_server.run_server]:[{self._name}]:[{host}:{port}] 开始接收数据",outputMode.saveLog,enum_Error.localInfo)
                while self.is_run is True:
                    # 接受客户端连接（阻塞调用）
                    try:
                       data,addr = server_socket.recvfrom(1024)
                       if self.socketClient :self.socketClient.callback(data)
                    except Exception as e:
                        
                        break
                        #tools.output(f"处理客户端连接时出错: {e}",outputMode.saveLog,enum_Error.localInfo)
            except Exception as e:
                pass
                #tools.output(f"处理客户端连接时出错: {e}",outputMode.saveLog,enum_Error.localInfo)

        output(f"[ctcp_server]:[{self._name}]监听线程退出",outputMode.saveLog,enum_Error.localInfo)
