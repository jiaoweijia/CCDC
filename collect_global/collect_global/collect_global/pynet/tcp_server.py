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
class ctcp_server(socket_base):
    def __init__(self,name, host,port,param):
        super().__init__(name, host,port,socket_type.tcp_server.value)#调用基类
        self.serverThreadHandle     = None              #服务监听线程句柄
        self.is_run                 = False             #运行标识
        self.socketLock             = threading.Lock()  #socket锁
        self.server_socket          = None              #监听的socket句柄，用于停止时强制关闭，会有一个问题，调试时会报错，但是不影响正常使用
        self.listLock               = threading.Lock()  #增加连接成员到列表的句柄
        self.clientList             = []                #客户端列表
        self.callback               = None              #用于生成实例类
        self.checkThreadHandle      = None              #检测线程句柄
        self.is_run_check           = False             #检测线程运行标识
        self.check_ExitEvent        = threading.Event() #检测信号，用于超时和快速停止线程使用，比sleep灵活
        self.check_Event_timeout_ms = 1000              #等待检测信息时间，超时单位毫秒
    #检测socket是否无效，如果无效则释放管理部分，如果有效考虑心跳
    def check_client(self):
        try:
            #超时时间转换成秒，因为wait是秒
            nCheckTimeout = self.check_Event_timeout_ms / 1000
            while self.is_run_check is True:
                try:
                    #等待检查退出信息
                    self.check_ExitEvent.wait(nCheckTimeout)
                    #注意要执行clear，否则会一直有信号
                    self.check_ExitEvent.clear()
                    #判断是否需要退出，为了快速退出，设置self.is_run_check设置False后设置信息，这样会在立即响应
                    if self.is_run_check is False:
                        break;

                    while True:
                        if self.is_run_check is False:
                            break;
                        invalid_client = None
                        with self.listLock:
                            for client in self.clientList:
                                if client.isValidSocket() is False:
                                    invalid_client = client
                                    self.clientList.remove(client)
                                    break
                                else:
                                    client.checkHeart()
                        if invalid_client is not None:
                            invalid_client.close()
                            del invalid_client
                            invalid_client=None
                        else:
                            break
                    gc.collect()#强制回收
                except Exception as e:
                    output(f"[ctcp_server]:检测socket时出错: {e}",outputMode.saveLog,enum_Error.localInfo)
        except Exception as e:
            output(f"[ctcp_server]:检测socket时出错: {e}",outputMode.saveLog,enum_Error.localInfo)
        output("[ctcp_server]:检测线程退出",outputMode.saveLog,enum_Error.localInfo)
    #数据直接发送给客户端，如果有特殊处理，需要单独重写
    def send_data(self,data):
        try:
            with self.listLock:
                for client in self.clientList:
                    client.socket_send_data(data)
        except Exception as e:
            output(f"[ctcp_server]:发送数据时出错: {e}",outputMode.saveLog,enum_Error.localInfo)
        return
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
        #停止检测线程
        if self.checkThreadHandle is not None:
            self.is_run_check = False
            self.check_ExitEvent.set()
            #等待线程正确退出
            self.checkThreadHandle.join()

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
            if self.checkThreadHandle is None:
                self.is_run_check = True
                self.checkThreadHandle = threading.Thread(
                    target=self.check_client,
                    daemon=True
                )
                self.checkThreadHandle.start()
                output(f'ctcp_server:[{self._name}]启动成功 ',outputMode.saveLog,enum_Error.localInfo)  # 写日志
            return True
        except Exception as e:
            self.is_run = False  # 运行标志 - 未运行
            output(f'[ctcp_server]:[{self._name}] 启动异常: {e}',outputMode.saveLog,enum_Error.localInfo)  # 写日志
        return False
    #监听线程
    def run_server(self,host, port):
        """启动TCP服务器"""
        # 创建TCP套接字
        while self.is_run is True:
            try:
                #with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
                server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                
                # 设置套接字选项，允许地址重用
                server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                # 绑定地址和端口
                server_socket.bind((host, port))
                # 开始监听，最大连接数5
                server_socket.listen(5)
                with self.socketLock:
                    self.server_socket = server_socket
                output(f"[run_server]:[{self._name}] 监听开始",outputMode.saveLog,enum_Error.localInfo)
                client_socket=None
                while self.is_run is True:
                    # 接受客户端连接（阻塞调用）
                    try:
                        client_socket=None
                        client_socket, client_address = server_socket.accept()
                        if self.callback is not None:
                            #client_socket.settimeout(1)
                            socketClient = self.callback(client_socket, client_address)#构建具体的数据处理类中，开启读写，处理线程
                            with self.listLock:
                                if socketClient is not None:
                                    self.clientList.append(socketClient)#增加到监控列表，用于心跳检测和状态检测
                                    del socketClient
                    except Exception as e:
                        if client_socket is not None:
                            client_socket.close()
                        break
                        #tools.output(f"处理客户端连接时出错: {e}",outputMode.saveLog,enum_Error.localInfo)
            except KeyboardInterrupt:
                pass
                #tools.output("服务器关闭",outputMode.saveLog,enum_Error.localInfo)
            except Exception as e:
                pass
                #tools.output(f"处理客户端连接时出错: {e}",outputMode.saveLog,enum_Error.localInfo)
        output(f"[ctcp_server]:[{self._name}]监听线程退出",outputMode.saveLog,enum_Error.localInfo)