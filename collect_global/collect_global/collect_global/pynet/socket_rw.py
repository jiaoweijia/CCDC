import threading
import time
import socket
from ..common import commonTools as tools
from ..common.logManager import output, outputMode,enum_Error
#异步读写类,这里没有业务逻辑处理，只负责发送和接收，同时根据回调函数返回信息
class socket_rw():
    #构造函数
    def __init__(self, socket,address,name = "socket_rw"):
        try:
            self.socketLock             = threading.Lock()#socket锁，主要是在关闭的时候防止并发
            self.socket                 = socket#socket对象
            self.address                = address#连接地址
            self._name                  = name#名称
            #断开回调
            self.disconnect_callback    = None
            #读取线程
            self.h_read_thread          = None#读取线程句柄
            self.read_callback          = None#读取回调函数
            ######
            #写线程
            self.h_write_thread         = None#写线程句柄
            self.write_data_lock        = threading.Lock()#写数据列表锁
            self.write_data             = []#写数据列表
            self.write_Event            = threading.Event()#写信号，用于判断实时写入
            self.write_Event_timeout_ms = 1000#超时没什么用，主要怕卡死，在插入数据时会使用Event.set通知立即执行
        except Exception as e:
            output(f"sock_rw:初始化{self._name}:{self.address}时出错: {e}",outputMode.saveLog,enum_Error.localErr)
    
    def disconnect(self):
        with  self.socketLock:
            try:
                if self.socket != None:
                    self.socket.shutdown(socket.SHUT_RDWR)
                    self.socket.close()
                    self.socket = None
            except Exception as e:
                output(f"sock_rw:{self.address}断开时出错: {e}",outputMode.saveLog,enum_Error.localErr)
            try:
               if self.disconnect_callback is not None:
                    self.disconnect_callback()
                    self.disconnect_callback = None#防止重复调用
            except Exception as e:
                output(f"sock_rw:{self.address}回调时出错: {e}",outputMode.saveLog,enum_Error.localErr)
    def send_count(self):
        nRes = 0
        with self.write_data_lock:
             nRes = len(self.write_data)
        return nRes
    def socket_send_data(self,data):
        """发送数据"""
        try:
            if data is None:
                return
            #增加到发送列表
            with self.write_data_lock:
                self.write_data.append(data)
            self.write_Event.set()#设置写信号，立即执行
        except Exception as e:
            output(f"sock_rw:{self.address}增加到发送列表时出错: {e}",outputMode.saveLog,enum_Error.localErr)
    def read_thread(self,client_socket,address):
        """处理接收"""
        output(f"sock_rw:{address}开启数据接收",outputMode.saveLog,enum_Error.localInfo)
        try:
            while True:
                # 接收客户端消息（缓冲区大小1024字节）
                try:
                    data = client_socket.recv(1024)
                    if not data:  # 客户端关闭连接
                        break
                    if self.read_callback != None:
                        self.read_callback(data)
                except socket.timeout:
                    if self.isValidSocket() == False:
                        break
                    continue
                except Exception as e:
                    output(f"sock_rw:{self.address}读取socket时出错，错误: {e}",outputMode.saveLog,enum_Error.localErr)
                    break
        except Exception as e:
            output(f"sock_rw:{address}处理客户端读取线程时出错: {e}",outputMode.saveLog,enum_Error.localInfo)
        #finally:
        # 关闭客户端连接
        self.disconnect()
        output(f"sock_rw:与{address}的连接已关闭",outputMode.saveLog,enum_Error.localInfo)

    def write_thread(self,socket,address):
        """处理接收"""
        output(f"socket_rw:{address}发送线程启动",outputMode.saveLog,enum_Error.localInfo)
        try:
            while True:
                res = self.write_Event.wait(self.write_Event_timeout_ms / 1000)
                self.write_Event.clear()
                with self.write_data_lock:
                    if self.socket == None:
                        break
                #output(f"socket_rw:发送数据数量 {len(self.write_data)}",outputMode.saveLog,enum_Error.localInfo)
                #curT = tools.timeManager.get_timestamp_ms() 
                while True:
                    data=None
                    with self.write_data_lock:
                        if len(self.write_data) == 0:
                            break
                        data = self.write_data.pop(0)
                
                    # 接收客户端消息（缓冲区大小1024字节）
                    if data is not None:
                        socket.sendall(data)
                #curEndT = tools.timeManager.get_timestamp_ms() 
                #consum = (curEndT - curT)
                #output(f'[write_thread]:[总耗时]:{consum}',outputMode.saveLog,enum_Error.localInfo)
        except Exception as e:
            output(f"socket_rw:处理客户端 {self.address} 时出错: {e}",outputMode.saveLog,enum_Error.localInfo)
        # 关闭客户端连接
        self.disconnect()
        output(f"socket_rw:{address}发送线程退出",outputMode.saveLog,enum_Error.localInfo)
    #运行读写线程,readflag开启读线程,writeflag开启写线程,如果不开启需要对socket单独操作
    def run(self,readflag=True,writeflag=True):
        #开启异步读取
        try:
            if readflag:
                self.h_read_thread = threading.Thread(
                    target=self.read_thread,
                    args=(self.socket, self.address),
                    daemon=True  # 设置为守护线程，主程序退出时自动终止
                )
                self.h_read_thread.start()
            if writeflag:
                self.h_write_thread = threading.Thread(
                    target=self.write_thread,
                    args=(self.socket, self.address),
                    daemon=True  # 设置为守护线程，主程序退出时自动终止
                )
                self.h_write_thread.start()
        except Exception as e:
            output(f"sock_rw:{self.address}开启异步线程时出错: {e}",outputMode.saveLog,enum_Error.localErr)
    #停止socket,一般是在socket无效的时候调用，处理无用资源
    def stop(self):
        try:
            self.disconnect()

            if self.h_read_thread != None:  
                self.h_read_thread.join()
            if self.h_write_thread != None: 
                self.h_write_thread.join()
            self.read_callback = None
            output(f"停止客户端{self.address}成功",outputMode.saveLog,enum_Error.localInfo)
        except Exception as e:
            output(f"停止 {self.address} 时出错: {e}",outputMode.saveLog,enum_Error.localInfo)

    #判断socket是否有效，由外部线程调用，如果是无效，外部调用stop
    def isValidSocket(self)->bool:
        with self.socketLock:
            if self.socket != None and self.socket.fileno() != -1:
                return True
        return False
    #得到socket，如果没有开启接收和发送线程时，可能在外部调用
    def socket(self)->socket:
        socketRes = None
        with self.socketLock:
            socketRes = self.socket
        return socketRes
