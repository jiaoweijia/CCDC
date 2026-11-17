# UDP通讯模块类，封装UDP套接字操作
# 用于接收和发送UDP消息 7020收船的状态向船的7021发送指令向船
import socket #udp通信的根基
import json   #解析船发来的json报文
import threading #线程
from PySide6.QtCore import QObject, Signal #导入俩个类，QObject是所有能发信号的基类，Signal定义信号用的
import time  #时间库
class UDPComm(QObject):
    """UDP通讯模块类，封装UDP套接字操作"""
    
    
    # 定义信号，用于在接收到消息时通知其他模块
    message_received = Signal(dict, str, int)  # 参数：消息内容, 发送方IP, 发送方端口signal是Qt信号机制发布订阅式通知
    connection_status_changed = Signal(str, bool)  # body_id, connected  GUI更新船的在线离线
    #收包在后台线程，GUI更新在主线程
    def __init__(self, local_port=7020): #UDPComm(7020)被创建时自动执行这个构造函数。不传默认就是7020是本地监听端口，7021是船的端口
        """初始化UDP通讯模块
        Args:
            local_port: 本地UDP监听端口
        """
        super().__init__() #调用QObject的构造函数，初始化父类的属性
        self.local_port = local_port
        self.udp_socket = None #对象创建时，UDP套接字为空 没启动服务器
        self.listener_thread = None #后台收包线程对象
        self.running = False #收包循环的开端
        
        # 心跳检测相关变量（参考动捕系统的超时检测机制）
        self.last_heartbeat_time = {}  # 记录每个USV的最后心跳时间
        self.connection_states = {}  # 跟踪连接状态变化 记录在线状态
        
    def start_server(self):
        """启动UDP服务器，开始监听消息"""
        if self.running:  #防止重复启动
            return False
        
        try:
            # 创建UDP套接字
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #第一个是IPV4，第二个是UDP（数据报）
            self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)#设置socket选项=1，允许重复使用地址程序关了立刻重开时，允许马上重绑同一端口（否则系统要等分钟释放，重开程序会报“端口被占用”）
            # 绑定到本地端口
            # self.udp_socket.bind(('192.168.0.102', self.local_port))
            self.udp_socket.bind(('192.168.6.10', self.local_port))#告诉系统“发到 192.168.6.10 这块网卡的 7020 端口的包，都交给我这个 socket 收件箱地址，船的状态注册包都发到这里
            # self.udp_socket.bind(('192.168.170.144', self.local_port))
            
            self.running = True
            # 启动监听线程
            self.listener_thread = threading.Thread(target=self._listener, daemon=True)#新线程要跑的函数 是`_listener`上位机收包 daemon=True 表示线程是守护线程，主线程退出时会自动结束
            self.listener_thread.start()
            
            return True
            
        except Exception as e: #出错后处理 错误信息在e里

            print(f"启动: {e}")
            return False
    
    def stop_server(self):
        """停止UDP服务器"""
        self.running = False
        # if self.udp_socket:
        #     self.udp_socket.close()
        return True
    
    def send_message(self, message, ip, port=7020):#7020上位机自己的
        """发送UDP消息"""
        try:
            message_with_newline = message + '\r\n'#船端解析会trim（）所以加换行符
            
            # 对ESP32C3使用ASCII编码确保兼容性 udp只能发字节不能发字符串`'ascii'` 指定编码；`'ignore'` 表示遇到非 ASCII 字符（比如中文）直接丢弃而不是报错——所以指令只能用数字和英文。
            message_bytes = message_with_newline.encode('ascii', 'ignore')
            
            if not self.udp_socket:
                # 使用临时套接字发送
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as temp_socket:
                    temp_socket.sendto(message_bytes, (ip, port))
            else:  #服务器已经启动，直接发送
                self.udp_socket.sendto(message_bytes, (ip, port))
            return True
        except Exception as e:
            print(f"发送UDP消息失败: {e}")
            return False
    
    def _listener(self):     #收包线程逻辑                            
        """UDP监听线程"""        
        while self.running:
            try:
                # 设置socket超时，以便能够定期检查心跳超时
                self.udp_socket.settimeout(1.0)# recvfrom 最多等1秒`recvfrom` 默认会 永远阻塞 等包，那样下面的“超时检查”（L116）就永远执行不到。设了 1 秒超时后：1 秒内有包→正常收；1 秒没包→抛 timeout 异常跳过收包， 继续往下做心跳检查 。这就是“能定期检查心跳”的注释含义。
                data, addr = self.udp_socket.recvfrom(1024)#从7020捞包，最多收1024字节，返回一个元组
                #data包的内容 addr发送方IP端口
                message = data.decode('utf-8').strip() #按UTF8翻译字节成字符串

                ip, port = addr
                current_time = time.time() #记下此刻时间
                message = json.loads(message)            #转换为python对象
                if message != '':
                    # 普通消息 发Qt信号给GUI 把信号广播出去
                    self.message_received.emit(message, ip, port)            
                    # 心跳消息，使用IP地址作为USV标识
                    usv_id = ip  # 使用IP地址标识USV                 
                    # 更新最后心跳时间
                    self.last_heartbeat_time[usv_id] = current_time  #新增键值对 船来的时刻        
                    # 检查是否是新连接   
                    if not self.connection_states.get(usv_id, False):  #如果之前没有连接记录返回false
                        self.connection_states[usv_id] = True
                        self.connection_status_changed.emit(usv_id, True)  #连接建立 发Qt信号给GUI 把信号广播出去
                        print(f"USV {usv_id} 连接建立")
                
            except socket.timeout:
                # 超时异常正常，继续检查心跳超时
                pass
            except Exception as e:
                if self.running:
                    print(f"UDP接收错误: {e}")
                
            # 检查心跳超时（6秒）
            current_time = time.time() #记下此刻时间
            for usv_id in list(self.last_heartbeat_time.keys()):
                last_heartbeat = self.last_heartbeat_time[usv_id]
                if current_time - last_heartbeat > 6.0:  # 超过6秒
                    if self.connection_states.get(usv_id, False):
                        self.connection_states[usv_id] = False
                        self.connection_status_changed.emit(usv_id, False)
                        print(f"USV {usv_id} 连接超时，已断开")
                        # 清理超时USV的记录
                        del self.last_heartbeat_time[usv_id]
            
            # 短暂休眠，减少CPU占用
            time.sleep(0.1)
