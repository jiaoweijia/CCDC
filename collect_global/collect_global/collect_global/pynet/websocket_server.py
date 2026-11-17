import os
import time
import threading
import queue

for _ in range(2):
    try:

        from flask import Flask, render_template
        from flask_socketio import SocketIO,Namespace, join_room, leave_room, send, emit
        from flask_cors import CORS
        break
    except ModuleNotFoundError:
        os.system('pip install flask')
        os.system('pip install flask_socketio')
        os.system('pip install flask_cors')
        

class webSocket_namespace(Namespace):
    def __init__(self, namespace=None,title='server_message'):
        super().__init__(namespace)  # 调用父类构造函数
        self.login_counter = 0  # 登录计数器

    # 当客户端连接时触发
    def on_connect(self):
        self.login_counter = self.login_counter + 1  # 增加登录计数器
        self.parent.startWrite()#第一次客户连接时触发
        info = f"[Connected to Namespace " + self.namespace + "] login " + str(self.login_counter) + " [OK]"
        tools.output(info,outputMode.saveLog,enum_Error.localInfo.value)
    # 当客户端断开连接时触发
    def on_disconnect(self):
        tools.output(f"[Namespace:+{self.namespace}] logout",outputMode.saveLog,enum_Error.localInfo.value)
        self.login_counter = self.login_counter - 1  # 增加登录计数器
    # 当收到消息时触发
    def on_message(self, data):
        
        #info = f"[Namespace " + self.namespace + "] Msg: " + data
        #tools.output(info,outputMode.saveLog,enum_Error.localInfo.value)
        pass
    # 当收到客户端消息时触发
    def on_client_message(self, data):
        #info = f"[Namespace " + self.namespace + "] Msg: " + data
        #tools.output(info,outputMode.saveLog,enum_Error.localInfo.value)
        pass


class websocket_server():
    """WebSocket服务器类"""
    def __init__(self,name, host,port,title='server_message',
                 cors_allowed_origins='*',namespace:str='',secretKey:str="",delay_time=1,delay_time2=1):
        self._name = name
        self.host = host
        self.port = port
        self.title = title
        self.cors_allowed_origins = cors_allowed_origins
        self.namespace = namespace
        self.secretKey = secretKey
        self.delay_time = delay_time

        self.socketioLock = threading.Lock()
        self.writeThread = None
        self.isRun = False
        self.write_data_lock = threading.Lock()
        self.write_data = []
        self.write_Event = threading.Event()#写信号，用于判断实时写入
        self.write_Event_timeout_ms = 1000

        self.serverThread = None
        try:
            self.app = Flask(__name__)
            if not tools.is_empty(self.secretKey):
                self.app.config['SECRET_KRY'] = self.secretKey#'HUI!WEBSOCKET'
            CORS(self.app)#CORS（Cross-Origin Resource Sharing）属于一种机制，它借助 HTTP 头信息，允许浏览器访问跨域资源
            self.socketioLock = threading.Lock()
            self.socketio = None
            self.space = None
            self.socketio = SocketIO(self.app)
            self.socketio.init_app(self.app, cors_allowed_origins="*")
            self.space = webSocket_namespace(self.namespace,self.title)
            self.space.parent = self
            self.socketio.on_namespace(self.space)
             #注册事件和命名空间
            self._register_events()
        except Exception as err:
            tools.output(f'[{self._name}] Init Exception: ' + str(err.args),outputMode.saveLog,enum_Error.localInfo.value)  # 写日志
       
    def _register_events(self):
        """注册SocketIO事件"""
        @self.socketio.on('connect')
        def handle_connect():
            print(f'客户端连接到 {self.port}')
            
        @self.socketio.on('message')
        def handle_message(data):
            print(f'收到消息: {data}')
            self.socketio.emit('response', {'data': f'服务器 {self.port} 收到: {data}'})
            self.socketio.sleep(self.delay_time) # 休眠
    def send_message(self, msg):
        try:
            self.socketio.emit(self.title, msg, namespace=self.namespace)
        except Exception as e:
            tools.output('send_message 异常:' + str(e.args),outputMode.saveLog,enum_Error.localInfo.value)  # 写日志

    def run(self):
        try:
            if self.serverThread is None or not self.serverThread.is_alive():
                self.serverThread = threading.Thread(
                    target=self.socketio.run,
                    args=(self.app,),
                    kwargs={'host': self.host, 'port': self.port, 'debug': False},
                    daemon=True  # 设置为守护线程，主线程退出时自动终止
                )
                self.serverThread.start()
            tools.output(f'[{self._name}] Start [OK] ',outputMode.saveLog,enum_Error.localInfo.value)  # 写日志
            return True
        except Exception as err:
            tools.output(f'[{self._name}] Start Exception: ' + str(err.args),outputMode.saveLog,enum_Error.localInfo.value)  # 写日志
        return False
    def stop(self):
        try:
            if self.serverThread is not None:
                with self.socketioLock:
                    try:
                        self.socketio.stop()  # 停止服务器
                    except Exception as e:
                        tools.output(f'[{self._name}] Stop Exception:{e} ' ,outputMode.saveLog,enum_Error.localInfo.value)  # 写日志
                        pass
                self.serverThread.join()  # 等待线程结束
                tools.output(f'[{self._name}] 停止socketIo ' ,outputMode.saveLog,enum_Error.localInfo.value)  # 写日志
                with self.socketioLock:
                    self.socketio = None
                    self.space = None
            if self.writeThread is not None:
                self.isRun = True
                self.write_Event.set() #设置写信号，立即执行
                self.writeThread.join()
        except Exception as err:
            tools.output(f'[{self._name}] Stop Exception: ' + str(err.args),outputMode.saveLog,enum_Error.localInfo.value)  # 写日志
    def startWrite(self):
        #一定要用后台任务创建线程，否则出现跨线程错误
        if self.writeThread is None:
            self.isRun = True
            self.writeThread = self.socketio.start_background_task(target=self.sending_thread)
    # 发送数据的线程
    def send_data(self,bEncode,needCall,data, cmdCode,serialNum,proMgr,uuid,bBlxx=False):
        """发送数据"""
        try:
            if data is None:
                return
            #增加到发送列表
            with self.write_data_lock:
                self.write_data.append(data)
            self.write_Event.set()#设置写信号，立即执行
        except Exception as e:
            tools.output(f"socketIo:{self._name}增加发送列表错误: {e}",outputMode.saveLog,enum_Error.localInfo.value)
    def sending_thread(self):
        """处理接收"""
        tools.output(f"socketIo:{self._name}发送线程启动",outputMode.saveLog,enum_Error.localInfo.value)
        while self.isRun:
            try:
                res = self.write_Event.wait(self.write_Event_timeout_ms / 1000)
                self.write_Event.clear()
                if not res:
                    self.send_message(f'{{"fzxx":[],"status": [], "t": {tools.timeManager.get_timestamp_s()}, "type": 202}}')
                    self.socketio.sleep(self.delay_time) # 休眠
                    continue
                self.write_Event.clear()
                while True:
                    data = None
                    with self.write_data_lock:
                        if len(self.write_data) == 0:
                            break
                        data = self.write_data.pop(0)
                
                    # 接收客户端消息（缓冲区大小1024字节）
                    if data is not None:
                        self.send_message(data)
                        if self.delay_time and self.delay_time > 0:
                            self.socketio.sleep(self.delay_time) # 休眠
            except Exception as e:
                tools.output(f"处理客户端 {self._name} 时出错: {e}",outputMode.saveLog,enum_Error.formatErr.value)
            
        tools.output(f"与 {self._name} 的连接已关闭",outputMode.saveLog,enum_Error.localInfo.value)
