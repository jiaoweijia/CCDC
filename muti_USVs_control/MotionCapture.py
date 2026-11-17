# This Python file uses the following encoding: utf-8
# 动捕数据管理模块类，负责与动捕服务器交互，获取最新位置姿态数据和速度解算
import os
import sys
import time
import threading
import math
from ctypes import *
from PySide6.QtCore import QObject, Signal
from pathlib import Path


class MotionCapture(QObject):
    """动捕数据管理类"""
    
    # 定义信号
    connection_status_changed = Signal(str, bool)  # body_id, connected
    
    def __init__(self):
        super().__init__()
        self.cmVrpn = None
        self.host = None
        self.frame_count = 0
        self.is_running = False
        self.capture_thread = None
        
        # 动捕服务器配置
        # self.server_address = "MCServer@192.168.0.100"
        self.server_address = "MCServer@192.168.6.100"
        self.dll_path = Path(__file__).parent / "ChingmuDLL" / "CMVrpn.dll"
        
        # USV与动捕body ID的映射关系
        self.usv_body_mapping = {
            "USV1": 0,
            "USV2": 1, 
            "USV3": 2,
            "USV4": 3,
            "USV5": 4
        }
        
        # 最新位置姿态数据缓存
        self.latest_pose_data = {}
        self.last_update_time = {}  # 记录每个USV的最后更新时间
        
        # 速度解算相关缓存
        self.prev_pose_data = {}  # 存储前一时刻的姿态数据用于速度计算
        self.velocity_data = {}  # 存储计算的速度分量
    
    def load_dll(self):
        """加载动捕DLL"""
        if os.path.exists(self.dll_path):
            return CDLL(str(self.dll_path))
        else:
            print("Chingmu's dynamic Library does not exist")
            return None
    
    def initialize(self):
        """初始化动捕系统"""
        # 加载DLL
        self.cmVrpn = self.load_dll()
        if not self.cmVrpn:
            return False
            
        try:
            # 设置服务器地址 (使用gbk编码)
            self.host = bytes(self.server_address, "gbk")
            
            # 启动vrpn线程
            self.cmVrpn.CMVrpnStartExtern()
            
            # 启用日志记录
            self.cmVrpn.CMVrpnEnableLog(True)
            
            print(f"动捕系统初始化完成，服务器地址: {self.server_address}")
            return True
            
        except Exception as e:
            print(f"动捕系统初始化失败: {e}")
            return False
    
    def get_body_pose(self, body_id, frame_count):
        """获取指定body的位置姿态数据"""
        try:
            # 获取位置数据 (x, y, z)
            bodyPos = (c_double * 3)()
            
            # 设置DLL函数属性
            trackerExtern = self.cmVrpn.CMTrackerExtern
            trackerExtern.restype = c_double
            
            bodyPos[0] = 0.001 * self.cmVrpn.CMTrackerExtern(self.host, body_id, 0, frame_count)  # 转换为米
            bodyPos[1] = 0.001 * self.cmVrpn.CMTrackerExtern(self.host, body_id, 1, frame_count)  # 转换为米
            bodyPos[2] = 0.001 * self.cmVrpn.CMTrackerExtern(self.host, body_id, 2, frame_count)  # 转换为米
            
            # 获取四元数数据 (rx, ry, rz, rw)
            bodyRot = (c_double * 4)()
            bodyRot[0] = self.cmVrpn.CMTrackerExtern(self.host, body_id, 3, frame_count)
            bodyRot[1] = self.cmVrpn.CMTrackerExtern(self.host, body_id, 4, frame_count)
            bodyRot[2] = self.cmVrpn.CMTrackerExtern(self.host, body_id, 5, frame_count)
            bodyRot[3] = self.cmVrpn.CMTrackerExtern(self.host, body_id, 6, frame_count)
            
            # 检查body是否被检测到
            is_detected = self.cmVrpn.CMTrackerExternIsDetected(self.host, body_id, frame_count)
            
            # 计算艏向角（绕z轴）
            heading = math.atan2(2 * (bodyRot[3] * bodyRot[2] + bodyRot[0] * bodyRot[1]), 
                                1 - 2 * (bodyRot[1] * bodyRot[1] + bodyRot[2] * bodyRot[2]))
            
            return is_detected, bodyPos[0], bodyPos[1], bodyPos[2], heading
            
        except Exception as e:
            print(f"获取body {body_id} 姿态数据失败: {e}")
            return False, 0, 0, 0, 0
    
    def capture_loop(self):
        """动捕数据采集循环"""
        connection_states = {}  # 跟踪连接状态变化
        
        # 初始化连接状态
        for usv_id in self.usv_body_mapping.keys():
            connection_states[usv_id] = False
        
        while self.is_running:
            try:
                # 控制采集频率
                time.sleep(0.05)  # 10Hz采样率
                current_time = time.time()
                
                # 遍历所有USV对应的body
                for usv_id, body_id in self.usv_body_mapping.items():
                    try:
                        # 获取姿态数据
                        is_detected, x, y, z, heading = self.get_body_pose(body_id, self.frame_count)
                        
                        if is_detected:
                            # 更新最后更新时间
                            self.last_update_time[usv_id] = current_time
                            
                            # 缓存当前时刻数据
                            current_pose_data = {
                                'x': x, 'y': y, 'z': z, 'heading': heading,
                                'timestamp': current_time
                            }
                            
                            # 计算速度分量
                            u, v, V, yaw_rate = 0.0, 0.0, 0.0, 0.0  # 首帧无速度
                            
                            # 如果有前一时刻数据，计算速度
                            if usv_id in self.prev_pose_data:
                                prev_pose_data = self.prev_pose_data[usv_id]
                                dt = current_time - prev_pose_data['timestamp']
                                
                                # 计算三方向速度分量
                                u, v, V = self.calculate_velocity(usv_id, current_pose_data, prev_pose_data, dt)
                                
                                if 0.001 <= dt <= 0.5:
                                    delta_heading = math.atan2(
                                        math.sin(heading-prev_pose_data['heading']),
                                        math.cos(heading-prev_pose_data['heading']))
                                    yaw_rate = delta_heading / dt

                            self.velocity_data[usv_id] = {'u': u, 'v': v, 'V': V, 'r': yaw_rate}
                            
                            # 缓存数据
                            self.latest_pose_data[usv_id] = current_pose_data
                                                        
                            # 保存当前数据为下次计算使用
                            self.prev_pose_data[usv_id] = current_pose_data
                            
                            # 只有在连接状态变化时才发射状态信号和打印
                            if not connection_states[usv_id]:
                                connection_states[usv_id] = True
                                self.connection_status_changed.emit(usv_id, True)
                                print(f"{usv_id}: pos({x:.3f}, {y:.3f}, {z:.3f}) heading({heading:.3f})")
                    
                    except Exception as e:
                        pass  # 静默处理错误
                
                # 检查数据超时（超过2秒没更新数据）
                for usv_id in self.usv_body_mapping.keys():
                    last_update = self.last_update_time.get(usv_id, 0)
                    if current_time - last_update > 2.0:  # 超过2秒
                        if connection_states[usv_id]:
                            connection_states[usv_id] = False
                            self.connection_status_changed.emit(usv_id, False)
                
                self.frame_count += 1
                
                # 防止frame_count过大
                if self.frame_count > 1000000:
                    self.frame_count = 0
                    
            except Exception as e:
                print(f"动捕采集循环错误: {e}")
                time.sleep(1)  # 出错时等待1秒再继续
    
    def start(self):
        """启动动捕数据采集"""
        if not self.initialize():
            return False
            
        self.is_running = True
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()
        
        print("动捕数据采集已启动")
        return True
    
    def stop(self):
        """停止动捕数据采集"""
        self.is_running = False
        
        if self.capture_thread and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2)
        
        # 退出vrpn线程
        if self.cmVrpn:
            try:
                self.cmVrpn.CMVrpnQuitExtern()
            except:
                pass
        
        print("动捕数据采集已停止")
    
    def get_latest_pose(self, usv_id):
        """获取指定USV的最新姿态数据"""
        return self.latest_pose_data.get(usv_id)
    
    def set_usv_body_mapping(self, usv_id, body_id):
        """设置USV与body ID的映射关系"""
        if usv_id in self.usv_body_mapping:
            self.usv_body_mapping[usv_id] = body_id
            print(f"设置映射: {usv_id} -> body {body_id}")
    
    def calculate_velocity(self, usv_id, current_pose, prev_pose, dt):
        """计算USV的三方向速度分量
        
        Args:
            usv_id: USV标识符
            current_pose: 当前时刻的姿态数据 {'x': x, 'y': y, 'z': z, 'heading': heading, 'timestamp': t}
            prev_pose: 前一时刻的姿态数据
            dt: 时间间隔（秒）
        
        Returns:
            tuple: (u, v, V) - 机体坐标系下的三方向速度
        """
        if dt <= 0:
            return 0.0, 0.0, 0.0
        
        try:
            # 计算世界坐标系下的位置变化率
            dx = current_pose['x'] - prev_pose['x']
            dy = current_pose['y'] - prev_pose['y']
            dz = current_pose['z'] - prev_pose['z']
            
            # 世界坐标系下的速度分量
            vx = dx / dt  # 世界x方向速度
            vy = dy / dt  # 世界y方向速度
            vz = dz / dt  # 世界z方向速度
            
            # 使用当前时刻的艏向角进行坐标转换
            heading = current_pose['heading']
            
            # 转换到机体坐标系
            # u: 前进方向速度（机体x轴）
            # v: 横向速度（机体y轴）
            u = vx * math.cos(heading) + vy * math.sin(heading)
            v = -vx * math.sin(heading) + vy * math.cos(heading)
            
            # 计算总速度V
            V = math.sqrt(u * u + v * v)
            
            return u, v, V
            
        except Exception as e:
            print(f"计算USV {usv_id} 速度失败: {e}")
            return 0.0, 0.0, 0.0

    def get_usv_state(self, usv_id):
        """获取指定USV的最新状态向量"""
        return [self.velocity_data[usv_id]['u'],
                self.velocity_data[usv_id]['v'],
                self.velocity_data[usv_id].get('r', 0.0),  # wrapped heading difference / measured dt
                self.latest_pose_data[usv_id]['x'],
                self.latest_pose_data[usv_id]['y'],
                self.latest_pose_data[usv_id]['heading'],
        ]
        
    
    def get_usv_body_mapping(self):
        """获取当前映射关系"""
        return self.usv_body_mapping.copy()