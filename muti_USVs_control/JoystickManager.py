# 摇杆管理模块类，负责读取BetaFPV遥控器数据并转换为USV控制命令
import pygame
import time
import math
from PySide6.QtCore import QObject, Signal, QTimer


class JoystickManager(QObject):
    """摇杆管理类，负责读取BetaFPV遥控器数据并转换为USV控制命令"""
    
    # 定义信号
    rc_data_updated = Signal(dict)  # 摇杆数据更新信号
    rc_connected = Signal(bool)     # 连接状态信号
    control_command = Signal(int, int)  # 控制命令信号: pwm1, pwm2
    
    def __init__(self):
        """初始化摇杆管理模块"""
        super().__init__()
        self.joystick = None
        self.initialized = False
        self.running = False
        
        # 摇杆通道映射 (BetaFPV LiteRadio 2 SIM / Mode 2)
        self.axis_mapping = {
            'roll': 0,      # Axis 0: Roll (右手左右)
            'pitch': 1,     # Axis 1: Pitch (右手前后)
            'throttle': 2,  # Axis 2: Throttle (左手油门)
            'yaw': 3,       # Axis 3: Yaw (左手左右)
            'sa': 4,        # Axis 4: SA
            'sb': 5,        # Axis 5: SB
            'sc': 7,        # Axis 7: SC
            'sd': 6,        # Axis 6: SD
        }
        
        # 当前摇杆状态
        self.current_state = {
            'roll': 0.0,
            'pitch': 0.0,
            'throttle': -1.0,  # 油门初始值-1（低位）
            'yaw': 0.0,
            'sa': -1.0,
            'sb': -1.0,
            'sc': -1.0,
            'sd': -1.0,
        }
        
        # 死区设置（消除摇杆漂移）
        self.deadzone = 0.05
        
        # PWM范围
        self.pwm_min = 1000
        self.pwm_max = 2000
        self.pwm_mid = 1500
        
        # 定时器用于定时读取数据
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_loop)
        self.update_timer.setInterval(10)  # 10Hz更新频率
        
    def initialize(self):
        """初始化pygame和摇杆设备"""
        try:
            pygame.init()
            pygame.joystick.init()
            
            # 检查是否有摇杆连接
            joystick_count = pygame.joystick.get_count()
            if joystick_count == 0:
                print("未检测到任何摇杆设备")
                self.rc_connected.emit(False)
                return False
            
            # 使用第一个摇杆
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            
            print(f"已连接摇杆: {self.joystick.get_name()}")
            print(f"轴数量: {self.joystick.get_numaxes()}, 按钮数量: {self.joystick.get_numbuttons()}")
            
            self.initialized = True
            self.rc_connected.emit(True)
            return True
            
        except Exception as e:
            print(f"初始化摇杆失败: {e}")
            self.rc_connected.emit(False)
            return False
    
    def start(self):
        """开始读取摇杆数据"""
        if not self.initialized:
            if not self.initialize():
                return False
        
        if self.running:
            return True
            
        # 检查油门是否在低位（1000位置）
        pygame.event.pump()
        throttle_value = self.joystick.get_axis(self.axis_mapping['throttle'])
        
        # 油门初始值应该接近-1（低位），允许一定误差
        if throttle_value > -0.9:
            print("警告：油门未归位，请将油门杆拉至最低位置后再启动遥控")
            self.rc_connected.emit(False)
            return False
            
        self.running = True
        self.update_timer.start()
        print("摇杆控制已启动")
        return True
    
    def stop(self):                
        # 发送初始状态命令（1000,1500），确保无人船停止
        self.control_command.emit(self.pwm_mid+10, self.pwm_mid+10)
        time.sleep(1)
        self.control_command.emit(self.pwm_mid+10, self.pwm_mid+10)
        print("摇杆控制已停止，已发送停止命令")
        """停止读取摇杆数据"""
        self.running = False
        self.update_timer.stop()
    
    def shutdown(self):
        """关闭摇杆模块"""
        self.stop()
        if self.initialized:
            pygame.quit()
            self.initialized = False
            print("摇杆模块已关闭")
    
    def _apply_deadzone(self, value):
        """应用死区处理"""
        if abs(value) < self.deadzone:
            return 0.0
        return value
    
    def _map_to_pwm(self, value, invert=False):
        """将摇杆值(-1~1)映射到PWM范围(1000~2000)
        
        Args:
            value: 摇杆值 (-1 到 1)
            invert: 是否反向
            
        Returns:
            int: PWM值 (1000 到 2000)
        """
        if invert:
            value = -value
        
        # 映射: -1 -> 1000, 0 -> 1500, 1 -> 2000
        pwm = int(self.pwm_mid + value * (self.pwm_max - self.pwm_min) / 2)
        
        # 限制范围
        pwm = max(self.pwm_min, min(self.pwm_max, pwm))
        
        return pwm
    
    def _update_loop(self):
        """定时更新摇杆数据"""
        if not self.running or not self.joystick:
            return
        
        try:
            pygame.event.pump()
            
            # 读取各通道数据
            self.current_state['roll'] = self._apply_deadzone(self.joystick.get_axis(self.axis_mapping['roll']))
            self.current_state['pitch'] = self._apply_deadzone(self.joystick.get_axis(self.axis_mapping['pitch']))
            self.current_state['throttle'] = self._apply_deadzone(self.joystick.get_axis(self.axis_mapping['throttle']))
            self.current_state['yaw'] = self._apply_deadzone(self.joystick.get_axis(self.axis_mapping['yaw']))
            self.current_state['sa'] = self._apply_deadzone(self.joystick.get_axis(self.axis_mapping['sa']))
            self.current_state['sb'] = self._apply_deadzone(self.joystick.get_axis(self.axis_mapping['sb']))
            self.current_state['sc'] = self._apply_deadzone(self.joystick.get_axis(self.axis_mapping['sc']))
            self.current_state['sd'] = self._apply_deadzone(self.joystick.get_axis(self.axis_mapping['sd']))
            
            # 发送摇杆数据更新信号
            self.rc_data_updated.emit(self.current_state.copy())
            
            # 计算控制命令
            self._calculate_control_command()
            time.sleep(0.02)
            
        except Exception as e:
            print(f"读取摇杆数据失败: {e}")
    
    def _calculate_control_command(self):
        """计算USV控制命令
        
        USV控制逻辑:
        - 油门控制前进后退 (对应PWM1)
        - 方向(roll/yaw)控制左右转向 (对应PWM2)
        """
        # # 油门通道: -1(低位)到1(高位)，映射到PWM1
        # # -1 -> 1000 (后退), 1 -> 2000 (前进)
        # throttle_pwm = self._map_to_pwm(self.current_state['throttle'], invert=False)
        
        # # 方向通道: -1(左)到1(右)，映射到PWM2
        # # -1 -> 1000 (左转), 1 -> 2000 (右转)
        # # 也可以使用yaw通道或roll通道
        # direction_pwm = self._map_to_pwm(self.current_state['roll'], invert=False)
        pwml = self._map_to_pwm(self.current_state['pitch']) + self._map_to_pwm(self.current_state['roll']) - self.pwm_mid
        pwmr = self._map_to_pwm(self.current_state['pitch']) - self._map_to_pwm(self.current_state['roll']) + self.pwm_mid
        
        # 发送控制命令
        self.control_command.emit(pwml, pwmr)
    
    
    def get_rc_values(self):
        """获取当前RC通道值（归一化到0~1，便于UI显示）
        
        Returns:
            dict: 各通道的0~1归一化值
        """
        # 将-1~1映射到0~1
        normalized = {}
        for key, value in self.current_state.items():
            normalized[key] = (value + 1) / 2
            normalized[key] = max(0.0, min(1.0, normalized[key]))
        return normalized
    
    def get_pwm_values(self):
        """获取当前PWM值
        
        Returns:
            dict: 各通道的PWM值
        """
        return {
            'throttle': self._map_to_pwm(self.current_state['throttle']),
            'roll': self._map_to_pwm(self.current_state['roll']),
            'pitch': self._map_to_pwm(self.current_state['pitch']),
            'yaw': self._map_to_pwm(self.current_state['yaw']),
            'sa': self._map_to_pwm(self.current_state['sa']),
            'sb': self._map_to_pwm(self.current_state['sb']),
            'sc': self._map_to_pwm(self.current_state['sc']),
            'sd': self._map_to_pwm(self.current_state['sd']),
        }
