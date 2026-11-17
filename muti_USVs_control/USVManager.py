# 无人船管理模块类，负责管理所有无人船的连接和状态
# 用于注册、注销、查询无人船状态每船一个独立控制系统
from PySide6.QtCore import QObject, Signal
from USVController import USVControlSystem
from typing import Dict, Optional, Tuple, Any #`typing` 模块提供 类型注解工具 ，只用于标注参数/返回值类型

class USVManager(QObject):
    """无人船管理模块类，负责管理无人船的连接和状态"""
    
    # 定义信号，用于通知其他模块无人船状态变化
    usv_connected = Signal(str, str, int)  # 参数：无人船ID, IP, 端口
    usv_disconnected = Signal(str)  # 参数：无人船ID
    #初始化 QObject，建两个空字典（连接台账 + 控制系统台账
    def __init__(self):
        """初始化无人船管理模块"""
        super().__init__()  #QObject 父类初始化（用信号的前提）
        self.usvs: Dict[str, Dict[str, Any]] = {}  # 存储无人船信息，格式：{id: {'ip': ip, 'port': port}}
        self.control_systems: Dict[str, USVControlSystem] = {}  # 存储无人船控制系统实例，格式：{id: USVControlSystem实例}
   #设置 新注册船 的默认制导律
    def set_law(self, guidance_law='PP', control_law='NMPC'):#设置导航律（PP纯跟踪）和控制律（怎么算PWM）
        """设置无人船的导航律和控制律"""
        self.guidance_law = guidance_law
        self.control_law = control_law
    #登记船的 IP/端口（无效端口回退 7021）；若该船还没有控制系统，就新建一个   
    def register_usv(self, usv_id, ip, port=7021):
        """注册无人船"""
        # 参数验证和端口处理
        if not all([usv_id, ip]):
            return False
        
        # 处理无效端口
        if port is None or port == 0 or port == '':
            port = 7021
        
        try:
            # 更新或添加无人船信息
            self.usvs[usv_id] = {'ip': ip, 'port': int(port)}
            
            # 创建或更新控制系统实例
            if usv_id not in self.control_systems:
                self.control_systems[usv_id] = USVControlSystem(usv_id, self.guidance_law, self.control_law)
            
            # 发射信号
            self.usv_connected.emit(usv_id, ip, port)
            return True
            
        except Exception:
            return False
    #删除船
    def unregister_usv(self, usv_id):
        """注销无人船"""
        if usv_id in self.usvs:
            del self.usvs[usv_id]
            # 移除对应的控制系统实例
            if usv_id in self.control_systems:
                del self.control_systems[usv_id]
            self.usv_disconnected.emit(usv_id)
            return True
        return False
    # 取单船信息
    def get_usv_info(self, usv_id):
        return self.usvs.get(usv_id)
    # 取所有船副本信息
    def get_all_usvs(self):
        return self.usvs.copy() #拿 全部 船的台账副本
    # 判断船是否已连接
    def is_usv_connected(self, usv_id):
        return usv_id in self.usvs
    # 取已连接船数量
    def get_usv_count(self):
        """获取已连接无人船数量
        
        Returns:
            int: 已连接无人船数量
        """
        return len(self.usvs)
    #获取USV实例
    def get_control_system(self, usv_id):
        """
        获取USV的控制系统实例
        
        参数:
            usv_id: USV标识符
            
        返回:
            USVControlSystem实例，如果USV不存在则返回None
        """
        return self.control_systems.get(usv_id)
    # 重置单USV的控制系统
    def reset_control_system(self, usv_id, initial_state=None):
        """
        重置USV的控制系统
        
        参数:
            usv_id: USV标识符
            initial_state: 初始状态向量
            
        返回:
            bool: 重置是否成功
        """
        if usv_id in self.control_systems:
            self.control_systems[usv_id].reset(initial_state)
            return True
        return False
    # 重置所有USV的控制系统
    def reset_all_control_systems(self):
        """
        重置所有USV的控制系统
        """
        for control_system in self.control_systems.values():
            control_system.reset()
            
    #把"当前参考点、下一参考点、实测状态、障碍物列表"原样转发给对应船
    def control_usv(self, usv_id, reference_point=None, next_reference_point=None, usv_state=None, obstacles=None, reference_preview=None, reference_curvature=None):
        """
        执行USV的控制循环

        参数:
            usv_id: USV标识符
            reference_point: 预先计算好的参考轨迹点 (x, y)，避免重复计算
            next_reference_point: 预先计算好的下一个参考轨迹点 (x, y)，避免重复计算
            usv_state: USV当前状态向量
            obstacles: 障碍物列表 [Obstacle, ...]，传给避障模块（无障碍物时为None）

        返回:
            tuple: 更新后的位置坐标 (x, y)，如果控制失败则返回None
        """
        if usv_id in self.control_systems:
            return self.control_systems[usv_id].control(usv_id, reference_point, next_reference_point, usv_state, obstacles, reference_preview, reference_curvature)
        return None
    
    # 更新USV控制系统的参数
    def update_control_params(self, usv_id, **params):
        """
        更新USV控制系统的参数
        
        参数:
            usv_id: USV标识符
            params: 控制系统参数，包括:
                lookahead_distance: 前视距离
                heading_gain: 航向增益
                K_PP: PP制导率增益
                speed_kp: 速度PID比例系数
                speed_ki: 速度PID积分系数
                speed_kd: 速度PID微分系数
                heading_kp: 航向PID比例系数
                heading_ki: 航向PID积分系数
                heading_kd: 航向PID微分系数
        
        返回:
            bool: 参数更新是否成功
        """
        if usv_id in self.control_systems:
            self.control_systems[usv_id].update_params(**params)
            return True
        return False
    
    def set_guidance_law(self, usv_id, guidance_law):
        """
        设置USV的制导率
        
        参数:
            usv_id: USV标识符
            guidance_law: 制导率名称，可选值: 'LOS', 'PP'
        
        返回:
            bool: 设置是否成功
        """
        if usv_id in self.control_systems:
            return self.control_systems[usv_id].set_guidance_law(guidance_law)
        return False
    
    def set_control_law(self, usv_id, control_law):
        """
        设置USV的控制率
        
        参数:
            usv_id: USV标识符
            control_law: 控制率名称，可选值: 'PID', 'NMPC'
        
        返回:
            bool: 设置是否成功
        """
        if usv_id in self.control_systems:
            return self.control_systems[usv_id].set_control_law(control_law)
        return False
    
    def get_registered_usvs(self):
        """
        获取已注册的USV列表
        
        返回:
            list: 已注册的USV ID列表
        """
        return list(self.control_systems.keys())
