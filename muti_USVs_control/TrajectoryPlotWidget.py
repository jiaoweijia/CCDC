import sys
import time
from PySide6.QtWidgets import QApplication
from PySide6.QtWidgets import QWidget, QVBoxLayout, QMessageBox
from PySide6.QtCore import Signal, QTimer, QEvent
from PySide6.QtCore import Qt
import pyqtgraph as pg
import numpy as np
import math
from USVManager import USVManager
from CommandManager import CommandManager
from MotionCapture import MotionCapture
from CoveragePathPlanner import CoveragePathPlanner
from ObstacleAvoidance import CircleObstacle, RectangleObstacle
from SoftTerminalPanel import SoftTerminalPanelMixin

class TrajectoryPlotWidget(SoftTerminalPanelMixin, QWidget):
    """参数方程轨迹绘制组件"""
    
    terminal_control_failed = Signal(str)
    trajectory_started = Signal(str)
    trajectory_reset = Signal(str)
    params_changed = Signal()
    usv_state_updated = Signal(str, dict)  # 信号：USV状态更新，参数：(usv_id, 状态字典)
    obstacle_position_selected = Signal(float, float)  # 画布放置障碍物：(x, y)
    obstacle_placement_mode_changed = Signal(bool)     # 放置模式开关
    
    MAX_TRAJECTORY_POINTS = 100
    animation_interval = 100  # 动画更新间隔（毫秒）
    
    def __init__(self, usv_manager: USVManager = None, command_manager: CommandManager = None, motion_capture: MotionCapture = None, parent=None):
        super().__init__(parent)
        
        self.is_drawing = False
        self.animation_timer = QTimer()
        self.animation_timer.timeout.connect(self.update_animation)
        
        self.t = 0.0
        self.t_max = 2 * np.pi
        self.t_step = 1.0  # 时间步长（会根据轨迹参数和期望速度自适应计算）
        self.current_t = 0.0
        self.base_positive = 1630
        self.base_nagitive = 1400
        # 期望跟踪速度（m/s）：参考点沿轨迹推进的速度应匹配实物船能达到的速度，
        # 否则参考点跑得太快，船永远在追，航向剧烈调整导致轨迹歪扭
        self.desired_tracking_speed = 0.5
        
        self.usv_colors = {
            'USV1': (255, 0, 0),
            'USV2': (0, 200, 0), 
            'USV3': (0, 0, 255),
            'USV4': (165, 42, 42),
            'USV5': (255, 165, 0)  # 棕色，提高辨识度
        }
        
        # 默认参数值
        self.traj_type = '圆形'
        self.x_coeff = 1.0
        self.y_coeff = 1.0
        self.x_offset = 0.0
        self.y_offset = 0.0
        self.omega_val = 1.0
        
        # 轨迹周期控制：'infinite'（无限圈）或'single'（单圈）
        self.loop_type = 'infinite'
        
        # 多USV轨迹数据和状态
        self.active_usvs = {}
        self.usv_offsets = {}
        self.trajectory_curves = {}
        self.trajectory_data = {}
        self.current_point_markers = {}
        self.crosshair_markers = {}
        
        # 真实轨迹数据
        self.real_trajectory_curves = {}
        self.real_trajectory_data = {}
        self.usv_markers = {}
        
        # 虚拟领航点原点
        self.virtual_leader_marker = None
        
        # 返航点标记（五角星）
        self.return_point_star = None
        self.return_point_x = -1.81  # 默认值
        self.return_point_y = 0.0  # 默认值
        
        # 动力学仿真相关属性
        self.dynamics_enabled = False  # 是否启用动力学仿真
        self.usv_manager = usv_manager  # USV管理器实例
        self.command_manager = command_manager  # 命令管理器实例
        self.motion_capture = motion_capture  # 运动捕捉实例
        self.dynamics_trajectory_curves = {}  # 动力学仿真轨迹曲线
        self.dynamics_trajectory_data = {}  # 动力学仿真轨迹数据
        
        # 前一个控制状态，用于加权平均
        self.previous_control_states = {}  # 存储每个USV的前一个控制状态
        
        # 覆盖路径规划相关属性
        self.coverage_mode = False  # 是否处于覆盖路径规划模式
        self.temp_coverage_points = []  # 临时存储覆盖区域顶点
        self.coverage_area = []  # 覆盖区域顶点
        self.coverage_path = []  # 覆盖路径点
        self.coverage_waypoints = []  # 覆盖路径航点
        self.dense_coverage_path = []  # 稠密覆盖路径点
        
        # 覆盖路径规划器
        self.coverage_planner = CoveragePathPlanner()
        
        # 覆盖路径曲线和标记
        self.coverage_area_curve = None
        self.coverage_path_curve = None
        self.coverage_waypoint_markers = None
        self.coverage_spacing = 0.5
        self.coverage_angle = 0
        self.coverage_speed = 0.2

        # 打点航点（WTracking）模式相关属性
        self.waypoint_edit_mode = False  # 是否处于鼠标打点采集模式
        self.temp_waypoints = []  # 本次打点过程中临时采集的航点
        self.manual_waypoint_path = []  # 右键确认后的手动航点航线
        self.manual_waypoint_curve = None  # 手动航线曲线
        self.manual_waypoint_markers = None  # 手动航点标记

        # 障碍物（避障）：id -> Obstacle 对象；对应的绘图项字典
        self.obstacle_placement_mode = False
        self.obstacles = {}
        self.obstacle_plot_items = {}
        self._obstacle_seq = 0
        # 路径跟踪相关属性
        self.tracking_mode = 'TTracking'  # PTracking路径跟踪模式,TTracking轨迹跟踪类型
        self.current_paths = {}  # 存储每个USV的路径点
        self.current_waypoint_indices = {}  # 存储每个USV当前目标航迹点索引
        self.waypoint_reached_threshold = 0.2  # 航迹点到达阈值（米）

        # 切换控制相关属性
        self.exchange_control_enabled = False  # 是否启用切换控制
        self.exchange_time = 0  # 切换时间（实时秒）
        self.exchange_time_threshold = 0  # 切换时间对应的current_t阈值
        self.exchange_biases = {}  # 存储需要切换的USV的第二组偏置 {usv_id: (x_bias, y_bias)}
        self.exchange_biases_from = {}  # 存储USV的原始偏置（过渡起始值）{usv_id: (x_bias, y_bias)}
        self.exchange_duration = 0  # 过渡持续时间（实时秒）
        self.exchange_duration_threshold = 0  # 过渡持续时间对应的current_t单位

        # 时间显示相关属性
        self.start_time = time.time()  # 软件启动时间
        self.experiment_start_time = None  # 实验开始时间（动画启动时记录）
        self.time_display = None  # 时间显示文本
        
        # 创建布局
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        
        # 创建绘图组件
        self.plot_widget = pg.PlotWidget()
        self.layout.addWidget(self.plot_widget)
        
        # 初始化绘图相关组件
        self.setup_plot()
        self._setup_soft_terminal_controls()
        
        # 安装事件过滤器以捕获鼠标事件
        self.plot_widget.viewport().installEventFilter(self)
    #绘图界面的初始化函数：pyqtgraph 画布样式，并预先创建画面上所有"图元"——参考轨迹线、虚拟领航点、返航五角星、水池边界框、时间文本
    def setup_plot(self):
        """初始化绘图组件"""
        # 设置绘图区域属性
        self.plot_widget.setBackground('w')
        self.plot_widget.setAspectLocked(True)
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel('left', 'Y (m)')
        self.plot_widget.setLabel('bottom', 'X (m)')
        self.plot_widget.setXRange(-2, 2)
        self.plot_widget.setYRange(-2, 2)
        
        # 初始化静态轨迹（所有USV共享的参考轨迹）
        self.static_curve = self.plot_widget.plot(pen=pg.mkPen('gray', style=Qt.DashLine, width=2))
        
        # 初始化虚拟领航点原点标记
        self.virtual_leader_marker = self.plot_widget.scatterPlot(
            [0], [0], size=10, brush=pg.mkBrush('red'), symbol='o'
        )
        
        # 初始化返航点五角星标记（默认隐藏）
        self.return_point_star = self.plot_widget.scatterPlot(
            [self.return_point_x], [self.return_point_y],
            size=20, brush=pg.mkBrush(255, 215, 0), pen=pg.mkPen('black', width=2),
            symbol='star'
        )
        self.return_point_star.hide()
        
        # 绘制边界矩形
        # 给定的两个点：(2.342, -0.72) 和 (-1.943, 1.334)
        min_x = min(2.014, -2.186)
        max_x = max(2.014, -2.186)
        min_y = min(-0.856, 0.908)
        max_y = max(-0.856, 0.908)
        
        # 矩形的四个顶点
        rect_points = [
            (min_x, min_y),
            (max_x, min_y),
            (max_x, max_y),
            (min_x, max_y),
            (min_x, min_y)  # 回到起点闭合矩形
        ]
        
        # 提取x和y坐标
        rect_x, rect_y = zip(*rect_points)
        
        # 绘制矩形
        self.plot_widget.plot(
            rect_x, rect_y,
            pen=pg.mkPen('blue', width=2, style=Qt.SolidLine)
        )
        
        # 初始化静态轨迹
        self.draw_static_trajectory()

        # 在画布左上角添加时间显示
        self.time_display = pg.TextItem(
            text="",
            anchor=(0, 0),
            color='k',
            fill=pg.mkBrush(255, 255, 255, 180)
        )
        self.time_display.setPos(self.plot_widget.viewRange()[0][0], self.plot_widget.viewRange()[1][1])
        self.plot_widget.addItem(self.time_display)

    def set_dead_zone(self, base_positive, base_nagitive):
        """设置死区"""
        self.base_positive = base_positive
        self.base_nagitive = base_nagitive
    #用户在界面上改了轨迹类型/系数/频率等参数后，调用它一次性完成"存参数 → 重算步长 → 重绘轨迹 → 重置领航点 → 发信号通知"全套刷新动作
    def update_params(self, traj_type, x_coeff, y_coeff, x_offset, y_offset, omega, loop_type=None, experiment_type=None, desired_speed=None):
        """更新轨迹参数"""
        self.traj_type = traj_type
        self.x_coeff = x_coeff
        self.y_coeff = y_coeff
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.omega_val = omega

        # 如果提供了期望跟踪速度，则更新（参考点沿轨迹推进的速度）
        if desired_speed is not None:
            self.desired_tracking_speed = float(desired_speed)

        # 如果提供了loop_type，则更新
        if loop_type is not None:
            self.loop_type = loop_type

        # 根据轨迹参数和期望速度自适应计算 t_step，保证参考点推进速度匹配船速
        self._compute_t_step()

        # 更新静态轨迹
        self.draw_static_trajectory()
        
        # 更新虚拟领航点位置
        reference_x, reference_y = self.calculate_point(0)
        self.virtual_leader_marker.setData([reference_x], [reference_y])
        
        # 如果正在绘制，重置轨迹
        if not self.is_drawing:
            self.reset_trajectory()
        
        # 发出参数变更信号
        self.params_changed.emit()
    #整个轨迹系统的"位置计算器"——输入轨迹参数 t，输出虚拟领航点（参考点）在水池世界坐标系中的位置 (x, y)，单位米。
    def calculate_point(self, t, x_coeff=None, y_coeff=None, x_offset=None, y_offset=None, omega=None, interval=0.0):
        # 如果没有提供参数，使用实例变量
        if x_coeff is None:
            x_coeff = self.x_coeff
        if y_coeff is None:
            y_coeff = self.y_coeff
        if x_offset is None:
            x_offset = self.x_offset
        if y_offset is None:
            y_offset = self.y_offset
        if omega is None:
            omega = self.omega_val
            
        traj_type = self.traj_type
        
        if traj_type == '圆形':
            x = y_coeff * 0.6 * np.sin(omega * t + interval) + x_offset
            y = y_coeff * 0.6 * np.cos(omega * t + interval) + y_offset
        elif traj_type == '椭圆':
            x = x_coeff * np.sin(omega * t + interval) + x_offset
            y = y_coeff * 0.5 * np.cos(omega * t + interval) + y_offset
        elif traj_type == '8字形':
            x = x_coeff * np.sin(omega * t + interval) + x_offset
            y = y_coeff * np.sin(omega * t + interval) * np.cos(omega * t + interval) + y_offset
        elif traj_type == 'SIN':
            y = x_coeff/10.0 * np.sin(4*omega * t + interval) + x_offset
            x = y_coeff * 0.015 * t + y_offset-1.5
        elif traj_type == 'COS':
            y = x_coeff/10.0 * np.cos(4*omega * t + interval) + x_offset
            x = y_coeff * 0.015 * t + y_offset-1.5
        elif traj_type == '螺旋线':
            x = x_coeff * 0.1 * t * np.cos(omega * t + interval) + x_offset
            y = y_coeff * 0.1 * t * np.sin(omega * t + interval) + y_offset
        elif traj_type == '自定义':
            x = x_coeff * np.sin(omega * t + interval) + x_offset
            y = y_coeff * np.cos(omega * t + interval) + y_offset
        else:
            x = np.sin(omega * t + interval) + x_offset
            y = np.cos(omega * t + interval) + y_offset
        
        return x, y
    #自适应计算每帧推进轨迹参数 t 的步长`t_step` ，让虚拟领航点沿轨迹移动的世界速度恰好等于船的期望跟踪速度。
    def _compute_t_step(self):
        """根据轨迹参数和期望跟踪速度自适应计算时间步长 t_step。

        原理：参考点沿轨迹推进的世界速度应匹配实物船能达到的速度。
        参考点世界速度 = (弧长速率 m/单位t) * (t_step / 控制周期s)
        令其等于 desired_tracking_speed，反解 t_step。
        """
        # dt = 0.01
        dt = 1e-4  # 更小的数值微步长，降低微分误差
        # x1, y1 = self.calculate_point(0)
        t_now = self.t
        x1, y1 = self.calculate_point(t_now)
        x2, y2 = self.calculate_point(t_now + dt)

        # x2, y2 = self.calculate_point(dt)
        # 参数 t 的弧长速率（m / 单位t），取 t=0 附近的局部值
        arc_rate = math.hypot(x2 - x1, y2 - y1) / dt #t每增1，参考点沿轨迹移动的距离为arc_rate * dt
        if arc_rate < 1e-6:  #轨迹几乎不动
            self.t_step = 1.0 
            return
        control_period = self.animation_interval / 1000.0  # 100/1000=0.1秒
        # V_ref = arc_rate * t_step / control_period = desired_tracking_speed
        self.t_step = self.desired_tracking_speed * control_period / arc_rate
        # 限制范围，防止参数异常导致 t_step 过大或过小
        self.t_step = float(np.clip(self.t_step, 0.001, 2.0))
    #把"一个完整周期的参考轨迹"采样成 200 个点，画成画面上那条灰色虚线。 它只负责静态显示，不参与控制。
    def draw_static_trajectory(self):
        # 静态轨迹始终显示一个完整的2π周期，不受loop_type影响
        if self.omega_val != 0:
            # 计算显示一个完整周期所需的时间
            t_max = 2 * np.pi / self.omega_val
        else:
            t_max = self.t_max
        
        t_values = np.linspace(0, t_max, 200)
        x_values = []
        y_values = []
        
        for t in t_values:
            x, y = self.calculate_point(t)
            x_values.append(x)
            y_values.append(y)
        
        self.static_curve.setData(x_values, y_values)
        # self.update_axis_range(x_values, y_values)#更新坐标轴范围
    #"开始实验"按钮的入口函数：做防重入检查 → 复位进度 → 按模式初始化画面/编队 → 启动 100ms 控制定时器、记录实验时间、发出启动信号。 启动后每帧的工作交给定时器回调     
    def start_drawing(self):
        if self.is_drawing:
            return
        
        self.is_drawing = True
        self.current_t = 0.0
        
        # RTracking模式：返航模式，不需要轨迹曲线，只启动计时器
        if self.tracking_mode == 'RTracking':
            self.animation_timer.start(self.animation_interval)
            self.experiment_start_time = time.time()
            self.trajectory_started.emit("started")
            return
        
        # 初始化所有激活USV的轨迹数据
        self.setup_usv_trajectory_curves()
        
        # 计算参考轨迹的当前方向角（所有USV共享相同的参考轨迹方向）
        reference_angle = self.calculate_leader_direction(self.current_t)
        
        # 计算参考轨迹的初始位置并更新虚拟领航点原点
        reference_x, reference_y = self.calculate_point(0)
        self.virtual_leader_marker.setData([reference_x], [reference_y])
        
        # 计算初始位置并更新标记
        for usv_id in self.active_usvs:
            x_bias, y_bias, interval = self.usv_offsets[usv_id]
            
            # 将相对偏移量转换为全局坐标系下的偏移量
            # 基于参考轨迹的初始方向角进行旋转变换
            global_x_bias = x_bias * np.cos(reference_angle) - y_bias * np.sin(reference_angle)
            global_y_bias = x_bias * np.sin(reference_angle) + y_bias * np.cos(reference_angle)
            
            # 计算当前时间点的轨迹坐标（应用轨迹整体偏移量和转换后的全局偏移量）
            x, y = self.calculate_point(0, x_offset=self.x_offset + global_x_bias, y_offset=self.y_offset + global_y_bias, interval=interval * np.pi/100)
            
            self.current_point_markers[usv_id].setData([x], [y])
            self.crosshair_markers[usv_id].setData([x], [y])
        
        self.animation_timer.start(self.animation_interval)  # 定时器间隔改为animation_interval毫秒

        self.experiment_start_time = time.time()

        self.trajectory_started.emit("started")
    #计算虚拟领航点在轨迹参数 t 处的"前进方向角"——即轨迹切线方向，单位弧度，范围 (-π, π]。
    def calculate_leader_direction(self, t):
        """计算领航者在指定时间点的方向（轨迹的切线方向）"""
        # 使用中心差分法计算方向
        delta_t = 0.01
        x1, y1 = self.calculate_point(t - delta_t)
        x2, y2 = self.calculate_point(t + delta_t)
        
        # 计算方向向量
        dx = x2 - x1
        dy = y2 - y1
        
        # 计算方向角（弧度）
        angle = np.arctan2(dy, dx)
        
        return angle
    #定时心跳 + 模式分发 刷新左上角时钟
    def update_animation(self):
        if not self._terminal_is_active():
            return self._update_animation_frame()
        try:
            self._update_animation_frame()
        except (RuntimeError, ValueError) as exc:
            self.stop_drawing()
            self.terminal_status_label.setText('NMPC运行停止：'+str(exc))
            self.terminal_control_failed.emit(str(exc))

    def _update_animation_frame(self):
        # 更新时间显示
        if self.time_display is not None:
            view_range = self.plot_widget.viewRange()
            self.time_display.setPos(view_range[0][0], view_range[1][1])
            current_time = time.time()
            elapsed_software = current_time - self.start_time
            software_time_str = f"{int(elapsed_software // 3600):02d}:{int((elapsed_software % 3600) // 60):02d}:{int(elapsed_software % 60):02d}"
            if self.experiment_start_time is not None:
                elapsed_experiment = current_time - self.experiment_start_time
                experiment_time_str = f"{elapsed_experiment:.1f}s"
                self.time_display.setText(f"系统运行: {software_time_str}\n实验仿真: {experiment_time_str}")
            else:
                self.time_display.setText(f"系统运行: {software_time_str}")

        # 检查是否处于路径跟踪模式
        if self.tracking_mode == 'PTracking':  
            if self.current_paths:
                self.update_path_tracking()
            else:
                # 弹窗提示，没有路径，
                 QMessageBox.warning(
                    self,
                    "路径跟踪启动失败",
                    "请先加载路径。",
                    QMessageBox.Ok
                ) 
                 self.animation_timer.stop()
        elif self.tracking_mode == 'WTracking':
            if self.current_paths:
                self.update_waypoint_tracking()
            else:
                # 弹窗提示，还没有打点航线
                QMessageBox.warning(
                    self,
                    "打点航点启动失败",
                    "请先在“路径规划-打点航点”中打点并右键确认生成航线。",
                    QMessageBox.Ok
                )
                self.animation_timer.stop()
        elif self.tracking_mode == 'TTracking':
            self.update_trajevtory_tracking()
        elif self.tracking_mode == 'RTracking':
            self.update_return_tracking()
    #返航模式
    def update_return_tracking(self):
        """
        更新返航跟踪状态：引导选中的USV回到预设的返航点
        """
        return_reached_threshold = 0.2  # 到达返航点阈值（米）
        all_reached_return = False
        
        for usv_id in self.active_usvs:                  
            # 使用USV控制器导航到返航点
            target_point = (self.return_point_x, self.return_point_y)
            next_target = (self.return_point_x - 0.14, self.return_point_y - 0.14)
            if self.dynamics_enabled is False: #实船走下面
                # 获取USV当前状态
                current_state = self.motion_capture.get_usv_state(usv_id)   
                result = self._control_with_terminal(usv_id, target_point, next_target, current_state, obstacles=self.get_obstacles())
                if result is not None:
                    pwml = result['pwml']
                    pwmr = result['pwmr']
                    
                    # 应用死区
                    if pwml > 0:
                        pwml = self.base_positive + pwml  #正转中位加增量
                    elif pwml < 0:
                        pwml = self.base_nagitive + pwml
                    if pwmr > 0:
                        pwmr = self.base_positive + pwmr
                    elif pwmr < 0:
                        pwmr = self.base_nagitive + pwmr
                    
                    # 发送控制指令
                    if self.command_manager is not None:
                        self.command_manager.send_control_command(usv_id, pwml, pwmr)
            else:
                # 动力学仿真模式
                result = self._control_with_terminal(usv_id, target_point, next_target, obstacles=self.get_obstacles())
            
            if result is not None:
                dynamic_x = result.get('x', 0)
                dynamic_y = result.get('y', 0)
                
                # 发出USV状态更新信号
                self.usv_state_updated.emit(str(usv_id), result)
                
                # 更新动力学轨迹数据
                if usv_id in self.dynamics_trajectory_data:
                    self.dynamics_trajectory_data[usv_id]['x'].append(dynamic_x)
                    self.dynamics_trajectory_data[usv_id]['y'].append(dynamic_y)
                    
                    if len(self.dynamics_trajectory_data[usv_id]['x']) > self.MAX_TRAJECTORY_POINTS:
                        self.dynamics_trajectory_data[usv_id]['x'] = self.dynamics_trajectory_data[usv_id]['x'][-self.MAX_TRAJECTORY_POINTS:]
                        self.dynamics_trajectory_data[usv_id]['y'] = self.dynamics_trajectory_data[usv_id]['y'][-self.MAX_TRAJECTORY_POINTS:]
                    
                    if usv_id in self.dynamics_trajectory_curves:
                        self.dynamics_trajectory_curves[usv_id].setData(
                            self.dynamics_trajectory_data[usv_id]['x'],
                            self.dynamics_trajectory_data[usv_id]['y']
                        )
                
                # 更新当前点和十字准线为实际位置
                if usv_id in self.current_point_markers:
                    self.current_point_markers[usv_id].setData([dynamic_x], [dynamic_y])
                if usv_id in self.crosshair_markers:
                    self.crosshair_markers[usv_id].setData([dynamic_x], [dynamic_y])

                if((dynamic_x - target_point[0])**2 + (dynamic_y - target_point[1])**2 < return_reached_threshold**2):
                    all_reached_return = True
                else:
                    all_reached_return = False
        
        # 所有USV都到达返航点，自动停止
        if all_reached_return:
            self.stop_drawing()
    #PtracKing路径跟踪模式
    def update_path_tracking(self):
        """
        更新路径跟踪状态
        """
        # 检查是否所有USV都到达了终点
        all_reached_end = True
        
        for usv_id in self.active_usvs:
            if usv_id not in self.current_paths or not self.current_paths[usv_id]:#该USV没有路径等，跳过控制
                continue
            
            # 获取当前路径和目标航迹点索引
            path = self.current_paths[usv_id]
            index = self.current_waypoint_indices[usv_id]
            
            # 检查是否到达最后一个航迹点
            if index >= len(path):
                # 到达终点，停止控制
                continue
            
            all_reached_end = False
            
            # 获取当前目标航迹点和下一个航迹点
            target_point = path[index]
            next_target_point = path[min(index + 1, len(path) - 1)]
            
            # 实船 使用动力学仿真更新位置，传递预先计算的参考点和下一个参考点
            if self.dynamics_enabled is False:
                # 获取USV当前状态
                current_state = self.motion_capture.get_usv_state(usv_id)                        
                result = self._control_with_terminal(usv_id, target_point, next_target_point, current_state, obstacles=self.get_obstacles())
                if result is not None:
                    pwml = result['pwml']
                    pwmr = result['pwmr']
                    
                    # 应用死区
                    if pwml > 0:
                        pwml = self.base_positive + pwml
                    elif pwml < 0:
                        pwml = self.base_nagitive + pwml
                    if pwmr > 0:
                        pwmr = self.base_positive + pwmr
                    elif pwmr < 0:
                        pwmr = self.base_nagitive + pwmr
                    
                    # 发送控制指令
                    self.command_manager.send_control_command(usv_id, pwml, pwmr)
            else:
                # 动力学仿真模式
                # 传递当前目标点和下一个目标点
                result = self._control_with_terminal(usv_id, target_point, next_target_point, obstacles=self.get_obstacles())
            self.current_waypoint_indices[usv_id] += 1
            # 只有当control_usv返回有效结果时才进行解包
            if result is not None:
                dynamic_x = result['x']
                dynamic_y = result['y']
                                        
                # 发出USV状态更新信号，确保usv_id是字符串类型
                self.usv_state_updated.emit(str(usv_id), result)
                
                # 更新动力学轨迹数据
                if usv_id in self.dynamics_trajectory_data:
                    self.dynamics_trajectory_data[usv_id]['x'].append(dynamic_x)
                    self.dynamics_trajectory_data[usv_id]['y'].append(dynamic_y)
                    
                    # 限制动力学轨迹数据点数量，防止内存泄漏
                    if len(self.dynamics_trajectory_data[usv_id]['x']) > self.MAX_TRAJECTORY_POINTS:
                        self.dynamics_trajectory_data[usv_id]['x'] = self.dynamics_trajectory_data[usv_id]['x'][-self.MAX_TRAJECTORY_POINTS:]
                        self.dynamics_trajectory_data[usv_id]['y'] = self.dynamics_trajectory_data[usv_id]['y'][-self.MAX_TRAJECTORY_POINTS:]
                    
                    # 更新动力学轨迹曲线
                    if usv_id in self.dynamics_trajectory_curves:
                        self.dynamics_trajectory_curves[usv_id].setData(
                            self.dynamics_trajectory_data[usv_id]['x'],
                            self.dynamics_trajectory_data[usv_id]['y']
                        )
                    
                    # 更新当前点标记和十字准线为动力学仿真位置
                    if usv_id in self.current_point_markers:
                        self.current_point_markers[usv_id].setData([dynamic_x], [dynamic_y])
                    if usv_id in self.crosshair_markers:
                        self.crosshair_markers[usv_id].setData([dynamic_x], [dynamic_y])
        
        # 如果所有USV都到达了终点，停止动画
        if all_reached_end:
            self.stop_drawing()
    #WTracking（打点航点）模式 的核心循环，每 100ms 调用一次。
    def update_waypoint_tracking(self):
        """
        更新打点航点跟踪状态

        WTracking 与 PTracking 使用同一套逐航点跟踪内核（current_paths +
        current_waypoint_indices），区别只在航线来源：PTracking 的航线来自外部加载/
        覆盖规划；WTracking 的航线来自鼠标手动打点、右键确认后生成的航线。
        单独保留入口函数，方便后续为打点模式定制逻辑（如到点判断、不同速度）。
        """
        self.update_path_tracking()
    #TTracking（轨迹跟踪/虚拟领航者编队）模式 的核心循环，每 100ms 调用一次。            
    def update_trajevtory_tracking(self):
        """
        更新轨迹跟踪状态
        """
        # 为每个激活的USV更新轨迹
        for usv_id in self.active_usvs:
            # 获取USV相对于领航者的偏移量
            x_bias, y_bias, interval = self.usv_offsets[usv_id]

            # 切换控制：如果启用且达到切换时间，进行平滑过渡  慢慢从旧位置到新位置过渡
            if self.exchange_control_enabled and self.current_t >= self.exchange_time_threshold:
                if usv_id in self.exchange_biases and usv_id in self.exchange_biases_from:
                    from_x, from_y = self.exchange_biases_from[usv_id] #旧位置
                    to_x, to_y = self.exchange_biases[usv_id] #新位置
                    if self.current_t < self.exchange_time_threshold + self.exchange_duration_threshold:
                        # 过渡期间，使用(1-t)线性插值
                        t_factor = (self.current_t - self.exchange_time_threshold) / self.exchange_duration_threshold
                        x_bias = (1 - t_factor) * from_x + t_factor * to_x
                        y_bias = (1 - t_factor) * from_y + t_factor * to_y
                    else:
                        # 过渡结束后，完全使用目标偏置
                        x_bias = to_x
                        y_bias = to_y
            # 计算领航者的当前位置和方向角（不带有任何偏移量）
            leader_x, leader_y = self.calculate_point(self.current_t, x_offset=self.x_offset, y_offset=self.y_offset, interval=interval * np.pi/100)
            leader_angle = self.calculate_leader_direction(self.current_t)
            
            # 计算领航者的下一个参考点位置：增量取 t_step，使其与参考点每步实际推进一致，
            # 这样控制器用 segment_length/delta_t 估算的速度才等于真实参考点推进速度。
            delta_t = self.t_step
            leader_next_x, leader_next_y = self.calculate_point(self.current_t + delta_t, x_offset=self.x_offset, y_offset=self.y_offset, interval=interval * np.pi/100)
            
            # 更新虚拟领航点原点位置
            self.virtual_leader_marker.setData([leader_x], [leader_y])
            
            
            # 将相对偏移量转换为全局坐标系下的偏移量
            # 基于领航者的当前方向角进行旋转变换
            global_x_bias = x_bias * np.cos(leader_angle) - y_bias * np.sin(leader_angle)
            global_y_bias = x_bias * np.sin(leader_angle) + y_bias * np.cos(leader_angle)
            
            # 计算USV的期望位置（领航者位置加上相对偏移量）虚拟领航者编队法的基本几何关系
            x = leader_x + global_x_bias
            y = leader_y + global_y_bias
            
            # 更新轨迹数据
            self.trajectory_data[usv_id]['x'].append(x)
            self.trajectory_data[usv_id]['y'].append(y)
            
            # 限制轨迹数据点数量，防止内存泄漏
            if len(self.trajectory_data[usv_id]['x']) > self.MAX_TRAJECTORY_POINTS:
                self.trajectory_data[usv_id]['x'] = self.trajectory_data[usv_id]['x'][-self.MAX_TRAJECTORY_POINTS:]
                self.trajectory_data[usv_id]['y'] = self.trajectory_data[usv_id]['y'][-self.MAX_TRAJECTORY_POINTS:]
            
            # 更新轨迹曲线
            self.trajectory_curves[usv_id].setData(
                self.trajectory_data[usv_id]['x'], 
                self.trajectory_data[usv_id]['y']
            )
            # 计算USV的参考点（带偏移量），传递给控制函数避免重复计算
            usv_reference_point = (x, y)
            
            # 计算领航者下一个位置的方向角
            leader_next_angle = self.calculate_leader_direction(self.current_t + delta_t)
            
            # 将相对偏移量转换为下一个时间点的全局坐标系下的偏移量
            # 基于领航者下一个位置的方向角进行旋转变换
            next_global_x_bias = x_bias * np.cos(leader_next_angle) - y_bias * np.sin(leader_next_angle)
            next_global_y_bias = x_bias * np.sin(leader_next_angle) + y_bias * np.cos(leader_next_angle)
            
            # 计算USV的下一个期望位置（领航者下一个位置加上相对偏移量）
            next_x = leader_next_x + next_global_x_bias
            next_y = leader_next_y + next_global_y_bias
            usv_next_reference_point = (next_x, next_y)
            
            # 使用动力学仿真更新位置，传递预先计算的参考点和下一个参考点
            # 确保USV ID在激活列表中
            if usv_id in self.active_usvs and usv_id in self.usv_offsets:
                if self.dynamics_enabled is False:
                    usv_state = self.motion_capture.get_usv_state(usv_id)
                    # 直接传递参考点和下一个参考点，避免重复计算
                    result = self._control_with_terminal(usv_id, usv_reference_point, usv_next_reference_point, usv_state, obstacles=self.get_obstacles())
                    pwml = result['pwml']
                    pwmr = result['pwmr']                
                    if pwml>0:
                        pwml = self.base_positive + pwml
                    elif pwml<0:
                        pwml = self.base_nagitive + pwml
                    if pwmr>0:
                        pwmr = self.base_positive + pwmr
                    elif pwmr<0:
                        pwmr = self.base_nagitive + pwmr
                    # self.command_manager.send_control_command(usv_id, pwmr, pwml)#原版du的Ts是正
                    self.command_manager.send_control_command(usv_id, pwml, pwmr)#后版du的Ts是负数
                else:
                    # 直接传递参考点和下一个参考点，避免重复计算
                    result = self._control_with_terminal(usv_id, usv_reference_point, usv_next_reference_point, obstacles=self.get_obstacles())
                    
            # 只有当control_usv返回有效结果时才进行解包
            if result is not None:
                dynamic_x = result['x']
                dynamic_y = result['y']
                                        
                # 发出USV状态更新信号，确保usv_id是字符串类型
                self.usv_state_updated.emit(str(usv_id), result)
                
                # 更新动力学轨迹数据
                if usv_id in self.dynamics_trajectory_data:
                    self.dynamics_trajectory_data[usv_id]['x'].append(dynamic_x)
                    self.dynamics_trajectory_data[usv_id]['y'].append(dynamic_y)
                    
                    # 限制动力学轨迹数据点数量，防止内存泄漏
                    if len(self.dynamics_trajectory_data[usv_id]['x']) > self.MAX_TRAJECTORY_POINTS:
                        self.dynamics_trajectory_data[usv_id]['x'] = self.dynamics_trajectory_data[usv_id]['x'][-self.MAX_TRAJECTORY_POINTS:]
                        self.dynamics_trajectory_data[usv_id]['y'] = self.dynamics_trajectory_data[usv_id]['y'][-self.MAX_TRAJECTORY_POINTS:]
                    
                    # 更新动力学轨迹曲线
                    if usv_id in self.dynamics_trajectory_curves:
                        self.dynamics_trajectory_curves[usv_id].setData(
                            self.dynamics_trajectory_data[usv_id]['x'],
                            self.dynamics_trajectory_data[usv_id]['y']
                        )
                    
                    # 更新当前点标记和十字准线为动力学仿真位置
                    if usv_id in self.current_point_markers:
                        self.current_point_markers[usv_id].setData([dynamic_x], [dynamic_y])
                    if usv_id in self.crosshair_markers:
                        self.crosshair_markers[usv_id].setData([dynamic_x], [dynamic_y])
            
        # 增加时间步长
        self.current_t += self.t_step
        
        # 单圈模式下，当达到完整的2π周期时停止
        if self.loop_type == 'single' and self.omega_val != 0:
            # 计算当前累计角度
            current_angle = self.omega_val * self.current_t
            # 当累计角度达到或超过2π时停止
            if current_angle >= 2 * np.pi:
                self.stop_drawing()
    #停止绘画
    def stop_drawing(self):
        self.is_drawing = False
        # self.current_paths = {}  # 存储每个USV的路径点
        self.current_waypoint_indices = {}  # 存储每个USV当前目标航迹点索引
        self.animation_timer.stop()
        
        self.trajectory_started.emit("stopped")
    #复位轨迹
    def reset_trajectory(self):
        if self.is_drawing:
            self.stop_drawing()
        
        self.current_t = 0.0
        
        # 计算参考轨迹的初始位置和方向角
        reference_x, reference_y = self.calculate_point(0)
        reference_angle = self.calculate_leader_direction(0)
        
        # 更新虚拟领航点原点位置
        self.virtual_leader_marker.setData([reference_x], [reference_y])
        
        # 重置所有USV的轨迹数据和标记
        # 使用set交集确保只处理存在于所有必要字典中的USV ID
        usv_ids_to_process = set(self.active_usvs.keys()) & set(self.trajectory_curves.keys()) & set(self.current_point_markers.keys()) & set(self.crosshair_markers.keys())
        
        for usv_id in usv_ids_to_process:
            # 重置轨迹数据
            if usv_id in self.trajectory_data:
                self.trajectory_data[usv_id] = {'x': [], 'y': []}
            
            # 清除轨迹曲线
            if usv_id in self.trajectory_curves:
                self.trajectory_curves[usv_id].setData([], [])
            
            # 计算初始位置
            if usv_id in self.usv_offsets:
                x_bias, y_bias, interval = self.usv_offsets[usv_id]
                
                # 将相对偏移量转换为全局坐标系下的偏移量
                # 基于参考轨迹的初始方向角进行旋转变换
                global_x_bias = x_bias * np.cos(reference_angle) - y_bias * np.sin(reference_angle)
                global_y_bias = x_bias * np.sin(reference_angle) + y_bias * np.cos(reference_angle)
                
                # 计算当前时间点的轨迹坐标（应用轨迹整体偏移量和转换后的全局偏移量）
                x, y = self.calculate_point(0, x_offset=self.x_offset + global_x_bias, y_offset=self.y_offset + global_y_bias, interval=interval * np.pi/100)
                
                # 更新标记
                if usv_id in self.current_point_markers:
                    self.current_point_markers[usv_id].setData([x], [y])
                if usv_id in self.crosshair_markers:
                    self.crosshair_markers[usv_id].setData([x], [y])
                
                # 重置动力学轨迹数据和曲线
                if usv_id in self.dynamics_trajectory_data:
                    self.dynamics_trajectory_data[usv_id] = {'x': [], 'y': []}
                
                # 清除动力学轨迹曲线（只有当曲线存在时才操作）
                if usv_id in self.dynamics_trajectory_curves:
                    self.dynamics_trajectory_curves[usv_id].setData([], [])
                
                # 重置USV控制系统
                self.usv_manager.reset_control_system(usv_id, [0.0, 0.0, 0.0, x, y, reference_angle])
    #清除轨迹
    def clear_trajectory(self):
        """
        只清除当前绘图内容，不参与船舶实验
        """
        # 清除所有USV的轨迹数据和曲线，但不停止绘制，不重置USV控制系统
        # 使用set交集确保只处理存在于所有字典中的USV ID
        usv_ids_to_process = set(self.active_usvs.keys()) & set(self.trajectory_curves.keys())
        
        for usv_id in usv_ids_to_process:
            # 清除轨迹数据
            if usv_id in self.trajectory_data:
                self.trajectory_data[usv_id] = {'x': [], 'y': []}
            
            # 清除轨迹曲线
            if usv_id in self.trajectory_curves:
                self.trajectory_curves[usv_id].setData([], [])
            
            # 清除动力学轨迹数据和曲线
            if usv_id in self.dynamics_trajectory_data:
                self.dynamics_trajectory_data[usv_id] = {'x': [], 'y': []}
            if usv_id in self.dynamics_trajectory_curves:
                self.dynamics_trajectory_curves[usv_id].setData([], [])
        
        # 保持当前时间和位置不变
        
        self.trajectory_reset.emit("reset")
    #设置轨迹周期类型 单圈/多圈
    def set_loop_type(self, loop_type):
        """设置轨迹周期类型并更新轨迹"""
        self.loop_type = loop_type
        self.draw_static_trajectory()
    #更新激活的船队以及船队偏置        
    def set_usvs(self, usvs_config):
        """
        设置激活的USV及其偏移量
        参数:
            usvs_config: 字典，键为USV ID（如'USV1'），值为(x_bias, y_bias, interval)
        """
        # 保存旧的USV配置用于比较
        old_active_usvs = set(self.active_usvs.keys())
        
        # 清除旧的USV配置
        self.active_usvs.clear()
        self.usv_offsets.clear()
        
        # 设置新的USV配置
        for usv_id, (x_bias, y_bias, interval) in usvs_config.items():
            self.active_usvs[usv_id] = True
            self.usv_offsets[usv_id] = (x_bias, y_bias, interval)
        
        # 新的USV配置
        new_active_usvs = set(self.active_usvs.keys())
        
        # 计算需要添加和移除的USV
        usvs_to_add = new_active_usvs - old_active_usvs
        usvs_to_remove = old_active_usvs - new_active_usvs
        
        # 如果正在绘制，只更新必要的部分而不重置整个轨迹
        if self.is_drawing:
            # 处理需要移除的USV
            for usv_id in usvs_to_remove:
                # 从轨迹数据中移除
                if usv_id in self.trajectory_data:
                    del self.trajectory_data[usv_id]
                # 从轨迹曲线中移除
                if usv_id in self.trajectory_curves:
                    self.plot_widget.removeItem(self.trajectory_curves[usv_id])
                    del self.trajectory_curves[usv_id]
                # 从标记中移除
                if usv_id in self.current_point_markers:
                    self.plot_widget.removeItem(self.current_point_markers[usv_id])
                    del self.current_point_markers[usv_id]
                if usv_id in self.crosshair_markers:
                    self.plot_widget.removeItem(self.crosshair_markers[usv_id])
                    del self.crosshair_markers[usv_id]
                # 处理动力学相关数据                
                if usv_id in self.dynamics_trajectory_data:
                    del self.dynamics_trajectory_data[usv_id]
                if usv_id in self.dynamics_trajectory_curves:
                    self.plot_widget.removeItem(self.dynamics_trajectory_curves[usv_id])
                    del self.dynamics_trajectory_curves[usv_id]
            
            # 处理需要添加的USV
            if usvs_to_add:
                # 计算当前参考轨迹的方向角
                reference_angle = self.calculate_leader_direction(self.current_t)
                
                # 为新添加的USV创建轨迹曲线和标记
                for usv_id in usvs_to_add:
                    color_rgb = self.usv_colors.get(usv_id, (128, 128, 128))
                    
                    # 创建轨迹曲线
                    self.trajectory_curves[usv_id] = self.plot_widget.plot(
                        pen=pg.mkPen(color_rgb, width=2)
                    )
                    
                    # 创建当前点标记
                    self.current_point_markers[usv_id] = self.plot_widget.scatterPlot(
                        [0], [0], size=5, brush=pg.mkBrush(color_rgb), symbol='o'
                    )
                    
                    # 创建十字准线标记
                    self.crosshair_markers[usv_id] = self.plot_widget.scatterPlot(
                        [0], [0], size=15, symbol='+', brush=pg.mkBrush(color_rgb)
                    )
                    
                    # 初始化轨迹数据
                    self.trajectory_data[usv_id] = {'x': [], 'y': []}
                    
                    # 计算当前时间点的轨迹坐标
                    x_bias, y_bias, interval = self.usv_offsets[usv_id]
                    global_x_bias = x_bias * np.cos(reference_angle) - y_bias * np.sin(reference_angle)
                    global_y_bias = x_bias * np.sin(reference_angle) + y_bias * np.cos(reference_angle)
                    x, y = self.calculate_point(self.current_t, x_offset=self.x_offset + global_x_bias, y_offset=self.y_offset + global_y_bias, interval=interval * np.pi/100)
                    
                    # 更新标记位置
                    self.current_point_markers[usv_id].setData([x], [y])
                    self.crosshair_markers[usv_id].setData([x], [y])
                    
                    # 初始化动力学轨迹数据和曲线
                    self.dynamics_trajectory_data[usv_id] = {'x': [], 'y': []}
                    self.dynamics_trajectory_curves[usv_id] = self.plot_widget.plot(
                        pen=pg.mkPen(color_rgb, style=Qt.DashLine, width=2)
                    )
                    
                    # 重置USV控制系统，设置正确的初始状态
                    if self.usv_manager is not None:
                        initial_state = [0.0, 0.0, 0.0, x, y, reference_angle]
                        self.usv_manager.reset_control_system(usv_id, initial_state)
        else:
            # 如果没有在绘制，正常设置USV配置
            self.setup_usv_trajectory_curves()
            # 立即更新所有标记到正确的初始位置
            if self.active_usvs:
                # 计算参考轨迹的初始方向角
                reference_angle = self.calculate_leader_direction(0)
                
                # 计算参考轨迹的初始位置并更新虚拟领航点原点
                reference_x, reference_y = self.calculate_point(0)
                self.virtual_leader_marker.setData([reference_x], [reference_y])
                
                # 更新所有USV的标记位置
                for usv_id in self.active_usvs:
                    x_bias, y_bias, interval = self.usv_offsets[usv_id]
                    
                    # 将相对偏移量转换为全局坐标系下的偏移量
                    global_x_bias = x_bias * np.cos(reference_angle) - y_bias * np.sin(reference_angle)
                    global_y_bias = x_bias * np.sin(reference_angle) + y_bias * np.cos(reference_angle)
                    
                    # 计算当前时间点的轨迹坐标
                    x, y = self.calculate_point(0, x_offset=self.x_offset + global_x_bias, y_offset=self.y_offset + global_y_bias, interval=interval * np.pi/100)
                    
                    # 更新标记
                    self.current_point_markers[usv_id].setData([x], [y])
                    self.crosshair_markers[usv_id].setData([x], [y])
    
    #下发队形切换参数
    def set_exchange_control_params(self, exchange_enabled, exchange_time, exchange_biases, exchange_duration=0, exchange_biases_from=None):
        """
        设置切换控制参数

        参数:
            exchange_enabled: 是否启用切换控制
            exchange_time: 切换时间（实时秒）
            exchange_biases: 字典，键为USV ID（如'USV1'），值为(x_bias, y_bias)
            exchange_duration: 过渡持续时间（实时秒），从exchange_time起经过此时间后完全采用目标偏置
            exchange_biases_from: 字典，键为USV ID，值为原始(x_bias, y_bias)（过渡起始值）
        """
        self.exchange_control_enabled = exchange_enabled
        self.exchange_time = exchange_time
        # 将实时秒转换为current_t单位
        # current_t每帧增加t_step，帧间隔为animation_interval毫秒
        # 所以current_t速率 = t_step / (animation_interval/1000) 单位/秒
        self.exchange_time_threshold = exchange_time * (1000.0 / self.animation_interval) * self.t_step
        self.exchange_biases = exchange_biases
        self.exchange_duration = exchange_duration
        self.exchange_duration_threshold = exchange_duration * (1000.0 / self.animation_interval) * self.t_step
        self.exchange_biases_from = exchange_biases_from if exchange_biases_from else {}
    #设置制导率
    def set_guidance_law(self, guidance_law):
        """
        设置所有USV的制导率
        
        参数:
            guidance_law: 制导率名称，可选值: 'LOS', 'PP'
        """
        if self.usv_manager is not None:
            for usv_id in self.usv_manager.get_registered_usvs():
                self.usv_manager.set_guidance_law(usv_id, guidance_law)
    #设置控制率
    def set_control_law(self, control_law):
        """
        设置所有USV的控制率
        
        参数:
            control_law: 控制率名称，可选值: 'PID', 'NMPC'
        """
        if self.usv_manager is not None:
            for usv_id in self.usv_manager.get_registered_usvs():
                self.usv_manager.set_control_law(usv_id, control_law)
    #实物船/动力学仿真开关
        self._terminal_selected_control = control_law
        self._apply_soft_terminal_controls()

    def enable_dynamics(self, enabled):
        """
        启用或禁用动力学仿真
        
        参数:
            enabled: 是否启用动力学仿真
        """
        self.dynamics_enabled = enabled
        if enabled:
            # 确保USV已经注册到USVManager
            if self.usv_manager is not None:
                for usv_id in self.active_usvs:
                    # 如果USV还没有注册，就自动注册它
                    if not self.usv_manager.is_usv_connected(usv_id):
                        # 使用默认IP和端口注册USV（仿真模式）
                        self.usv_manager.register_usv(usv_id, "127.0.0.1", 7020)
                
        # 初始化动力学轨迹曲线
        self.setup_dynamics_trajectory_curves()
        # 重新应用所有UI参数到控制器算法
        if self.usv_manager is not None:
            # 获取主窗口实例，以便访问UI参数
            app = QApplication.instance()
            main_window = None
            for widget in app.topLevelWidgets():
                if hasattr(widget, 'ui') and hasattr(widget, 'usv_manager'):
                    main_window = widget
                    break
            
            if main_window is not None:
                # 重新加载所有USV的控制参数
                for usv_index in range(1, 6):
                    usv_id = f'USV{usv_index}'
                    if usv_id in self.active_usvs and hasattr(main_window, 'load_control_params'):
                        # 使用MainWindow的公共方法加载所有参数
                        main_window.load_control_params(usv_index)
    #重建参考轨迹图形 图形初始化
    def setup_usv_trajectory_curves(self):
        """为激活的USV创建轨迹曲线和标记"""
        # 清除旧的轨迹曲线和标记
        for curve in self.trajectory_curves.values():
            self.plot_widget.removeItem(curve)
        for marker in self.current_point_markers.values():
            self.plot_widget.removeItem(marker)
        for marker in self.crosshair_markers.values():
            self.plot_widget.removeItem(marker)
        
        self.trajectory_curves.clear()
        self.current_point_markers.clear()
        self.crosshair_markers.clear()
        self.trajectory_data.clear()
        
        # 为每个激活的USV创建轨迹曲线和标记
        for usv_id in self.active_usvs:
            # 获取USV颜色
            color_rgb = self.usv_colors.get(usv_id, (128, 128, 128))
            
            # 创建轨迹曲线
            self.trajectory_curves[usv_id] = self.plot_widget.plot(
                pen=pg.mkPen(color_rgb, width=2)
            )
            
            # 创建当前点标记
            self.current_point_markers[usv_id] = self.plot_widget.scatterPlot(
                [0], [0], size=5, brush=pg.mkBrush(color_rgb), symbol='o'
            )
            
            # 创建十字准线标记
            self.crosshair_markers[usv_id] = self.plot_widget.scatterPlot(
                [0], [0], size=15, symbol='+', brush=pg.mkBrush(color_rgb)
            )
            
            # 初始化轨迹数据
            self.trajectory_data[usv_id] = {'x': [], 'y': []}
    #重建虚线并重置控制器初态
    def setup_dynamics_trajectory_curves(self):
        """
        为激活的USV创建动力学仿真轨迹曲线
        """
        # 清除旧的动力学轨迹曲线
        for curve in self.dynamics_trajectory_curves.values():
            self.plot_widget.removeItem(curve)
        
        self.dynamics_trajectory_curves.clear()
        self.dynamics_trajectory_data.clear()
        
        # 计算参考轨迹的初始方向角
        reference_angle = self.calculate_leader_direction(0)
        
        # 为每个激活的USV创建动力学轨迹曲线
        for usv_id in self.active_usvs:
            # 获取USV颜色（使用半透明效果区分）
            color_rgb = self.usv_colors.get(usv_id, (128, 128, 128))
            
            # 创建动力学轨迹曲线（使用虚线样式区分）
            self.dynamics_trajectory_curves[usv_id] = self.plot_widget.plot(
                pen=pg.mkPen(color_rgb, style=Qt.DashLine, width=2)
            )
            
            # 初始化动力学轨迹数据
            self.dynamics_trajectory_data[usv_id] = {'x': [], 'y': []}
            
            # 计算USV的初始位置，考虑偏移量
            x_bias, y_bias, interval = self.usv_offsets[usv_id]
            
            # 将相对偏移量转换为全局坐标系下的偏移量
            global_x_bias = x_bias * np.cos(reference_angle) - y_bias * np.sin(reference_angle)
            global_y_bias = x_bias * np.sin(reference_angle) + y_bias * np.cos(reference_angle)
            
            # 计算初始位置
            x, y = self.calculate_point(0, x_offset=self.x_offset + global_x_bias, y_offset=self.y_offset + global_y_bias, interval=interval * np.pi/100)
            
            # 重置USV控制系统，设置正确的初始状态
            if self.usv_manager is not None:
                initial_state = [0.0, 0.0, 0.0, x, y, reference_angle]
                self.usv_manager.reset_control_system(usv_id, initial_state)
    #查询最新参考位置
    def get_current_position(self, usv_id=None):
        """获取当前位置
        
        参数:
            usv_id: USV ID，如果为None，则返回第一个激活的USV的位置
        """
        if not self.trajectory_data:
            return (0, 0)
        
        if usv_id is None:
            usv_id = next(iter(self.trajectory_data.keys()), None)
            if usv_id is None:
                return (0, 0)
        
        data = self.trajectory_data.get(usv_id, {'x': [], 'y': []})
        if data['x']:
            return (data['x'][-1], data['y'][-1])
        return (0, 0)
    #查询历史轨迹数据
    def get_trajectory_data(self, usv_id=None):
        """获取轨迹数据
        
        参数:
            usv_id: USV ID，如果为None，则返回所有USV的轨迹数据
        """
        if usv_id is None:
            return self.trajectory_data.copy()
        return self.trajectory_data.get(usv_id, {'x': [], 'y': []}).copy()
    #追加一个真实船位点并刷新画面，记录外部上报的实船实测位置
    def update_real_trajectory(self, usv_id, x, y):
        if usv_id not in self.real_trajectory_data:
            self.real_trajectory_data[usv_id] = {'x': [], 'y': []}
            color_rgb = self.usv_colors.get(usv_id, (128, 128, 128))
            color_name = '#%02x%02x%02x' % color_rgb
            self.real_trajectory_curves[usv_id] = self.plot_widget.plot(
                pen=pg.mkPen(color_name, width=2)
            )
            self.usv_markers[usv_id] = self.plot_widget.scatterPlot(
                [x], [y], size=20, brush=pg.mkBrush(color_name), symbol='t'
            )
        
        data = self.real_trajectory_data[usv_id]
        data['x'].append(x)
        data['y'].append(y)
        
        if len(data['x']) > self.MAX_TRAJECTORY_POINTS:
            data['x'] = data['x'][-self.MAX_TRAJECTORY_POINTS:]
            data['y'] = data['y'][-self.MAX_TRAJECTORY_POINTS:]
        
        self.real_trajectory_curves[usv_id].setData(data['x'], data['y'])
        self.usv_markers[usv_id].setData([x], [y])
    #查询某船真实历史轨迹
    def get_real_trajectory(self, usv_id):
        return self.real_trajectory_data.get(usv_id)
    #清空所有历史真实轨迹
    def clear_real_trajectories(self):
        for curve in self.real_trajectory_curves.values():
            curve.setData([], [])
        for marker in self.usv_markers.values():
            marker.setData([0], [0])
        self.real_trajectory_data = {}
        self.current_paths = {}
    #清空所有历史动力学轨迹 虚线轨迹
    def clear_dynamics_trajectories(self):
        """
        清除所有动力学轨迹曲线和数据
        """
        for curve in self.dynamics_trajectory_curves.values():
            curve.setData([], [])
        self.dynamics_trajectory_curves.clear()
        self.dynamics_trajectory_data.clear()
    #覆盖规划模式下的鼠标/键盘拦截
    def eventFilter(self, source, event):
        """
        事件过滤器，用于捕获鼠标事件
        """
        # ==================== [旧代码-已注释停用] 原 eventFilter 仅支持覆盖规划模式 ====================
        # if source == self.plot_widget.viewport() and self.coverage_mode:
        #     if event.type() == QEvent.MouseButtonPress and event.button() == Qt.LeftButton:
        #         # 鼠标左键点击，添加顶点
        #         pos = event.pos()
        #         view_pos = self.plot_widget.plotItem.vb.mapSceneToView(pos)
        #         x, y = view_pos.x(), view_pos.y()
        #         self.temp_coverage_points.append((x, y))
        #         self.draw_coverage_area()
        #         return True
        #     elif event.type() == QEvent.MouseButtonPress and event.button() == Qt.RightButton:
        #         # 鼠标右键点击结束绘制，计算覆盖路径
        #         if len(self.temp_coverage_points) >= 3:
        #             self.coverage_area = self.temp_coverage_points.copy()
        #             self.temp_coverage_points = []
        #             self.generate_coverage_path()
        #             self.draw_coverage_path()
        #         return True
        #     elif event.type() == QEvent.KeyPress:
        #         if event.key() == Qt.Key_Escape:
        #             # 取消绘制
        #             self.temp_coverage_points = []
        #             self.draw_coverage_area()
        #             return True
        # return super().eventFilter(source, event)
        # ==================== 外层先判视口，内部区分打点航点/覆盖规划两种模式 ====================
        if source == self.plot_widget.viewport():
            # 打点航点采集模式（WTracking）：左键添加航点，右键确认生成航线，Esc取消当次采集
            if self.waypoint_edit_mode:
                if event.type() == QEvent.MouseButtonPress and event.button() == Qt.LeftButton:
                    # 鼠标左键点击，添加一个航点
                    pos = event.pos()
                    view_pos = self.plot_widget.plotItem.vb.mapSceneToView(pos)
                    self.temp_waypoints.append((view_pos.x(), view_pos.y()))
                    self._draw_manual_waypoint_items(self.temp_waypoints)
                    return True
                elif event.type() == QEvent.MouseButtonPress and event.button() == Qt.RightButton:
                    # 鼠标右键点击结束打点，至少2个点才能构成航线
                    if len(self.temp_waypoints) >= 2:
                        self._confirm_manual_waypoints()
                    return True
                elif event.type() == QEvent.KeyPress and event.key() == Qt.Key_Escape:
                    # 取消本次打点，恢复显示已确认的航线（若有）
                    self.temp_waypoints = []
                    self._draw_manual_waypoint_items(self.manual_waypoint_path)
                    return True
            # 覆盖路径规划模式（逻辑与旧代码一致，仅因外层条件拆分而整体内移一层）
            elif self.coverage_mode:
                if event.type() == QEvent.MouseButtonPress and event.button() == Qt.LeftButton:
                    # 鼠标左键点击，添加顶点
                    pos = event.pos()
                    view_pos = self.plot_widget.plotItem.vb.mapSceneToView(pos)
                    x, y = view_pos.x(), view_pos.y()
                    self.temp_coverage_points.append((x, y))
                    self.draw_coverage_area()
                    return True
                elif event.type() == QEvent.MouseButtonPress and event.button() == Qt.RightButton:
                    # 鼠标右键点击结束绘制，计算覆盖路径
                    if len(self.temp_coverage_points) >= 3:
                        self.coverage_area = self.temp_coverage_points.copy()
                        self.temp_coverage_points = []
                        self.generate_coverage_path()
                        self.draw_coverage_path()
                    return True
                elif event.type() == QEvent.KeyPress:
                    if event.key() == Qt.Key_Escape:
                        # 取消绘制
                        self.temp_coverage_points = []
                        self.draw_coverage_area()
                        return True
            # 障碍物放置模式：左键点击画布发射位置信号，Esc退出放置模式
            elif self.obstacle_placement_mode:
                if event.type() == QEvent.MouseButtonPress and event.button() == Qt.LeftButton:
                    # 鼠标左键点击，发射障碍物放置位置（可连续放置）
                    pos = event.pos()
                    view_pos = self.plot_widget.plotItem.vb.mapSceneToView(pos)
                    self.obstacle_position_selected.emit(view_pos.x(), view_pos.y())
                    return True
                elif event.type() == QEvent.KeyPress and event.key() == Qt.Key_Escape:
                    # 退出障碍物放置模式
                    self.set_obstacle_placement_mode(False)
                    return True
        return super().eventFilter(source, event)
    #启动覆盖路径规划模式
    def start_coverage_planning(self, spacing=0.5, angle=0, speed=0.2):
        """
        开始覆盖路径规划
        """
        self.set_obstacle_placement_mode(False)  # 与障碍物放置模式互斥
        self.coverage_mode = True
        self.waypoint_edit_mode = False  # 与打点航点采集模式互斥
        self.temp_coverage_points = []
        self.coverage_area = []
        self.coverage_path = []
        self.coverage_waypoints = []
        self.dense_coverage_markers = {}
        self.coverage_spacing = spacing
        self.coverage_angle = angle
        self.coverage_speed = speed
        self.clear_coverage_display()
    #停止覆盖路径规划
    def stop_coverage_planning(self):
        """
        停止覆盖路径规划
        """
        self.coverage_mode = False

    # ===== 打点航点（WTracking）模式 =====
    def start_waypoint_picking(self):
        """
        进入打点航点采集模式：清除旧航线，开始接收鼠标左键打点
        """
        self.set_obstacle_placement_mode(False)  # 与障碍物放置模式互斥
        self.waypoint_edit_mode = True
        self.coverage_mode = False  # 与覆盖规划模式互斥
        self.temp_waypoints = []
        self.manual_waypoint_path = []
        self._remove_manual_waypoint_items()
    #确认打点航线 右键确认
    def _confirm_manual_waypoints(self):
        """右键确认：保存打点航线、下发给所有激活USV，并退出采集模式"""
        self.manual_waypoint_path = self.temp_waypoints.copy()
        self.temp_waypoints = []
        self.waypoint_edit_mode = False
        # 将手动航线下发给所有激活USV（set_path内部会按0.2m/s、0.1s插值稠密化）
        for usv_id in self.active_usvs:
            self.set_path(usv_id, self.manual_waypoint_path)
        self._draw_manual_waypoint_items(self.manual_waypoint_path)
    #清除打点航线  ESC键清除
    def clear_manual_waypoints(self):
        """清除打点航线（图元、数据和已下发路径），并退出采集模式"""
        self.waypoint_edit_mode = False
        self.temp_waypoints = []
        self.manual_waypoint_path = []
        self._remove_manual_waypoint_items()
        self.current_paths.clear()
        self.current_waypoint_indices.clear()
    #移除手动航线曲线和航点标记
    def _remove_manual_waypoint_items(self):
        """从画布移除手动航线曲线和航点标记"""
        if self.manual_waypoint_curve is not None:
            self.plot_widget.removeItem(self.manual_waypoint_curve)
            self.manual_waypoint_curve = None
        if self.manual_waypoint_markers is not None:
            self.plot_widget.removeItem(self.manual_waypoint_markers)
            self.manual_waypoint_markers = None
    #重绘手动航线：洋红色折线 + 航点圆点
    def _draw_manual_waypoint_items(self, points):
        """重绘手动航线：洋红色折线 + 航点圆点"""
        self._remove_manual_waypoint_items()
        if not points:
            return
        x, y = zip(*points)
        self.manual_waypoint_curve = self.plot_widget.plot(
            list(x), list(y), pen=pg.mkPen('magenta', width=2)
        )
        self.manual_waypoint_markers = pg.ScatterPlotItem(
            x=list(x), y=list(y), size=8, symbol='o',
            brush=pg.mkBrush('magenta'), pen=pg.mkPen('black', width=1)
        )
        self.plot_widget.addItem(self.manual_waypoint_markers)

    # ===== 障碍物（避障）模式 =====
    def set_obstacle_placement_mode(self, enabled):
        """开关画布放置障碍物模式（与打点航点/覆盖规划模式互斥）。"""
        if enabled:
            self.waypoint_edit_mode = False
            self.coverage_mode = False
        if self.obstacle_placement_mode == enabled:
            return
        self.obstacle_placement_mode = enabled
        if enabled:
            self.plot_widget.viewport().setFocus()
            self.plot_widget.viewport().setCursor(Qt.CrossCursor)
        else:
            self.plot_widget.viewport().unsetCursor()
        self.obstacle_placement_mode_changed.emit(enabled)

    def add_obstacle(self, shape, x, y, radius=None, width=None, height=None):
        """添加障碍物并绘制，返回 obs_id。shape: 'circle' 或 'rectangle'。"""
        self._obstacle_seq += 1
        obs_id = f"obs{self._obstacle_seq}"
        if shape == 'circle':
            obs = CircleObstacle(obs_id, radius=radius if radius else 0.15, position=(x, y))
        else:
            obs = RectangleObstacle(obs_id, width=width if width else 0.15,
                                    height=height if height else 0.15, angle=0.0, position=(x, y))
        self.obstacles[obs_id] = obs
        self._draw_obstacle_item(obs_id, obs)
        return obs_id

    def get_obstacle_count(self):
        return len(self.obstacles)

    def get_obstacles(self):
        """返回当前障碍物对象列表（供控制循环传给USVController避障）。"""
        return list(self.obstacles.values())

    def clear_obstacles(self):
        for obs_id in list(self.obstacle_plot_items.keys()):
            item = self.obstacle_plot_items.pop(obs_id)
            self.plot_widget.removeItem(item)
        self.obstacles.clear()

    def _draw_obstacle_item(self, obs_id, obs):
        """在画布上绘制障碍物轮廓（橙色虚线）。"""
        if isinstance(obs, CircleObstacle):
            theta = np.linspace(0, 2 * np.pi, 64)
            xs = obs.position[0] + obs.radius * np.cos(theta)
            ys = obs.position[1] + obs.radius * np.sin(theta)
        else:  # RectangleObstacle
            hw, hh = obs.width / 2.0, obs.height / 2.0
            ca, sa = np.cos(obs.angle), np.sin(obs.angle)
            corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh), (-hw, -hh)]
            xs, ys = [], []
            for lx, ly in corners:
                xs.append(obs.position[0] + lx * ca - ly * sa)
                ys.append(obs.position[1] + lx * sa + ly * ca)
        pen = pg.mkPen(color='#ff5500', width=2, style=Qt.PenStyle.DashLine)
        item = self.plot_widget.plot(xs, ys, pen=pen)
        self.obstacle_plot_items[obs_id] = item
    #生成覆盖路径
    def generate_coverage_path(self):
        """
        生成覆盖路径
        """
        if not self.coverage_area or len(self.coverage_area) < 3:
            return
        
        # 使用覆盖路径规划器生成路径
        self.coverage_planner.set_coverage_area(self.coverage_area)
        self.coverage_planner.set_parameters(spacing=self.coverage_spacing, angle=self.coverage_angle)

        path, waypoints = self.coverage_planner.generate_coverage_path()
        
        # 对路径点进行拟合插值，生成稠密路径点序列
        dense_path = self.interpolate_path(path, speed=self.coverage_speed, time_step=0.1)
        
        self.coverage_path = path
        self.coverage_waypoints = waypoints
        self.dense_coverage_path = dense_path  # 存储稠密路径点
        # 将生成的路径赋值给所有USV
        for usv_id in [f'USV{i}' for i in range(1, 6)]:
            self.set_path(usv_id, dense_path)
        
    #对路径点进行拟合插值，生成稠密路径点序列
    def interpolate_path(self, path, speed=0.2, time_step=0.1):
        """
        对路径点进行拟合插值，生成稠密路径点序列
        
        参数:
            path: 原始路径点列表，每个点为(x, y)元组
            speed: 行驶速度（米/秒）
            time_step: 时间步长（秒） 
            
        返回:
            dense_path: 稠密路径点列表
        """
        if not path or len(path) < 2:
            return path
        
        dense_path = []
        step_distance = speed * time_step  # 每步的距离
        
        for i in range(len(path) - 1):
            start = path[i]
            end = path[i + 1]
            
            # 计算两点之间的距离
            distance = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
            
            # 计算需要的插值点数量
            num_points = max(1, int(distance / step_distance))
            
            # 线性插值
            for j in range(num_points + 1):
                t = j / num_points
                x = start[0] + t * (end[0] - start[0])
                y = start[1] + t * (end[1] - start[1])
                dense_path.append((x, y))
        
        # 添加最后一个点
        dense_path.append(path[-1])
        
        return dense_path
    #绘制覆盖区域
    def draw_coverage_area(self):
        """
        绘制覆盖区域
        """
        if self.coverage_area_curve:
            self.plot_widget.removeItem(self.coverage_area_curve)
        
        if self.temp_coverage_points:
            # 绘制临时顶点和连线
            x, y = zip(*self.temp_coverage_points)
            self.coverage_area_curve = self.plot_widget.plot(
                x, y, pen=pg.mkPen('orange', width=2), symbol='o', symbolSize=5, symbolBrush='orange'
            )
        elif self.coverage_area:
            # 绘制完整的覆盖区域
            x, y = zip(*self.coverage_area)
            # 闭合多边形
            x = list(x) + [x[0]]
            y = list(y) + [y[0]]
            self.coverage_area_curve = self.plot_widget.plot(
                x, y, pen=pg.mkPen('blue', width=2), fill=True, fillLevel=0, fillBrush=pg.mkBrush('cyan', alpha=80)
            )
    #绘制覆盖路径
    def draw_coverage_path(self):
        """
        绘制覆盖路径
        """
        # 清除旧的路径和航点
        if self.coverage_path_curve:
            self.plot_widget.removeItem(self.coverage_path_curve)
        if self.coverage_waypoint_markers:
            self.plot_widget.removeItem(self.coverage_waypoint_markers)
        if self.dense_coverage_markers:
            self.plot_widget.removeItem(self.dense_coverage_markers)
        
        # 绘制路径
        if self.coverage_path:
            x, y = zip(*self.coverage_path)
            self.coverage_path_curve = self.plot_widget.plot(
                x, y, pen=pg.mkPen('green', width=2)
            )
        
        # 绘制稠密路径点
        if self.dense_coverage_path:
            x, y = zip(*self.dense_coverage_path)
            self.dense_coverage_markers = pg.ScatterPlotItem(
                x=x, y=y, size=3, brush='blue', symbol='+'
            )
            self.plot_widget.addItem(self.dense_coverage_markers)
        
        # 绘制航点
        if self.coverage_waypoints:
            pos = [wp['pos'] for wp in self.coverage_waypoints]
            x, y = zip(*pos)
            sizes = [6 if wp['turn'] else 4 for wp in self.coverage_waypoints]
            colors = ['yellow' if wp['turn'] else 'green' for wp in self.coverage_waypoints]
            self.coverage_waypoint_markers = pg.ScatterPlotItem(
                x=x, y=y, size=sizes, brush=colors, pen=pg.mkPen('black', width=1)
            )
            self.plot_widget.addItem(self.coverage_waypoint_markers)
    #清除覆盖路径显示
    def clear_coverage_display(self):
        """
        清除覆盖路径显示
        """
        if self.coverage_area_curve:
            self.plot_widget.removeItem(self.coverage_area_curve)
            self.coverage_area_curve = None
        if self.coverage_path_curve:
            self.plot_widget.removeItem(self.coverage_path_curve)
            self.coverage_path_curve = None
        if self.coverage_waypoint_markers:
            self.plot_widget.removeItem(self.coverage_waypoint_markers)
            self.coverage_waypoint_markers = None
        if self.dense_coverage_markers:
            self.plot_widget.removeItem(self.dense_coverage_markers)
            self.dense_coverage_markers = None
    #设置USV路径点
    def set_path(self, usv_id, waypoints):
        """
        设置USV的路径点
        
        参数:
            usv_id: USV ID
            waypoints: 路径点列表，每个点为(x, y)元组
        """
        # 对路径点进行拟合插值，生成稠密路径点序列
        dense_waypoints = self.interpolate_path(waypoints, speed=0.2, time_step=0.1)
        self.current_paths[usv_id] = dense_waypoints
        self.current_waypoint_indices[usv_id] = 0
    #设置路径跟踪模式
    def set_tracking_mode(self, Type):
        """
        设置路径跟踪模式
        
        参数:
            Type: 路径跟踪模式类型
        """
        self.tracking_mode = Type
    #设置返航点位置
    def set_return_point(self, x, y):
        """
        设置返航点位置并更新五角星标记
        
        参数:
            x: 返航点X坐标
            y: 返航点Y坐标
        """
        self.return_point_x = x
        self.return_point_y = y
        self.return_point_star.setData([x], [y])
        self.return_point_star.show()


if __name__ == "__main__":

    
    app = QApplication(sys.argv)
    
    widget = TrajectoryPlotWidget()
    widget.show()
    
    sys.exit(app.exec())
