#你好
# This Python file uses the following encoding: utf-8
import os # 导入os模块，用于操作操作系统
import time
import json  # 导入json模块，用于解析JSON数据
import configparser
import threading
import traceback
import numpy as np
from pathlib import Path  # 导入Path模块，用于操作文件路径
import sys
from PySide6.QtWidgets import QMainWindow, QApplication, QMessageBox, QLineEdit, QVBoxLayout, QFileDialog, QLabel, QGraphicsProxyWidget, QScrollArea, QFormLayout, QPushButton
from PySide6.QtGui import QFont
from PySide6.QtCore import QFile, QObject, Qt, Signal, QTimer
from PySide6.QtUiTools import QUiLoader
import pyqtgraph as pg
from UDPComm import UDPComm
from USVManager import USVManager
from CommandManager import CommandManager
from DataProcessor import DataProcessor
from MotionCapture import MotionCapture
from JoystickManager import JoystickManager
from TrajectoryPlotWidget import TrajectoryPlotWidget
from ModelIdentification import run_identification
from ExcitationProtocol import ExcitationProtocol, PWM_MID, PWM_DEAD_POS, PWM_DEAD_NEG


class IdentificationWorker(QObject):
    """模型辨识后台工作器：避免优化计算阻塞UI线程"""
    finished = Signal(object)   # 辨识成功，携带结果字典
    failed = Signal(str)        # 辨识失败，携带错误信息

    def __init__(self, cfg):
        super().__init__()
        self.cfg = cfg

    def run(self):
        try:
            result = run_identification(**self.cfg)
            self.finished.emit(result)
        except Exception as e:
            self.failed.emit(f"{e}")

# import  collect_global.common.commonTools as tools
# from collect_global.common.logManager import buildLogHelper, stopLog
# from collect_global.common.logManager import output, outputMode,enum_Error
# from navigation_global.zmqlib.zmq_client import *
# from navigation_global.zmqlib.zmq_define import *
# from navigation_global.common.topic_define import topic_define
# 注册pyqtgraph的PlotWidget控件，以便QFormBuilder能够识别它
# QUiLoader.registerCustomWidget(pg.PlotWidget)


class MainWindow(QMainWindow):
    """主界面类，整合所有模块"""
    
    def __init__(self):
        super(MainWindow, self).__init__()
        self.load_ui()
        # 设置最小窗口大小，防止界面缩成一团
        self.setMinimumSize(1289, 746)
        
        # 配置文件路径
        self.config_file = Path(__file__).parent / "config.ini"
        # 在模块初始化后再加载配置，确保UI控件已准备就绪
        self.setup_modules()
        self.load_config()
        self.connect_signals_slots()
        self.start_udp_server()

        #主线程消息zeromq
        # self.sub = ZMQClient("sub",None,zmqMember('192.168.0.104',5555),[topic_define.senson_radar.value[0]],zmqMember('192.168.0.104',5556))  #定义主管理类
        # #服务管理
        # output(f'[主程序]:[启动总线发布]',outputMode.saveLog,enum_Error.localInfo)
        # self.sub._run()
        time.sleep(2)                   #等待2秒，确保服务启动完成
            # 添加舟山东福山原点坐标
        self.origin_lat = 30.317  # 北纬30.317度
        self.origin_lon = 122.375  # 东经122.375度


    def closeEvent(self, event):
        """窗口关闭事件，自动保存配置并停止动捕系统"""
        # 停止动捕系统
        if hasattr(self, 'motion_capture'):
            self.motion_capture.stop()
        
        # 停止摇杆控制
        if hasattr(self, 'joystick_manager'):
            self.joystick_manager.shutdown()
        
        self.save_config()
        super().closeEvent(event)
    
    def load_config(self):
        """加载配置文件，参考C#反射机制自动遍历所有相关控件"""
        try:
            if self.config_file.exists():
                config = configparser.ConfigParser()
                config.read(self.config_file, encoding='utf-8')
                
                # 加载UI控件配置
                if 'UI_CONFIG' in config:
                    ui_config = config['UI_CONFIG']
                    
                    # 获取所有UI控件
                    ui_widgets = vars(self.ui)
                    
                    # 需要排除的控件名称
                    excluded_widgets = {
                        'statusBar', 'menuBar', 'centralWidget', 'toolBar',
                        'dockWidgetArea', 'statusBar', 'menubar'
                    }
                    
                    loaded_count = 0
                    
                    for widget_name, widget in ui_widgets.items():
                        # 跳过排除的控件
                        if widget_name in excluded_widgets:
                            continue
                        
                        # 跳过私有和内部控件
                        if widget_name.startswith('_') or widget_name.startswith('metaObject'):
                            continue
                        
                        # 如果配置中存在这个控件
                        if widget_name in ui_config:
                            config_value = ui_config[widget_name]
                            
                            try:
                                # 获取控件类型名称
                                widget_type = type(widget).__name__
                                
                                if widget_type == 'QLineEdit':
                                    # 文本输入框 - 设置text()
                                    widget.setText(config_value)
                                    loaded_count += 1
                                
                                elif widget_type in ['QCheckBox', 'QRadioButton']:
                                    # 复选框和单选按钮 - 设置checked状态
                                    checked = config_value.lower() == 'true'
                                    widget.setChecked(checked)
                                    loaded_count += 1
                                
                                elif widget_type == 'QComboBox':
                                    # 下拉选择框 - 设置当前选中项
                                    index = widget.findText(config_value)
                                    if index >= 0:
                                        widget.setCurrentIndex(index)
                                        loaded_count += 1
                                
                                elif widget_type == 'QSpinBox':
                                    # 数字输入框 - 设置value()
                                    value = int(config_value)
                                    widget.setValue(value)
                                    loaded_count += 1
                                
                                elif widget_type == 'QDoubleSpinBox':
                                    # 浮点数输入框 - 设置value()
                                    value = float(config_value)
                                    widget.setValue(value)
                                    loaded_count += 1
                                
                                elif widget_type == 'QSlider':
                                    # 滑块 - 设置value()
                                    value = int(config_value)
                                    widget.setValue(value)
                                    loaded_count += 1
                                
                                elif widget_type == 'QTextEdit':
                                    # 多行文本框 - 设置plainText()
                                    widget.setPlainText(config_value)
                                    loaded_count += 1
                                
                                # 其他控件类型可以在这里扩展
                                
                            except Exception as e:
                                print(f"加载控件 {widget_name} ({widget_type}) 失败: {e}")
                                continue
                    
                    print(f"UI控件配置加载完成: {loaded_count} 个控件")                
                # 更新轨迹绘制组件参数（UI控件已经通过反射机制加载完成）
                if hasattr(self, 'trajectory_plot'):
                    self.update_trajectory_params()
                
                # 加载窗口配置
                if 'WINDOW_CONFIG' in config:
                    window_config = config['WINDOW_CONFIG']
                    try:
                        # 恢复窗口大小和位置
                        width = int(window_config.get('width', 1289))
                        height = int(window_config.get('height', 746))

                        # 高缩放/多显示器下，保存的尺寸位置可能超出当前屏幕可用区域
                        # （缩放比改变后逻辑坐标系变化）——先钳制到目标屏幕内，防止窗口部分不可见
                        x = int(window_config['x']) if 'x' in window_config else None
                        y = int(window_config['y']) if 'y' in window_config else None
                        from PySide6.QtCore import QPoint, QRect
                        anchor = QPoint(x, y) if x is not None else self.frameGeometry().center()
                        screen = QApplication.screenAt(anchor) or QApplication.primaryScreen()
                        ag = screen.availableGeometry()
                        width = min(width, ag.width())
                        height = min(height, ag.height())
                        self.resize(width, height)

                        # 如果配置了位置，恢复位置（保证窗口完整落在屏幕内）
                        if x is not None and y is not None:
                            if not ag.intersects(QRect(x, y, width, height)):
                                x, y = ag.x() + 50, ag.y() + 50
                            x = max(ag.left(), min(x, ag.right() - width))
                            y = max(ag.top(), min(y, ag.bottom() - height))
                            self.move(x, y)
                    except (ValueError, TypeError):
                        # 如果配置无效，使用默认值
                        self.resize(1289, 746)
                
                print(f"配置加载成功: {self.config_file}")
            else:
                print("配置文件不存在，使用默认配置")
        except Exception as e:
            print(f"加载配置文件失败: {e}")
    
    def save_config(self):
        """保存配置文件，参考C#反射机制自动遍历所有相关控件"""
        try:
            config = configparser.ConfigParser()
            
            # 获取所有UI控件
            ui_widgets = vars(self.ui)
            
            # 需要排除的控件名称（类似C#中的TB_Recv、TB_Send、TB_Status）
            excluded_widgets = {
                'statusBar', 'menuBar', 'centralWidget', 'toolBar',
                'dockWidgetArea', 'statusBar', 'menubar'
            }
            
            # 保存UI控件配置
            config['UI_CONFIG'] = {}
            
            for widget_name, widget in ui_widgets.items():
                # 跳过排除的控件
                if widget_name in excluded_widgets:
                    continue
                
                # 跳过私有和内部控件
                if widget_name.startswith('_') or widget_name.startswith('metaObject'):
                    continue
                
                # 获取控件类型名称
                widget_type = type(widget).__name__
                
                try:
                    if widget_type == 'QLineEdit':
                        # 文本输入框 - 保存text()
                        text = widget.text()
                        if text:  # 只保存非空值
                            config['UI_CONFIG'][widget_name] = text
                    
                    elif widget_type in ['QCheckBox', 'QRadioButton']:
                        # 复选框和单选按钮 - 保存checked状态
                        checked = "true" if widget.isChecked() else "false"
                        config['UI_CONFIG'][widget_name] = checked
                    
                    elif widget_type == 'QComboBox':
                        # 下拉选择框 - 保存当前选中的文本
                        text = widget.currentText()
                        if text:  # 只保存非空值
                            config['UI_CONFIG'][widget_name] = text
                    
                    elif widget_type == 'QSpinBox':
                        # 数字输入框 - 保存value()
                        config['UI_CONFIG'][widget_name] = str(widget.value())
                    
                    elif widget_type == 'QDoubleSpinBox':
                        # 浮点数输入框 - 保存value()
                        config['UI_CONFIG'][widget_name] = str(widget.value())
                    
                    elif widget_type == 'QSlider':
                        # 滑块 - 保存value()
                        config['UI_CONFIG'][widget_name] = str(widget.value())
                    
                    elif widget_type == 'QTextEdit':
                        # 多行文本框 - 保存plainText()
                        text = widget.toPlainText()
                        if text:  # 只保存非空值
                            config['UI_CONFIG'][widget_name] = text
                    
                    # 其他控件类型可以在这里扩展
                    
                except Exception as e:
                    print(f"保存控件 {widget_name} ({widget_type}) 失败: {e}")
                    continue
            
            # 保存窗口配置
            config['WINDOW_CONFIG'] = {
                'width': str(self.width()),
                'height': str(self.height()),
                'x': str(self.x()),
                'y': str(self.y())
            }
            
            # 保存到文件
            with open(self.config_file, 'w', encoding='utf-8') as f:
                config.write(f)
            
            print(f"配置保存成功: {self.config_file}，保存了 {len(config['UI_CONFIG'])} 个UI控件配置")
        except Exception as e:
            print(f"保存配置文件失败: {e}")
        
    def load_ui(self):
        """加载UI界面"""
        loader = QUiLoader()
        path = os.fspath(Path(__file__).resolve().parent / "form.ui")
        ui_file = QFile(path)
        ui_file.open(QFile.ReadOnly)
        
        # 加载UI到central widget
        self.central_widget = loader.load(ui_file, self)
        self.setCentralWidget(self.central_widget)
        
        ui_file.close()
        
        # 设置窗口标题3
        self.setWindowTitle("室内无人船集群开发软件")
        
        # 设置初始窗口大小
        self.resize(1289, 746)
        
        # 保存原始字体大小
        self.original_font_size = self.font().pointSize()
        self.original_width = 1289
        self.original_height = 746
        
        # 将UI组件引用赋值给self.ui，保持向后兼容
        self.ui = self.central_widget

        # 避障面板预处理
        self._prepare_obstacle_panel()

        # 模型辨识页：动态创建 S0-S11 激励采集按钮
        self._setup_excitation_controls()


    def _prepare_obstacle_panel(self):
        """障碍物位置通过画布点击获取，隐藏中心X/Y输入行（仅保留形状与尺寸）。"""
        form = self.ui.findChild(QFormLayout, "formLayout_obstacle")
        if form is None:
            return
        try:
            # 仅当布局中存在“中心X/Y”标签行时才隐藏（当前form.ui的避障页签不含这两行）
            for row in range(form.rowCount()):
                label_item = form.itemAt(row, QFormLayout.LabelRole)
                widget = label_item.widget() if label_item is not None else None
                if widget is not None and ('中心X' in widget.text() or '中心Y' in widget.text()):
                    form.setRowVisible(row, False)
        except AttributeError:
            pass  # 旧版Qt无setRowVisible则保持可见

    def setup_modules(self):
        """初始化各个模块"""
        # 创建UDP通讯模块
        self.udp_comm = UDPComm(local_port=7020)
        self.adc_resolution = 4095.0
        self.adc_reference_voltage = 3.3
        self.battery_divider_ratio = 3.0  # R1=200k, R2=100k: Vbat = Vadc * (R1 + R2) / R2
        self.usv_battery_data = {}
        
        # 创建无人船管理模块
        self.usv_manager = USVManager()
        
        # 创建命令管理模块
        self.command_manager = CommandManager(self.udp_comm, self.usv_manager)
        
        # 创建数据处理模块
        self.data_processor = DataProcessor()
        
        # 创建动捕数据管理模块
        self.motion_capture = MotionCapture()
        
        # 连接动捕数据信号
        self.motion_capture.connection_status_changed.connect(self.handle_motion_capture_status)
        
        # 自动启动动捕数据采集
        self.start_motion_capture()
        
        # 创建摇杆管理模块
        self.joystick_manager = JoystickManager()
        
        # 连接摇杆信号
        self.joystick_manager.rc_data_updated.connect(self.handle_rc_data)
        self.joystick_manager.rc_connected.connect(self.handle_rc_connection)
        self.joystick_manager.control_command.connect(self.handle_joystick_command)

        # 创建轨迹绘制组件，直接传递USVManager实例
        self.trajectory_plot = TrajectoryPlotWidget(self.usv_manager, self.command_manager, self.motion_capture)
        
        # 连接TrajectoryPlotWidget的usv_state_updated信号到update_usv_state方法
        self.trajectory_plot.usv_state_updated.connect(self.update_usv_state)
        # 将轨迹绘制组件添加到graphicsView
        if hasattr(self.ui, 'graphicsView'):
            layout = self.ui.graphicsView.layout()
            if layout is None:                
                layout = QVBoxLayout(self.ui.graphicsView)
                self.ui.graphicsView.setLayout(layout)
            layout.addWidget(self.trajectory_plot)

        # 初始化模型辨识页绘图区
        self._setup_identification_view()

    # ===================== 模型辨识页 =====================
    def _setup_identification_view(self):
        """初始化模型辨识页右侧的绘图区域（轨迹对比 + u,v,r,psi 对比 + 模型方程）"""
        self.ident_data_path = ''
        self.ident_running = False
        self.ident_worker = None
        self.ident_thread = None
        if not hasattr(self.ui, 'graphicsView_2'):
            return
        if self.ui.graphicsView_2.layout() is None:
            layout = QVBoxLayout(self.ui.graphicsView_2)
            layout.setContentsMargins(0, 0, 0, 0)
        self.ident_plot = pg.GraphicsLayoutWidget()
        self.ident_plot.setBackground('w')
        self.ui.graphicsView_2.layout().addWidget(self.ident_plot)

    # ===================== S0-S11 开环激励采集 =====================
    def _setup_excitation_controls(self):
        """在模型辨识页动态创建“激励采集”按钮并初始化状态。

        该按钮不经过任何控制率/制导率，也不追踪虚拟领航点：点击后按 S0-S11 协议
        直接给选中 USV 下发原始 PWM，同时开始记录数据，结束后自动保存到 data/。
        （form.ui 中无此按钮；模型辨识页为绝对坐标布局，故运行时按 geometry
        放到“输出位置”按钮下方。）
        """
        self.excitation_running = False
        self.excitation_protocol = None
        self.excitation_usv_id = None
        self.excitation_timer = None
        self.excitation_start_time = None
        self._exc_last_seg = None
        self.bt_excitation = None
        try:
            anchor = self.ui.findChild(QPushButton, 'bt_Output_path')
            if anchor is None:
                self._exc_log('未找到模型辨识页锚点按钮 bt_Output_path，激励采集按钮创建失败')
                return
            parent = anchor.parentWidget()
            if parent is None:
                return
            btn = QPushButton('激励采集(S0-S11)', parent)
            btn.setObjectName('bt_excitation')
            btn.clicked.connect(self.toggle_excitation_protocol)
            layout = parent.layout()
            if layout is not None:
                # 页面有布局：直接追加（会随布局自动排布）
                layout.addWidget(btn)
            else:
                # 模型辨识页为绝对坐标布局（无 layout），按 geometry 放到“输出位置”按钮下方
                btn.setGeometry(anchor.x(), anchor.y() + anchor.height() + 8,
                                140, max(anchor.height(), 28))
                btn.show()
                btn.raise_()
            self.bt_excitation = btn
        except Exception as e:
            self._exc_log(f'创建激励采集按钮失败: {e}')

    @staticmethod
    def _pwm_to_offset(pwm, base_positive, base_nagitive):
        """把绝对 PWM 换算成与 NMPC 数据同口径的推力偏置（0 为中心、死区基准对齐）

        NMPC 路径下发的物理 PWM = 死区基准 + 偏置，而 DataProcessor 存表时会再 +1500，
        因此辨识管线解出的 Tp = 存储值 - 1500 = 偏置。激励数据必须做同样换算，
        否则会比同一物理工况的 NMPC 数据多出约 130 的常偏移，导致增益/常数项失真。
        """
        if pwm >= base_positive:
            return pwm - base_positive
        if pwm <= base_nagitive:
            return pwm - base_nagitive
        return 0

    def toggle_excitation_protocol(self):
        """激励采集按钮：未运行则开始 S0-S11；运行中再点即急停"""
        if self.excitation_running:
            self._stop_excitation(aborted=True)
        else:
            self._start_excitation()

    def _start_excitation(self):
        if getattr(self, 'is_recording', False):
            QMessageBox.warning(self, '提示', '当前正在记录数据，请先停止记录再启动激励采集')
            return
        usv_id = self.get_selected_usv()
        if not usv_id:
            QMessageBox.warning(self, '提示', '请先在「系统配置」页的「船舶测试」分组中选中要激励的 USV')
            return
        if not self.usv_manager.get_usv_info(usv_id):
            QMessageBox.warning(self, '提示', f'{usv_id} 尚未注册/连接，无法下发激励命令')
            return
        # 读取当前生效的电机死区基准（与 NMPC 下发路径 TrajectoryPlotWidget 保持一致）
        tp = getattr(self, 'trajectory_plot', None)
        base_pos = getattr(tp, 'base_positive', PWM_DEAD_POS)
        base_neg = getattr(tp, 'base_nagitive', PWM_DEAD_NEG)
        proto = ExcitationProtocol(base_positive=base_pos, base_nagitive=base_neg)
        bad = proto.validate()
        if bad:
            self._exc_log(f'警告：协议有 {len(bad)} 个采样点落入电机死区 {base_neg}~{base_pos}'
                          f'（首例 {bad[0]}），这些段电机不会响应，会污染辨识数据')
        reply = QMessageBox.question(
            self, '确认激励采集',
            f'将对 {usv_id} 执行 S0-S11 开环激励（共约 {proto.total_duration:.0f}s）。\n'
            f'期间直接下发原始 PWM，不使用控制率/制导率、不追踪领航点。\n'
            f'请确保水域空旷、人员远离；运行中再次点击按钮即可急停。\n\n是否开始？',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply != QMessageBox.Yes:
            return
        self.excitation_protocol = proto
        self.excitation_usv_id = usv_id
        self.excitation_start_time = time.monotonic()
        self.excitation_base_pos = base_pos
        self.excitation_base_neg = base_neg
        proto.start(self.excitation_start_time)
        self._exc_last_seg = None
        # 开始记录（数据在 _excitation_tick 中直接读动捕状态入库）
        self.is_recording = True
        self.data_processor.start_experiment('excitation')
        if hasattr(self.ui, 'bt_Record'):
            self.ui.bt_Record.setText('停止记录')
            self.ui.bt_Record.setStyleSheet('background-color: rgb(255, 0, 0);')
        self.excitation_running = True
        self.bt_excitation.setText('停止激励')
        self.excitation_timer = QTimer(self)
        self.excitation_timer.setInterval(100)   # 10Hz，与摇杆更新频率一致
        self.excitation_timer.timeout.connect(self._excitation_tick)
        self.excitation_timer.start()
        self._exc_log(f'{usv_id} S0-S11 激励开始，数据记录中...')

    def _excitation_tick(self):
        if not self.excitation_running:
            return
        now = time.monotonic()
        name, pwm_l, pwm_r, finished = self.excitation_protocol.tick(now)
        self.command_manager.send_control_command(self.excitation_usv_id, pwm_l, pwm_r)
        # 激励不走轨迹跟踪，usv_state_updated 回调不会触发，故在此直接读动捕状态入库
        try:
            st = self.motion_capture.get_usv_state(self.excitation_usv_id)   # [u, v, r, x, y, heading]
            bpos = getattr(self, 'excitation_base_pos', PWM_DEAD_POS)
            bneg = getattr(self, 'excitation_base_neg', PWM_DEAD_NEG)
            self.data_processor.collect_data(self.excitation_usv_id, {
                'x': st[3], 'y': st[4], 'heading': st[5],
                'u': st[0], 'v': st[1],
                'pwml': self._pwm_to_offset(pwm_l, bpos, bneg),
                'pwmr': self._pwm_to_offset(pwm_r, bpos, bneg),
            })
        except Exception:
            pass
        if name != self._exc_last_seg:
            self._exc_last_seg = name
            self._exc_log(f'[激励] {name}  L={pwm_l} R={pwm_r}  t={now - self.excitation_start_time:.1f}s')
        if finished:
            self._stop_excitation(aborted=self.excitation_protocol.aborted)

    def _stop_excitation(self, aborted):
        if self.excitation_timer is not None:
            self.excitation_timer.stop()
            self.excitation_timer = None
        usv_id = self.excitation_usv_id
        if usv_id:
            # 冗余下发停桨，防止丢包导致电机保持上一拍输出
            for _ in range(3):
                self.command_manager.send_control_command(usv_id, PWM_MID, PWM_MID)
        if self.excitation_protocol is not None:
            self.excitation_protocol.abort()
        rows = len(self.data_processor.get_data_buffer().get(usv_id, [])) if usv_id else 0
        if rows == 0:
            self._exc_log(f'{usv_id} 未采集到状态数据（动捕无数据或船未连接），本次不会生成文件')
        self.excitation_running = False
        self.excitation_usv_id = None
        self.excitation_start_time = None
        if getattr(self, 'bt_excitation', None) is not None:
            self.bt_excitation.setText('激励采集(S0-S11)')
        if getattr(self, 'is_recording', False):
            self.is_recording = False
            self.data_processor.stop_experiment()   # 自动保存到 data/data_时间戳.xlsx
            if hasattr(self.ui, 'bt_Record'):
                self.ui.bt_Record.setText('开始记录')
                self.ui.bt_Record.setStyleSheet('')
        self._exc_log('激励已' + ('急停' if aborted else '结束') +
                      (f'，共 {rows} 帧，已保存到 data/' if rows else '，无数据可保存'))

    def _exc_log(self, text):
        if hasattr(self.ui, 'USVsConnectState'):
            self.ui.USVsConnectState.appendPlainText(text)

    def select_identification_data(self):
        """选择数据按钮：从data目录选择Excel数据文件"""
        data_dir = Path(__file__).resolve().parent / 'data'
        path, _ = QFileDialog.getOpenFileName(
            self, '选择数据文件', str(data_dir), 'Excel 文件 (*.xlsx *.xls)')
        if path:
            self.ident_data_path = path
            self.ui.ql_CSV_path.setText(os.path.basename(path))
            self.ui.ql_CSV_path.setToolTip(path)

    def start_identification(self):
        """输出按钮：读取所选Excel并执行模型辨识，完成后在右侧绘图区显示结果"""
        if self.ident_running:
            return
        if not self.ident_data_path:
            QMessageBox.warning(self, '提示', '请先点击"选择数据"选择Excel数据文件')
            return
        try:
            start_row = int(self.ui.ql_Start_row.text().strip())
            n_rows = int(self.ui.ql_Row_count.text().strip())
            h_multi = int(self.ui.ql_H_multi.text().strip())
        except ValueError:
            QMessageBox.warning(self, '提示', '开始行/数据行/H_multi 必须为整数')
            return
        if start_row < 0 or n_rows < 10:
            QMessageBox.warning(self, '提示', '开始行需≥0，数据行需≥10')
            return
        if h_multi < 10:
            h_multi = 10

        cfg = {
            'path': self.ident_data_path,
            'start_row': start_row,
            'n_rows': n_rows,
            'filter_name': self.ui.Filter_Sel.currentText(),
            'optimizer_name': self.ui.ControlLaw_Sel_2.currentText(),
            'h_multi': h_multi,
        }
        self.ident_running = True
        self.ui.bt_Select_path.setEnabled(False)
        self.ui.bt_Output_path.setEnabled(False)
        self.ui.ql_Output_path.setText('辨识中...')
        self.ident_worker = IdentificationWorker(cfg)
        self.ident_worker.finished.connect(self._on_identification_finished)
        self.ident_worker.failed.connect(self._on_identification_failed)
        self.ident_thread = threading.Thread(target=self.ident_worker.run, daemon=True)
        self.ident_thread.start()

    def _set_identification_ui_enabled(self, enabled):
        self.ui.bt_Select_path.setEnabled(enabled)
        self.ui.bt_Output_path.setEnabled(enabled)

    def _on_identification_finished(self, result):
        self.ident_running = False
        self._set_identification_ui_enabled(True)
        try:
            self._draw_identification_result(result)
        except Exception as e:
            traceback.print_exc()   # 完整堆栈输出到控制台便于定位
            QMessageBox.warning(self, '绘图失败', f'辨识完成但绘图出错: {e}')
            return
        rmse = result['rmse']
        self.ui.ql_Output_path.setText(
            f"完成 loss={result['loss']:.3f} 轨迹RMSE={rmse['traj']:.3f}m")
        self.ui.ql_Output_path.setToolTip('\n'.join(result['equations']))
        self._show_identification_status(result)

    def _show_identification_status(self, result):
        """在模型辨识页左侧空白区显示辨识状态信息（对应 demo2 的 print 输出）"""
        if not hasattr(self, '_ident_status_label'):
            lbl = QLabel(self.ui.tb_Model_identification)
            lbl.setGeometry(20, 230, 300, 430)
            lbl.setFont(QFont('Microsoft YaHei', 11))
            lbl.setStyleSheet('color: black;')
            lbl.setAlignment(Qt.AlignLeft | Qt.AlignTop)
            lbl.setWordWrap(True)
            lbl.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self._ident_status_label = lbl
        lines = list(result.get('status_lines', []))
        rmse = result['rmse']
        lines.append(f"RMSE: u={rmse['u']:.4f} v={rmse['v']:.4f} r={rmse['r']:.4f}\n"
                     f"      psi={rmse['psi']:.4f} 轨迹={rmse['traj']:.4f}m")
        self._ident_status_label.setText('\n\n'.join(lines))
        self._ident_status_label.show()

    def _on_identification_failed(self, msg):
        self.ident_running = False
        self._set_identification_ui_enabled(True)
        self.ui.ql_Output_path.setText('辨识失败')
        QMessageBox.critical(self, '辨识失败', msg)

    def _draw_identification_result(self, res):
        """在右侧绘图区绘制轨迹对比、u/v/r/psi对比，方程显示在底部QLabel"""
        gw = self.ident_plot
        gw.clear()
        # 紧凑布局：减小边距与间距，让图占更大空间
        gw.ci.layout.setContentsMargins(2, 2, 8, 2)
        gw.ci.layout.setHorizontalSpacing(8)
        gw.ci.layout.setVerticalSpacing(4)
        tick_font = QFont('Arial', 7)

        def _style(p):
            """缩小刻度与轴标签字体，给曲线留更多空间"""
            p.getAxis('bottom').setStyle(tickFont=tick_font)
            p.getAxis('left').setStyle(tickFont=tick_font)

        pen_meas = pg.mkPen(color='#1f77b4', width=2)
        pen_sim = pg.mkPen(color='#d62728', width=2, style=Qt.PenStyle.DashLine)

        # 轨迹对比
        p_traj = gw.addPlot(row=0, col=0, colspan=2, title='轨迹对比 (实际 vs 辨识模型)')
        p_traj.plot(res['x'], res['y'], pen=pen_meas, name='实际轨迹')
        p_traj.plot(res['x_sim'], res['y_sim'], pen=pen_sim, name='辨识模型')
        p_traj.setLabel('bottom', 'x', units='m', **{'font-size': '8pt'})
        p_traj.setLabel('left', 'y', units='m', **{'font-size': '8pt'})
        p_traj.setAspectLocked(True)
        p_traj.addLegend()
        p_traj.showGrid(x=True, y=True, alpha=0.3)
        _style(p_traj)

        # u v r psi du dv dr 时间对比 (2列布局)
        t = res['t']
        channels = [
            ('u (m/s)', res['u'], res['u_sim'], '实际', '开环仿真'),
            ('v (m/s)', res['v'], res['v_sim'], '实际', '开环仿真'),
            ('r (rad/s)', res['r'], res['r_sim'], '实际', '开环仿真'),
            ('psi (rad)', np.unwrap(res['psi']), np.unwrap(res['psi_sim']), '实际', '开环仿真'),
            ('du (m/s²)', res['du'], res['du_model'], '实际(差分)', '模型预测'),
            ('dv (m/s²)', res['dv'], res['dv_model'], '实际(差分)', '模型预测'),
            ('dr (rad/s²)', res['dr'], res['dr_model'], '实际(差分)', '模型预测'),
        ]
        for i, (label, meas, sim, name_m, name_s) in enumerate(channels):
            p = gw.addPlot(row=1 + i // 2, col=i % 2, title=label)
            p.plot(t, meas, pen=pen_meas, name=name_m)
            p.plot(t, sim, pen=pen_sim, name=name_s)
            p.setLabel('bottom', '时间', units='s', **{'font-size': '8pt'})
            p.addLegend()
            p.showGrid(x=True, y=True, alpha=0.3)
            _style(p)

        gw.ci.layout.setRowStretchFactor(0, 2)   # 降低轨迹图占比，放大各小图

        # 辨识的模型方程显示在dr图右侧的空白格
        eq_label = QLabel('\n'.join(res['equations']))
        eq_label.setFont(QFont('Consolas', 8))
        eq_label.setStyleSheet('color: black; padding: 4px;')
        eq_label.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        eq_label.setWordWrap(True)
        eq_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        eq_proxy = QGraphicsProxyWidget()
        eq_proxy.setWidget(eq_label)
        gw.addItem(eq_proxy, row=4, col=1)

    def connect_signals_slots(self):
        
        """连接信号和槽函数"""
        # 连接UDP模块的消息接收信号
        self.udp_comm.message_received.connect(self.handle_udp_message)
        
        # 连接UDP模块的心跳检测信号
        self.udp_comm.connection_status_changed.connect(self.handle_udp_heartbeat_status)
        
        # 连接无人船管理模块的信号
        self.usv_manager.usv_connected.connect(self.handle_usv_connected)
        self.usv_manager.usv_disconnected.connect(self.handle_usv_disconnected)
        
        # 连接命令管理模块的信号
        self.command_manager.command_sent.connect(self.handle_command_sent)
        
        # 连接数据处理模块的信号
        self.data_processor.data_collected.connect(self.handle_data_collected)
        
        # 连接数据记录按钮信号
        self.ui.bt_Record.clicked.connect(self.handle_record)

        # 连接模型辨识页按钮信号
        if hasattr(self.ui, 'bt_Select_path'):
            self.ui.bt_Select_path.clicked.connect(self.select_identification_data)
        if hasattr(self.ui, 'bt_Output_path'):
            self.ui.bt_Output_path.clicked.connect(self.start_identification)
        
        # 连接UI按钮信号
        self.ui.StartConnect.clicked.connect(self.start_udp_server)
        
        self.ui.StopConnect.clicked.connect(self.stop_udp_server)
        
        # 连接USV控制按钮信号
        self.ui.USV_Up.clicked.connect(lambda: self.send_control_command(1900, 1900))
        self.ui.USV_Down.clicked.connect(lambda: self.send_control_command(1100, 1100))
        self.ui.USV_Left.clicked.connect(lambda: self.send_control_command(1100, 1900))
        self.ui.USV_Right.clicked.connect(lambda: self.send_control_command(1900, 1100))
        self.ui.USV_Stop.clicked.connect(lambda: self.send_control_command(1500, 1500))
        self.ui.USV_Send.clicked.connect(lambda: self.send_control_command(self.ui.sb_PWML.value(), self.ui.sb_PWMR.value()))
        self.ui.bt_Set_Dead.clicked.connect(lambda: self.trajectory_plot.set_dead_zone(self.ui.sb_PWML.value(), self.ui.sb_PWMR.value()))
        # 连接OTA相关按钮信号
        self.ui.bt_USV_OTA.clicked.connect(self.send_ota_command)
        self.ui.bt_USV_Restart.clicked.connect(self.send_restart_command)
        self.ui.bt_USV_camera.clicked.connect(self.send_camera_command)
        self.ui.bt_Emergency_Stop.clicked.connect(self.send_emergency_stop_command)
        
        # 连接遥控按钮信号
        self.ui.bt_USV_Remote.clicked.connect(self.toggle_joystick_control)
        
        # 连接轨迹参数控件信号
        self.ui.cb_Trajectory_type.currentTextChanged.connect(self.update_trajectory_params)
        self.ui.sb_param_a.valueChanged.connect(self.update_trajectory_params)
        self.ui.sb_param_b.valueChanged.connect(self.update_trajectory_params)
        self.ui.sb_param_c.valueChanged.connect(self.update_trajectory_params)
        self.ui.sb_param_d.valueChanged.connect(self.update_trajectory_params)
        self.ui.sb_param_w.valueChanged.connect(self.update_trajectory_params)
        # form.ui中没有sb_param_speed控件时跳过连接，否则启动会报AttributeError
        # if hasattr(self.ui, 'sb_param_speed'):
        #     self.ui.sb_param_speed.valueChanged.connect(self.update_trajectory_params)

        # 连接轨迹周期控制单选按钮信号
        self.ui.rb_Single_turn.toggled.connect(lambda checked: self.trajectory_plot.set_loop_type("single") if checked else None)
        self.ui.rb_Infinite_loop.toggled.connect(lambda checked: self.trajectory_plot.set_loop_type("infinite") if checked else None)
        
                
        # 连接轨迹绘制按钮信号
        self.ui.StartExpButton.clicked.connect(self.start_trajectory_drawing)
        self.ui.ResetCurveButton.clicked.connect(self.reset_trajectory_drawing)
        
        # 连接 trajectory_started 信号，自动处理按钮状态
        self.trajectory_plot.trajectory_started.connect(self.on_trajectory_status_changed)
        
        # 连接覆盖路径规划按钮信号
        self.ui.bt_set_coverage.clicked.connect(self.start_coverage_planning)
        self.ui.bt_clear_coverage.clicked.connect(self.clear_coverage_planning)

        # 连接打点航点（WTracking）按钮信号
        if hasattr(self.ui, 'bt_set_waypoints'):
            self.ui.bt_set_waypoints.clicked.connect(self.start_waypoint_picking)
        if hasattr(self.ui, 'bt_clear_waypoints'):
            self.ui.bt_clear_waypoints.clicked.connect(self.clear_waypoint_picking)

        # 连接避障障碍物按钮信号
        self.ui.bt_add_obstacle.clicked.connect(self.toggle_obstacle_placement)
        self.trajectory_plot.obstacle_position_selected.connect(self.add_obstacle_at)
        self.trajectory_plot.obstacle_placement_mode_changed.connect(
            self.update_obstacle_placement_button)
        self.ui.bt_clear_obstacles.clicked.connect(self.clear_obstacles)
        
        # 连接USV复选框与偏移量控件信号
        for i in range(1, 6):
            # 连接USV复选框信号
            cb_name = f'cb_USV_{i}'
            if hasattr(self.ui, cb_name):
                self.ui.__getattribute__(cb_name).toggled.connect(self.update_trajectory_params)
            
            # 连接USV偏移量输入框信号
            x_bias_name = f'sb_X_bias_{i}'
            y_bias_name = f'sb_Y_bias_{i}'
            interval_name = f'sb_Interval_Data_{i}'
            if hasattr(self.ui, x_bias_name):
                self.ui.__getattribute__(x_bias_name).valueChanged.connect(self.update_trajectory_params)
            if hasattr(self.ui, y_bias_name):
                self.ui.__getattribute__(y_bias_name).valueChanged.connect(self.update_trajectory_params)
            if hasattr(self.ui, interval_name):
                self.ui.__getattribute__(interval_name).valueChanged.connect(self.update_trajectory_params)
        
        # 连接切换控制控件信号
        if hasattr(self.ui, 'cb_exchange_control'):
            self.ui.cb_exchange_control.toggled.connect(self.update_trajectory_params)
        if hasattr(self.ui, 'sb_exchange_time'):
            self.ui.sb_exchange_time.valueChanged.connect(self.update_trajectory_params)
        if hasattr(self.ui, 'sb_trainsition_time'):
            self.ui.sb_trainsition_time.valueChanged.connect(self.update_trajectory_params)
        # 连接第二组偏置输入框信号（USV1~3切换）
        exchange_bias_names = ['sb_X_bias_11', 'sb_Y_bias_11', 'sb_X_bias_22', 'sb_Y_bias_22', 'sb_X_bias_33', 'sb_Y_bias_33']
        for bias_name in exchange_bias_names:
            if hasattr(self.ui, bias_name):
                self.ui.__getattribute__(bias_name).valueChanged.connect(self.update_trajectory_params)
        
        # 连接动力学仿真参数控件信号 - USV1~USV5的PID和MPC参数
        self.param_types = ['AngleKp', 'AngleKi', 'AngleKd', 'SpeedKp', 'SpeedKi', 'SpeedKd', 
                      'MpcQ', 'MpcR', 'MpcT', 'Mpca', 'BpkP', 'BpkV', 'BpkH', 'BpkR', 'BpkX', 'BpkY']
        
        # 加载USV控制参数的公共方法
        def load_control_params(usv_index, param_type=None):
            if hasattr(self, 'usv_manager'):
                usv_id = f'USV{usv_index}'
                
                # 构建参数字典
                param_dict = {}
                
                # 如果指定了参数类型，只加载该参数
                if param_type:
                    param_name = f'ql_{param_type}_Data_{usv_index}'
                    if hasattr(self.ui, param_name):
                        try:
                            # 获取参数值
                            param_value = float(getattr(self.ui, param_name).text() or '1.0')
                            
                            # 构建参数字典
                            if param_type.startswith('Angle'):
                                # PID角度参数
                                param_key = f'angle_{param_type[5:].lower()}'  # AngleKp -> angle_kp
                                param_dict[param_key] = param_value
                            elif param_type.startswith('Speed'):
                                # PID速度参数
                                param_key = f'speed_{param_type[5:].lower()}'  # SpeedKp -> speed_kp
                                param_dict[param_key] = param_value
                            elif param_type.startswith('Mpc'):
                                # MPC参数
                                param_key = f'mpc_{param_type[3:].lower()}'  # MpcQ -> mpc_q
                                param_dict[param_key] = param_value
                            elif param_type.startswith('Bp'):
                                # 反步法参数
                                param_key = f'bp_{param_type[2:].lower()}'  # BpkP -> bp_kp
                                param_dict[param_key] = param_value
                            
                            # 更新参数
                            self.usv_manager.update_control_params(usv_id, **param_dict)
                        except Exception as e:
                            print(f"Error updating parameter {param_type} for USV{usv_index}: {e}")
                else:
                    # 加载所有参数
                    for param_type in self.param_types:
                        param_name = f'ql_{param_type}_Data_{usv_index}'
                        if hasattr(self.ui, param_name):
                            try:
                                # 获取参数值
                                param_value = float(getattr(self.ui, param_name).text() or '1.0')
                                
                                # 构建参数字典
                                if param_type.startswith('Angle'):
                                    # PID角度参数
                                    param_key = f'angle_{param_type[5:].lower()}'
                                    param_dict[param_key] = param_value
                                elif param_type.startswith('Speed'):
                                    # PID速度参数
                                    param_key = f'speed_{param_type[5:].lower()}'
                                    param_dict[param_key] = param_value
                                elif param_type.startswith('Mpc'):
                                    # MPC参数
                                    param_key = f'mpc_{param_type[3:].lower()}'
                                    param_dict[param_key] = param_value
                                elif param_type.startswith('Bp'):
                                    # 反步法参数
                                    param_key = f'bp_{param_type[2:].lower()}'
                                    param_dict[param_key] = param_value
                            except Exception as e:
                                print(f"Error reloading parameter {param_type} for {usv_id}: {e}")
                    
                    # 更新所有参数
                    if param_dict:
                        self.usv_manager.update_control_params(usv_id, **param_dict)
        
        # 保存为实例方法，以便其他地方调用
        self.load_control_params = load_control_params
        
        # 创建一个参数更新的辅助函数
        def on_param_changed(usv_index, param_type):
            load_control_params(usv_index, param_type)
        
        # 为每个参数连接信号并初始化参数
        for usv_index in range(1, 6):
            for param_type in self.param_types:
                param_name = f'ql_{param_type}_Data_{usv_index}'
                if hasattr(self.ui, param_name):
                    # 使用lambda函数捕获usv_index和param_type
                    self.ui.__getattribute__(param_name).textChanged.connect(
                        lambda text, ui=usv_index, pt=param_type: on_param_changed(ui, pt)
                    )
                    # 初始化时加载参数
                    on_param_changed(usv_index, param_type)
        
        # 连接制导率和控制率选择信号
        self.ui.GuidenceLaw_Sel.currentTextChanged.connect(self.handle_guidance_law_changed)
        self.ui.ControlLaw_Sel.currentTextChanged.connect(self.handle_control_law_changed)
        self.usv_manager.set_law(self.ui.GuidenceLaw_Sel.currentText(), self.ui.ControlLaw_Sel.currentText())
        # 初始化实验类型
        self.ui.TestProject_Sel.currentTextChanged.connect(self.trajectory_plot.set_tracking_mode)
        # 初始化USV ID映射
        self.update_usv_mapping()
    
    def handle_guidance_law_changed(self, guidance_law):
        """
        处理制导率选择变化
        
        参数:
            guidance_law: 选择的制导率名称
        """
        print(f"选择的制导率: {guidance_law}")
        # 同步到管理器，保证之后才注册的船也使用当前选择
        if hasattr(self, 'usv_manager'):
            self.usv_manager.set_law(guidance_law, self.ui.ControlLaw_Sel.currentText())
        # 确保trajectory_plot已初始化并设置了制导率
        if hasattr(self, 'trajectory_plot') and hasattr(self.trajectory_plot, 'set_guidance_law'):
            self.trajectory_plot.set_guidance_law(guidance_law)
    
    def handle_control_law_changed(self, control_law):
        """
        处理控制率选择变化
        
        参数:
            control_law: 选择的控制率名称
        """
        print(f"选择的控制率: {control_law}")
        # 同步到管理器，保证之后才注册的船也使用当前选择
        if hasattr(self, 'usv_manager'):
            self.usv_manager.set_law(self.ui.GuidenceLaw_Sel.currentText(), control_law)
        # 确保trajectory_plot已初始化并设置了控制率
        if hasattr(self, 'trajectory_plot') and hasattr(self.trajectory_plot, 'set_control_law'):
            self.trajectory_plot.set_control_law(control_law)
    
    def start_motion_capture(self):
        """启动动捕数据采集"""
        try:
            success = self.motion_capture.start()
            if success:
                self.ui.USVsConnectState.appendPlainText("动捕数据采集已启动")
            else:
                self.ui.USVsConnectState.appendPlainText("动捕数据采集启动失败")
        except Exception as e:
            self.ui.USVsConnectState.appendPlainText(f"启动动捕系统时出错: {e}")
    
    def stop_motion_capture(self):
        """停止动捕数据采集"""
        try:
            self.motion_capture.stop()
            self.ui.USVsConnectState.appendPlainText("动捕数据采集已停止")
        except Exception as e:
            self.ui.USVsConnectState.appendPlainText(f"停止动捕系统时出错: {e}")
    
    def toggle_joystick_control(self):
        """切换摇杆控制开关"""
        if hasattr(self, 'joystick_active') and self.joystick_active:
            # 关闭摇杆控制
            self.joystick_manager.stop()
            self.joystick_active = False
            self.ui.bt_USV_Remote.setText("启动遥控")
            self.ui.USVsConnectState.appendPlainText("摇杆控制已关闭")
        else:
            # 启动摇杆控制
            if self.joystick_manager.start():
                self.joystick_active = True
                self.ui.bt_USV_Remote.setText("关闭遥控")
                self.ui.USVsConnectState.appendPlainText("摇杆控制已启动")
            else:
                # 检查是否是油门未归位导致的启动失败
                # 弹窗提示用户
                QMessageBox.warning(
                    self,
                    "遥控启动失败",
                    "油门未归位！请将油门杆拉至最低位置后再启动遥控。\n\n这是为了防止无人船意外飞车。",
                    QMessageBox.Ok
                )
                self.ui.USVsConnectState.appendPlainText("摇杆控制启动失败：油门未归位")
    
    
    def handle_rc_data(self, rc_data):
        """处理摇杆数据更新
        
        Args:
            rc_data: 摇杆数据字典
        """
        # 更新UI显示的RC值（如果有对应的显示控件）
        try:
            # 获取归一化后的值 (0~1)
            self.joystick_manager.get_rc_values()
            
            # 更新UI标签显示
            # 可以在此添加QLabel更新代码
            
        except Exception as e:
            print(f"更新RC数据显示失败: {e}")
    
    def handle_rc_connection(self, connected):
        """处理摇杆连接状态变化
        
        Args:
            connected: 是否已连接
        """
        if connected:
            self.ui.USVsConnectState.appendPlainText("遥控器已连接")
        else:
            self.ui.USVsConnectState.appendPlainText("遥控器已断开连接")
            if hasattr(self, 'joystick_active') and self.joystick_active:
                self.joystick_active = False
                self.ui.bt_USV_Remote.setText("启动遥控")
    
    def handle_joystick_command(self, pwm1, pwm2):
        """处理摇杆控制命令
        
        Args:
            pwm1: 第一个PWM值 (油门)
            pwm2: 第二个PWM值 (方向)
        """
        # 发送控制命令
        self.send_control_command(pwm1, pwm2)
    
    def start_trajectory_drawing(self):
        """开始/停止绘制轨迹（连接StartExpButton）"""
        # 设置USV的制导率和控制率
        self.usv_manager.set_law(self.ui.GuidenceLaw_Sel.currentText(), self.ui.ControlLaw_Sel.currentText())
        tracking_mode = self.ui.TestProject_Sel.currentText()
        self.trajectory_plot.set_tracking_mode(tracking_mode)
        self.trajectory_plot.enable_dynamics(self.ui.rb_Simulated_ship.isChecked())
        
        # RTracking模式：设置返航点，且只用选中的测试USV
        if tracking_mode == 'RTracking':
            return_x = self.ui.sb_X_Return.value()
            return_y = self.ui.sb_Y_Return.value()
            self.trajectory_plot.set_return_point(return_x, return_y)
            # # 使用选中的测试USV进行返航
            # test_usv = self.get_selected_usv()
            # if test_usv:
            #     self.trajectory_plot.set_usvs({test_usv: (0.0, 0.0, 0.0)})
        
        if hasattr(self, 'trajectory_plot'):
            if self.trajectory_plot.is_drawing:
                # 如果正在绘制，则停止绘制
                self.trajectory_plot.stop_drawing()
                if not self.ui.rb_Simulated_ship.isChecked():
                    self.send_emergency_stop_command()
                self.ui.StartExpButton.setText("开始实验")
                self.ui.StartExpButton.setStyleSheet("background-color: rgb(255, 255, 255);")
                self.ui.USVsConnectState.appendPlainText("停止实验")
            else:              
                self.trajectory_plot.start_drawing()
                self.ui.StartExpButton.setText("停止实验")
                self.ui.StartExpButton.setStyleSheet("background-color: rgb(255, 0, 0);")
                self.ui.USVsConnectState.appendPlainText("开始实验")
    
    def reset_trajectory_drawing(self):
        """重置轨迹（连接ResetCurveButton）"""
        if hasattr(self, 'trajectory_plot'):
            self.trajectory_plot.clear_trajectory()
            self.ui.USVsConnectState.appendPlainText("轨迹已清除")
    
    def on_trajectory_status_changed(self, status):
        """轨迹状态变化回调"""
        if status == "stopped":
            self.ui.StartExpButton.setText("开始实验")
            self.ui.StartExpButton.setStyleSheet("background-color: rgb(255, 255, 255);")
            self.ui.USVsConnectState.appendPlainText("实验已停止")
    
    def start_coverage_planning(self):
        """开始覆盖路径规划（连接bt_set_coverage）"""
        if hasattr(self, 'trajectory_plot'):
            self.trajectory_plot.start_coverage_planning(self.ui.sb_Coverage_Interval.value(), self.ui.sb_Coverage_Angle.value(), self.ui.sb_Coverage_Speed.value())
            self.ui.USVsConnectState.appendPlainText("进入覆盖路径规划模式，请在地图上点击鼠标左键设置多边形顶点，点击鼠标右键结束并生成路径")
        
    def clear_coverage_planning(self):
        """清除覆盖路径规划（连接bt_clear_coverage）"""
        if hasattr(self, 'trajectory_plot'):
            self.trajectory_plot.clear_coverage_display()
            self.ui.USVsConnectState.appendPlainText("覆盖路径规划已清除")

    def start_waypoint_picking(self):
        """进入打点航点采集模式（连接bt_set_waypoints）"""
        if hasattr(self, 'trajectory_plot'):
            self.trajectory_plot.start_waypoint_picking()
            self.ui.USVsConnectState.appendPlainText(
                "进入打点航点模式：在地图上左键添加航点（至少2个），右键确认生成航线，Esc取消；实验类型选择WTracking后开始实验"
            )

    def clear_waypoint_picking(self):
        """清除打点航线（连接bt_clear_waypoints）"""
        if hasattr(self, 'trajectory_plot'):
            self.trajectory_plot.clear_manual_waypoints()
            self.ui.USVsConnectState.appendPlainText("打点航线已清除")

    def toggle_obstacle_placement(self):
        """打开或结束画布放置障碍物模式。"""
        if not hasattr(self, 'trajectory_plot'):
            return
        enabled = not self.trajectory_plot.obstacle_placement_mode
        self.trajectory_plot.set_obstacle_placement_mode(enabled)
        if enabled:
            self.ui.USVsConnectState.appendPlainText(
                "障碍物放置模式：点击轨迹画布添加，可连续放置；按 Esc 或再次点击按钮退出")
        else:
            self.ui.USVsConnectState.appendPlainText("已结束障碍物放置")

    def update_obstacle_placement_button(self, enabled):
        """根据画布放置状态更新按钮提示。"""
        if enabled:
            self.ui.bt_add_obstacle.setText("完成放置")
            self.ui.bt_add_obstacle.setStyleSheet("background-color: #ffe08a;")
        else:
            self.ui.bt_add_obstacle.setText("点击画布放置")
            self.ui.bt_add_obstacle.setStyleSheet("")

    def add_obstacle_at(self, x, y):
        """按用户在轨迹画布上点击的位置添加障碍物。"""
        if not hasattr(self, 'trajectory_plot'):
            return
        shape_text = self.ui.cb_obs_shape.currentText()
        shape = 'circle' if shape_text == '圆形' else 'rectangle'
        radius = self.ui.sb_obs_radius.value()
        height = self.ui.sb_obs_height.value()
        if shape == 'circle':
            obs_id = self.trajectory_plot.add_obstacle('circle', x, y, radius=radius)
            self.ui.USVsConnectState.appendPlainText(
                f"添加圆形障碍物: 中心({x:.2f}, {y:.2f}) 半径={radius:.2f}m  ID={obs_id}")
        else:
            obs_id = self.trajectory_plot.add_obstacle(
                'rectangle', x, y, width=radius, height=height)
            self.ui.USVsConnectState.appendPlainText(
                f"添加矩形障碍物: 中心({x:.2f}, {y:.2f}) 宽={radius:.2f} 高={height:.2f}m  ID={obs_id}")
        count = self.trajectory_plot.get_obstacle_count()
        self.ui.ql_obs_count.setText(f"当前障碍物: {count} 个")

    def clear_obstacles(self):
        """清除所有障碍物（连接bt_clear_obstacles）"""
        if hasattr(self, 'trajectory_plot'):
            self.trajectory_plot.set_obstacle_placement_mode(False)
            self.trajectory_plot.clear_obstacles()
            self.ui.ql_obs_count.setText("当前障碍物: 0 个")
            self.ui.USVsConnectState.appendPlainText("已清除所有障碍物")

    def _parse_usv_id(self, content):
        """解析USV ID消息"""
        if content.startswith("id:"):
            return content.split(":")[1]  # 格式：id:6789
        elif content in self.usv_mapping:
            return content  # 格式：6789
        return None
    
    def update_usv_mapping(self):
        """从界面动态读取USV ID配置"""
        usv_configs = {}
        try:
            # 简化的配置读取
            for i in range(1, 6):  # USV1-USV5  
                for j in range(1, 3):  # UUID1, UUID2
                    widget_name = f"USV{i}_UUID{j}"
                    if hasattr(self.ui, widget_name):
                        usv_id = getattr(self.ui, widget_name).text().strip()
                        if usv_id:
                            usv_configs[usv_id] = f"USV{i}"
            
            self.usv_mapping = usv_configs
            # 打印当前映射表用于调试
            if usv_configs:
                print(f"USV映射表已更新: {usv_configs}")
            
        except Exception as e:
            print(f"读取USV映射配置失败，使用默认配置: {e}")
            # 使用默认配置
            self.usv_mapping = {
                "7000": "USV1", "4373": "USV1",
                "9836": "USV2", "6463": "USV2", 
                "9839": "USV3", "9704": "USV3",
                "7589": "USV4", "7989": "USV4",
                "8000": "USV5", "8001": "USV5"
            }
                
    def handle_udp_message(self, message, ip, port):
        """处理接收到的UDP消息"""        
        if 'type' not in message or 'content' not in message:
            return False
        # 更新UI显示接收到的消息
        self.ui.USVsConnectState.appendPlainText(f"接收到来自 {ip}:{port} 的消息: {message}")
        type = message['type']
        content = message['content']
        if type == 0:
            self._handle_register_message(content, ip, port)
            return True
        elif type == 1:
            self._handle_status_message(content, ip, port)
            return True
        elif type == 2:
            # self._handle_control_message(content, ip)
            return True
        else:
            return False
    
    def _handle_register_message(self, content, ip, port):
        """处理注册消息"""
        # 动态更新USV映射配置（确保使用最新的UI配置）
        if not self._find_usv_id_by_ip(ip):
            self.update_usv_mapping()            
            # 处理无效端口，统一使用7020
            if port is None or port == 0 or port == '':
                port = 7020
            
            # 解析并注册USV ID
            id = content['id']
            esp_id = self._parse_usv_id(id)
            if esp_id:
                usv_id = self.usv_mapping.get(esp_id, esp_id)
                result = self.usv_manager.register_usv(usv_id, ip, port)
                return result

    def _handle_status_message(self, content, ip, port):
        """解析下位机回传的状态数据，并换算为实际电池电压。"""
        usv_id = self._find_usv_id_by_ip(ip)
        if not usv_id:
            self.ui.USVsConnectState.appendPlainText(f"收到 {ip} 的状态数据，但未找到对应USV")
            self._handle_register_message(content, ip, port)
            return True

        adc0 = int(content.get('adc0', 0))        
        adc_voltage = adc0 / self.adc_resolution * self.adc_reference_voltage
        battery_voltage = adc_voltage * self.battery_divider_ratio
        temperature = float(content.get('temperature', 0.0))
        led_left = int(content.get('led_left', 0))
        led_right = int(content.get('led_right', 0))
        led_text = f"({led_left}/{led_right})"

        self.usv_battery_data[usv_id] = {
            'adc0': adc0,
            'adc_voltage': adc_voltage,
            'battery': battery_voltage,
            'battery_voltage': battery_voltage,
            'temperature': temperature,
            'led_left': led_left,
            'led_right': led_right,
            'led': led_text
        }

        self._update_battery_label(usv_id, battery_voltage)
        self._update_led_label(usv_id, led_text)
        self.ui.USVsConnectState.appendPlainText(
            f"{usv_id} ADC0={adc0:.0f}, ADC电压={adc_voltage:.2f}V, 电池电压={battery_voltage:.2f}V, 灯={led_text}"
        )
        return True


    def _find_usv_id_by_ip(self, ip):
        """根据下位机IP查找已经注册的USV编号。"""
        for usv_id, usv_info in self.usv_manager.get_all_usvs().items():
            if usv_info.get('ip') == ip:
                return usv_id
        return None

    def _update_battery_label(self, usv_id, battery_voltage):
        """直接刷新电池电压显示，避免等待下一帧动捕状态。"""
        usv_num = usv_id.replace("USV", "")
        label_name = f"ql_Battery_Data_{usv_num}"
        if hasattr(self.ui, label_name):
            getattr(self.ui, label_name).setText(f"{battery_voltage:.2f}")

    def _update_led_label(self, usv_id, led_text):
        """直接刷新左右灯状态显示。"""
        usv_num = usv_id.replace("USV", "")
        label_name = f"ql_LED_Data_{usv_num}"
        if hasattr(self.ui, label_name):
            getattr(self.ui, label_name).setText(led_text)
        
        
    def handle_usv_connected(self, usv_id, ip, port):
        """处理无人船连接事件"""
        # 更新UI显示无人船连接信息
        self.ui.USVsConnectState.appendPlainText(f"无人船 {usv_id} 已连接，IP: {ip}, 端口: {port}")
        
        # 更新无人船连接状态显示
        self.update_usv_connection_status(usv_id, True)
        
    def handle_usv_disconnected(self, usv_id):
        """处理无人船断开连接事件"""
        # 更新UI显示无人船断开连接信息
        self.ui.USVsConnectState.appendPlainText(f"无人船 {usv_id} 已断开连接")
        
        # 更新无人船连接状态显示
        self.update_usv_connection_status(usv_id, False)
        
    def handle_command_sent(self, usv_id, command, success):
        """处理命令发送事件"""
        if success:
            self.ui.USVsConnectState.appendPlainText(f"向无人船 {usv_id} 发送命令成功: {command}")
        else:
            self.ui.USVsConnectState.appendPlainText(f"向无人船 {usv_id} 发送命令失败: {command}")
    
    def handle_data_collected(self, usv_id, data):
        """处理数据采集事件"""
        # 更新UI显示采集到的数据
        self.ui.USVsConnectState.appendPlainText(f"采集到无人船 {usv_id} 的数据: {data}")
    
        
    def handle_motion_capture_status(self, usv_id, connected):
        """处理动捕连接状态变化，设置USV checkbox勾选状态"""
        # 获取对应的checkbox控件名称
        usv_num = usv_id.replace("USV", "")
        checkbox_name = f"cb_body{usv_num}"
        
        if hasattr(self.ui, checkbox_name):
            checkbox = getattr(self.ui, checkbox_name)
            checkbox.setChecked(connected)
        
        
    def handle_udp_heartbeat_status(self, usv_id, connected):
        """处理UDP心跳检测连接状态变化"""
        """处理动捕连接状态变化，设置USV checkbox勾选状态"""
        # usv_id现在传过来的是IP地址，需要根据IP地址找到对应的USV编号
        all_usvs = self.usv_manager.get_all_usvs()
        target_usv_id = None
        
        # 根据IP地址查找对应的USV
        for usv_key, usv_info in all_usvs.items():
            if usv_info.get('ip') == usv_id:
                target_usv_id = usv_key
                break
        
        if target_usv_id:
            # 获取对应的checkbox控件名称
            usv_num = target_usv_id.replace("USV", "")
            checkbox_name = f"cb_Status_USV{usv_num}"
            
            if hasattr(self.ui, checkbox_name):
                checkbox = getattr(self.ui, checkbox_name)
                checkbox.setChecked(connected)
                
            print(f"USV {target_usv_id} ({usv_id}) 连接状态: {'连接' if connected else '断开'}")
        else:
            print(f"未找到IP地址 {usv_id} 对应的USV记录")
        
    def generate_equation(self, traj_type, x_coeff, y_coeff, x_offset, y_offset, omega):
        """根据轨迹类型和参数生成方程文本"""
        a = round(x_coeff, 2)
        b = round(y_coeff, 2)
        c = round(x_offset, 2)
        d = round(y_offset, 2)
        w = round(omega, 3)# 保留3位小数
        
        if traj_type == '圆形':
            return f"x = {a}·sin({w}t) + {c}, y = {b}·cos({w}t) + {d}"
        elif traj_type == '椭圆':
            return f"x = {a}·sin({w}t) + {c}, y = {b}·cos({w}t)/2 + {d}"
        elif traj_type == '8字形':
            return f"x = {a}·sin({w}t) + {c}, y = {b}·sin({w}t)·cos({w}t) + {d}"
        elif traj_type == '螺旋线':
            return f"x = {a}·0.1t·cos({w}t) + {c}, y = {b}·0.1t·sin({w}t) + {d}"
        elif traj_type == 'SIN':
            return f"y = {a}/10·sin(4·{w}·t) + {c}, x = {b}·{w}·t + {d}-1.5"
        elif traj_type == 'COS':
            return f"y = {a}/10·cos(4·{w}·t) + {c}, x = {b}·{w}·t + {d}-1.5"
        elif traj_type == '自定义':
            return f"x = {a}·sin({w}t) + {c}, y = {b}·cos({w}t) + {d}"
        else:
            return f"x = sin({w}t) + {c}, y = cos({w}t) + {d}"
    
    def update_trajectory_params(self):
        """更新轨迹参数并传递给TrajectoryPlotWidget"""
        if hasattr(self, 'trajectory_plot'):
            traj_type = self.ui.cb_Trajectory_type.currentText() if hasattr(self.ui, 'cb_Trajectory_type') else '圆形'
            x_coeff = self.ui.sb_param_a.value() if hasattr(self.ui, 'sb_param_a') else 1.0
            y_coeff = self.ui.sb_param_b.value() if hasattr(self.ui, 'sb_param_b') else 1.0
            x_offset = self.ui.sb_param_c.value() if hasattr(self.ui, 'sb_param_c') else 0.0
            y_offset = self.ui.sb_param_d.value() if hasattr(self.ui, 'sb_param_d') else 0.0
            omega = self.ui.sb_param_w.value() if hasattr(self.ui, 'sb_param_w') else 1.0
            desired_speed = self.ui.sb_param_speed.value() if hasattr(self.ui, 'sb_param_speed') else 0.5
            
            # 获取当前选中的轨迹周期类型
            loop_type = "infinite"  # 默认无限圈
            if hasattr(self.ui, 'rb_Single_turn') and self.ui.rb_Single_turn.isChecked():
                loop_type = "single"
            elif hasattr(self.ui, 'rb_Infinite_loop') and self.ui.rb_Infinite_loop.isChecked():
                loop_type = "infinite"
            
            experiment_type = "simulated_ship" 
            if hasattr(self.ui, 'rb_Simulated_ship') and self.ui.rb_Simulated_ship.isChecked():
                experiment_type = "simulated_ship"
            elif hasattr(self.ui, 'rb_Real_ship') and self.ui.rb_Real_ship.isChecked():
                experiment_type = "real_ship"
            # 更新轨迹参数
            self.trajectory_plot.update_params(traj_type, x_coeff, y_coeff, x_offset, y_offset, omega, loop_type, experiment_type, desired_speed)
            
            # 检测USV复选框状态和偏移量
            usvs_config = {}
            for i in range(1, 6):
                # 检查复选框是否存在且被勾选
                cb_name = f'cb_USV_{i}'
                if hasattr(self.ui, cb_name) and self.ui.__getattribute__(cb_name).isChecked():
                    # 获取偏移量
                    x_bias_name = f'sb_X_bias_{i}'
                    y_bias_name = f'sb_Y_bias_{i}'
                    interval_name = f'sb_Interval_Data_{i}'
                    x_bias = self.ui.__getattribute__(x_bias_name).value() if hasattr(self.ui, x_bias_name) else 0.0
                    y_bias = self.ui.__getattribute__(y_bias_name).value() if hasattr(self.ui, y_bias_name) else 0.0
                    interval = self.ui.__getattribute__(interval_name).value() if hasattr(self.ui, interval_name) else 0.0
                    
                    # 添加到USV配置
                    usv_id = f'USV{i}'
                    usvs_config[usv_id] = (x_bias, y_bias, interval)
            
            # 设置USV配置
            if usvs_config:
                self.trajectory_plot.set_usvs(usvs_config)
            
            # 传递切换控制参数
            exchange_enabled = hasattr(self.ui, 'cb_exchange_control') and self.ui.cb_exchange_control.isChecked()
            exchange_time = self.ui.sb_exchange_time.value() if hasattr(self.ui, 'sb_exchange_time') else 0
            exchange_duration = self.ui.sb_trainsition_time.value() if hasattr(self.ui, 'sb_trainsition_time') else 0
            exchange_biases = {}
            exchange_biases_from = {}
            if exchange_enabled:
                exchange_mapping = {
                    'USV1': ('sb_X_bias_11', 'sb_Y_bias_11'),
                    'USV2': ('sb_X_bias_22', 'sb_Y_bias_22'),
                    'USV3': ('sb_X_bias_33', 'sb_Y_bias_33'),
                }
                original_mapping = {
                    'USV1': ('sb_X_bias_1', 'sb_Y_bias_1'),
                    'USV2': ('sb_X_bias_2', 'sb_Y_bias_2'),
                    'USV3': ('sb_X_bias_3', 'sb_Y_bias_3'),
                }
                for usv_id, (x_name, y_name) in exchange_mapping.items():
                    if hasattr(self.ui, x_name) and hasattr(self.ui, y_name):
                        x_bias = self.ui.__getattribute__(x_name).value()
                        y_bias = self.ui.__getattribute__(y_name).value()
                        exchange_biases[usv_id] = (x_bias, y_bias)
                for usv_id, (x_name, y_name) in original_mapping.items():
                    if hasattr(self.ui, x_name) and hasattr(self.ui, y_name):
                        x_bias = self.ui.__getattribute__(x_name).value()
                        y_bias = self.ui.__getattribute__(y_name).value()
                        exchange_biases_from[usv_id] = (x_bias, y_bias)
            self.trajectory_plot.set_exchange_control_params(exchange_enabled, exchange_time, exchange_biases, exchange_duration, exchange_biases_from)
            
            # 更新参数方程显示
            if hasattr(self.ui, 'ql_Equation'):
                equation_text = self.generate_equation(traj_type, x_coeff, y_coeff, x_offset, y_offset, omega)
                self.ui.ql_Equation.setText(equation_text)
    
    def start_udp_server(self):
        """启动UDP服务器"""
        if self.udp_comm.start_server():
            self.ui.USVsConnectState.appendPlainText("UDP服务器已启动，开始监听端口 7020")
            self.ui.StartConnect.setEnabled(False)
            self.ui.StopConnect.setEnabled(True)
        else:
            QMessageBox.critical(self, "错误", "启动UDP服务器失败")
            
    def stop_udp_server(self):
        """停止UDP服务器"""
        if self.udp_comm.stop_server():
            self.ui.USVsConnectState.appendPlainText("UDP服务器已停止")
            self.ui.StartConnect.setEnabled(True)
            self.ui.StopConnect.setEnabled(False)
        else:
            QMessageBox.critical(self, "错误", "停止UDP服务器失败")
            
    def send_control_command(self, pwm1, pwm2):
        """发送控制命令"""
        # 获取当前选中的无人船
        usv_id = self.get_selected_usv()
        if usv_id:
            self.command_manager.send_control_command(usv_id, pwm1, pwm2)
        else:
            QMessageBox.warning(self, "警告", "请先选择要控制的无人船")
            
    def send_ota_command(self):
        """发送OTA命令"""
        # 获取当前选中的无人船
        usv_id = self.get_selected_ota_usv()
        if usv_id:
            self.command_manager.send_ota_command(usv_id)
        else:
            QMessageBox.warning(self, "警告", "请先选择要OTA的无人船")
            
    def send_restart_command(self):
        """发送重启命令"""
        # 获取当前选中的无人船
        usv_id = self.get_selected_ota_usv()
        if usv_id:
            self.command_manager.send_restart_command(usv_id)
        else:
            QMessageBox.warning(self, "警告", "请先选择要重启的无人船")
            
    def send_camera_command(self):
        """发送相机命令"""
        # 获取当前选中的无人船
        usv_id = self.get_selected_ota_usv()
        if usv_id:
            self.command_manager.send_camera_command(usv_id)
        else:
            QMessageBox.warning(self, "警告", "请先选择要相机的无人船")
            
    def send_emergency_stop_command(self):
        """发送紧急停止命令"""
        self.command_manager.send_command_to_all("1500,1500")
        
            
    def get_selected_usv(self):
        """获取当前选中的测试无人船"""
        for i in range(1, 6):
            if getattr(self.ui, f'rb_Test_USV{i}').isChecked():
                return f"USV{i}"
        return None
        
    def get_selected_ota_usv(self):
        """获取当前选中的 OTA 无人船"""
        for i in range(1, 6):
            if getattr(self.ui, f'rb_OTA_USV{i}').isChecked():
                return f"USV{i}"
        return None
        
    def update_usv_connection_status(self, usv_id, connected):
        """更新无人船连接状态显示（使用颜色标记）"""
        # 提取USV编号（如"USV1" -> "1"）
        usv_num = usv_id.replace("USV", "")
        
        # 设置对应的 LineEdit 颜色样式
        for j in range(1, 3):
            widget_name = f"USV{usv_num}_UUID{j}"
            widget = getattr(self.ui, widget_name, None)
            if connected:
                # 连接状态：绿色背景
                widget.setStyleSheet("background-color: #90EE90; border: 2px solid #228B22;")
            else:
                # 断开状态：恢复默认样式
                widget.setStyleSheet("")
                
    def update_usv_state(self, usv_id, state):
        """更新无人船状态显示"""
        # 根据usv_id更新对应的状态显示
        usv_num = usv_id.replace("USV", "")
        state = state.copy()
        state.update(self.usv_battery_data.get(usv_id, {}))
        # 将原点坐标添加到state字典中
        state_with_origin = state.copy()  # 创建副本避免修改原始state
        state_with_origin['origin_lat'] = self.origin_lat
        state_with_origin['origin_lon'] = self.origin_lon
        state_with_origin['id'] = usv_id
        # 构造控件名称并更新状态显示
        try:
            # 更新基本状态信息
            id_label = getattr(self.ui, f"ql_TP_Data_{usv_num}")
            x_label = getattr(self.ui, f"ql_X_Data_{usv_num}")
            y_label = getattr(self.ui, f"ql_Y_Data_{usv_num}")
            u_label = getattr(self.ui, f"ql_u_Data_{usv_num}")
            v_label = getattr(self.ui, f"ql_v_Data_{usv_num}")
            heading_label = getattr(self.ui, f"ql_Heading_Data_{usv_num}")
            speed_label = getattr(self.ui, f"ql_Speed_Data_{usv_num}")
            battery_label = getattr(self.ui, f"ql_Battery_Data_{usv_num}")
            led_label = getattr(self.ui, f"ql_LED_Data_{usv_num}")
            pwml_label = getattr(self.ui, f"ql_PWML_Data_{usv_num}")
            pwmr_label = getattr(self.ui, f"ql_PWMR_Data_{usv_num}")
            desired_heading_label = getattr(self.ui, f"ql_Set_Angle_{usv_num}")
            desired_speed_label = getattr(self.ui, f"ql_Set_Speed_{usv_num}")

            # 更新显示
            id_label.setText(f"{state.get('temperature', 0.0):.2f}")  # 使用temperature作为温度显示
            x_label.setText(f"{state.get('x', 0.0):.2f}")
            y_label.setText(f"{state.get('y', 0.0):.2f}")
            u_label.setText(f"{state.get('u', 0.0):.2f}")
            v_label.setText(f"{state.get('v', 0.0):.2f}")
            heading_label.setText(f"{state.get('heading', 0.0):.2f}")
            speed_label.setText(f"{state.get('V', 0.0):.2f}")  # 使用V作为总速度显示
            battery_label.setText(f"{state.get('battery', 0.0):.2f}")  # 使用battery作为电池电量显示
            led_label.setText(state.get('led', '(0/0)'))
            pwml_label.setText(f"{state.get('pwml', 0.0):.2f}")  # 使用pwml作为左电机PWM显示
            pwmr_label.setText(f"{state.get('pwmr', 0.0):.2f}")  # 使用pwmr作为右电机PWM显示
            desired_heading_label.setText(f"{state.get('desired_heading', 0.0):.2f}")  # 使用desired_heading作为期望航向显示
            desired_speed_label.setText(f"{state.get('desired_speed', 0.0):.2f}")  # 使用desired_speed作为期望速度显示
            
            # 发布总线数据
            # self.sub.pubInfo("sense/boat", json.dumps(state_with_origin, ensure_ascii=False))
            
            # 当正在记录数据时，采集当前状态
            if hasattr(self, 'is_recording') and self.is_recording:
                self.data_processor.collect_data(usv_id, state)
                    
        except AttributeError as e:
            print(f"更新USV{usv_num}状态显示失败，控件不存在: {e}")
        except Exception as e:
            print(f"更新USV{usv_num}状态显示失败: {e}")
    
    def handle_record(self):
        """处理数据记录的开始和停止"""
        # 检查是否正在记录
        if not hasattr(self, 'is_recording'):
            self.is_recording = False
        
        if not self.is_recording:
            # 开始记录
            self.is_recording = True
            self.ui.bt_Record.setText("停止记录")
            self.ui.bt_Record.setStyleSheet("background-color: rgb(255, 0, 0);")
            self.ui.USVsConnectState.appendPlainText("开始记录数据")
            
            # 开始实验数据采集
            self.data_processor.start_experiment("manual_record")
        else:
            # 停止记录
            self.is_recording = False
            self.ui.bt_Record.setText("开始记录")
            self.ui.bt_Record.setStyleSheet("")
            self.ui.USVsConnectState.appendPlainText("停止记录数据")
            
            # 停止实验数据采集
            self.data_processor.stop_experiment()
    
    def resizeEvent(self, event):
        """窗口大小变化时的事件处理"""
        # 调用父类的resizeEvent方法
        super(MainWindow, self).resizeEvent(event)
        
        # 获取当前窗口大小
        current_width = self.width()
        current_height = self.height()
        
        # 计算缩放比例
        scale_x = current_width / self.original_width
        scale_y = current_height / self.original_height
        # 只允许缩小字体、不允许放大：窗口放大时字体同比放大会撑破按钮/标签等控件，
        # 在高DPI缩放下保存的窗口尺寸恢复后scale>1，导致控件显示不全
        scale = min(scale_x, scale_y, 1.0)  # 使用最小的缩放比例，保持宽高比
        
        # 计算新的字体大小
        new_font_size = max(8, int(self.original_font_size * scale))  # 确保字体大小不小于8
        
        # 更新所有组件的字体大小
        self.set_font_size_recursive(self.central_widget, new_font_size)
    
    def set_font_size_recursive(self, widget, font_size):
        """递归地设置所有子组件的字体大小"""
        # 检查widget是否具有font属性
        if hasattr(widget, 'font') and hasattr(widget, 'setFont'):
            # 设置当前组件的字体大小
            font = widget.font()
            font.setPointSize(font_size)
            widget.setFont(font)
        
        # 递归设置所有子组件的字体大小
        for child in widget.children():
            try:
                # 尝试获取子组件的children方法
                child.children()
                # 如果有children方法，说明是容器组件，递归设置
                self.set_font_size_recursive(child, font_size)
            except AttributeError:
                # 如果没有children方法，检查是否具有font属性
                if hasattr(child, 'font') and hasattr(child, 'setFont'):
                    font = child.font()
                    font.setPointSize(font_size)
                    child.setFont(font)


if __name__ == "__main__":
    # 高DPI适配：缩放取整策略改为PassThrough（严格按系统缩放比例、不取整），
    # 避免在125%/150%/175%等档位下控件与字体缩放不一致导致显示不全。
    # 必须在创建QApplication之前设置（API与环境变量双保险）。
    os.environ.setdefault("QT_ENABLE_HIGHDPI_SCALING", "1")
    os.environ.setdefault("QT_SCALE_FACTOR_ROUNDING_POLICY", "PassThrough")
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    app = QApplication([])
    window = MainWindow()
    window.show()
    # 高缩放下窗口最小尺寸可能超过屏幕可用区域：收缩最小尺寸并适配屏幕（UI为全布局，会自动重排）
    screen = window.screen().availableGeometry()
    if screen.width() < 1289 or screen.height() < 746:
        window.setMinimumSize(min(1289, screen.width()), min(746, screen.height()))
        window.resize(min(window.width(), screen.width()),
                      min(window.height(), screen.height()))
    sys.exit(app.exec())
