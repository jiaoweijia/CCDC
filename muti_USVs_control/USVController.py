import numpy as np
import casadi as ca
import math
import re
from ObstacleAvoidance import ObstacleAvoidance
def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class LOSGuidance:


    def __init__(self, lookahead_distance=0.3, heading_gain=1.0):
        """
        初始化LOS制导率
        
        参数:
            lookahead_distance: 前视距离
            heading_gain: 航向增益
        """
        self.lookahead_distance = lookahead_distance
        self.heading_gain = heading_gain
    
    def calculate_guidance(self, current_position, current_heading,reference_point=None, next_reference_point=None):
        """
        计算LOS制导率的控制指令
        
        参数:
            current_position: 当前位置 (x, y)
            current_heading: 当前航向角 (rad)
            reference_point: 预先计算好的参考轨迹点 (x, y)，避免重复计算
            next_reference_point: 预先计算好的下一个参考轨迹点 (x, y)，避免重复计算
            
        返回:
            desired_heading: 期望航向角 (rad)
            desired_speed: 期望速度 (m/s)
        """
        # 获取当前参考点位置
        ref_x, ref_y = reference_point
        # 获取下一个参考点位置（用于计算参考轨迹的方向）
        delta_t = 0.1
        next_ref_x, next_ref_y = next_reference_point
        # 计算参考轨迹的方向角
        pi_h = math.atan2(next_ref_y - ref_y, next_ref_x - ref_x)

                
        # 计算当前位置到参考轨迹的垂直距离（LOS标准公式）
        dx = current_position[0] - ref_x
        dy = current_position[1] - ref_y
        # 计算沿航段和横向误差
        R = np.array([[math.cos(pi_h), math.sin(pi_h)],
                      [-math.sin(pi_h), math.cos(pi_h)]])
        along_track, cross_track = R @ np.array([dx, dy])
    
        # LOS制导率计算期望航向角（与单元测试保持一致，移除heading_gain参数）
        los_angle_offset = math.atan2(cross_track, self.lookahead_distance)
        desired_heading = pi_h - los_angle_offset
        # 归一化
        desired_heading = (desired_heading + math.pi) % (2 * math.pi) - math.pi
        
        # 计算期望速度（与单元测试保持一致，使用固定期望速度）
        # desired_speed = (0.1 * np.linalg.norm([dx,dy]) + np.linalg.norm([next_ref_y - ref_y, next_ref_x - ref_x])) / delta_t  # 固定期望速度为0.25 m/s，与单元测试保持一致
        # Symbol = np.sign(np.cos(current_heading-psi_d))
        # desired_speed =  Symbol * np.linalg.norm([next_ref_y - ref_y, next_ref_x - ref_x]) / delta_t  # 固定期望速度为0.25 m/s，与单元测试保持一致

        kp = 0.5
        Ud = np.linalg.norm([dx,dy])
        desired_speed = (Ud - kp * along_track) * np.linalg.norm([cross_track,self.lookahead_distance]) / self.lookahead_distance
        if desired_speed <= 0:
            desired_speed = Ud
        if desired_speed > Ud*2.0:
            desired_speed = Ud*2.0
         

        return desired_heading, desired_speed
class PPGuidance:
    """纯PP制导率类"""
    
    def __init__(self, lookahead_distance=0.1, heading_gain=1.0):
        """
        初始化纯PP制导率

        参数:
            lookahead_distance: 前视距离(米)
            heading_gain: 航向增益
        """
        self.lookahead_distance = lookahead_distance
        self.heading_gain = heading_gain
    
    def calculate_guidance(self, current_position, current_heading, reference_point=None,
                           next_reference_point=None, *, legacy=False):
        """
        计算纯PP制导率的控制指令
        
        参数:
            current_position: 当前位置 (x, y)
            current_heading: 当前航向角 (rad)
            reference_trajectory: 参考轨迹对象，具有calculate_point(t)方法
            current_time: 当前时间
            offset: USV的偏移量 (x_bias, y_bias)
            reference_point: 预先计算好的参考轨迹点 (x, y)，避免重复计算
            next_reference_point: 预先计算好的下一个参考轨迹点 (x, y)，避免重复计算
            
        返回:
            desired_heading: 期望航向角 (rad)
            desired_speed: 期望速度 (m/s)
        """
        # 获取当前参考点位置
        current_x, current_y = current_position
        xk, yk = reference_point
        xk1, yk1 = next_reference_point
        delta_t = 0.1
        dist_to_ref = np.linalg.norm([current_x - xk, current_y - yk])
        segment_length = np.linalg.norm([xk1 - xk, yk1 - yk])

        if legacy:
            # 旧版 PID/反步法使用当前参考点的方位和原速度公式。
            psi_d = math.atan2(yk - current_y, xk - current_x)
            return psi_d, (0.1 * dist_to_ref + segment_length) / delta_t

        psi_d = math.atan2(yk1 - current_y, xk1 - current_x)
        # 期望速度：基于到参考点的距离做比例收敛，并限制上限，
        # 避免参考点距离大时输出远超船速的速度导致速度环饱和、航向环失调。
        # 基础速度与前一段弧长匹配（即参考点推进速度），加上距离误差的比例项
        desired_speed = segment_length / delta_t + 0.3 * dist_to_ref
        # 限制期望速度不超过实物船能达到的速度
        max_speed = 0.8
        desired_speed = float(np.clip(desired_speed, 0.0, max_speed))

        return psi_d, desired_speed

# class PIDController:
#     """PID控制率类"""
    
#     def __init__(self, kp, ki, kd, dt, output_limits=(-float('inf'), float('inf'))):
#         """
#         初始化PID控制器
        
#         参数:
#             kp: 比例增益
#             ki: 积分增益
#             kd: 微分增益
#             dt: 时间步长
#             output_limits: 输出限制 (min, max)
#         """
#         self.kp = kp
#         self.ki = ki
#         self.kd = kd
#         self.dt = dt
#         self.output_limits = output_limits
#         self.error = 0.0
#         self.integral = 0.0
#         self.derivative = 0.0
#         self.prev_error = 0.0
#         self.angle_mode = angle_mode
    
#     def reset(self):
#         """重置PID控制器"""
#         self.error = 0.0
#         self.integral = 0.0
#         self.derivative = 0.0
#         self.prev_error = 0.0
    
#     def update(self, setpoint, measured_value):
#         """
#         更新PID控制器
        
#         参数:
#             setpoint: 设定值
#             measured_value: 测量值
            
#         返回:
#             output: PID输出值
#         """

#         @staticmethod
#         def wrap_angle(angle):
#             return math.atan2(math.sin(angle), math.cos(angle))
#         # 计算误差
#         self.error = setpoint - measured_value 
#         if self.angle_mode:
#             self.error = self.wrap_angle(self.error)
      
                
#         # 计算积分项
#         self.integral += self.error * self.dt
        
#         # 计算微分项
#         self.derivative = (self.error - self.prev_error) / self.dt
#         self.prev_error = self.error
        
#         # 计算PID输出
#         output = self.kp * self.error + self.ki * self.integral + self.kd * self.derivative

#         # 限制输出范围
#         output = max(self.output_limits[0], min(self.output_limits[1], output))

#         return output

class PIDController:
    """通用 PID，支持航向角误差处理，角度单位为弧度。"""

    def __init__(
        self,
        kp,
        ki,
        kd,
        dt,
        output_limits=(-float('inf'), float('inf')),
        angle_mode=False,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.output_limits = output_limits
        self.angle_mode = angle_mode

        self.reset()

    @staticmethod
    def wrap_angle(angle):
        """将角度转换为最短角差。"""
        return math.atan2(math.sin(angle), math.cos(angle))

    def reset(self):
        """重置 PID 内部状态。"""
        self.error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.prev_error = 0.0
        self._has_previous_error = False

    def update(self, setpoint, measured_value):
        """根据目标值和测量值计算控制输出。"""

        # 计算当前误差。
        self.error = setpoint - measured_value

        if self.angle_mode:
            self.error = self.wrap_angle(self.error)

        # 计算积分项。
        self.integral += self.error * self.dt

        # 计算微分项。
        if self.angle_mode:
            if self._has_previous_error:
                error_difference = self.wrap_angle(
                    self.error - self.prev_error
                )
                self.derivative = error_difference / self.dt
            else:
                # 航向 PID 首拍没有历史误差，微分设为零。
                self.derivative = 0.0
        else:
            self.derivative = (
                self.error - self.prev_error
            ) / self.dt

        self.prev_error = self.error
        self._has_previous_error = True

        # 计算 PID 输出。
        output = (
            self.kp * self.error
            + self.ki * self.integral
            + self.kd * self.derivative
        )

        # 限制输出范围。
        lower, upper = self.output_limits
        output = max(lower, min(upper, output))

        return output
class ADRCController:
    """纵向速度与航向双通道线性自抗扰控制器。"""

    # 执行输入增益按实船试验数据标定：u̇ = b0·(Tp+Ts) + f，ṙ = b0d·(Ts-Tp) + f_r。
    # 早期直接沿用 ARX 系数 0.03163 / 0.00162，比实船大约两个数量级，
    # 会让 ESO 的扰动估计无限累积、输出长期贴顶，实测 88.6% 的帧饱和。
    SPEED_INPUT_GAIN = 3.5e-4  #bo
    HEADING_INPUT_GAIN = 1.2e-3
    SPEED_BANDWIDTH = 1.2  #H，速度响应带宽  追赶目标力度
    SPEED_OBSERVER_BANDWIDTH = 3.0 #DH 速度观测带宽 速度ESO更新状态和扰动估计的快慢
    HEADING_BANDWIDTH = 1.4  #TH，航向响应带宽  纠正航向力度
    HEADING_OBSERVER_BANDWIDTH = 2.8 #DTH 航向观测带宽 航向ESO更新状态和扰动估计的快慢
    # 扰动估计限幅，避免模型失配时扰动项积分饱和。
    SPEED_DISTURBANCE_LIMIT = 0.75  # m/s²
    YAW_DISTURBANCE_LIMIT = 0.35  # rad/s²
    # 该船满推约 0.36 m/s，而参考轨迹只需 0.19 m/s；
    # 直行允许 0.28，航向偏差大时按 cos 降速，避免速度把转弯半径撑大。
    MAX_TRACKED_SPEED = 0.28  #ED 目标速度上限，进入速度环的目标速度
    TURN_SLOWDOWN_FLOOR = 0.35
    # 参考航向角速度前馈，消除跟踪圆周参考时的稳态航向滞后。
    YAW_RATE_FEEDFORWARD_LIMIT = 0.6  # rad/s
    YAW_RATE_FEEDFORWARD_FILTER = 0.35
    # 上位机把指令叠加到 1630 基准后再被 2000 截断，实际可用偏置只有 370。
    SERVO_MAX_OFFSET = 370.0

    def __init__(self):
        self.reset()

    def reset(self):
        self._speed_estimate = None  #估计船现在前进速度
        self._speed_disturbance = 0.0 #除推进器以外的帮助或阻碍
        self._heading_estimate = 0.0 #估计船头指向
        self._yaw_rate_estimate = 0.0 #估计船头转速
        self._yaw_disturbance = 0.0 #除船头转速以外的帮助或阻碍转向
        self._previous_sum = 0.0
        self._previous_difference = 0.0
        self._previous_desired_heading = None
        self._heading_rate_reference = 0.0

    def update(self, state, desired_speed, desired_heading, dt, min_output, max_output):
        """从六维状态取 u、r、psi，返回已限幅的左右推进器模型指令。"""
        speed, yaw_rate, heading = state[0], state[2], state[5]
        if not all(math.isfinite(value) for value in
                   (speed, yaw_rate, heading, desired_speed, desired_heading)):
            self.reset()
            return 0.0, 0.0

        if self._speed_estimate is None:
            self._speed_estimate = speed
            self._heading_estimate = heading
            self._yaw_rate_estimate = yaw_rate
        else:
            # 纵向二阶 ESO，离散观测器极点按 e^(-带宽 * dt) 配置。
            speed_pole = math.exp(-self.SPEED_OBSERVER_BANDWIDTH * dt)
            predicted_speed = self._speed_estimate + dt * (
                self.SPEED_INPUT_GAIN * self._previous_sum + self._speed_disturbance
            )
            speed_innovation = speed - predicted_speed
            self._speed_estimate = predicted_speed + (1.0 - speed_pole**2) * speed_innovation
            speed_disturbance = self._speed_disturbance + (1.0 - speed_pole)**2 / dt * speed_innovation
            self._speed_disturbance = float(np.clip(
                speed_disturbance, -self.SPEED_DISTURBANCE_LIMIT, self.SPEED_DISTURBANCE_LIMIT
            ))

            # 航向三阶 ESO，航向创新用最短角差，跨越 ±π 时保持连续。
            heading_pole = math.exp(-self.HEADING_OBSERVER_BANDWIDTH * dt)
            pole_delta = 1.0 - heading_pole
            yaw_accel = self.HEADING_INPUT_GAIN * self._previous_difference + self._yaw_disturbance
            predicted_heading = self._heading_estimate + dt * self._yaw_rate_estimate + 0.5 * dt**2 * yaw_accel
            heading_innovation = math.remainder(heading - predicted_heading, math.tau)
            self._heading_estimate = math.remainder(
                predicted_heading + (1.0 - heading_pole**3) * heading_innovation, math.tau
            )
            self._yaw_rate_estimate += (
                dt * yaw_accel + 1.5 * pole_delta**2 * (1.0 + heading_pole) / dt * heading_innovation
            )
            yaw_disturbance = self._yaw_disturbance + pole_delta**3 / dt**2 * heading_innovation
            self._yaw_disturbance = float(np.clip(
                yaw_disturbance, -self.YAW_DISTURBANCE_LIMIT, self.YAW_DISTURBANCE_LIMIT
            ))

        # 参考航向角速度前馈：对旋转参考做差分并限幅、一阶平滑，避免 ±π 跳变冲击。
        if self._previous_desired_heading is None:
            self._previous_desired_heading = desired_heading
        raw_rate = math.remainder(desired_heading - self._previous_desired_heading, math.tau) / dt
        raw_rate = float(np.clip(raw_rate, -self.YAW_RATE_FEEDFORWARD_LIMIT,
                                 self.YAW_RATE_FEEDFORWARD_LIMIT))
        self._heading_rate_reference += self.YAW_RATE_FEEDFORWARD_FILTER * (
            raw_rate - self._heading_rate_reference
        )
        self._previous_desired_heading = desired_heading

        # 目标速度限制在该船实际可达范围内，并在大航向偏差时降速换取转弯半径。
        speed_target = min(max(desired_speed, 0.0), self.MAX_TRACKED_SPEED)
        heading_error = math.remainder(desired_heading - self._heading_estimate, math.tau)
        speed_target *= max(self.TURN_SLOWDOWN_FLOOR, math.cos(heading_error))
        total = (self.SPEED_BANDWIDTH * (speed_target - self._speed_estimate)
                 - self._speed_disturbance) / self.SPEED_INPUT_GAIN
        difference = (self.HEADING_BANDWIDTH**2 * heading_error
                      + 2.0 * self.HEADING_BANDWIDTH
                      * (self._heading_rate_reference - self._yaw_rate_estimate)
                      - self._yaw_disturbance) / self.HEADING_INPUT_GAIN
        # 单侧推力约束：优先保证差动（航向）权限，必要时压缩合力（速度）。
        upper = min(float(max_output), self.SERVO_MAX_OFFSET)
        feasible = 2.0 * upper
        difference = float(np.clip(difference, -feasible, feasible))
        total = float(np.clip(total, 2.0 * min_output, feasible - abs(difference)))
        left = float(np.clip(0.5 * (total - difference), min_output, upper))
        right = float(np.clip(0.5 * (total + difference), min_output, upper))

        # ESO 回用分配并限幅后的指令，单位与模型一致。
        self._previous_sum = left + right
        self._previous_difference = right - left
        return left, right


class MPCController:    
    """NMPC控制率类"""
   # du = 0.605503*v*r - 1.6*u + 0.858201*v + 0.055049*r + 0.059478*Tp + 0.060716*Ts + 0.314724 - 0.900000*du' + 0.000000*dv' + 0.000000*dr'
   # dv = -0.680488*u*r - 0.134844*u - 1.266535*v + 0.021621*r - 0.006550*Tp + 0.005212*Ts - 0.016607 + 0.000000*du' + 0.000000*dv' + 0.000000*dr'
   # dr = -0.032429*u*v + 0.073954*u + 0.610138*v - 1.0*r - 0.008000*Tp + 0.006000*Ts - 0.002771 + 0.000000*du' + 0.000000*dv' - 0.30000*dr'
   # du = 0.318686*v*r - 1.069696*u + 0.451685*v + 0.028973*r + 0.031304*Tp + 0.031956*Ts + 0.165644 + 0.000000*du' + 0.000000*dv' + 0.000000*dr'
    # dv = -0.680488*u*r - 0.134844*u - 1.266535*v + 0.021621*r - 0.006550*Tp + 0.005212*Ts - 0.016607 + 0.000000*du' + 0.000000*dv' + 0.000000*dr'
    # dr = -0.020507*u*v + 0.046766*u + 0.385833*v - 0.780707*r - 0.006720*Tp + 0.005100*Ts - 0.001752 + 0.000000*du' + 0.000000*dv' + 0.000000*dr'
    # ===== 30参数ARX模型（与 ModelIdentification 辨   # du = 0.318686*v*r - 1.069696*u + 0.451685*v + 0.028973*r + 0.031304*Tp + 0.031956*Ts + 0.165644 + 0.000000*du' + 0.000000*dv' + 0.000000*dr'
    # dv = -0.680488*u*r - 0.134844*u - 1.266535*v + 0.021621*r - 0.006550*Tp + 0.005212*Ts - 0.016607 + 0.000000*du' + 0.000000*dv' + 0.000000*dr'
    # dr = -0.020507*u*v + 0.046766*u + 0.385833*v - 0.780707*r - 0.006720*Tp + 0.005100*Ts - 0.001752 + 0.000000*du' + 0.000000*dv' + 0.000000*dr'识输出方程格式完全一致）=====
    # 每次辨识完成后：把辨识界面上显示的三条方程整块复制、替换下面三引号内的内容即可，
    # 系数由 _parse_arx_equations 按项名自动解析（v*r / u / v / r / Tp / Ts / 常数 / du' / dv' / dr'）。
    #dr的Tp Ts系数调小，提前打刹车  否则模型以为一点差速就能甩动船头，MPC 就敢晚刹车 r前系数调小，阻尼减小，船头转头靠自己停不下来提前制动
    ARX_EQUATIONS = """
    du = 0.605503*v*r - 1.6*u + 0.858201*v + 0.055049*r + 0.059478*Tp + 0.060716*Ts + 0.314724 - 0.900000*du' + 0.000000*dv' + 0.000000*dr'
    dv = -0.680488*u*r - 0.134844*u - 1.266535*v + 0.021621*r - 0.006550*Tp + 0.005212*Ts - 0.016607 + 0.000000*du' + 0.000000*dv' + 0.000000*dr'
    dr = -0.032429*u*v + 0.073954*u + 0.610138*v - 1.0*r - 0.008000*Tp + 0.006000*Ts - 0.002771 + 0.000000*du' + 0.000000*dv' - 0.30000*dr'
    """

    # 方程项名 -> 系数列索引（du/dv/dr 通道的非线性项均为第0列）
    _TERM_INDEX = {'v*r': 0, 'u*r': 0, 'u*v': 0,
                   'u': 1, 'v': 2, 'r': 3, 'Tp': 4, 'Ts': 5,
                   '': 6, "du'": 7, "dv'": 8, "dr'": 9}
    _TERM_RE = re.compile(r"([+-]?)\s*((?:\d+\.?\d*|\.\d+))\s*(?:\*\s*([\w'*]+))?")

    @classmethod
    def _parse_arx_equations(cls, text):
        """解析三条辨识方程文本，返回 (du, dv, dr) 各10个系数（列顺序=方程项顺序）"""
        coefs = {'du': [0.0] * 10, 'dv': [0.0] * 10, 'dr': [0.0] * 10}
        seen = set()
        for line in text.splitlines():
            line = line.strip()
            if not line or '=' not in line:
                continue
            ch, body = line.split('=', 1)
            ch = ch.strip().lower()
            if ch not in coefs:
                raise ValueError(f"ARX方程通道无法识别（应为du/dv/dr开头）: {line!r}")
            for sign, num, name in cls._TERM_RE.findall(body):
                key = name.strip()
                if key not in cls._TERM_INDEX:
                    raise ValueError(f"ARX方程含未知项 {name!r}: {line!r}")
                coefs[ch][cls._TERM_INDEX[key]] += float(num) * (-1.0 if sign == '-' else 1.0)
            seen.add(ch)
        if len(seen) != 3:
            raise ValueError(f"ARX方程缺少通道，只识别到: {sorted(seen)}")
        return tuple(coefs['du']), tuple(coefs['dv']), tuple(coefs['dr'])

    def __init__(self, prediction_horizon=10, control_horizon=10, dt=0.1, max_output=500, min_output=-500):
        """
        初始化NMPC控制器
        
        参数:
            prediction_horizon: 预测时域
            control_horizon: 控制时域
            dt: 采样时间
            max_output: 最大输出值
            min_output: 最小输出值
        """
        self.N = prediction_horizon  # 预测步长
        self.T = dt  # 采样时间
        self.motor_power = max_output  # 推进器功率限制
        self.max_output = max_output
        self.min_output = min_output
        
        # 初始化权重参数
        # 状态权重 (Q)
        self.Q = [0.1, 0.001, 0.001, 100, 100, 0.1]  # 状态权重主参数
        self.baseQ = [0.01 * q for q in self.Q]  # 基础状态权重（缩放后）
        
        # 控制输入权重 (R)
        self.R = [0.05, 0.05]  # 控制输入权重主参数 R越大惩罚越大越平缓
        self.baseR = [20 * r for r in self.R]  # 基础控制输入权重（缩放后）
        
        # 终端状态权重 (Qf)
        self.Qf = [0.1, 0.002, 0.002, 100, 100, 0.1]  # 终端状态权重
        
        # 状态约束权重 (F)
        self.F_low = [0.1, 0.001, 0.001, 100, 100, 0.1]  # 低状态约束权重
        self.F_high = [1, 0.002, 0.002, 5000, 5000, 0.5]  # 高状态约束权重
        self.baseF_high = [0.0001 * f for f in self.F_high]  # 基础高状态约束权重（缩放后）
        self.kQ = 1
        self.kR = 1
        self.kT = 1
        self.a = 1
        
        # 30参数ARX系数：解析 ARX_EQUATIONS 中的辨识方程（列顺序=方程项顺序）
        (du_c, dv_c, dr_c) = self._parse_arx_equations(self.ARX_EQUATIONS)
        (a1, a2, a3, a4, a5, a6, a7, a8, a9, a10) = du_c
        (b1, b2, b3, b4, b5, b6, b7, b8, b9, b10) = dv_c
        (c1, c2, c3, c4, c5, c6, c7, c8, c9, c10) = dr_c
        # 留存数值系数：update 中 ARX 滞后项自递归播种用
        self._du_c, self._dv_c, self._dr_c = du_c, dv_c, dr_c

        # 状态和控制变量定义（状态增广 du/dv/dr：ARX滞后项使用的上一时刻导数）
        u = ca.SX.sym('u')
        v = ca.SX.sym('v')
        r = ca.SX.sym('r')
        x_pos = ca.SX.sym('x')
        y_pos = ca.SX.sym('y')
        psi = ca.SX.sym('psi')
        du_prev = ca.SX.sym('du_prev')
        dv_prev = ca.SX.sym('dv_prev')
        dr_prev = ca.SX.sym('dr_prev')
        Tp = ca.SX.sym('Tp')
        Ts = ca.SX.sym('Ts')

        self.state = ca.vertcat(u, v, r, x_pos, y_pos, psi, du_prev, dv_prev, dr_prev)
        self.control = ca.vertcat(Tp, Ts)

        # 30参数ARX导数（与 ModelIdentification.arx_derivs 完全一致）
        du_arx = a1 * v * r + a2 * u + a3 * v + a4 * r + a5 * Tp + a6 * Ts + a7 \
                 + a8 * du_prev + a9 * dv_prev + a10 * dr_prev
        dv_arx = b1 * u * r + b2 * u + b3 * v + b4 * r + b5 * Tp + b6 * Ts + b7 \
                 + b8 * du_prev + b9 * dv_prev + b10 * dr_prev
        dr_arx = c1 * u * v + c2 * u + c3 * v + c4 * r + c5 * Tp + c6 * Ts + c7 \
                 + c8 * du_prev + c9 * dv_prev + c10 * dr_prev

        # 动力学模型：增广状态 [u,v,r,x,y,psi,du',dv',dr']，其中 du'/dv'/dr' 为上一时刻导数
        # u/v/r 由当前ARX新导数推进，ARX递推与 ModelIdentification.simulate 严格一致：
        #   D_{k+1} = Phi(u_k, ctrl_k) + A*D_k,  u_{k+1} = u_k + dt*D_{k+1}
        rhs = ca.vertcat(  # du纵向加速度 dv横向加速度 dr转向角加速度
            du_arx,
            dv_arx,
            dr_arx,
            u * ca.cos(psi) - v * ca.sin(psi),
            u * ca.sin(psi) + v * ca.cos(psi),
            r,
            (du_arx - du_prev) / self.T,
            (dv_arx - dv_prev) / self.T,
            (dr_arx - dr_prev) / self.T
        )

        self.f = ca.Function('f', [self.state, self.control], [rhs])
        
        # 构建NLP问题
        self.U = ca.SX.sym('U', 2, self.N)
        self.P_state = ca.SX.sym('P_state', 15)  # [当前增广状态9: u,v,r,x,y,psi,du,dv,dr, 期望状态6]
        self.P_weight = ca.SX.sym('P_weight', 14)  # [Q, R, Qf]
        self.P = ca.vertcat(self.P_state, self.P_weight)

        self.X = ca.SX.sym('X', 9, self.N + 1)
        self.X[:, 0] = self.P_state[0:9]
        
        # 预测模型
        for k in range(self.N):
            self.X[:, k + 1] = self.X[:, k] + self.T * self.f(self.X[:, k], self.U[:, k])
        
        # 构建目标函数
        obj = 0
        Qk = ca.diagcat(self.P_weight[0], self.P_weight[1], self.P_weight[2], self.P_weight[3], self.P_weight[4], self.P_weight[5])
        Rk = ca.diagcat(self.P_weight[6], self.P_weight[7])
        Qf = ca.diagcat(self.P_weight[8], self.P_weight[9], self.P_weight[10], self.P_weight[11], self.P_weight[12], self.P_weight[13])
        
        for k in range(self.N):
            st_err = self.X[0:6, k] - self.P_state[9:15]  # 仅跟踪 u,v,r,x,y,psi（增广导数态不进目标）
            con = self.U[:, k]
            obj += st_err.T @ Qk @ st_err + con.T @ Rk @ con

        obj += (self.X[0:6, self.N] - self.P_state[9:15]).T @ Qf @ (self.X[0:6, self.N] - self.P_state[9:15])
        
        # 构建约束
        g = []
        for k in range(self.N + 1):
            g.append(self.X[0, k])  # u约束
            g.append(self.X[1, k])  # v约束
            g.append(self.X[2, k])  # r约束
        g = ca.vertcat(*g)
        
        # 优化变量
        OPT_variables = ca.reshape(self.U, 2 * self.N, 1)
        
        # 定义NLP问题
        nlp = {'x': OPT_variables, 'f': obj, 'g': g, 'p': self.P}
        
        # 求解器选项
        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.max_iter': 50,
            'ipopt.acceptable_tol': 1e-7,
            'ipopt.tol': 1e-8,
            'ipopt.constr_viol_tol': 1e-6,
            'ipopt.hessian_approximation': 'exact'
        }
        
        # 创建求解器
        self.solver = ca.nlpsol('solver', 'ipopt', nlp, opts)
        
        # 约束边界
        # 禁止倒车，提前打转转向，过冲更小，模型高估刹车能力
        # 实船执行不了 → 拐弯/停车过冲（仿真验证：禁倒车过冲 2.65m→0.58m）
        self.lbx = [0.0] * (2 * self.N) 
        self.ubx = [self.motor_power] * (2 * self.N)
        self.lbg = [-2, -0.5, -0.5*math.pi] * (self.N+1)
        self.ubg = [2, 0.5, 0.5*math.pi] * (self.N+1)
        
        # 初始化控制输入
        self.u0 = np.zeros(2 * self.N).tolist()
        # ARX滞后项自递归缓存：上一拍实测 u/v/r、滞后导数估计、实际下发推力
        self._prev_uvr = None
        self._prev_derivs = np.zeros(3)
        self._prev_ctrl = None
        self.terminal_diagnostics = {'mode': 'legacy', 'nominal_plan_checked': False,
                                     'certificate_scope': 'original heuristic; no certificate'}

    def reset(self):
        """重置NMPC内部状态（滚动优化初值与ARX滞后项自递归缓存）"""
        self.u0 = np.zeros(2 * self.N).tolist()
        self._prev_uvr = None
        self._prev_derivs = np.zeros(3)
        self._prev_ctrl = None

    def _arx_step(self, uvr_prev, d_prev, ctrl):
        """ARX 单步递推：D_k = ARX(上拍实测状态, 上拍实际下发推力, 上拍滞后项)。
        与 ModelIdentification.arx_derivs / simulate 的递推式严格一致。"""
        u_, v_, r_ = uvr_prev
        du_p, dv_p, dr_p = d_prev
        Tp_, Ts_ = ctrl
        (a1, a2, a3, a4, a5, a6, a7, a8, a9, a10) = self._du_c
        (b1, b2, b3, b4, b5, b6, b7, b8, b9, b10) = self._dv_c
        (c1, c2, c3, c4, c5, c6, c7, c8, c9, c10) = self._dr_c
        du = a1 * v_ * r_ + a2 * u_ + a3 * v_ + a4 * r_ + a5 * Tp_ + a6 * Ts_ + a7 \
             + a8 * du_p + a9 * dv_p + a10 * dr_p
        dv = b1 * u_ * r_ + b2 * u_ + b3 * v_ + b4 * r_ + b5 * Tp_ + b6 * Ts_ + b7 \
             + b8 * du_p + b9 * dv_p + b10 * dr_p
        dr = c1 * u_ * v_ + c2 * u_ + c3 * v_ + c4 * r_ + c5 * Tp_ + c6 * Ts_ + c7 \
             + c8 * du_p + c9 * dv_p + c10 * dr_p
        return du, dv, dr

    def update(self, current_state, desired_state, dt):
        """
        更新MPC控制器

        参数:
            current_state: 当前状态 [u, v, r, x, y, psi]
            desired_state: 期望状态 [u, v, r, x, y, psi]
            dt: 时间步长

        返回:
            (Tp, Ts): 左右推进器输出
        """
        self.terminal_diagnostics = {'mode': 'legacy', 'nominal_plan_checked': False,
                                     'certificate_scope': 'original heuristic; no certificate'}
        current_state = np.asarray(current_state, dtype=float).flatten()

        # 计算到LOS目标点的距离误差
        distance_error = np.linalg.norm(np.array(desired_state) - current_state)
        distance_coeff = 1 - math.exp(-self.a* distance_error)
        self.Qf = [self.F_low[i] + distance_coeff * (self.F_high[i] - self.F_low[i]) for i in range(6)]

        # ARX滞后项初值：模型自递归估计当前导数 du/dv/dr
        # （上拍实测状态 + 上拍实际下发推力 + 上拍滞后项），不用动捕状态差分——
        # 动捕帧率抖动会把差分变成噪声放大器，污染预测起点
        if self._prev_uvr is not None and self._prev_ctrl is not None:
            d0 = np.array(self._arx_step(self._prev_uvr, self._prev_derivs, self._prev_ctrl))
        else:
            d0 = np.zeros(3)
        d0 = np.clip(d0, [-5.0, -3.0, -10.0], [5.0, 3.0, 10.0])
        self._prev_uvr = current_state[0:3].copy()
        self._prev_derivs = d0.copy()

        # 构建参数向量 [当前增广状态(u,v,r,x,y,psi,du,dv,dr), 期望状态, Q, R, Qf]
        p = np.concatenate([current_state, d0, desired_state, self.Q, self.R, self.Qf])
        
        # 求解NLP问题
        res = self.solver(x0=self.u0, lbx=self.lbx, ubx=self.ubx, lbg=self.lbg, ubg=self.ubg, p=p)
        
        # 提取最优控制输入
        u_opt = res['x'].full().flatten().tolist()
        
        # 更新初始控制输入（滚动时域）
        self.u0 = u_opt[2:] + u_opt[-2:]
        
        # 返回第一个控制输入
        Tp = u_opt[0]
        Ts = u_opt[1]
        
        # 限制输出范围
        Tp = max(self.min_output, min(self.max_output, Tp))
        Ts = max(self.min_output, min(self.max_output, Ts))

        # 记录实际下发推力（限幅后），供下一拍 ARX 滞后项自递归
        self._prev_ctrl = (Tp, Ts)

        return Tp, Ts

class BacksteppingController:
    """稳定版反步法控制率类 - 消除振荡"""
    
    def __init__(self, k1=0.8, k2=1.5, k3=1.2, k4=1.8, 
                 k_ff_x=0.1, k_ff_y=0.1, max_output=200, min_output=-200):
        """
        初始化反步法控制器（超保守参数，消除振荡）
        
        参数:
            k1, k2: 位置和速度跟踪增益
            k3, k4: 航向和偏航率跟踪增益
            k_ff_x, k_ff_y: 前馈增益
            max_output: 最大输出值
            min_output: 最小输出值
        """
        self.k1 = k1
        self.k2 = k2
        self.k3 = k3
        self.k4 = k4
        self.k_ff_x = k_ff_x
        self.k_ff_y = k_ff_y
        self.max_output = max_output
        self.min_output = min_output
        
        # 误差积分项（降低积分增益，防止振荡）
        self.integral_xe = 0
        self.integral_ye = 0
        self.integral_psi_e = 0
        self.integral_limit = 1.0  # 积分限幅
        self.ki_pos = 0.02  # 降低位置积分增益
        self.ki_psi = 0.01  # 降低航向积分增益
        
        # 误差限幅
        self.error_limit_pos = 30
        self.error_limit_psi = 0.8
        
        # 低通滤波（关键：消除振荡）
        self.last_Tp = 0
        self.last_Ts = 0
        self.filter_alpha = 0.9  # 强滤波，消除高频振荡
        
        # 控制量变化率限制
        self.last_Tp_raw = 0
        self.last_Ts_raw = 0
        self.delta_limit = 50  # 每步最大变化量
        
        # 上次期望航向（用于平滑）
        self.last_psi_desired = 0
        
    def reset(self):
        """重置控制器"""
        self.integral_xe = 0
        self.integral_ye = 0
        self.integral_psi_e = 0
        self.last_Tp = 0
        self.last_Ts = 0
        self.last_Tp_raw = 0
        self.last_Ts_raw = 0
        self.last_psi_desired = 0
        
    def _saturate(self, value, limit):
        return np.clip(value, -limit, limit)
    
    def _low_pass_filter(self, new_val, last_val, alpha):
        """一阶低通滤波"""
        return alpha * new_val + (1 - alpha) * last_val
    
    def _rate_limit(self, new_val, last_val, delta_limit, dt):
        """变化率限制"""
        max_change = delta_limit * dt
        return np.clip(new_val, last_val - max_change, last_val + max_change)
        
    def update(self, current_state, desired_state, dt):
        """
        更新反步法控制器（稳定版）
        """
        # 固定dt防止异常
        dt = np.clip(dt, 0.05, 0.2)
        
        # 提取当前状态
        u_curr = current_state[0]
        v_curr = current_state[1]
        r_curr = current_state[2]
        x_curr = current_state[3]
        y_curr = current_state[4]
        psi_curr = current_state[5]
        
        # 提取参考状态
        x_ref = desired_state[3]
        y_ref = desired_state[4]
        psi_ref = desired_state[5]
        u_ref = np.clip(desired_state[0], 0.3, 1.2)
        
        # ========== 位置误差（带限幅）==========
        xe = self._saturate(x_curr - x_ref, self.error_limit_pos)
        ye = self._saturate(y_curr - y_ref, self.error_limit_pos)
        
        # 积分项（很弱的积分，防止振荡）
        self.integral_xe += self.ki_pos * xe * dt
        self.integral_ye += self.ki_pos * ye * dt
        self.integral_xe = np.clip(self.integral_xe, -self.integral_limit, self.integral_limit)
        self.integral_ye = np.clip(self.integral_ye, -self.integral_limit, self.integral_limit)
        
        # 位置误差控制量（弱积分）
        xe_control = xe + 0.3 * self.integral_xe
        ye_control = ye + 0.3 * self.integral_ye
        
        # 转换到船体坐标系
        cos_psi = np.cos(psi_curr)
        sin_psi = np.sin(psi_curr)
        xe_body = xe_control * cos_psi + ye_control * sin_psi
        ye_body = -xe_control * sin_psi + ye_control * cos_psi
        
        # ========== LOS制导（保守参数，减少振荡）==========
        look_ahead = 2.5  # 增大前视距离，减少振荡
        # 平滑的LOS角
        los_angle = -np.arctan2(ye_body, look_ahead + abs(xe_body) * 0.5)
        
        # 期望航向（带低通，处理±π边界跳变）
        psi_desired_raw = psi_ref + los_angle
        # 归一化到[-π, π]
        psi_desired_raw = np.arctan2(np.sin(psi_desired_raw), np.cos(psi_desired_raw))
        # 角度解缠：消除与上一时刻的±π跳变，确保连续性
        diff = psi_desired_raw - self.last_psi_desired
        diff = np.arctan2(np.sin(diff), np.cos(diff))  # 最短角度差
        psi_desired_unwrapped = self.last_psi_desired + diff
        # 低通滤波（在连续域上进行）
        psi_desired = self._low_pass_filter(psi_desired_unwrapped, self.last_psi_desired, 0.5)
        self.last_psi_desired = psi_desired
        
        # ========== 航向误差 ==========
        psi_e = psi_curr - psi_desired
        psi_e = np.arctan2(np.sin(psi_e), np.cos(psi_e))
        psi_e = self._saturate(psi_e, self.error_limit_psi)
        
        # 航向误差积分（很弱）
        self.integral_psi_e += self.ki_psi * psi_e * dt
        self.integral_psi_e = np.clip(self.integral_psi_e, -0.5, 0.5)
        psi_e_control = psi_e + 0.2 * self.integral_psi_e
        
        # ========== 速度误差 ==========
        u_e = u_curr - u_ref
        u_e = np.clip(u_e, -1.0, 1.0)
        
        # ========== 偏航率误差（简化版，减少计算复杂度）==========
        # 直接使用偏航率反馈，不计算复杂的期望偏航率
        r_e = r_curr
        r_e = np.clip(r_e, -1.0, 1.0)
        
        # ========== 使用与USVDynamics一致的模型参数 ==========
        # 动力学非线性项
        # f_u = (-0.578486 * v_curr * r_curr - 0.590191 * u_curr 
        #        - 0.502736 * v_curr + 0.080375 * r_curr - 0.119302)
        
        # f_r = (-0.020507 * u_curr * v_curr + 0.046766 * u_curr 
        #        + 0.385833 * v_curr - 0.780707 * r_curr - 0.001752)
        f_u = (0.318686 * v_curr * r_curr -1.069696 * u_curr 
               + 0.451685 * v_curr + 0.028973 * r_curr + 0.165644)
        
        f_r = (-0.020507 * u_curr * v_curr + 0.046766 * u_curr 
               + 0.385833 * v_curr - 0.780707 * r_curr - 0.001752)

        # 控制矩阵系数
        g_u1 = 0.056161
        g_u2 = 0.041063
        g_r1 = -0.00672
        g_r2 = 0.00510
        
        # ========== 简化的控制律（减少振荡）==========
        # 纵向控制 - 简化为PD形式
        tau_u = -f_u - self.k2 * u_e - 0.3 * u_curr  # 增加阻尼
        
        # 偏航控制 - 简化为PD形式，不使用复杂的耦合项
        tau_r = -f_r - self.k4 * r_e - self.k3 * psi_e_control - 0.2 * r_curr
        
        # 限制虚拟控制输入
        tau_u = np.clip(tau_u, -500, 500)
        tau_r = np.clip(tau_r, -500, 500)
        
        # ========== 解算控制输入 ==========
        det = g_u1 * g_r2 - g_u2 * g_r1
        
        if abs(det) > 1e-4:
            Tp_raw = (tau_u * g_r2 - g_u2 * tau_r) / det
            Ts_raw = (g_u1 * tau_r - tau_u * g_r1) / det
        else:
            # 伪逆
            Tp_raw = (g_u1 * tau_u + g_r1 * tau_r) / (g_u1**2 + g_r1**2 + 0.01)
            Ts_raw = (g_u2 * tau_u + g_r2 * tau_r) / (g_u2**2 + g_r2**2 + 0.01)
        
        # ========== 控制量处理（关键：消除振荡）==========
        # 1. 硬限幅
        Tp_raw = np.clip(self.k_ff_x * Tp_raw, self.min_output, self.max_output)
        Ts_raw = np.clip(self.k_ff_y * Ts_raw, self.min_output, self.max_output)
        
        # 2. 变化率限制（防止突变）
        Tp_limited = self._rate_limit(Tp_raw, self.last_Tp_raw, self.delta_limit, dt)
        Ts_limited = self._rate_limit(Ts_raw, self.last_Ts_raw, self.delta_limit, dt)
        
        # 3. 低通滤波（消除高频振荡）
        Tp_filtered = self._low_pass_filter(Tp_limited, self.last_Tp, self.filter_alpha)
        Ts_filtered = self._low_pass_filter(Ts_limited, self.last_Ts, self.filter_alpha)
        
        # 保存状态
        self.last_Tp_raw = Tp_limited
        self.last_Ts_raw = Ts_limited
        self.last_Tp = Tp_filtered
        self.last_Ts = Ts_filtered
        
        return Tp_raw, Ts_raw
class USVDynamics:
    """USV动力学和运动学模型类，该模块采用数据驱动的模型"""
    def __init__(self):
        """初始化USV动力学模型"""
        # 状态向量: [u, v, r, x, y, psi]
        # u: 前向速度 (m/s)
        # v: 横向速度 (m/s)
        # r: 偏航角速度 (rad/s)
        # x: x坐标 (m)
        # y: y坐标 (m)
        # psi: 偏航角 (rad)
        self.state = np.zeros(6)
        self.reference_point = np.zeros(2)
        
        # 控制参数
        self.T = 0.1  # 采样时间 (s)

        # # ===== Fossen 3自由度模型参数（当前为占位值，待实船辨识后替换） =====
        # self.thruster_separation = 0.4  # 两桨间距 (m)，差动产生艏摇力矩的力臂
        # # 惯性矩阵 M = diag(m11, m22, m66)，含附加质量（Xu_dot/Yv_dot/Nr_dot 为负贡献）
        # self.m11 = 11.0  # 纵荡：m + 附加质量 (kg)
        # self.m22 = 25.0  # 横荡：m + 附加质量 (kg)，横向附加质量明显更大
        # self.m66 = 1.5   # 艏摇：Iz + 附加惯量 (kg·m²)
        # # 线性阻尼 (N·s/m, N·m·s/rad)
        # self.Xu = 2.0
        # self.Yv = 10.0
        # self.Nr = 1.0
        # # 二次阻尼 (N·s²/m², N·m·s²/rad²)
        # self.Xuq = 5.0
        # self.Yvq = 20.0
        # self.Nrq = 2.0
    
    def reset_state(self, initial_state=None):
        """
        重置USV状态
        
        参数:
            initial_state: 初始状态向量，如果为None则重置为零向量
        """
        if initial_state is None:
            self.state = np.zeros(6)
        else:
            self.state = np.array(initial_state)
    
    def update_state(self, Tp, Ts):
        """
        使用动力学和运动学方程更新USV状态
        
        参数:
            Tp: 左推力器推力 (N)
            Ts: 右推力器推力 (N)
        """
        # 获取当前状态
        u = self.state[0]
        v = self.state[1]
        r = self.state[2]
        x = self.state[3]
        y = self.state[4]
        psi = self.state[5]
        
        # 计算状态导数
        du_dt = -0.578486 * v * r + -0.590191 * u + -0.502736 * v + 0.080375 * r + 0.056161 * Tp + 0.041063 * Ts + -0.119302      
        dv_dt = -0.680488 * u * r + -0.134844 * u + -1.266535 * v + 0.021621 * r + -0.006550 * Tp + 0.005212 * Ts + -0.016607
        dr_dt = -0.020507 * u * v + 0.046766 * u + 0.385833 * v + -0.780707 * r + -0.00672 * Tp + 0.00510 * Ts + -0.001752
    
        # x位置导数 (dx/dt)
        dx_dt = u * np.cos(psi) - v * np.sin(psi)
        
        # y位置导数 (dy/dt)
        dy_dt = u * np.sin(psi) + v * np.cos(psi)
        
        # 偏航角导数 (dpsi/dt)
        dpsi_dt = r
        
        # 更新状态
        self.state[0] += self.T * du_dt
        self.state[1] += self.T * dv_dt
        self.state[2] += self.T * dr_dt
        self.state[3] += self.T * dx_dt
        self.state[4] += self.T * dy_dt
        self.state[5] += self.T * dpsi_dt
        
        # 数值限制和检查，防止溢出和NaN值
        # 限制速度范围
        self.state[0] = np.clip(self.state[0], -10.0, 10.0)  # 前向速度限制在±10 m/s
        self.state[1] = np.clip(self.state[1], -5.0, 5.0)    # 横向速度限制在±5 m/s
        self.state[2] = np.clip(self.state[2], -np.pi, np.pi) # 偏航角速度限制在±π rad/s
        
        # 限制位置范围（防止数值溢出）
        self.state[3] = np.clip(self.state[3], -1000.0, 1000.0)  # x位置限制
        self.state[4] = np.clip(self.state[4], -1000.0, 1000.0)  # y位置限制
        
        # 确保偏航角在[-pi, pi]范围内
        self.state[5] = np.arctan2(np.sin(self.state[5]), np.cos(self.state[5]))
        
        # 检查NaN值，如果出现NaN则重置状态
        if np.any(np.isnan(self.state)):
            print("警告：检测到NaN值，重置USV状态")
            self.state = np.zeros(6)

    def update_state_physics(self, Tp, Ts):
        """
        使用动力学和运动学方程更新USV状态

        参数:
            Tp: 左推力器推力 (N)
            Ts: 右推力器推力 (N)
        """
        # 获取当前状态
        u = self.state[0]
        v = self.state[1]
        r = self.state[2]
        x = self.state[3]
        y = self.state[4]
        psi = self.state[5]
        #控制量合二为一
        T_sum = Tp + Ts
        T_diff = Tp - Ts
        # # Fossen 3自由度模型:  M·ν̇ + C(ν)·ν + D(ν)·ν = τ，ν = [u, v, r]
        # # 控制输入合成：双桨差动 → 纵荡合力 X 与艏摇力矩 N（横荡无直接推力）
        # X = Tp + Ts
        # N = (Tp - Ts) * self.thruster_separation / 2.0
        #简化增益
        k_thrust = 0.5 #T_sum  u
        k_yaw = 1.0 #T_diff  r
        tau_u = 0.5 #surge一阶常数
        tau_r = 0.3 #yaw一阶常数
        
        #这里需要Fossen模型
        du_dt = (-u + k_thrust * T_sum) / tau_u
        dv_dt = 0
        dr_dt = (-r + k_yaw * T_diff) / tau_r

        # # 科氏-向心力项（x_g=0 的水面船标准形式）
        # cor_u = -self.m22 * v * r
        # cor_v = self.m11 * u * r
        # cor_r = (self.m22 - self.m11) * u * v

        # # 阻尼项（线性 + 二次）
        # damp_u = self.Xu * u + self.Xuq * u * abs(u)
        # damp_v = self.Yv * v + self.Yvq * v * abs(v)
        # damp_r = self.Nr * r + self.Nrq * r * abs(r)

        # # M 为对角阵，直接解出加速度
        # du_dt = (X - cor_u - damp_u) / self.m11
        # dv_dt = (0.0 - cor_v - damp_v) / self.m22
        # dr_dt = (N - cor_r - damp_r) / self.m66
        # 欧拉积分：更新速度
        u += du_dt * self.T
        v += dv_dt * self.T
        r += dr_dt * self.T

        # x位置导数 (dx/dt)
        dx_dt = u * np.cos(psi) - v * np.sin(psi)

        # y位置导数 (dy/dt)
        dy_dt = u * np.sin(psi) + v * np.cos(psi)

        # 偏航角导数 (dpsi/dt)
        dpsi_dt = r

        # 更新状态
        self.state[0] += self.T * du_dt
        self.state[1] += self.T * dv_dt
        self.state[2] += self.T * dr_dt
        self.state[3] += self.T * dx_dt
        self.state[4] += self.T * dy_dt
        self.state[5] += self.T * dpsi_dt

        # 数值限制和检查，防止溢出和NaN值
        # 限制速度范围
        self.state[0] = np.clip(self.state[0], -10.0, 10.0)  # 前向速度限制在±10 m/s
        self.state[1] = np.clip(self.state[1], -5.0, 5.0)  # 横向速度限制在±5 m/s
        self.state[2] = np.clip(self.state[2], -np.pi, np.pi)  # 偏航角速度限制在±π rad/s

        # 限制位置范围（防止数值溢出）
        self.state[3] = np.clip(self.state[3], -1000.0, 1000.0)  # x位置限制
        self.state[4] = np.clip(self.state[4], -1000.0, 1000.0)  # y位置限制

        # 确保偏航角在[-pi, pi]范围内
        self.state[5] = np.arctan2(np.sin(self.state[5]), np.cos(self.state[5]))

        # 检查NaN值，如果出现NaN则重置状态
        if np.any(np.isnan(self.state)):
            print("警告：检测到NaN值，重置USV状态")
            self.state = np.zeros(6)

    def update_state_stupid(self, goal_u, goal_psi):
        """
        使用动力学和运动学方程更新USV状态

        参数:
            Tp: 左推力器推力 (N)
            Ts: 右推力器推力 (N)
        """
        # 获取当前状态
        u = self.state[0]
        v = self.state[1]
        r = self.state[2]
        x = self.state[3]
        y = self.state[4]
        psi = self.state[5]

        # 这里采用无人船傻蛋模型，去他**的du/dv/dr
        u += goal_u / 150 * self.T
        v = 0
        r += goal_psi / 300 * self.T

        # x位置导数 (dx/dt)
        dx_dt = u * np.cos(psi) - v * np.sin(psi)

        # y位置导数 (dy/dt)
        dy_dt = u * np.sin(psi) + v * np.cos(psi)

        # 偏航角导数 (dpsi/dt)
        dpsi_dt = r

        # 更新状态
        self.state[0] = u
        self.state[1] = v
        self.state[2] = r
        self.state[3] += self.T * dx_dt
        self.state[4] += self.T * dy_dt
        self.state[5] += self.T * dpsi_dt

        # 数值限制和检查，防止溢出和NaN值
        # 限制速度范围
        self.state[0] = np.clip(self.state[0], -10.0, 10.0)  # 前向速度限制在±10 m/s
        self.state[1] = np.clip(self.state[1], -5.0, 5.0)  # 横向速度限制在±5 m/s
        self.state[2] = np.clip(self.state[2], -np.pi, np.pi)  # 偏航角速度限制在±π rad/s

        # 限制位置范围（防止数值溢出）
        self.state[3] = np.clip(self.state[3], -1000.0, 1000.0)  # x位置限制
        self.state[4] = np.clip(self.state[4], -1000.0, 1000.0)  # y位置限制

        # 确保偏航角在[-pi, pi]范围内
        self.state[5] = np.arctan2(np.sin(self.state[5]), np.cos(self.state[5]))

        # 检查NaN值，如果出现NaN则重置状态
        if np.any(np.isnan(self.state)):
            print("警告：检测到NaN值，重置USV状态")
            self.state = np.zeros(6)

    def set_state(self, state):
        """
        设置USV状态
        
        参数:
            state: 状态向量 [u, v, r, x, y, psi]
        """
        self.state = np.array(state)
    
    def get_state(self):
        """
        获取当前USV状态
        
        返回:
            state: 当前状态向量 [u, v, r, x, y, psi]
        """
        return self.state.copy()
    
    def get_position(self):
        """
        获取当前USV位置
        
        返回:
            (x, y): 当前位置坐标
        """
        return (self.state[3], self.state[4])
    
    def get_heading(self):
        """
        获取当前USV航向角
        
        返回:
            psi: 当前航向角 (rad)
        """
        return self.state[5]
    
    def get_velocity(self):
        """
        获取当前USV速度
        
        返回:
            (u, v): 前向速度和横向速度
        """
        return (self.state[0], self.state[1])
    
    def get_yaw_rate(self):
        """
        获取当前USV偏航角速度
        
        返回:
            r: 当前偏航角速度 (rad/s)
        """
        return self.state[2]
class USVControlSystem:
    """USV控制系统类，整合制导率和控制率"""
    
    def __init__(self, usv_id, guidance_law='PP', control_law='NMPC', mode='real'):
        """
        初始化USV控制系统
        
        参数:
            usv_id: USV标识符
            mode: 运行模式，'real'表示实船模式，'simulation'表示仿真模式
        """
        self.usv_id = usv_id
        self.mode = mode
        self.simulation_enabled = False
        # 初始化动力学模型
        self.dynamics = USVDynamics()
        # 初始化制导率实例
        self.los_guidance = LOSGuidance()
        self.pp_guidance = PPGuidance()
        # 初始化制导率字典（保持向后兼容）
        self.guidance_laws = {
            'LOS': self.los_guidance,
            'PP': self.pp_guidance
        }
        
        # 初始化PID控制器
        self.speed_pid = PIDController(kp=6.0,ki=0,kd=0.5, dt=self.dynamics.T, output_limits=(-500, 500))
        self.heading_pid = PIDController(kp=2.0, ki=0.1, kd=0, dt=self.dynamics.T, output_limits=(-500, 500))
        self.heading_pid.angle_mode = True
        # 初始化MPC控制器
        self.mpc = MPCController(max_output=500, min_output=0)
        # 初始化反步法控制器
        self.backstepping = BacksteppingController(max_output=500, min_output=-500)
        # 初始化ADRC控制器
        self.adrc = ADRCController()
        # 初始化控制率字典（保持向后兼容）
        self.control_laws = {
            'PID': {
                'speed_pid': self.speed_pid,
                'heading_pid': self.heading_pid
            },
            'NMPC': {
                'mpc': self.mpc
            },
            'Backstepping': {
                'backstepping': self.backstepping
            },
            'ADRC': {
                'adrc': self.adrc
            }
        }
        
        # 设置默认制导率和控制率
        self.current_guidance = guidance_law
        self.current_control = control_law
        self.Tp = 0
        self.Ts = 0
        self.desired_heading = 0.0
        self.desired_speed = 0.0

        # 避障模块（基于人工势场法）
        self.obstacle_avoidance = ObstacleAvoidance(
            influence_distance=0.8,
            safety_margin=0.15,
            repulsive_gain=2.0,
            max_avoidance_angle=math.pi * 0.6,
            min_speed_ratio=0.1
        )
        self.obstacles = []  # 当前帧的障碍物列表

    def reset(self, initial_state=None):
        """
        重置USV控制系统
        
        参数:
            initial_state: 初始状态向量
        """
        # 重置动力学模型
        self.dynamics.reset_state(initial_state)
        
        # 重置所有控制器
        self.speed_pid.reset()
        self.heading_pid.reset()
        self.mpc.reset()
        self.backstepping.reset()
        self.adrc.reset()
        # 重置避障平滑状态
        self.obstacle_avoidance.smoothed_heading_diff = 0.0
        self.obstacle_avoidance.smoothed_offset_mag = 0.0
        self.obstacle_avoidance._prev_smoothed_heading_diff = 0.0
        self.obstacle_avoidance.path_detour_sides = {}

    def clone_for_simulation(self, initial_state=None):
        """Create an independent controller with the same tuning for comparison runs."""
        clone = USVControlSystem(
            self.usv_id,
            guidance_law=self.current_guidance,
            control_law=self.current_control,
            mode='simulation'
        )
        clone.dynamics.T = self.dynamics.T

        for guidance_name in ('los_guidance', 'pp_guidance'):
            source_guidance = getattr(self, guidance_name)
            target_guidance = getattr(clone, guidance_name)
            target_guidance.lookahead_distance = source_guidance.lookahead_distance
            target_guidance.heading_gain = source_guidance.heading_gain

        for controller_name in ('speed_pid', 'heading_pid'):
            source_pid = getattr(self, controller_name)
            target_pid = getattr(clone, controller_name)
            for attribute in ('kp', 'ki', 'kd', 'dt', 'output_limits'):
                setattr(target_pid, attribute, getattr(source_pid, attribute))

        source_mpc = self.mpc
        target_mpc = clone.mpc
        if source_mpc.N != target_mpc.N:
            target_mpc = MPCController(
                prediction_horizon=source_mpc.N,
                dt=source_mpc.T,
                max_output=source_mpc.max_output,
                min_output=source_mpc.min_output
            )
            clone.mpc = target_mpc
            clone.control_laws['NMPC']['mpc'] = target_mpc
        for attribute in (
            'N', 'T', 'motor_power', 'max_output', 'min_output',
            'Q', 'baseQ', 'R', 'baseR', 'Qf', 'F_low', 'F_high',
            'baseF_high', 'kQ', 'kR', 'kT', 'a'
        ):
            value = getattr(source_mpc, attribute)
            setattr(target_mpc, attribute, value.copy() if isinstance(value, list) else value)

        clone.backstepping.__dict__.update(self.backstepping.__dict__)
        for attribute in (
            'influence_distance', 'safety_margin', 'repulsive_gain',
            'max_avoidance_angle', 'min_speed_ratio', 'smoothing_factor',
            'max_heading_rate'
        ):
            setattr(clone.obstacle_avoidance, attribute,
                    getattr(self.obstacle_avoidance, attribute))

        state = initial_state if initial_state is not None else self.dynamics.get_state()
        clone.reset(np.array(state, dtype=float).copy())
        return clone
    
    def set_guidance_law(self, guidance_name):
        """
        设置制导率
        
        参数:
            guidance_name: 制导率名称，可选值: 'LOS', 'PP'
        """
        if guidance_name in self.guidance_laws:
            self.current_guidance = guidance_name
            return True
        return False
    
    def set_control_law(self, control_name):
        """
        设置控制率

        参数:
            control_name: 控制率名称，可选值: 'PID', 'NMPC', 'Backstepping', 'ADRC'
        """
        if control_name in self.control_laws:
            if control_name == 'ADRC' and self.current_control != 'ADRC':
                self.adrc.reset()
            self.current_control = control_name
            return True
        return False

    def set_obstacles(self, obstacles):
        """设置当前帧的障碍物列表"""
        self.obstacles = obstacles if obstacles is not None else []

    def set_obstacle_avoidance_params(self, **params):
        """更新避障模块参数"""
        oa = self.obstacle_avoidance
        if 'influence_distance' in params:
            oa.influence_distance = params['influence_distance']
        if 'safety_margin' in params:
            oa.safety_margin = params['safety_margin']
        if 'repulsive_gain' in params:
            oa.repulsive_gain = params['repulsive_gain']
        if 'max_avoidance_angle' in params:
            oa.max_avoidance_angle = params['max_avoidance_angle']
        if 'min_speed_ratio' in params:
            oa.min_speed_ratio = params['min_speed_ratio']
    #一帧完整控制循环
    def control(self, usv_id, reference_point=None, next_reference_point=None, usv_state=None, obstacles=None):
        """
        执行控制循环

        参数:
            usv_id: USV标识符
            reference_point: 预先计算好的参考轨迹点 (x, y)
            next_reference_point: 预先计算好的下一个参考轨迹点 (x, y)
            usv_state: USV当前状态向量
            obstacles: 障碍物列表 [Obstacle, ...]，用于避障（可选）
            
        返回:
            (x, y): 更新后的位置坐标
        """
        self.usv_id = usv_id
        self.reference_point = reference_point
        if usv_state is not None:
            self.dynamics.set_state(usv_state)
            self.simulation_enabled = False
        else:
            self.simulation_enabled = True
        # 获取当前状态
        current_position = self.dynamics.get_position()
        current_heading = self.dynamics.get_heading()
        current_velocity = self.dynamics.get_velocity()
        current_state = self.dynamics.get_state()
        
        # 使用当前选择的制导率计算期望航向角和速度
        if self.current_guidance == 'LOS':
            self.desired_heading, self.desired_speed = self.los_guidance.calculate_guidance(
                current_position, current_heading, reference_point, next_reference_point
            )
        elif self.current_guidance == 'PP':  # 默认使用PP制导率
            self.desired_heading, self.desired_speed = self.pp_guidance.calculate_guidance(
                current_position, current_heading, reference_point, next_reference_point,
                legacy=self.current_control in ('PID', 'Backstepping'),
            )

        # ===== 避障修正：在制导之后、控制率之前修正期望航向、速度和目标点 =====
        if obstacles is None:
            obstacles = self.obstacles
        if obstacles:
            self.desired_heading, self.desired_speed, reference_point = self.obstacle_avoidance.compute_avoidance(
                current_position, current_heading,
                self.desired_heading, self.desired_speed,
                obstacles, reference_point
            )

        speed_output = 0
        heading_output = 0
        # 根据当前选择的控制率计算控制输出
        if self.current_control == 'PID':
            # 使用PID控制器计算控制输出
            speed_output = self.speed_pid.update(self.desired_speed, np.linalg.norm(current_velocity[0:2]))
            heading_error = math.atan2(math.sin(self.desired_heading - current_heading), math.cos(self.desired_heading - current_heading))
            heading_output = self.heading_pid.update(heading_error,0)
            
            # 计算左右推力器推力
            self.Tp = 0.5 * (speed_output - heading_output)  # 左推力器Portboard Thruster
            self.Ts = 0.5 * (speed_output + heading_output)  # 右推力器Starboard Thruster

        elif self.current_control == 'NMPC':
            # 构建期望状态
            desired_state = [self.desired_speed, 0, 0, 
                           reference_point[0], 
                           reference_point[1], 
                           self.desired_heading]
            
            # 使用NMPC控制器计算控制输出
            self.Tp, self.Ts = self.mpc.update(current_state, desired_state, self.dynamics.T)
        elif self.current_control == 'Backstepping':
            # 构建期望状态
            desired_state = [self.desired_speed, 0, 0, 
                           reference_point[0], 
                           reference_point[1], 
                           self.desired_heading]
            
            # 旧版动捕给反步法的艏摇角速度为零。仅修改实船反步法的输入副本。
            backstepping_state = current_state
            if not self.simulation_enabled:
                backstepping_state = np.array(current_state, dtype=float, copy=True)
                backstepping_state[2] = 0.0
            self.Tp, self.Ts = self.backstepping.update(backstepping_state, desired_state, self.dynamics.T)
        elif self.current_control == 'ADRC':
            self.Tp, self.Ts = self.adrc.update(
                current_state, self.desired_speed, self.desired_heading, self.dynamics.T,
                self.mpc.min_output, self.mpc.max_output,
            )
        else:
            # 默认使用PID控制
            self.Tp, self.Ts = 0, 0

        if self.simulation_enabled and self.current_control != 'PID':
            # 更新USV状态
            self.dynamics.update_state(self.Tp, self.Ts)
        elif self.simulation_enabled and self.current_control == 'PID':
            # 如果是PID的话直接用安泉的傻蛋模型
            self.dynamics.update_state_stupid(speed_output, heading_output)
        else:
            # 不更新状态，直接返回当前位置
            pass
        # 返回更新后的位置
        return self.get_result()            
    #内部状态和指令 组装成一个 dict ，供界面显示、数据记录、PWM 发送使用
    def get_result(self):
        """
        计算结果
        
        返回:
            (x, y, psi, u, v, V, pwm_left, pwm_right): 更新后的位置坐标、航向角和左右推力器PWM值
        """
        # 获取USV的当前状态
        state = self.dynamics.get_state()
        
        # 计算结果
        result = {
            'usv_id': self.usv_id,
            'x': state[3],
            'y': state[4],
            'rx': self.reference_point[0],
            'ry': self.reference_point[1],
            'heading': state[5],
            'u': state[0],
            'v': state[1],
            'V': np.linalg.norm(state[0:2]),
            'pwml': self.Tp,
            'pwmr': self.Ts,
            'desired_heading': self.desired_heading,
            'desired_speed': self.desired_speed,
            'terminal': dict(self.mpc.terminal_diagnostics) if self.current_control == 'NMPC' else {},
        }
        return result
    #取完整状态向量
    def get_state(self):
        """
        获取当前USV状态
        
        返回:
            state: 当前状态向量 [u, v, r, x, y, psi]
        """
        return self.dynamics.get_state()
    
    def get_position(self):
        """
        获取当前USV位置
        
        返回:
            (x, y): 当前位置坐标
        """
        return self.dynamics.get_position()
    
    def get_heading(self):
        """
        获取当前USV航向角
        
        返回:
            psi: 当前航向角 (rad)
        """
        return self.dynamics.get_heading()
    
    def update_params(self, **params):
        """
        更新控制系统参数
        
        参数:
            params: 控制系统参数，包括:
                lookahead_distance: 前视距离
                heading_gain: 航向增益
                K_PP: PP制导率增益
                speed_kp: 速度PID比例系数
                speed_ki: 速度PID积分系数
                speed_kd: 速度PID微分系数
                heading_kp/angle_kp: 航向PID比例系数
                heading_ki/angle_ki: 航向PID积分系数
                heading_kd/angle_kd: 航向PID微分系数
                mpc_q: MPC权重Q
                mpc_r: MPC权重R
                mpc_t: MPC预测时域
                mpca: MPC参数a
        """
        # 更新LOS制导率参数
        if 'lookahead_distance' in params:
            if 'LOS' in self.guidance_laws:
                self.los_guidance.lookahead_distance = params['lookahead_distance']
            if 'PP' in self.guidance_laws:
                self.pp_guidance.lookahead_distance = params['lookahead_distance']
        if 'heading_gain' in params:
            if 'LOS' in self.guidance_laws:
                self.los_guidance.heading_gain = params['heading_gain']
            if 'PP' in self.guidance_laws:
                self.pp_guidance.heading_gain = params['heading_gain']
        
        # 更新速度PID参数
        if 'speed_kp' in params:
            self.speed_pid.kp = params['speed_kp']
        if 'speed_ki' in params:
            self.speed_pid.ki = params['speed_ki']
        if 'speed_kd' in params:
            self.speed_pid.kd = params['speed_kd']
        
        # 更新航向PID参数 - 支持两种参数名称
        if 'heading_kp' in params or 'angle_kp' in params:
            kp = params.get('heading_kp', params.get('angle_kp', None))
            if kp is not None:
                self.heading_pid.kp = kp
        if 'heading_ki' in params or 'angle_ki' in params:
            ki = params.get('heading_ki', params.get('angle_ki', None))
            if ki is not None:
                self.heading_pid.ki = ki
        if 'heading_kd' in params or 'angle_kd' in params:
            kd = params.get('heading_kd', params.get('angle_kd', None))
            if kd is not None:
                self.heading_pid.kd = kd
        
        # 更新MPC参数
        if 'NMPC' in self.control_laws:
            if 'mpc_q' in params:
                # MPC权重Q
                self.mpc.kQ = params['mpc_q']
                self.mpc.Q = [self.mpc.kQ * q for q in self.mpc.baseQ]
            if 'mpc_r' in params:
                # MPC权重R
                self.mpc.kR = params['mpc_r']
                self.mpc.R = [self.mpc.kR * r for r in self.mpc.baseR]
            if 'mpc_t' in params:
                # MPC预测时域
                self.mpc.kT = int(params['mpc_t'])
                self.mpc.F_high = [self.mpc.kT * q for q in self.mpc.baseF_high]
            if 'mpca' in params:
                # MPC参数a（如果需要）
                self.mpc.a = params['mpca']
                # pass  # Mpca参数在当前MPC实现中没有使用
        
        # 更新反步法控制器参数
        if 'Backstepping' in self.control_laws:
            if 'bp_kp' in params:
                self.backstepping.k1 = params['bp_kp']
            if 'bp_kv' in params:
                self.backstepping.k2 = params['bp_kv']
            if 'bp_kh' in params:
                self.backstepping.k3 = params['bp_kh']
            if 'bp_kr' in params:
                self.backstepping.k4 = params['bp_kr']
            if 'bp_kx' in params:
                self.backstepping.k_ff_x = params['bp_kx']
            if 'bp_ky' in params:
                self.backstepping.k_ff_y = params['bp_ky']

        # 更新避障参数
        oa_params = {}
        for key in ('influence_distance', 'safety_margin', 'repulsive_gain',
                    'max_avoidance_angle', 'min_speed_ratio'):
            if key in params:
                oa_params[key] = params[key]
        if oa_params:
            self.set_obstacle_avoidance_params(**oa_params)