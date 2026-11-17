# 这个是以米为单位辨识的模型
#纯追踪(Pure Pursuit) + PID
import casadi as ca
import numpy as np
import math
# import pandas as pd
import matplotlib.pyplot as plt

# ==================== 常量定义 ====================
M_PI = math.pi
N = 10           # 预测步长
T = 0.1          # 采样时间
motor_power = 500
Adaptive_flag = True
position_noise_std = 0.005   # 位置噪声标准差
w = 0.25
# 生成自定义椭圆轨迹
t_total = 10 * np.pi / w  # 计算一个周期的总时间 (wt = 2π)
num_points = int(t_total / T)  # 根据采样时间计算点的数量
t_values = np.linspace(0, t_total, num_points)  # 生成时间向量

x = 1.4 * np.sin(w * t_values) + 0.5  # 计算x坐标
y = np.cos(w * t_values) - 0.88     # 计算y坐标

path_points = np.column_stack((x, y)).tolist()  # 组合为路径点列表
sim_steps = len(path_points)  # 设置仿真步数
t = t_values  # 更新时间向量
# ==================== 权重参数 ====================
Q_low = [0.0005, 0.001, 0.001, 3, 3, 0.1]
Q_high = [0.0005, 0.001, 0.001, 1000, 1000, 0.1]
R_low = [0.003, 0.003]
R_high = [0.001, 0.001]
F_low = [0.001, 0.002, 0.002, 10, 10, 0.15]
F_high = [0.001, 0.002, 0.002, 5000, 5000, 0.15]

# ==================== 状态和控制变量定义 ====================
u = ca.SX.sym('u')
v = ca.SX.sym('v')
r = ca.SX.sym('r')
x_pos = ca.SX.sym('x')
y_pos = ca.SX.sym('y')
psi = ca.SX.sym('psi')
Tp = ca.SX.sym('Tp')
Ts = ca.SX.sym('Ts')

state = ca.vertcat(u, v, r, x_pos, y_pos, psi)
control = ca.vertcat(Tp, Ts)

# ==================== 动力学模型 ====================
rhs = ca.vertcat(
    # 2.388776 * v * r + -0.223360 * u + 0.031403 * (Tp + Ts) + 0.045647,
    # -0.275308 * u * r + -0.545202 * v + -0.003912,
    # 0.701675 * u * v + -0.689482 * r + -0.177370 * (Tp - Ts) + -0.015698,
    # 1.428555 * v * r + -0.296273 * u + -0.050424 * v + 0.048005 * r + 0.040889 * Tp + 0.020753 * Ts + -0.164785,
    # -0.642374 * u * r + -0.117058 * u + -1.051617 * v + 0.018381 * r + -0.008968 * Tp + 0.005212 * Ts + -0.012565,
    # -4.503596 * u * v + 1.871949 * u + 18.340192 * v + -0.692702 * r + -0.184732 * Tp + 0.149389 * Ts + -0.504434,
    -0.578486 * v * r + -0.590191 * u + -0.502736 * v + 0.080375 * r + 0.056161 * Tp + 0.041063 * Ts + -0.119302,
    -0.680488 * u * r + -0.134844 * u + -1.266535 * v + 0.021621 * r + -0.006550 * Tp + 0.005212 * Ts + -0.016607,
    -0.020507 * u * v + 0.046766 * u + 0.385833 * v + -0.780707 * r + -0.00672 * Tp + 0.00510 * Ts + -0.001752,
    u * ca.cos(psi) - v * ca.sin(psi),
    u * ca.sin(psi) + v * ca.cos(psi),
    r
)

f = ca.Function('f', [state, control], [rhs])

# ==================== PID控制器参数 ====================
# 航速PID参数
desired_speed = 0.25  # 期望前进速度 (m/s)
Kp_speed = 55.0     # 比例增益 - 增加以提高响应速度
Ki_speed = 0.1       # 积分增益 - 减少以避免积分饱和
Kd_speed = 0.0     # 微分增益 - 增加以减少振荡

# 航向PID参数
Kp_heading = 35.0   # 比例增益 - 增加以提高航向响应
Ki_heading = 0.1     # 积分增益 - 减少以避免积分饱和
Kd_heading = 0.5   # 微分增益 - 增加以减少航向振荡

# ==================== PID控制器类 ====================
class PIDController:
    def __init__(self, Kp, Ki, Kd, dt, output_limits=(-float('inf'), float('inf'))):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.output_limits = output_limits
        
        self.error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.prev_error = 0.0
    
    def reset(self):
        self.error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.prev_error = 0.0
    
    def compute(self, setpoint, measured_value):
        # 计算误差
        self.error = setpoint - measured_value
        
        # 计算积分项
        self.integral += self.error * self.dt
        
        # 计算微分项
        self.derivative = (self.error - self.prev_error) / self.dt
        self.prev_error = self.error
        
        # 计算PID输出
        output = self.Kp * self.error + self.Ki * self.integral + self.Kd * self.derivative
        
        # 限制输出范围
        output = max(self.output_limits[0], min(self.output_limits[1], output))
        
        return output

# ==================== LOS参数 ====================
lookahead_distance = 0.5  # 前向观测距离 - 增加以获得更平滑的航向指令
K_PP = 0.5  # 航点切换阈值

# ==================== 约束边界 ====================
# 状态约束 (保持原有约束)
speed_limit = 5.0       # 前进速度限制 (m/s)
sway_limit = 1.0        # 横漂速度限制 (m/s)
yaw_rate_limit = 0.2*M_PI  # 偏航角速度限制 (rad/s)

# 控制输入约束
motor_limit = motor_power  # 推进器输出限制

# ==================== 初始状态 ====================
x0 = [0, 0, 0, path_points[0][0] , path_points[0][1] , 0]
xs_list = [[0, 0, 0, point[0], point[1], 0] for point in path_points]
state_history = [x0]
control_history = []

# ==================== 初始化PID控制器 ====================
speed_pid = PIDController(Kp_speed, Ki_speed, Kd_speed, T, output_limits=(-motor_power, motor_power))
heading_pid = PIDController(Kp_heading, Ki_heading, Kd_heading, T, output_limits=(-motor_power, motor_power))

# ==================== 初始化误差记录 ====================
lateral_errors = []
heading_errors = []

# ==================== 辅助函数 ====================
def normalize_angle_diff(delta):
    return math.atan2(math.sin(delta), math.cos(delta))


def compute_PP_heading(current_pos, path_points, K_PP, current_index, current_psi):
    """Pure Pursuit 制导算法"""
    current_x, current_y = current_pos
    dx = path_points[current_index][0] - current_x
    dy = path_points[current_index][1] - current_y
    psi_d = np.arctan2(dy, dx)
    r_d = K_PP * (psi_d - current_psi)
    num_wpts = len(path_points)
    # 获取当前航段的两个航点
    xk, yk = path_points[current_index]
    xk1, yk1 = path_points[(current_index + 1) % num_wpts]
    
    # 计算航段向量
    path_vec = np.array([xk1 - xk, yk1 - yk])
    
    # 计算航段角度（北东坐标系）
    pi_h = math.atan2(path_vec[1], path_vec[0])
    
    # 计算沿航段和横向误差
    R = np.array([[math.cos(pi_h), math.sin(pi_h)],
                  [-math.sin(pi_h), math.cos(pi_h)]])
    along_track, cross_track = R @ np.array([current_x - xk, current_y - yk])
    
    return psi_d, r_d, cross_track, along_track


# ==================== LOS+PID 仿真 ====================
current_index = 0  # 初始轨迹点索引
for sim in range(sim_steps):
    current_x, current_y = x0[3], x0[4]
    current_u = x0[0]  # 当前前进速度
    current_psi = x0[5]  # 当前航向角
    
    # 使用用户提供的LOS算法计算期望航向角
    target_psi, r_d, cross_track, along_track = compute_PP_heading(
        (current_x, current_y), path_points, K_PP, current_index, current_psi
    )
    
    # 航点切换判断（在主循环中维护index）
    current_index += 1
    
    # 记录横向误差（使用LOS算法计算的cross_track）
    lateral_error = abs(cross_track)
    lateral_errors.append(lateral_error)
    
    # 计算航向误差
    heading_error = normalize_angle_diff(target_psi - current_psi)
    heading_errors.append(abs(heading_error))
    
    # 使用PID控制器计算控制输入
    # 航速PID控制：计算总推进力（控制前进速度）
    if current_index < len(path_points) - 1:
        Symbol = np.sign(np.cos(heading_error))
        desired_speed =  Symbol * np.linalg.norm(np.array(path_points[current_index]) - np.array(path_points[current_index-1])) / T
    else:
        desired_speed = desired_speed
    total_thrust = speed_pid.compute(desired_speed, current_u)
    
    # 航向PID控制：计算左右推进器的差值（控制航向）
    thrust_diff = heading_pid.compute(0, heading_error)  # 期望航向误差为0
    
    # 计算左右推进器输出
    # Tp: 左推进器, Ts: 右推进器
    # 总推进力 = Tp + Ts
    # 推力差 = Tp - Ts
    Tp = 0.5 * (total_thrust + thrust_diff)
    Ts = 0.5 * (total_thrust - thrust_diff)
    
    # 限制推进器输出在允许范围内
    Tp = max(-motor_limit, min(motor_limit, Tp))
    Ts = max(-motor_limit, min(motor_limit, Ts))
    
    u0_step = [Tp, Ts]
    
    # 使用动力学模型计算下一个状态
    next_state = f(x0, u0_step)
    x_next = x0 + T * next_state.full().flatten()
    
    # 应用状态约束
    x_next[0] = max(-speed_limit, min(speed_limit, x_next[0]))  # 前进速度约束
    x_next[1] = max(-sway_limit, min(sway_limit, x_next[1]))    # 横漂速度约束
    x_next[2] = max(-yaw_rate_limit, min(yaw_rate_limit, x_next[2]))  # 偏航角速度约束
    
    # 加入位置白噪声
    noise = np.random.normal(0, position_noise_std, size=2)
    x_next[3] += noise[0]
    x_next[4] += noise[1]
    
    # 更新状态
    x0 = x_next.tolist()
    control_history.append(u0_step)
    state_history.append(x0)

# ==================== 提取轨迹用于绘图 ====================
traj_u =   [s[0] for s in state_history][1:]
traj_v =   [s[1] for s in state_history][1:]
traj_r =   [s[2] for s in state_history][1:]
traj_x =   [s[3] for s in state_history][1:]
traj_y =   [s[4] for s in state_history][1:]
traj_psi = [s[5] for s in state_history][1:]
ref_x = [p[0] for p in path_points]
ref_y = [p[1] for p in path_points]
PWML = [u[0] for u in control_history]
PWMR = [u[1] for u in control_history]
# ==================== 输出统计数据 ====================
print(f"Lateral Error: Mean={np.mean(lateral_errors):.2f}, Std={np.std(lateral_errors):.2f}")
print(f"Heading Error: Mean={np.mean(heading_errors):.2f} rad, Std={np.std(heading_errors):.2f} rad")
# ==================== 绘图 ====================
plt.figure(figsize=(10, 6))
plt.plot(ref_x, ref_y, 'r--', label='Reference Path (LosXF/YF)')
plt.plot(traj_x, traj_y, 'b-', label='LOS+PID Trajectory')
plt.scatter([traj_x[0]], [traj_y[0]], c='g', label='Start')
plt.scatter([traj_x[-1]], [traj_y[-1]], c='orange', label='End')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('LOS+PID Path Tracking vs Reference Path')
plt.legend()
plt.grid(True)
plt.axis('equal')
# plt.show()

# ==================== 误差绘图 ====================
plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(lateral_errors, label='Lateral Tracking Error', color='b')
plt.axhline(y=np.mean(lateral_errors), color='b', linestyle='--', label=f'Mean Lateral Error: {np.mean(lateral_errors):.2f}')
plt.subplot(2, 1, 2)
plt.plot(heading_errors, label='Heading Angle Error (rad)', color='r')
plt.axhline(y=np.mean(heading_errors), color='r', linestyle='--', label=f'Mean Heading Error: {np.mean(heading_errors):.2f} rad')
plt.xlabel('Time Step')
plt.ylabel('Error')
plt.title('Lateral and Heading Errors During LOS+PID Path Tracking')
plt.legend()
plt.grid(True)

# 可视化状态变量
plt.figure(figsize=(12, 6))
plt.subplot(4, 1, 1)
plt.plot(t[:-1], traj_u[:-1], label='u')
# plt.plot(t[:-1], u_est[:-1], '--', label='Estimated u')
plt.legend()
# plt.title('u: True vs Estimated')

plt.subplot(4, 1, 2)
plt.plot(t[:-1], traj_v[:-1], label='v')
# plt.plot(t[:-1], v_est[:-1], '--', label='Estimated v')
plt.legend()
# plt.title('v: True vs Estimated')

plt.subplot(4, 1, 3)
plt.plot(t[:-1], traj_r[:-1], label=' r')
# plt.plot(t[:-1], r_est[:-1], '--', label='Estimated r')
plt.legend()
# plt.title('r: True vs Estimated')

plt.subplot(4, 1, 4)
plt.plot(t[:-1], traj_psi[:-1], label='True psi')
# plt.plot(t[:-1], psi_est[:-1], '--', label='Estimated psi')
plt.legend()
# plt.title('psi: True vs Estimated')
plt.tight_layout()

# 新增：7. 推进器输出差值对比 (plt.figure(3))
plt.figure(figsize=(12, 6))
# Ts（右推进器）
plt.subplot(2, 1, 1)
plt.plot(t, PWML, label=' Propeller rotational speed (Ts)', color='blue')
plt.legend(loc='upper center')
plt.title('Propeller Outputs')
plt.xlabel('Time (s)')
plt.ylabel('PWM Value (us)')
plt.grid(True)

# Tp（左推进器）
plt.subplot(2, 1, 2)
plt.plot(t, PWMR, label='Rudder angle (Tp)', color='blue')
plt.legend(loc='upper center')
plt.xlabel('Time (s)')
plt.ylabel('PWM Value (us)')
plt.grid(True)

plt.tight_layout()
plt.show()

# # ==================== 保存为CSV ====================
# import os

# output_data = {
#     'X': traj_x,
#     'Y': traj_y,
#     'LosXF': ref_x,
#     'LosYF': ref_y,
#     'PWML': PWML,
#     'PWMR': PWMR
# }

# df_output = pd.DataFrame(output_data)
# output_path = os.path.join(os.path.dirname(csv_file_path), 's0621atnmpc.csv')
# df_output.to_csv(output_path, index=False)

# print(f"数据已保存至: {output_path}")