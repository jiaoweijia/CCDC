# 反步法轨迹跟踪USV - 改进版（解决字体乱码和控制效果问题）
#反步法控制
import casadi as ca
import numpy as np
import math
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.font_manager import FontProperties
import platform

# ==================== 解决中文字体乱码 ====================
# 设置中文字体
# 解决负号显示问题
plt.rcParams['axes.unicode_minus'] = False  # 关键：解决U+2212错误

# 根据操作系统设置中文字体
system = platform.system()
if system == 'Windows':
    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Arial']
elif system == 'Darwin':  # macOS
    plt.rcParams['font.sans-serif'] = ['Arial Unicode MS', 'Heiti TC', 'PingFang SC']
else:  # Linux
    plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'DejaVu Sans']

# 确保负号正确显示
plt.rcParams['axes.unicode_minus'] = False

# ==================== 常量定义 ====================
M_PI = math.pi
T = 0.1          # 采样时间
motor_power = 500
position_noise_std = 0.005   # 位置噪声标准差

# 生成自定义椭圆轨迹
t_total = 2 * np.pi / 0.15
num_points = int(t_total / T)
t_values = np.linspace(0, t_total, num_points)

x = 2 * np.sin(0.15 * t_values) + 0.5
y = np.cos(0.15 * t_values) - 0.88

path_points = np.column_stack((x, y)).tolist()
sim_steps = len(path_points)
t = t_values

# ==================== 反步法控制器参数（优化后）====================
# 经过调优的控制增益，提高跟踪性能
k1 = 4.0      # 增加位置响应
k2 = 10.0     # 增加速度响应
k3 = 6.0      # ⚠️ 关键：增加航向增益，减小航向误差
k4 = 8.0      # 增加角速度阻尼

# 前馈增益
k_ff_x = 0.8   # x方向前馈
k_ff_y = 0.8   # y方向前馈

# 控制器约束
max_Tp = 300
min_Tp = -300
max_Ts = 300
min_Ts = -300

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
    1.428555 * v * r + -0.296273 * u + -0.050424 * v + 0.048005 * r + 0.040889 * Tp + 0.020753 * Ts + 0.064785,
    -0.642374 * u * r + -0.117058 * u + -1.051617 * v + 0.018381 * r + -0.008968 * Tp + 0.005212 * Ts + 0.012565,
    -4.503596 * u * v + 1.871949 * u + 18.340192 * v + -0.692702 * r + -0.184732 * Tp + 0.149389 * Ts + -0.504434,
    u * ca.cos(psi) - v * ca.sin(psi),
    u * ca.sin(psi) + v * ca.cos(psi),
    r
)

f = ca.Function('f', [state, control], [rhs])

# ==================== 改进的反步法控制器类 ====================
class ImprovedBacksteppingController:
    def __init__(self, k1, k2, k3, k4, k_ff_x=0.5, k_ff_y=0.5):
        """
        改进的反步法控制器
        
        参数:
            k1, k2: 位置和速度跟踪增益
            k3, k4: 航向和偏航率跟踪增益
            k_ff_x, k_ff_y: 前馈增益
        """
        self.k1 = k1
        self.k2 = k2
        self.k3 = k3
        self.k4 = k4
        self.k_ff_x = k_ff_x
        self.k_ff_y = k_ff_y
        
        # 误差积分项（用于消除稳态误差）
        self.integral_xe = 0
        self.integral_ye = 0
        self.integral_psi_e = 0
        self.integral_limit = 5.0  # 积分限幅
        
    def reset_integrals(self):
        """重置积分项"""
        self.integral_xe = 0
        self.integral_ye = 0
        self.integral_psi_e = 0
        
    def compute_control(self, state, ref_state, ref_dot, dt=T):
        """
        计算控制输入
        
        参数:
            state: 当前状态 [u, v, r, x, y, psi]
            ref_state: 参考状态 [u_r, v_r, r_r, x_r, y_r, psi_r]
            ref_dot: 参考状态导数
            dt: 时间步长
        """
        # 提取当前状态
        u_curr = state[0]
        v_curr = state[1]
        r_curr = state[2]
        x_curr = state[3]
        y_curr = state[4]
        psi_curr = state[5]
        
        # 提取参考状态
        x_ref = ref_state[3]
        y_ref = ref_state[4]
        psi_ref = ref_state[5]
        u_ref = max(0.2, ref_state[0])  # 确保参考速度为正
        
        # 提取参考状态导数
        x_ref_dot = ref_dot[3]
        y_ref_dot = ref_dot[4]
        psi_ref_dot = ref_dot[5]
        u_ref_dot = ref_dot[0]
        
        # 计算位置误差
        xe = x_curr - x_ref
        ye = y_curr - y_ref
        
        # 积分项更新
        self.integral_xe += xe * dt
        self.integral_ye += ye * dt
        self.integral_xe = np.clip(self.integral_xe, -self.integral_limit, self.integral_limit)
        self.integral_ye = np.clip(self.integral_ye, -self.integral_limit, self.integral_limit)
        
        # 带积分项的位置误差
        xe_control = xe + 0.1 * self.integral_xe
        ye_control = ye + 0.1 * self.integral_ye
        
        # 将位置误差转换到船体坐标系
        xe_body = xe_control * np.cos(psi_curr) + ye_control * np.sin(psi_curr)
        ye_body = -xe_control * np.sin(psi_curr) + ye_control * np.cos(psi_curr)
        
        # 计算期望航向角（改进的LOS制导）
        # 使用atan2避免奇异点，并加入前向距离
        look_ahead = 1.5  # 前视距离
        if abs(xe_body) > 0.01:
            psi_desired = -np.arctan2(ye_body, look_ahead)
        else:
            psi_desired = -np.arctan2(ye_body, look_ahead) if abs(ye_body) > 0.01 else 0
        
        # 航向误差（规范化）
        psi_e = psi_curr - psi_ref - psi_desired
        psi_e = np.arctan2(np.sin(psi_e), np.cos(psi_e))
        
        # 航向误差积分
        self.integral_psi_e += psi_e * dt
        self.integral_psi_e = np.clip(self.integral_psi_e, -self.integral_limit, self.integral_limit)
        psi_e_control = psi_e + 0.05 * self.integral_psi_e
        
        # 计算期望偏航率
        r_desired = psi_ref_dot - self.k3 * psi_e_control
        
        # 偏航率误差
        r_e = r_curr - r_desired
        
        # 计算纵向速度误差
        u_e = u_curr - u_ref
        
        # 计算动力学非线性项
        f_u = (1.428555 * v_curr * r_curr - 0.296273 * u_curr - 0.050424 * v_curr 
               + 0.048005 * r_curr + 0.064785)
        
        f_r = (-4.503596 * u_curr * v_curr + 1.871949 * u_curr + 18.340192 * v_curr 
               - 0.692702 * r_curr - 0.504434)
        
        # 控制矩阵系数
        g_u1 = 0.040889
        g_u2 = 0.020753
        g_r1 = -0.184732
        g_r2 = 0.149389
        
        # 前馈控制项
        ff_x = self.k_ff_x * x_ref_dot
        ff_y = self.k_ff_y * y_ref_dot
        
        # 虚拟控制输入（改进的控制律）
        tau_u = -f_u + u_ref_dot - self.k2 * u_e - (v_curr * r_curr) + ff_x
        
        # 偏航控制律
        tau_r = (-f_r + psi_ref_dot - self.k4 * r_e 
                 - (u_curr * v_curr) - 0.5 * psi_e_control) + ff_y
        
        # 解算实际控制输入
        det = g_u1 * g_r2 - g_u2 * g_r1
        
        if abs(det) > 1e-6:
            Tp = (tau_u * g_r2 - g_u2 * tau_r) / det
            Ts = (g_u1 * tau_r - tau_u * g_r1) / det
        else:
            # 伪逆解
            Tp = (g_u1 * tau_u + g_r1 * tau_r) / (g_u1**2 + g_r1**2 + 1e-6)
            Ts = (g_u2 * tau_u + g_r2 * tau_r) / (g_u2**2 + g_r2**2 + 1e-6)
        
        # 应用饱和约束
        Tp = np.clip(Tp, min_Tp, max_Tp)
        Ts = np.clip(Ts, min_Ts, max_Ts)
        
        return Tp, Ts

def compute_reference_states(path_points, t, sim_index, dt=T):
    """计算参考状态和参考状态导数（改进版）"""
    n_points = len(path_points)
    
    # 使用插值获得更平滑的参考轨迹
    if sim_index >= n_points - 1:
        sim_index = n_points - 2
    
    # 当前位置作为参考位置
    x_ref = path_points[sim_index][0]
    y_ref = path_points[sim_index][1]
    
    # 计算参考速度（基于路径弧长）
    if sim_index > 0:
        dx = x_ref - path_points[sim_index-1][0]
        dy = y_ref - path_points[sim_index-1][1]
        path_length = np.sqrt(dx**2 + dy**2)
        u_ref = path_length / dt if dt > 0 else 0.5
        u_ref = np.clip(u_ref, 0.2, 1.0)  # 限制速度范围
    else:
        u_ref = 0.5
    
    # 计算参考航向角（基于路径切线）
    if sim_index < n_points - 1:
        dx_next = path_points[sim_index+1][0] - x_ref
        dy_next = path_points[sim_index+1][1] - y_ref
        psi_ref = math.atan2(dy_next, dx_next)
    elif sim_index > 0:
        dx_prev = x_ref - path_points[sim_index-1][0]
        dy_prev = y_ref - path_points[sim_index-1][1]
        psi_ref = math.atan2(dy_prev, dx_prev)
    else:
        psi_ref = 0
    
    # 计算参考状态导数
    if 0 < sim_index < n_points - 1:
        x_ref_dot = (path_points[sim_index+1][0] - path_points[sim_index-1][0]) / (2 * dt)
        y_ref_dot = (path_points[sim_index+1][1] - path_points[sim_index-1][1]) / (2 * dt)
        psi_ref_dot = 0  # 航向角变化率
    else:
        x_ref_dot = 0
        y_ref_dot = 0
        psi_ref_dot = 0
    
    u_ref_dot = 0
    v_ref = 0
    r_ref = 0
    
    ref_state = [u_ref, v_ref, r_ref, x_ref, y_ref, psi_ref]
    ref_dot = [u_ref_dot, 0, 0, x_ref_dot, y_ref_dot, psi_ref_dot]
    
    return ref_state, ref_dot

def normalize_angle(angle):
    """规范化角度到[-pi, pi]"""
    return math.atan2(math.sin(angle), math.cos(angle))

# ==================== 仿真初始化 ====================
x0 = [0, 0, 0, path_points[0][0] - 0.1, path_points[0][1] - 0.1, 0]
state_history = [x0]
control_history = []
lateral_errors = []
heading_errors = []
position_errors = []

# 创建改进的反步法控制器
controller = ImprovedBacksteppingController(k1, k2, k3, k4, k_ff_x, k_ff_y)

# ==================== 反步法仿真 ====================
print("开始改进的反步法轨迹跟踪仿真...")
print(f"控制器参数: k1={k1}, k2={k2}, k3={k3}, k4={k4}")

for sim in range(sim_steps):
    current_state = x0
    
    # 计算参考状态和参考导数
    ref_state, ref_dot = compute_reference_states(path_points, t, sim, T)
    
    # 计算控制输入
    Tp, Ts = controller.compute_control(current_state, ref_state, ref_dot, T)
    
    # 保存控制输入
    control_history.append([Tp, Ts])
    
    # 使用动力学模型更新状态
    next_state = f(current_state, [Tp, Ts])
    x_next = current_state + T * next_state.full().flatten()
    
    # 添加过程噪声
    noise = np.random.normal(0, position_noise_std, size=2)
    x_next[3] += noise[0]
    x_next[4] += noise[1]
    
    # 规范化航向角
    x_next[5] = normalize_angle(x_next[5])
    
    # 更新状态
    x0 = x_next.tolist()
    state_history.append(x0)
    
    # 计算误差
    current_pos = (x0[3], x0[4])
    ref_pos = (ref_state[3], ref_state[4])
    
    # 位置误差
    pos_error = math.sqrt((current_pos[0] - ref_pos[0])**2 + (current_pos[1] - ref_pos[1])**2)
    position_errors.append(pos_error)
    
    # 横向误差（相对于参考航向）
    dx = current_pos[0] - ref_pos[0]
    dy = current_pos[1] - ref_pos[1]
    lateral_error = abs(-dx * np.sin(ref_state[5]) + dy * np.cos(ref_state[5]))
    lateral_errors.append(lateral_error)
    
    # 航向误差
    heading_error = abs(normalize_angle(x0[5] - ref_state[5]))
    heading_errors.append(heading_error)

# ==================== 提取轨迹数据 ====================
traj_u = [s[0] for s in state_history][1:]
traj_v = [s[1] for s in state_history][1:]
traj_r = [s[2] for s in state_history][1:]
traj_x = [s[3] for s in state_history][1:]
traj_y = [s[4] for s in state_history][1:]
traj_psi = [s[5] for s in state_history][1:]
ref_x = [p[0] for p in path_points]
ref_y = [p[1] for p in path_points]
PWML = [c[0] for c in control_history]
PWMR = [c[1] for c in control_history]

# ==================== 输出统计数据 ====================
print("\n" + "="*60)
print("改进的反步法轨迹跟踪仿真结果")
print("="*60)
print(f"位置误差: 均值={np.mean(position_errors):.4f}m, 标准差={np.std(position_errors):.4f}m")
print(f"横向误差: 均值={np.mean(lateral_errors):.4f}m, 标准差={np.std(lateral_errors):.4f}m")
print(f"航向误差: 均值={np.mean(heading_errors):.4f}rad, 标准差={np.std(heading_errors):.4f}rad")
print(f"最大位置误差: {np.max(position_errors):.4f}m")
print(f"最大横向误差: {np.max(lateral_errors):.4f}m")
print(f"最大航向误差: {np.max(heading_errors):.4f}rad")
print(f"均方根位置误差: {np.sqrt(np.mean(np.square(position_errors))):.4f}m")

# ==================== 绘图（使用英文标签避免乱码）====================
fig = plt.figure(figsize=(14, 10))

# 1. 轨迹跟踪图
ax1 = fig.add_subplot(2, 2, 1)
ax1.plot(ref_x, ref_y, 'r--', linewidth=2, label='Reference Path')
ax1.plot(traj_x, traj_y, 'b-', linewidth=1.5, label='Backstepping Trajectory')
ax1.scatter([traj_x[0]], [traj_y[0]], c='g', s=80, label='Start', zorder=5, marker='o')
ax1.scatter([traj_x[-1]], [traj_y[-1]], c='orange', s=80, label='End', zorder=5, marker='s')
ax1.set_xlabel('X Position (m)')
ax1.set_ylabel('Y Position (m)')
ax1.set_title('Trajectory Tracking Comparison')
ax1.legend(loc='best')
ax1.grid(True, alpha=0.3)
ax1.axis('equal')

# 2. 位置误差
ax2 = fig.add_subplot(2, 2, 2)
ax2.plot(position_errors, 'b-', linewidth=1.5, label='Position Error')
ax2.axhline(y=np.mean(position_errors), color='r', linestyle='--', 
            label=f'Mean: {np.mean(position_errors):.4f}m')
ax2.fill_between(range(len(position_errors)), 0, position_errors, alpha=0.3)
ax2.set_xlabel('Time Step')
ax2.set_ylabel('Position Error (m)')
ax2.set_title('Position Tracking Error')
ax2.legend()
ax2.grid(True, alpha=0.3)

# 3. 横向误差
ax3 = fig.add_subplot(2, 2, 3)
ax3.plot(lateral_errors, 'g-', linewidth=1.5, label='Lateral Error')
ax3.axhline(y=np.mean(lateral_errors), color='r', linestyle='--', 
            label=f'Mean: {np.mean(lateral_errors):.4f}m')
ax3.fill_between(range(len(lateral_errors)), 0, lateral_errors, alpha=0.3)
ax3.set_xlabel('Time Step')
ax3.set_ylabel('Lateral Error (m)')
ax3.set_title('Lateral Tracking Error')
ax3.legend()
ax3.grid(True, alpha=0.3)

# 4. 航向误差
ax4 = fig.add_subplot(2, 2, 4)
ax4.plot(heading_errors, 'm-', linewidth=1.5, label='Heading Error')
ax4.axhline(y=np.mean(heading_errors), color='r', linestyle='--', 
            label=f'Mean: {np.mean(heading_errors):.4f}rad')
ax4.fill_between(range(len(heading_errors)), 0, heading_errors, alpha=0.3)
ax4.set_xlabel('Time Step')
ax4.set_ylabel('Heading Error (rad)')
ax4.set_title('Heading Tracking Error')
ax4.legend()
ax4.grid(True, alpha=0.3)

plt.tight_layout()

# 第二组图：速度和状态
fig2 = plt.figure(figsize=(14, 10))

# 纵向速度
ax21 = fig2.add_subplot(3, 2, 1)
ax21.plot(traj_u, 'b-', linewidth=1.5, label='Surge velocity u')
ax21.set_ylabel('u (m/s)')
ax21.set_title('Surge Velocity')
ax21.legend()
ax21.grid(True, alpha=0.3)

# 横向速度
ax22 = fig2.add_subplot(3, 2, 2)
ax22.plot(traj_v, 'r-', linewidth=1.5, label='Sway velocity v')
ax22.set_ylabel('v (m/s)')
ax22.set_title('Sway Velocity')
ax22.legend()
ax22.grid(True, alpha=0.3)

# 偏航率
ax23 = fig2.add_subplot(3, 2, 3)
ax23.plot(traj_r, 'g-', linewidth=1.5, label='Yaw rate r')
ax23.set_ylabel('r (rad/s)')
ax23.set_title('Yaw Rate')
ax23.legend()
ax23.grid(True, alpha=0.3)

# 航向角
ax24 = fig2.add_subplot(3, 2, 4)
ax24.plot(traj_psi, 'm-', linewidth=1.5, label='Heading psi')
ax24.set_ylabel('psi (rad)')
ax24.set_title('Heading Angle')
ax24.legend()
ax24.grid(True, alpha=0.3)

# 左推进器控制
ax25 = fig2.add_subplot(3, 2, 5)
ax25.plot(PWML, 'orange', linewidth=1.5, label='Left Thruster Tp')
ax25.set_xlabel('Time Step')
ax25.set_ylabel('Tp')
ax25.set_title('Left Thruster Control')
ax25.legend()
ax25.grid(True, alpha=0.3)

# 右推进器控制
ax26 = fig2.add_subplot(3, 2, 6)
ax26.plot(PWMR, 'purple', linewidth=1.5, label='Right Thruster Ts')
ax26.set_xlabel('Time Step')
ax26.set_ylabel('Ts')
ax26.set_title('Right Thruster Control')
ax26.legend()
ax26.grid(True, alpha=0.3)

plt.tight_layout()

# 第三组图：误差分布直方图
fig3 = plt.figure(figsize=(14, 5))

ax31 = fig3.add_subplot(1, 3, 1)
ax31.hist(position_errors, bins=30, edgecolor='black', alpha=0.7, color='blue')
ax31.set_xlabel('Position Error (m)')
ax31.set_ylabel('Frequency')
ax31.set_title(f'Position Error Distribution\nMean={np.mean(position_errors):.4f}m')
ax31.axvline(x=np.mean(position_errors), color='r', linestyle='--', label='Mean')
ax31.legend()
ax31.grid(True, alpha=0.3)

ax32 = fig3.add_subplot(1, 3, 2)
ax32.hist(lateral_errors, bins=30, edgecolor='black', alpha=0.7, color='green')
ax32.set_xlabel('Lateral Error (m)')
ax32.set_ylabel('Frequency')
ax32.set_title(f'Lateral Error Distribution\nMean={np.mean(lateral_errors):.4f}m')
ax32.axvline(x=np.mean(lateral_errors), color='r', linestyle='--', label='Mean')
ax32.legend()
ax32.grid(True, alpha=0.3)

ax33 = fig3.add_subplot(1, 3, 3)
ax33.hist(heading_errors, bins=30, edgecolor='black', alpha=0.7, color='red')
ax33.set_xlabel('Heading Error (rad)')
ax33.set_ylabel('Frequency')
ax33.set_title(f'Heading Error Distribution\nMean={np.mean(heading_errors):.4f}rad')
ax33.axvline(x=np.mean(heading_errors), color='b', linestyle='--', label='Mean')
ax33.legend()
ax33.grid(True, alpha=0.3)

plt.tight_layout()

# 第四组图：误差收敛曲线
fig4 = plt.figure(figsize=(12, 5))

ax41 = fig4.add_subplot(1, 2, 1)
# 绘制对数尺度误差收敛曲线
ax41.semilogy(position_errors, 'b-', linewidth=1.5, label='Position Error')
ax41.semilogy(lateral_errors, 'g--', linewidth=1.5, label='Lateral Error')
ax41.set_xlabel('Time Step')
ax41.set_ylabel('Error (m) - Log Scale')
ax41.set_title('Error Convergence (Log Scale)')
ax41.legend()
ax41.grid(True, alpha=0.3)

ax42 = fig4.add_subplot(1, 2, 2)
ax42.plot(heading_errors, 'r-', linewidth=1.5, label='Heading Error')
ax42.set_xlabel('Time Step')
ax42.set_ylabel('Heading Error (rad)')
ax42.set_title('Heading Error Convergence')
ax42.legend()
ax42.grid(True, alpha=0.3)

plt.tight_layout()

# 保存数据到CSV
output_data = {
    'Time': t[:-1] if len(t) > len(traj_x) else t,
    'X_Position': traj_x,
    'Y_Position': traj_y,
    'Ref_X': ref_x,
    'Ref_Y': ref_y,
    'Position_Error': position_errors,
    'Lateral_Error': lateral_errors,
    'Heading_Error': heading_errors,
    'Tp': PWML,
    'Ts': PWMR,
    'u': traj_u,
    'v': traj_v,
    'r': traj_r,
    'psi': traj_psi
}

# 确保所有数组长度一致
min_len = min(len(traj_x), len(ref_x), len(position_errors))
for key in output_data:
    if len(output_data[key]) > min_len:
        output_data[key] = output_data[key][:min_len]

df_output = pd.DataFrame(output_data)
# df_output.to_csv('backstepping_tracking_results.csv', index=False)

plt.show()

print("\n仿真完成！")
print(f"数据已保存，共 {min_len} 个采样点")