# 这个是以米为单位辨识的模型
# LOS 制导 + 非线性模型预测控制，单船航点跟踪
# LOS + NMPC 变体
#LOS 制导 + 非线性模型预测控制，单船航点跟踪
import casadi as ca
import numpy as np
import math
import pandas as pd
import matplotlib.pyplot as plt

# ==================== 常量定义 ====================
M_PI = math.pi
N = 15           # 增加预测步长，提高预测能力
T = 0.1          # 采样时间
motor_power = 500
Adaptive_flag = True
position_noise_std = 0.005   # 位置噪声标准差

# 生成自定义椭圆轨迹
t_total = 2 * np.pi / 0.15  # 计算一个周期的总时间 (0.15t = 2π)
num_points = int(t_total / T)  # 根据采样时间计算点的数量
t_values = np.linspace(0, t_total, num_points)  # 生成时间向量

x = 2 * np.sin(0.15 * t_values) + 0.5  # 计算x坐标
y = np.cos(0.15 * t_values) - 0.88     # 计算y坐标

path_points = np.column_stack((x, y)).tolist()  # 组合为路径点列表
sim_steps = len(path_points)  # 设置仿真步数
t = t_values  # 更新时间向量
# ==================== 权重参数 ====================
# 优化后的权重参数，提高位置跟踪精度
Q_low = [0.0005, 0.001, 0.001, 100, 100, 0.1]  # 增加位置和航向误差权重
Q_high = [0.0005, 0.001, 0.001, 1000, 1000, 0.2]  # 进一步增加位置和航向误差权重
R_low = [0.05, 0.05]  # 适当增加控制输入权重，避免控制过于激进
R_high = [0.002, 0.002]
F_low = [0.001, 0.002, 0.002, 100, 100, 0.1]  # 增加状态约束权重
F_high = [0.001, 0.002, 0.002, 5000, 5000, 0.5]

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
    1.428555 * v * r + -0.296273 * u + -0.050424 * v + 0.048005 * r + 0.040889 * Tp + 0.020753 * Ts + 0.064785,
    -0.642374 * u * r + -0.117058 * u + -1.051617 * v + 0.018381 * r + -0.008968 * Tp + 0.005212 * Ts + 0.012565,
    -4.503596 * u * v + 1.871949 * u + 18.340192 * v + -0.692702 * r + -0.184732 * Tp + 0.149389 * Ts + -0.504434,
    u * ca.cos(psi) - v * ca.sin(psi),
    u * ca.sin(psi) + v * ca.cos(psi),
    r
)

f = ca.Function('f', [state, control], [rhs])

# ==================== 初始状态 ====================
x0 = [0, 0, 0, path_points[0][0] - 0.1, path_points[0][1] - 0.1, 0]
xs_list = [[0, 0, 0, point[0], point[1], 0] for point in path_points]
state_history = [x0]
u0 = np.zeros(2 * N).tolist()
control_history = []

# ==================== 构建NLP问题 ====================
U = ca.SX.sym('U', 2, N)
P_state = ca.SX.sym('P_state', 12)
P_weight = ca.SX.sym('P_weight', 14)
P = ca.vertcat(P_state, P_weight)

X = ca.SX.sym('X', 6, N + 1)
X[:, 0] = P_state[0:6]

for k in range(N):
    X[:, k + 1] = X[:, k] + T * f(X[:, k], U[:, k])

obj = 0
Qk = ca.diagcat(P_weight[0], P_weight[1], P_weight[2], P_weight[3], P_weight[4], P_weight[5])
Rk = ca.diagcat(P_weight[6], P_weight[7])
Qf = ca.diagcat(P_weight[8], P_weight[9], P_weight[10], P_weight[11], P_weight[12], P_weight[13])

for k in range(N):
    st_err = X[:, k] - P_state[6:12]
    con = U[:, k]
    obj += st_err.T @ Qk @ st_err + con.T @ Rk @ con

obj += (X[:, N] - P_state[6:12]).T @ Qf @ (X[:, N] - P_state[6:12])

g = []
for k in range(N + 1):
    g.append(X[0, k])
    g.append(X[1, k])
    g.append(X[2, k])
g = ca.vertcat(*g)

OPT_variables = ca.reshape(U, 2 * N, 1)
nlp = {'x': OPT_variables, 'f': obj, 'g': g, 'p': P}

opts = {
    'ipopt.print_level': 0,
    'print_time': 0,
    'ipopt.max_iter': 50,      # 增加最大迭代次数，提高求解质量
    'ipopt.acceptable_tol': 1e-7,  # 减小可接受容差
    'ipopt.tol': 1e-8,            # 减小优化容差
    'ipopt.constr_viol_tol': 1e-6,  # 约束违反容差
    'ipopt.hessian_approximation': 'exact'  # 使用精确Hessian近似
}
solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

# ==================== 约束边界 ====================
lbx = [-motor_power] * (2 * N)
ubx = [motor_power] * (2 * N)
lbg = [-5, -2, -0.3*M_PI] * (N+1)
ubg = [5, 2, 0.3*M_PI] * (N+1)

# ==================== 初始化误差记录 ====================
lateral_errors = []
heading_errors = []

# ==================== LOS参数 ====================
look_ahead = 0.3  # 前向观测距离
R_switch = 0.25  # 航点切换阈值

# ==================== 辅助函数 ====================
def normalize_angle_diff(delta):
    return math.atan2(math.sin(delta), math.cos(delta))



def compute_los_heading(current_pos, path_points, current_index, speed):
    """实现用户提供的LOS制导算法"""
    current_x, current_y = current_pos
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
    
    # LOS期望航向计算(论文式5.103)
    psi_d = pi_h - math.atan(cross_track / look_ahead) - 0  # 不考虑蟹角
    
    # 期望转艏速率(通过误差导数)
    dy_e = -speed * cross_track / math.sqrt(look_ahead**2 + cross_track**2)
    r_d = -dy_e / (look_ahead * (1 + (cross_track/look_ahead)**2))
    
    # 返回期望航向、目标位置（使用当前航段的终点作为目标点）
    return psi_d, (xk1, yk1), cross_track, along_track

def compute_lateral_error(current_pos, path_points, current_index):
    """计算横向误差"""
    current_x, current_y = current_pos
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
    
    return abs(cross_track)

def compute_target_psi(xs_list, sim_index):
    if sim_index >= len(xs_list) - 1:
        return xs_list[sim_index][5]
    current_x = xs_list[sim_index][3]
    current_y = xs_list[sim_index][4]
    next_x = xs_list[sim_index + 1][3]
    next_y = xs_list[sim_index + 1][4]
    return math.atan2(next_y - current_y, next_x - current_x)

# ==================== NMPC 仿真 ====================
current_index = 0  # 初始轨迹点索引
for sim in range(sim_steps):
    current_x, current_y = x0[3], x0[4]
    current_psi = x0[5]
    current_speed = x0[0]  # 当前前进速度
    
    # 使用用户提供的LOS算法计算期望航向角和目标位置
    target_psi, (target_x, target_y), cross_track, along_track = compute_los_heading(
        (current_x, current_y), path_points, current_index, current_speed
    )
    
    # 航点切换判断（在主循环中维护index）
    current_index += 1
    
    # 计算横向误差
    lateral_error = abs(cross_track)  # 直接使用LOS算法计算的cross_track
    lateral_errors.append(lateral_error)
    
    # 更新参考状态（使用LOS目标点）
    xs = [0, 0, 0, target_x, target_y, target_psi]

    delta_psi = normalize_angle_diff(current_psi - target_psi)
    heading_errors.append(abs(delta_psi))

    # 计算到LOS目标点的距离误差
    distance_error = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
    
    distance_coeff = max(0, min(1.0, distance_error / 2.0))
    if Adaptive_flag:
        Q_current = Q_low
        R_current = R_low
        F_current = [F_low[i] + distance_coeff * (F_high[i] - F_low[i]) for i in range(6)]
    else:
        Q_current = Q_low
        R_current = R_low
        F_current = F_low

    p_state = x0 + xs
    p_weight = Q_current + R_current + F_current
    p = p_state + p_weight

    res = solver(x0=u0, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg, p=p)

    u_opt = res['x'].full().flatten().tolist()
    u0_step = u_opt[:2]

    next_state = f(x0, u0_step)
    x_next = x0 + T * next_state.full().flatten()

    # 加入白噪声
    noise = np.random.normal(0, position_noise_std, size=2)
    x_next[3] += noise[0]
    x_next[4] += noise[1]
    x0 = x_next.tolist()
    control_history.append(u0_step)
    state_history.append(x0)
    u0 = u_opt[2:] + u_opt[-2:]

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
plt.plot(traj_x, traj_y, 'b-', label='NMPC Trajectory')
plt.scatter([traj_x[0]], [traj_y[0]], c='g', label='Start')
plt.scatter([traj_x[-1]], [traj_y[-1]], c='orange', label='End')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('NMPC Path Tracking vs Reference Path')
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
plt.title('Lateral and Heading Errors During NMPC Path Tracking')
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