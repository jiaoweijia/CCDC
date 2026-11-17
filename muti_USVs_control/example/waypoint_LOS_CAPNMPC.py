# CAP_NMPC_LOS_optimized.py - 高效版约束感知粒子滤波NMPC
#LOS + CAP-NMPC 变体
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
import time

# ==================== 常量定义 ====================
M_PI = math.pi
H = 10                  # 预测时域
T = 0.1                 # 采样时间
N_particles = 50        # 减少粒子数以提高速度
motor_power = 500
position_noise_std = 0.005

# 生成自定义椭圆轨迹
t_total = 2 * np.pi / 0.15
num_points = int(t_total / T)
t_values = np.linspace(0, t_total, num_points)
x = 2 * np.sin(0.15 * t_values) + 0.5
y = np.cos(0.15 * t_values) - 0.88
path_points = np.column_stack((x, y)).tolist()
sim_steps = len(path_points) - H  # 留出预测时域

# ==================== 权重参数 ====================
Q_pos = 100.0           # 位置误差权重
Q_psi = 10.0            # 航向误差权重
Q_vel = 0.1             # 速度误差权重
R = 0.01                # 控制输入权重

# 障碍函数参数
barrier_gain = 100.0    # 约束违反惩罚

# ==================== LOS参数 ====================
look_ahead = 0.3
R_switch = 0.25

# ==================== 动力学模型 ====================
def system_dynamics(x, u, dt=T):
    """简化版动力学模型"""
    u_vel, v_vel, r_rate, x_pos, y_pos, psi = x
    Tp, Ts = u
    
    # 状态导数
    u_dot = 1.428555 * v_vel * r_rate - 0.296273 * u_vel - 0.050424 * v_vel + 0.048005 * r_rate + 0.040889 * Tp + 0.020753 * Ts + 0.064785
    v_dot = -0.642374 * u_vel * r_rate - 0.117058 * u_vel - 1.051617 * v_vel + 0.018381 * r_rate - 0.008968 * Tp + 0.005212 * Ts + 0.012565
    r_dot = -4.503596 * u_vel * v_vel + 1.871949 * u_vel + 18.340192 * v_vel - 0.692702 * r_rate - 0.184732 * Tp + 0.149389 * Ts - 0.504434
    x_dot = u_vel * math.cos(psi) - v_vel * math.sin(psi)
    y_dot = u_vel * math.sin(psi) + v_vel * math.cos(psi)
    psi_dot = r_rate
    
    return np.array([u_dot, v_dot, r_dot, x_dot, y_dot, psi_dot])

# ==================== 辅助函数 ====================
def normalize_angle_diff(delta):
    return math.atan2(math.sin(delta), math.cos(delta))

def compute_los_heading(current_pos, path_points, current_index, speed):
    """LOS制导算法"""
    current_x, current_y = current_pos
    num_wpts = len(path_points)
    
    xk, yk = path_points[current_index % num_wpts]
    xk1, yk1 = path_points[(current_index + 1) % num_wpts]
    
    path_vec = np.array([xk1 - xk, yk1 - yk])
    pi_h = math.atan2(path_vec[1], path_vec[0])
    
    R_mat = np.array([[math.cos(pi_h), math.sin(pi_h)],
                      [-math.sin(pi_h), math.cos(pi_h)]])
    along_track, cross_track = R_mat @ np.array([current_x - xk, current_y - yk])
    
    psi_d = pi_h - math.atan(cross_track / look_ahead)
    
    return psi_d, (xk1, yk1), cross_track

def barrier_function(x, u):
    """约束障碍函数 - 惩罚约束违反"""
    u_vel, v_vel, r_rate, x_pos, y_pos, psi = x
    Tp, Ts = u
    
    # 控制输入约束
    penalty = 0
    if abs(Tp) > motor_power:
        penalty += (abs(Tp) - motor_power)**2
    if abs(Ts) > motor_power:
        penalty += (abs(Ts) - motor_power)**2
    
    # 速度约束
    speed = np.sqrt(u_vel**2 + v_vel**2)
    if speed > 5.0:
        penalty += (speed - 5.0)**2
    
    return barrier_gain * penalty

# ==================== 简化的粒子滤波器 ====================
class SimplifiedParticleFilter:
    """
    简化版粒子滤波器 - 只保留核心功能
    """
    def __init__(self, n_particles, state_dim=6, control_dim=2):
        self.N = n_particles
        self.state_dim = state_dim
        self.control_dim = control_dim
        
        # 粒子: 状态 + 控制序列 (H个控制输入)
        self.particles = None
        self.weights = None
        
        # 过程噪声
        self.process_noise = np.diag([0.01, 0.01, 0.01, 0.005, 0.005, 0.01])
        
    def initialize(self, current_state):
        """初始化粒子"""
        self.particles = []
        for _ in range(self.N):
            # 随机生成控制序列
            control_seq = np.random.uniform(-motor_power/2, motor_power/2, (H, self.control_dim))
            self.particles.append({
                'state': np.array(current_state),
                'control_seq': control_seq
            })
        self.weights = np.ones(self.N) / self.N
        
    def predict_trajectory(self, particle):
        """预测整个轨迹"""
        state = particle['state'].copy()
        control_seq = particle['control_seq']
        predicted_states = [state.copy()]
        
        for k in range(H):
            # 应用控制
            u = control_seq[k]
            # 系统动力学
            state_dot = system_dynamics(state, u)
            state = state + T * state_dot
            
            # 加入小噪声增加多样性
            noise = np.random.multivariate_normal(np.zeros(self.state_dim), self.process_noise * 0.1)
            state += noise
            
            predicted_states.append(state.copy())
        
        return predicted_states
    
    def evaluate_particle(self, particle, reference_states):
        """评估单个粒子的性能"""
        predicted = self.predict_trajectory(particle)
        total_cost = 0
        
        # 轨迹跟踪成本
        for k in range(H):
            state = predicted[k]
            ref = reference_states[k]
            
            # 位置误差
            pos_error = np.sqrt((state[3] - ref[3])**2 + (state[4] - ref[4])**2)
            total_cost += Q_pos * pos_error**2
            
            # 航向误差
            psi_error = normalize_angle_diff(state[5] - ref[5])
            total_cost += Q_psi * psi_error**2
            
            # 速度误差
            vel_error = state[0] - ref[0]
            total_cost += Q_vel * vel_error**2
            
            # 控制成本
            u = particle['control_seq'][k]
            total_cost += R * (u[0]**2 + u[1]**2)
            
            # 约束惩罚
            total_cost += barrier_function(state, u)
        
        # 终端成本
        final_state = predicted[H]
        final_ref = reference_states[H]
        pos_error = np.sqrt((final_state[3] - final_ref[3])**2 + (final_state[4] - final_ref[4])**2)
        total_cost += 2 * Q_pos * pos_error**2  # 终端权重加倍
        
        return -total_cost  # 返回负成本作为似然
    
    def update_weights(self, reference_states):
        """更新粒子权重"""
        log_likelihoods = []
        for particle in self.particles:
            log_lik = self.evaluate_particle(particle, reference_states)
            log_likelihoods.append(log_lik)
        
        # 稳定化处理
        max_log_lik = max(log_likelihoods)
        likelihoods = np.exp(log_likelihoods - max_log_lik)
        
        self.weights = likelihoods / (np.sum(likelihoods) + 1e-10)
        
    def resample(self):
        """重采样"""
        indices = np.random.choice(self.N, self.N, p=self.weights)
        self.particles = [self.particles[i].copy() for i in indices]
        self.weights = np.ones(self.N) / self.N
        
        # 添加小扰动增加多样性
        for particle in self.particles:
            particle['control_seq'] += np.random.randn(H, self.control_dim) * 10
        
    def get_optimal_control(self):
        """获取最优控制"""
        # 加权平均
        best_idx = np.argmax(self.weights)
        return self.particles[best_idx]['control_seq'][0]

# ==================== CAP-NMPC主循环 ====================
def run_cap_nmpc():
    """运行CAP-NMPC"""
    
    # 初始化
    current_state = np.array([0, 0, 0, path_points[0][0] - 0.1, 
                              path_points[0][1] - 0.1, 0])
    current_index = 0
    
    # 记录数据
    state_history = [current_state.copy()]
    control_history = []
    lateral_errors = []
    heading_errors = []
    computation_times = []
    
    # 初始化粒子滤波器
    pf = SimplifiedParticleFilter(N_particles)
    pf.initialize(current_state)
    
    # 主循环
    for sim in range(sim_steps):
        start_time = time.time()
        
        current_x, current_y = current_state[3], current_state[4]
        current_psi = current_state[5]
        current_speed = current_state[0]
        
        # 计算LOS参考
        target_psi, target_pos, cross_track = compute_los_heading(
            (current_x, current_y), path_points, current_index, current_speed
        )
        target_x, target_y = target_pos
        
        # 记录误差
        lateral_errors.append(abs(cross_track))
        delta_psi = normalize_angle_diff(current_psi - target_psi)
        heading_errors.append(abs(delta_psi))
        
        # 生成参考轨迹
        reference_states = []
        temp_index = current_index
        temp_state = current_state.copy()
        
        for k in range(H + 1):
            # 获取对应时刻的参考点
            ref_idx = min(temp_index, len(path_points) - 1)
            ref_x, ref_y = path_points[ref_idx]
            
            # 计算参考航向
            next_idx = min(ref_idx + 1, len(path_points) - 1)
            next_x, next_y = path_points[next_idx]
            ref_psi = math.atan2(next_y - ref_y, next_x - ref_x)
            
            reference_states.append([0, 0, 0, ref_x, ref_y, ref_psi])
            
            temp_index += 1
        
        # 粒子滤波更新
        pf.update_weights(reference_states)
        pf.resample()
        
        # 获取最优控制
        u_opt = pf.get_optimal_control()
        
        # 应用控制
        next_state_dot = system_dynamics(current_state, u_opt)
        next_state = current_state + T * next_state_dot
        
        # 加入噪声
        next_state[3] += np.random.normal(0, position_noise_std)
        next_state[4] += np.random.normal(0, position_noise_std)
        
        # 更新状态
        current_state = next_state
        current_index += 1
        
        # 记录
        control_history.append(u_opt)
        state_history.append(current_state.copy())
        
        # 更新粒子状态
        for particle in pf.particles:
            particle['state'] = current_state.copy()
        
        computation_times.append(time.time() - start_time)
        
        if sim % 50 == 0:
            print(f"Step {sim}/{sim_steps}, Lateral Error: {abs(cross_track):.4f}")
    
    return state_history, control_history, lateral_errors, heading_errors, computation_times

# ==================== 运行仿真 ====================
print("运行高效CAP-NMPC仿真...")
state_history, control_history, lateral_errors, heading_errors, comp_times = run_cap_nmpc()

# ==================== 提取数据 ====================
traj_x = [s[3] for s in state_history][1:]
traj_y = [s[4] for s in state_history][1:]
traj_psi = [s[5] for s in state_history][1:]
ref_x = [p[0] for p in path_points][:len(traj_x)]
ref_y = [p[1] for p in path_points][:len(traj_y)]
PWML = [u[0] for u in control_history]
PWMR = [u[1] for u in control_history]

# ==================== 输出统计 ====================
print(f"\n===== 仿真结果 =====")
print(f"横向误差: 均值={np.mean(lateral_errors):.4f}, 标准差={np.std(lateral_errors):.4f}")
print(f"航向误差: 均值={np.mean(heading_errors):.4f} rad, 标准差={np.std(heading_errors):.4f} rad")
print(f"平均计算时间: {np.mean(comp_times)*1000:.2f} ms/步")
print(f"控制输入范围: Tp [{np.min(PWML):.1f}, {np.max(PWML):.1f}], Ts [{np.min(PWMR):.1f}, {np.max(PWMR):.1f}]")

# ==================== 绘图 ====================
plt.figure(figsize=(15, 10))

# 轨迹跟踪
plt.subplot(2, 3, 1)
plt.plot(ref_x, ref_y, 'r--', linewidth=2, label='Reference')
plt.plot(traj_x, traj_y, 'b-', linewidth=2, label='CAP-NMPC')
plt.scatter([traj_x[0]], [traj_y[0]], c='g', s=100, label='Start')
plt.scatter([traj_x[-1]], [traj_y[-1]], c='orange', s=100, label='End')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Path Tracking')
plt.legend()
plt.grid(True)
plt.axis('equal')

# 横向误差
plt.subplot(2, 3, 2)
plt.plot(lateral_errors, 'b-', linewidth=1.5)
plt.axhline(y=np.mean(lateral_errors), color='r', linestyle='--', 
            label=f'Mean: {np.mean(lateral_errors):.3f}m')
plt.fill_between(range(len(lateral_errors)), 
                 np.mean(lateral_errors) - np.std(lateral_errors),
                 np.mean(lateral_errors) + np.std(lateral_errors),
                 alpha=0.2, color='r')
plt.xlabel('Step')
plt.ylabel('Error (m)')
plt.title('Lateral Error')
plt.legend()
plt.grid(True)

# 航向误差
plt.subplot(2, 3, 3)
plt.plot(np.array(heading_errors)*180/np.pi, 'r-', linewidth=1.5)
plt.axhline(y=np.mean(heading_errors)*180/np.pi, color='b', linestyle='--',
            label=f'Mean: {np.mean(heading_errors)*180/np.pi:.1f}°')
plt.xlabel('Step')
plt.ylabel('Error (deg)')
plt.title('Heading Error')
plt.legend()
plt.grid(True)

# 控制输入
plt.subplot(2, 3, 4)
plt.plot(PWML, 'g-', label='Tp', alpha=0.7)
plt.plot(PWMR, 'm-', label='Ts', alpha=0.7)
plt.axhline(y=motor_power, color='r', linestyle='--', alpha=0.5)
plt.axhline(y=-motor_power, color='r', linestyle='--', alpha=0.5)
plt.xlabel('Step')
plt.ylabel('PWM')
plt.title('Control Inputs')
plt.legend()
plt.grid(True)

# 速度状态
plt.subplot(2, 3, 5)
vel_u = [s[0] for s in state_history][1:]
vel_v = [s[1] for s in state_history][1:]
vel_r = [s[2] for s in state_history][1:]
plt.plot(vel_u, label='u (surge)')
plt.plot(vel_v, label='v (sway)')
plt.plot(vel_r, label='r (yaw rate)')
plt.xlabel('Step')
plt.ylabel('Velocity')
plt.title('Velocity States')
plt.legend()
plt.grid(True)

# 计算时间
plt.subplot(2, 3, 6)
plt.plot(np.array(comp_times)*1000, 'b-', linewidth=1)
plt.axhline(y=np.mean(comp_times)*1000, color='r', linestyle='--',
            label=f'Mean: {np.mean(comp_times)*1000:.1f}ms')
plt.xlabel('Step')
plt.ylabel('Time (ms)')
plt.title('Computation Time per Step')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()

# ==================== 误差分布 ====================
plt.figure(figsize=(12, 4))

plt.subplot(1, 3, 1)
plt.hist(lateral_errors, bins=30, edgecolor='black', alpha=0.7)
plt.axvline(np.mean(lateral_errors), color='r', linestyle='--', label=f'Mean: {np.mean(lateral_errors):.3f}')
plt.xlabel('Lateral Error (m)')
plt.ylabel('Frequency')
plt.title('Lateral Error Distribution')
plt.legend()

plt.subplot(1, 3, 2)
plt.hist(np.array(heading_errors)*180/np.pi, bins=30, edgecolor='black', alpha=0.7)
plt.axvline(np.mean(heading_errors)*180/np.pi, color='r', linestyle='--',
            label=f'Mean: {np.mean(heading_errors)*180/np.pi:.1f}°')
plt.xlabel('Heading Error (deg)')
plt.ylabel('Frequency')
plt.title('Heading Error Distribution')
plt.legend()

plt.subplot(1, 3, 3)
# 累积误差
cumulative_error = np.cumsum(lateral_errors)
plt.plot(cumulative_error, 'b-', linewidth=2)
plt.xlabel('Step')
plt.ylabel('Cumulative Error (m)')
plt.title(f'Total Path Deviation: {cumulative_error[-1]:.1f}m')
plt.grid(True)

plt.tight_layout()
plt.show()

print("\n===== 改进总结 =====")
print("1. 简化了粒子表示，直接优化控制序列")
print("2. 使用单步预测代替完整后向平滑")
print("3. 保留核心约束处理机制")
print("4. 显著提升计算速度")