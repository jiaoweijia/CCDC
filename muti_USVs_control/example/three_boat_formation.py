#三船编队 （CasADi + NMPC，每船一个 Boat 对象，含动力学模型和求解器
import casadi as ca
import numpy as np
import math
import matplotlib.pyplot as plt


class Boat:
    """小船基类，仅包含模型和求解器"""
    def __init__(self, name, motor_power=500):
        self.name = name
        self.motor_power = motor_power
        
        # 常量定义
        self.M_PI = np.pi
        self.N = 10  # 预测步长
        self.T = 0.1  # 采样时间
        
        # 状态和控制变量
        self.u = ca.SX.sym(f'u_{name}')
        self.v = ca.SX.sym(f'v_{name}')
        self.r = ca.SX.sym(f'r_{name}')
        self.x_pos = ca.SX.sym(f'x_{name}')
        self.y_pos = ca.SX.sym(f'y_{name}')
        self.psi = ca.SX.sym(f'psi_{name}')
        self.Tp = ca.SX.sym(f'Tp_{name}')
        self.Ts = ca.SX.sym(f'Ts_{name}')
        
        self.state = ca.vertcat(self.u, self.v, self.r, self.x_pos, self.y_pos, self.psi)
        self.control = ca.vertcat(self.Tp, self.Ts)
        
        # 动力学模型
        self._create_dynamics_model()
        
        # NMPC权重参数
        self.Q_min = [0.5, 0.1, 0.1, 100, 100, 0.1]
        self.Q_max = [5, 1, 1, 1000, 1000, 1]
        self.alpha = 0.02
        self.R_low = [0.001, 0.001]
        
        # 构建NLP问题
        self._create_nlp_problem()
        
    def _create_dynamics_model(self):
        """创建动力学模型"""
        rhs = ca.vertcat(
            -2.714753 * self.v * self.r + -1.265476 * self.u + 0.564581 * self.v + 0.138960 * self.r + 0.058919 * self.Tp + 0.057245 * self.Ts + 1.231774,
            0.002976 * self.u * self.r + -0.002729 * self.u + -1.162851 * self.v + -0.316429 * self.r + 0.000888 * self.Tp + -0.012990 * self.Ts + 0.006469,
            -1.823694 * self.u * self.v + -0.375174 * self.u + 1.273425 * self.v + -1.480984 * self.r + 0.027807 * self.Tp + -0.066450 * self.Ts + 0.384930,
            self.u * ca.cos(self.psi) - self.v * ca.sin(self.psi),
            self.u * ca.sin(self.psi) + self.v * ca.cos(self.psi),
            self.r
        )
        self.dynamics = ca.Function(f'f_{self.name}', [self.state, self.control], [rhs])
    
    def _create_nlp_problem(self):
        """创建NLP问题"""
        # 符号变量
        self.U = ca.SX.sym(f'U_{self.name}', 2, self.N)
        self.P_state = ca.SX.sym(f'P_state_{self.name}', 12)
        self.P_weight = ca.SX.sym(f'P_weight_{self.name}', 14)
        self.P = ca.vertcat(self.P_state, self.P_weight)
        
        self.X = ca.SX.sym(f'X_{self.name}', 6, self.N + 1)
        self.X[:, 0] = self.P_state[0:6]
        
        # 状态预测
        for k in range(self.N):
            self.X[:, k + 1] = self.X[:, k] + self.T * self.dynamics(self.X[:, k], self.U[:, k])
        
        # 目标函数
        self.obj = 0
        self.Qk = ca.diagcat(
            self.P_weight[0], self.P_weight[1], self.P_weight[2], 
            self.P_weight[3], self.P_weight[4], self.P_weight[5]
        )
        self.Rk = ca.diagcat(self.P_weight[6], self.P_weight[7])
        self.Qf = ca.diagcat(
            self.P_weight[8], self.P_weight[9], self.P_weight[10], 
            self.P_weight[11], self.P_weight[12], self.P_weight[13]
        )
        
        for k in range(self.N):
            st_err = self.X[:, k] - self.P_state[6:12]
            con = self.U[:, k]
            self.obj += st_err.T @ self.Qk @ st_err + con.T @ self.Rk @ con
        
        self.obj += (self.X[:, self.N] - self.P_state[6:12]).T @ self.Qf @ (self.X[:, self.N] - self.P_state[6:12])
        
        # 约束
        self.g = []
        for k in range(self.N + 1):
            self.g.append(self.X[0, k])
            self.g.append(self.X[1, k])
            self.g.append(self.X[2, k])
        self.g = ca.vertcat(*self.g)
        
        # 优化变量
        self.OPT_variables = ca.reshape(self.U, 2 * self.N, 1)
        
        # 创建NLP问题和求解器
        nlp = {'x': self.OPT_variables, 'f': self.obj, 'g': self.g, 'p': self.P}
        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.max_iter': 30,
            'ipopt.acceptable_tol': 1e-5
        }
        self.solver = ca.nlpsol(f'solver_{self.name}', 'ipopt', nlp, opts)
        
        # 约束边界
        self.lbx = [-self.motor_power, -self.motor_power] * self.N
        self.ubx = [self.motor_power, self.motor_power] * self.N
        self.lbg = [-3, -1, -0.3 * self.M_PI] * (self.N + 1)
        self.ubg = [3, 1, 0.3 * self.M_PI] * (self.N + 1)
    
    def normalize_angle_diff(self, delta):
        """归一化角度差"""
        if delta > self.M_PI:
            return delta - 2 * self.M_PI
        elif delta < -self.M_PI:
            return delta + 2 * self.M_PI
        else:
            return delta
    
    def compute_adaptive_weights(self, x0, xs):
        """计算自适应权重"""
        state_error = np.array(x0) - np.array(xs)
        Q_current = []
        for j in range(6):
            ej = abs(state_error[j])
            qj = self.Q_min[j] + (self.Q_max[j] - self.Q_min[j]) * (1 - math.exp(-self.alpha * ej))
            Q_current.append(qj)
        return Q_current
    
    def solve_nmpc(self, x0, xs, u0):
        """求解NMPC问题"""
        # 计算自适应权重
        Q_current = self.compute_adaptive_weights(x0, xs)
        R_current = self.R_low
        F_current = self.Q_min
        
        # 构建参数向量
        p_state = x0 + xs
        p_weight = Q_current + R_current + F_current
        p = p_state + p_weight
        
        # 求解
        res = self.solver(x0=u0, lbx=self.lbx, ubx=self.ubx, lbg=self.lbg, ubg=self.ubg, p=p)
        u_opt = res['x'].full().flatten().tolist()
        
        return u_opt
    
    def update_state(self, x0, u_apply):
        """更新状态"""
        next_state = self.dynamics(x0, u_apply)
        x_next = x0 + self.T * next_state.full().flatten()
        return x_next.tolist()


class ThreeBoatFormationControl:
    """三船编队控制器，负责生成参考路径和管理三船编队"""
    def __init__(self, leader, follower1, follower2, loop_num=1, 
                 follower1_relative_x=8, follower1_relative_y=0,
                 follower2_relative_x=8, follower2_relative_y=-5):
        self.leader = leader
        self.follower1 = follower1
        self.follower2 = follower2
        
        # 各船期望相对位置
        self.follower1_relative_x = follower1_relative_x
        self.follower1_relative_y = follower1_relative_y
        self.follower2_relative_x = follower2_relative_x
        self.follower2_relative_y = follower2_relative_y
        
        # 常量
        self.M_PI = np.pi
        self.loop_num = loop_num
        self.tol = 210 * self.loop_num  # 仿真总时间s
        self.t = np.linspace(0, 2 * self.M_PI * self.loop_num, int(self.tol / self.leader.T) + 1)
        
        # 生成参考路径
        self._create_reference_paths()
    
    def _create_reference_paths(self):
        """创建参考路径"""
        # 领航船参考路径
        x_r1 = 50 * np.sin(self.t) + 1
        y_r1 = 30 * np.cos(self.t) + 1
        
        x_r1 = x_r1[:, np.newaxis]
        y_r1 = y_r1[:, np.newaxis]
        self.path_points = np.concatenate([x_r1, y_r1], axis=1)
        self.sim_steps = len(self.path_points)
        
        # 生成三艘船的参考状态列表
        self.leader_xs_list = [[0, 0, 0, point[0], point[1], 0] for point in self.path_points]
        self.follower1_xs_list = []
        self.follower2_xs_list = []
        
        for i, point in enumerate(self.path_points):
            # 计算领航船在该点的航向角
            if i < len(self.leader_xs_list) - 1:
                next_point = self.leader_xs_list[i + 1]
                target_psi = math.atan2(next_point[4] - point[1], next_point[3] - point[0])
            else:
                target_psi = 0
            
            # 计算第一艘跟随船的参考位置
            follower1_x = point[0] + self.follower1_relative_x * math.cos(target_psi) - self.follower1_relative_y * math.sin(target_psi)
            follower1_y = point[1] + self.follower1_relative_x * math.sin(target_psi) + self.follower1_relative_y * math.cos(target_psi)
            
            # 计算第二艘跟随船的参考位置
            follower2_x = point[0] + self.follower2_relative_x * math.cos(target_psi) - self.follower2_relative_y * math.sin(target_psi)
            follower2_y = point[1] + self.follower2_relative_x * math.sin(target_psi) + self.follower2_relative_y * math.cos(target_psi)
            
            self.follower1_xs_list.append([0, 0, 0, follower1_x, follower1_y, target_psi])
            self.follower2_xs_list.append([0, 0, 0, follower2_x, follower2_y, target_psi])
    
    def compute_target_psi(self, xs_list, sim_index):
        """计算目标航向角"""
        if sim_index >= len(xs_list) - 1:
            return xs_list[sim_index][5]
        current_x = xs_list[sim_index][3]
        current_y = xs_list[sim_index][4]
        next_x = xs_list[sim_index + 1][3]
        next_y = xs_list[sim_index + 1][4]
        return math.atan2(next_y - current_y, next_x - current_x)


class ThreeBoatTrajectoryDisplay:
    """三船轨迹显示类，负责结果可视化"""
    def __init__(self, formation_control):
        self.formation_control = formation_control
        self.leader = formation_control.leader
        self.follower1 = formation_control.follower1
        self.follower2 = formation_control.follower2
        
        # 历史记录
        self.leader_state_history = []
        self.follower1_state_history = []
        self.follower2_state_history = []
        
        self.leader_control_history = []
        self.follower1_control_history = []
        self.follower2_control_history = []
        
        self.leader_lateral_errors = []
        self.follower1_lateral_errors = []
        self.follower2_lateral_errors = []
        
        self.leader_heading_errors = []
        self.follower1_heading_errors = []
        self.follower2_heading_errors = []
    
    def log_errors(self, x0, xs, xs_list, sim_index, boat_type="leader"):
        """记录误差"""
        current_x, current_y = x0[3], x0[4]
        target_x, target_y = xs[3], xs[4]
        
        # 距离误差
        distance_error = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        
        # 航向误差
        current_psi = x0[5]
        target_psi = self.formation_control.compute_target_psi(xs_list, sim_index)
        delta_psi = self.formation_control.leader.normalize_angle_diff(target_psi - current_psi)
        
        if boat_type == "leader":
            self.leader_lateral_errors.append(distance_error)
            self.leader_heading_errors.append(abs(delta_psi))
        elif boat_type == "follower1":
            self.follower1_lateral_errors.append(distance_error)
            self.follower1_heading_errors.append(abs(delta_psi))
        elif boat_type == "follower2":
            self.follower2_lateral_errors.append(distance_error)
            self.follower2_heading_errors.append(abs(delta_psi))
    
    def log_state(self, x0, boat_type="leader"):
        """记录状态"""
        if boat_type == "leader":
            self.leader_state_history.append(x0)
        elif boat_type == "follower1":
            self.follower1_state_history.append(x0)
        elif boat_type == "follower2":
            self.follower2_state_history.append(x0)
    
    def log_control(self, u_apply, boat_type="leader"):
        """记录控制输入"""
        if boat_type == "leader":
            self.leader_control_history.append(u_apply)
        elif boat_type == "follower1":
            self.follower1_control_history.append(u_apply)
        elif boat_type == "follower2":
            self.follower2_control_history.append(u_apply)
    
    def print_statistics(self):
        """打印统计信息"""
        print("\n=== Leader Ship Statistics ===")
        print(f"Lateral Error: Mean={np.mean(self.leader_lateral_errors):.4f}, Std={np.std(self.leader_lateral_errors):.4f}")
        print(f"Heading Error: Mean={np.mean(self.leader_heading_errors):.4f} rad, Std={np.std(self.leader_heading_errors):.4f} rad")
        
        print("\n=== Follower 1 Ship Statistics ===")
        print(f"Lateral Error: Mean={np.mean(self.follower1_lateral_errors):.4f}, Std={np.std(self.follower1_lateral_errors):.4f}")
        print(f"Heading Error: Mean={np.mean(self.follower1_heading_errors):.4f} rad, Std={np.std(self.follower1_heading_errors):.4f} rad")
        
        print("\n=== Follower 2 Ship Statistics ===")
        print(f"Lateral Error: Mean={np.mean(self.follower2_lateral_errors):.4f}, Std={np.std(self.follower2_lateral_errors):.4f}")
        print(f"Heading Error: Mean={np.mean(self.follower2_heading_errors):.4f} rad, Std={np.std(self.follower2_heading_errors):.4f} rad")
    
    def plot_results(self):
        """绘制结果"""
        # 提取轨迹数据
        leader_traj_x = [s[3] for s in self.leader_state_history][1:]
        leader_traj_y = [s[4] for s in self.leader_state_history][1:]
        
        follower1_traj_x = [s[3] for s in self.follower1_state_history][1:]
        follower1_traj_y = [s[4] for s in self.follower1_state_history][1:]
        
        follower2_traj_x = [s[3] for s in self.follower2_state_history][1:]
        follower2_traj_y = [s[4] for s in self.follower2_state_history][1:]
        
        ref_x = [p[0] for p in self.formation_control.path_points]
        ref_y = [p[1] for p in self.formation_control.path_points]
        
        # 提取控制数据
        leader_PWML = [u[0] for u in self.leader_control_history]
        leader_PWMR = [u[1] for u in self.leader_control_history]
        
        follower1_PWML = [u[0] for u in self.follower1_control_history]
        follower1_PWMR = [u[1] for u in self.follower1_control_history]
        
        follower2_PWML = [u[0] for u in self.follower2_control_history]
        follower2_PWMR = [u[1] for u in self.follower2_control_history]
        
        # 1. 轨迹对比图
        plt.figure(figsize=(10, 8))
        plt.plot(ref_x, ref_y, 'r--', label='Leader Reference Path', linewidth=2)
        plt.plot(leader_traj_x, leader_traj_y, 'b-', label='Leader Trajectory', linewidth=2)
        plt.plot(follower1_traj_x, follower1_traj_y, 'g-', label='Follower 1 Trajectory', linewidth=2)
        plt.plot(follower2_traj_x, follower2_traj_y, 'm-', label='Follower 2 Trajectory', linewidth=2)
        
        plt.scatter([leader_traj_x[0]], [leader_traj_y[0]], c='g', s=100, label='Leader Start')
        plt.scatter([leader_traj_x[-1]], [leader_traj_y[-1]], c='orange', s=100, label='Leader End')
        
        plt.scatter([follower1_traj_x[0]], [follower1_traj_y[0]], c='g', s=100, marker='^', label='Follower 1 Start')
        plt.scatter([follower1_traj_x[-1]], [follower1_traj_y[-1]], c='orange', s=100, marker='^', label='Follower 1 End')
        
        plt.scatter([follower2_traj_x[0]], [follower2_traj_y[0]], c='g', s=100, marker='s', label='Follower 2 Start')
        plt.scatter([follower2_traj_x[-1]], [follower2_traj_y[-1]], c='orange', s=100, marker='s', label='Follower 2 End')
        
        # 同时添加箭头显示和虚线连接，使用相同的步长参数
        display_step = 50  # 显示步长
        arrow_length = 1.2  # 箭头长度
        
        # 获取各船的航向角历史
        leader_psi_history = [s[5] for s in self.leader_state_history][1:]
        follower1_psi_history = [s[5] for s in self.follower1_state_history][1:]
        follower2_psi_history = [s[5] for s in self.follower2_state_history][1:]
        
        # 合并绘制箭头和虚线连接
        for i in range(0, len(leader_traj_x), display_step):
            if i < len(leader_psi_history) and i < len(follower1_psi_history) and i < len(follower2_psi_history):
                # 获取当前时刻三条船的位置
                leader_x, leader_y = leader_traj_x[i], leader_traj_y[i]
                follower1_x, follower1_y = follower1_traj_x[i], follower1_traj_y[i]
                follower2_x, follower2_y = follower2_traj_x[i], follower2_traj_y[i]
                
                # 1. 绘制箭头显示方向
                # 领航船
                leader_psi = leader_psi_history[i]
                leader_dx = arrow_length * math.cos(leader_psi)
                leader_dy = arrow_length * math.sin(leader_psi)
                plt.arrow(leader_x, leader_y, leader_dx, leader_dy, width=0.5, head_width=1, head_length=1, fc='b', ec='b', alpha=0.7)
                
                # 跟随船1
                follower1_psi = follower1_psi_history[i]
                follower1_dx = arrow_length * math.cos(follower1_psi)
                follower1_dy = arrow_length * math.sin(follower1_psi)
                plt.arrow(follower1_x, follower1_y, follower1_dx, follower1_dy, width=0.5, head_width=1, head_length=1, fc='g', ec='g', alpha=0.7)
                
                # 跟随船2
                follower2_psi = follower2_psi_history[i]
                follower2_dx = arrow_length * math.cos(follower2_psi)
                follower2_dy = arrow_length * math.sin(follower2_psi)
                plt.arrow(follower2_x, follower2_y, follower2_dx, follower2_dy, width=0.5, head_width=1, head_length=1, fc='m', ec='m', alpha=0.7)
                
                # 2. 绘制虚线连接三船
                plt.plot([leader_x, follower1_x], [leader_y, follower1_y], 'k--', alpha=0.3, linewidth=1)
                plt.plot([leader_x, follower2_x], [leader_y, follower2_y], 'k--', alpha=0.3, linewidth=1)
                plt.plot([follower1_x, follower2_x], [follower1_y, follower2_y], 'k--', alpha=0.3, linewidth=1)
        
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.title('Formation Control - Three Boat Trajectories')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        
        # 2. 误差对比图
        plt.figure(figsize=(12, 8))
        plt.subplot(2, 2, 1)
        plt.plot(self.leader_lateral_errors, 'b-', label='Leader')
        plt.plot(self.follower1_lateral_errors, 'g-', label='Follower 1')
        plt.plot(self.follower2_lateral_errors, 'm-', label='Follower 2')
        plt.ylabel('Lateral Error (m)')
        plt.title('Lateral Tracking Error')
        plt.legend()
        plt.grid(True)
        
        plt.subplot(2, 2, 2)
        plt.plot(self.leader_heading_errors, 'b-', label='Leader')
        plt.plot(self.follower1_heading_errors, 'g-', label='Follower 1')
        plt.plot(self.follower2_heading_errors, 'm-', label='Follower 2')
        plt.ylabel('Heading Error (rad)')
        plt.title('Heading Tracking Error')
        plt.legend()
        plt.grid(True)
        
        # 3. 控制输入图 - PWML
        plt.subplot(2, 2, 3)
        plt.plot(leader_PWML, 'b-', label='Leader PWML')
        plt.plot(follower1_PWML, 'g-', label='Follower 1 PWML')
        plt.plot(follower2_PWML, 'm-', label='Follower 2 PWML')
        plt.xlabel('Time Step')
        plt.ylabel('PWML')
        plt.title('Left Propeller Output')
        plt.legend()
        plt.grid(True)
        
        # 4. 控制输入图 - PWMR
        plt.subplot(2, 2, 4)
        plt.plot(leader_PWMR, 'b-', label='Leader PWMR')
        plt.plot(follower1_PWMR, 'g-', label='Follower 1 PWMR')
        plt.plot(follower2_PWMR, 'm-', label='Follower 2 PWMR')
        plt.xlabel('Time Step')
        plt.ylabel('PWMR')
        plt.title('Right Propeller Output')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        
        # 5. 相对位置图
        plt.figure(figsize=(10, 6))
        # 计算相对距离
        relative_distances_1 = []  # follower1与leader
        relative_distances_2 = []  # follower2与leader
        
        for i in range(len(leader_traj_x)):
            dx1 = follower1_traj_x[i] - leader_traj_x[i]
            dy1 = follower1_traj_y[i] - leader_traj_y[i]
            distance1 = math.sqrt(dx1**2 + dy1**2)
            relative_distances_1.append(distance1)
            
            dx2 = follower2_traj_x[i] - leader_traj_x[i]
            dy2 = follower2_traj_y[i] - leader_traj_y[i]
            distance2 = math.sqrt(dx2**2 + dy2**2)
            relative_distances_2.append(distance2)
        
        desired_distance1 = math.sqrt(self.formation_control.follower1_relative_x**2 + self.formation_control.follower1_relative_y**2)
        desired_distance2 = math.sqrt(self.formation_control.follower2_relative_x**2 + self.formation_control.follower2_relative_y**2)
        
        plt.plot(relative_distances_1, 'g-', label='Follower 1 - Leader Distance')
        plt.plot(relative_distances_2, 'm-', label='Follower 2 - Leader Distance')
        plt.axhline(y=desired_distance1, color='g', linestyle='--', label=f'Follower 1 Desired: {desired_distance1:.2f}m')
        plt.axhline(y=desired_distance2, color='m', linestyle='--', label=f'Follower 2 Desired: {desired_distance2:.2f}m')
        
        plt.xlabel('Time Step')
        plt.ylabel('Relative Distance (m)')
        plt.title('Distance Between Leader and Followers')
        plt.legend()
        plt.grid(True)
        
        plt.show()


class ThreeBoatSimulationRunner:
    """三船仿真运行类，负责主循环和仿真控制"""
    def __init__(self, formation_control, trajectory_display):
        self.formation_control = formation_control
        self.trajectory_display = trajectory_display
        self.leader = formation_control.leader
        self.follower1 = formation_control.follower1
        self.follower2 = formation_control.follower2
        
        # 初始化状态
        self._initialize_states()
    
    def _initialize_states(self):
        """初始化状态"""
        # 领航船初始状态
        self.leader_x0 = [0, 0, 0, self.formation_control.path_points[0][0] - 5, self.formation_control.path_points[0][1] - 5, 0]
        
        # 第一艘跟随船初始状态
        leader_psi = self.leader_x0[5]
        follower1_x = self.leader_x0[3] + self.formation_control.follower1_relative_x * math.cos(leader_psi) - self.formation_control.follower1_relative_y * math.sin(leader_psi)
        follower1_y = self.leader_x0[4] + self.formation_control.follower1_relative_x * math.sin(leader_psi) + self.formation_control.follower1_relative_y * math.cos(leader_psi)
        self.follower1_x0 = [0, 0, 0, follower1_x + 2, follower1_y + 2, leader_psi]
        
        # 第二艘跟随船初始状态
        follower2_x = self.leader_x0[3] + self.formation_control.follower2_relative_x * math.cos(leader_psi) - self.formation_control.follower2_relative_y * math.sin(leader_psi)
        follower2_y = self.leader_x0[4] + self.formation_control.follower2_relative_x * math.sin(leader_psi) + self.formation_control.follower2_relative_y * math.cos(leader_psi)
        self.follower2_x0 = [0, 0, 0, follower2_x - 2, follower2_y - 2, leader_psi]
        
        # 历史记录初始化
        self.trajectory_display.log_state(self.leader_x0, boat_type="leader")
        self.trajectory_display.log_state(self.follower1_x0, boat_type="follower1")
        self.trajectory_display.log_state(self.follower2_x0, boat_type="follower2")
        
        # 初始控制猜测
        self.leader_u0 = np.zeros(2 * self.leader.N).tolist()
        self.follower1_u0 = np.zeros(2 * self.follower1.N).tolist()
        self.follower2_u0 = np.zeros(2 * self.follower2.N).tolist()
    
    def run_simulation(self):
        """运行仿真"""
        print(f"Starting three-boat formation control simulation for {self.formation_control.sim_steps} steps...")
        
        for sim in range(self.formation_control.sim_steps):
            # 获取当前参考状态
            leader_xs = self.formation_control.leader_xs_list[sim] if sim < len(self.formation_control.leader_xs_list) else self.formation_control.leader_xs_list[-1]
            follower1_xs = self.formation_control.follower1_xs_list[sim] if sim < len(self.formation_control.follower1_xs_list) else self.formation_control.follower1_xs_list[-1]
            follower2_xs = self.formation_control.follower2_xs_list[sim] if sim < len(self.formation_control.follower2_xs_list) else self.formation_control.follower2_xs_list[-1]
            
            # 归一化航向角
            self.leader_x0[5] = self.leader.normalize_angle_diff(self.leader_x0[5])
            self.follower1_x0[5] = self.follower1.normalize_angle_diff(self.follower1_x0[5])
            self.follower2_x0[5] = self.follower2.normalize_angle_diff(self.follower2_x0[5])
            
            # 记录误差
            self.trajectory_display.log_errors(self.leader_x0, leader_xs, self.formation_control.leader_xs_list, sim, boat_type="leader")
            self.trajectory_display.log_errors(self.follower1_x0, follower1_xs, self.formation_control.follower1_xs_list, sim, boat_type="follower1")
            self.trajectory_display.log_errors(self.follower2_x0, follower2_xs, self.formation_control.follower2_xs_list, sim, boat_type="follower2")
            
            # 求解NMPC问题
            leader_u_opt = self.leader.solve_nmpc(self.leader_x0, leader_xs, self.leader_u0)
            follower1_u_opt = self.follower1.solve_nmpc(self.follower1_x0, follower1_xs, self.follower1_u0)
            follower2_u_opt = self.follower2.solve_nmpc(self.follower2_x0, follower2_xs, self.follower2_u0)
            
            # 应用控制输入（带平滑处理）
            # 领航船
            if len(self.trajectory_display.leader_control_history) > 0:
                prev_control = np.array(self.trajectory_display.leader_control_history[-1])
                current_control = np.array(leader_u_opt[:2])
                leader_u_apply = (0.9 * current_control + 0.1 * prev_control).tolist()
            else:
                leader_u_apply = leader_u_opt[:2]
            
            # 第一艘跟随船
            if len(self.trajectory_display.follower1_control_history) > 0:
                prev_control = np.array(self.trajectory_display.follower1_control_history[-1])
                current_control = np.array(follower1_u_opt[:2])
                follower1_u_apply = (0.9 * current_control + 0.1 * prev_control).tolist()
            else:
                follower1_u_apply = follower1_u_opt[:2]
            
            # 第二艘跟随船
            if len(self.trajectory_display.follower2_control_history) > 0:
                prev_control = np.array(self.trajectory_display.follower2_control_history[-1])
                current_control = np.array(follower2_u_opt[:2])
                follower2_u_apply = (0.9 * current_control + 0.1 * prev_control).tolist()
            else:
                follower2_u_apply = follower2_u_opt[:2]
            
            # 更新状态
            self.leader_x0 = self.leader.update_state(self.leader_x0, leader_u_apply)
            self.follower1_x0 = self.follower1.update_state(self.follower1_x0, follower1_u_apply)
            self.follower2_x0 = self.follower2.update_state(self.follower2_x0, follower2_u_apply)
            
            # 记录历史
            self.trajectory_display.log_state(self.leader_x0, boat_type="leader")
            self.trajectory_display.log_state(self.follower1_x0, boat_type="follower1")
            self.trajectory_display.log_state(self.follower2_x0, boat_type="follower2")
            
            self.trajectory_display.log_control(leader_u_apply, boat_type="leader")
            self.trajectory_display.log_control(follower1_u_apply, boat_type="follower1")
            self.trajectory_display.log_control(follower2_u_apply, boat_type="follower2")
            
            # 更新初始控制猜测
            self.leader_u0 = leader_u_opt[2:] + leader_u_opt[-2:]
            self.follower1_u0 = follower1_u_opt[2:] + follower1_u_opt[-2:]
            self.follower2_u0 = follower2_u_opt[2:] + follower2_u_opt[-2:]
            
            # 打印进度
            if sim % 200 == 0:
                print(f"Step {sim}/{self.formation_control.sim_steps}")
        
        print("Simulation completed!")
        self.trajectory_display.print_statistics()


# ==================== 主程序 ====================
if __name__ == "__main__":
    # 创建小船实例
    leader_boat = Boat("Leader")
    follower1_boat = Boat("Follower1")
    follower2_boat = Boat("Follower2")
    
    # 创建三船编队控制器
    formation_control = ThreeBoatFormationControl(
        leader_boat, follower1_boat, follower2_boat, 
        loop_num=1, 
        follower1_relative_x=5, follower1_relative_y=-5,
        follower2_relative_x=-5, follower2_relative_y=-5
    )
    
    # 创建轨迹显示器
    trajectory_display = ThreeBoatTrajectoryDisplay(formation_control)
    
    # 创建仿真运行器
    simulator = ThreeBoatSimulationRunner(formation_control, trajectory_display)
    
    # 运行仿真
    simulator.run_simulation()
    
    # 绘制结果
    trajectory_display.plot_results()