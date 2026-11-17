# -*- coding: utf-8 -*-
"""LOS 制导 + 非线性模型预测控制（NMPC）—— 单船航点跟踪
（航点切换：进入接纳圆立即切换 / 超过额定切换间隔强制切换，与原代码一致的 plt.show 出图版）
=========================================================
航点切换规则（本次核心改动）：
  1) 每个参考点设置一个半径为 R_ACCEPT 的接纳圆
  2) 船进入接纳圆 -> 立即切换到下一个参考点，并用新航段重算 LOS 航向角、横迹误差等
  3) 若超过「额定参考点切换间隔 x TIMEOUT_GAIN」仍未进入圆内 -> 强制切换
  4) 额定切换间隔 = 相邻参考点弧长间距 / 额定航速（由代码自动算出，非写死常数）
  5) 支持级联推进（一次最多 MAX_ADVANCE 个点）

运行方式：
  python los_nmpc_waypoint.py              # 跑完 418 步并弹出 4 张图
  python los_nmpc_waypoint.py --steps 60   # 只跑 60 步
  python los_nmpc_waypoint.py --no-plot    # 不出图，只看数值结果
"""

import sys
import os
import time
import math

# ---------- 0) 依赖检查（缺失时立即报错，不静默卡住）----------
try:
    import numpy as np
except ImportError:
    print("[错误] 缺少 numpy，请先: pip install numpy"); sys.exit(1)
try:
    import casadi as ca
except ImportError:
    print("[错误] 缺少 casadi，请先: pip install casadi  (约 90MB)"); sys.exit(1)

# ---------- 1) 用默认交互式后端，plt.show() 直接弹窗 ----------
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib import font_manager

# ---------- 1.6) 中文字体自动探测；没有就改用英文标签，避免刷屏警告 ----------
_CJK = ['Microsoft YaHei', 'SimHei', 'SimSun', 'WenQuanYi Micro Hei',
        'Noto Sans CJK SC', 'Source Han Sans SC', 'Heiti SC']
USE_CN = False
for _f in _CJK:
    try:
        font_manager.findfont(_f, fallback_to_default=False)
        plt.rcParams['font.sans-serif'] = [_f]
        USE_CN = True
        break
    except Exception:
        continue
plt.rcParams['axes.unicode_minus'] = False

if USE_CN:
    LB = dict(title1='LOS + NMPC 路径跟踪（接纳圆切换 + 超时强制切换）',
              ref='参考路径', traj='NMPC 轨迹', wp='航点(参考点)',
              circle='接纳圆 (R=%.2f)', sw_in='切换: 进圈', sw_to='切换: 超时',
              start='起点', end='终点',
              e1='横迹误差', e2='航向误差 (rad)', e3='航点索引',
              iu='进圈切换', it='超时切换',
              st='状态变量', err_t='跟踪误差与航点切换',
              tp='Tp (左推进器)', ts='Ts (右推进器)', pr='推进器输出',
              x='X (m)', y='Y (m)', step='时间步')
else:
    LB = dict(title1='LOS + NMPC Path Tracking (circle switch + timeout)',
              ref='Reference Path', traj='NMPC Trajectory', wp='Waypoints',
              circle='Acceptance Circle (R=%.2f)', sw_in='Switch: in-circle',
              sw_to='Switch: timeout', start='Start', end='End',
              e1='Cross-track Error (m)', e2='Heading Error (rad)',
              e3='Waypoint Index', iu='switch: circle', it='switch: timeout',
              st='State Variables', err_t='Tracking Errors & Waypoint Switching',
              tp='Tp (left thruster)', ts='Ts (right thruster)',
              pr='Propeller Outputs', x='X (m)', y='Y (m)', step='Time Step')

# ---------- 2) 命令行开关 ----------
DO_PLOT = "--no-plot" not in sys.argv
MAX_STEPS = None
if "--steps" in sys.argv:
    try:
        MAX_STEPS = int(sys.argv[sys.argv.index("--steps") + 1])
    except Exception:
        MAX_STEPS = None

np.random.seed(0)

# ==================== 常量定义 ====================
M_PI = math.pi
N = 15
T = 0.1
motor_power = 500
Adaptive_flag = True
position_noise_std = 0.005

# ---------- 航点（参考点）与接纳圆参数 ----------
WP_SPACING   = 0.35   # 相邻参考点弧长间距 (m)
R_ACCEPT     = 0.25   # 接纳圆半径 (m)
TIMEOUT_GAIN = 2.5    # 超时系数
TIMEOUT_MIN_STEPS = 5
MAX_ADVANCE  = 4

# ---------- 圈数控制：走满 LAPS 圈就结束（只走一圈，轨迹不重叠，看得清）----------
LAPS = 1.0            # 1.0 = 一圈；0.5 = 半圈；2.0 = 两圈

# ==================== 生成椭圆参考轨迹 ====================
t_total = 2 * np.pi / 0.15
num_points = int(t_total / T)
t_values = np.linspace(0, t_total, num_points)
x = 2 * np.sin(0.15 * t_values) + 0.5
y = np.cos(0.15 * t_values) - 0.88
path_points = np.column_stack((x, y)).tolist()
sim_steps = len(path_points)
if MAX_STEPS is not None:
    sim_steps = min(sim_steps, MAX_STEPS)
t = t_values


# ==================== 按弧长重采样出参考点 ====================
def resample_by_arclength(points, spacing):
    pts = np.asarray(points, dtype=float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = s[-1]
    n = max(2, int(np.floor(total / spacing)))
    s_new = np.linspace(0, total, n + 1)
    out = []
    for si in s_new:
        i = int(np.searchsorted(s, si, side='right') - 1)
        i = min(max(i, 0), len(seg) - 1)
        ratio = (si - s[i]) / max(seg[i], 1e-12)
        out.append(tuple(pts[i] + ratio * (pts[i + 1] - pts[i])))
    return out


waypoints = resample_by_arclength(path_points, WP_SPACING)


# ==================== 航点管理器：接纳圆 + 超时强制切换 ====================
class WaypointManager:
    def __init__(self, wps, dt, spacing, v_nominal, r_accept,
                 timeout_gain, min_timeout_steps, max_advance, closed=True):
        self.wps = [tuple(p) for p in wps]
        self.n = len(self.wps)
        self.dt = dt
        self.r_accept = float(r_accept)
        self.max_advance = int(max_advance)
        self.closed = closed
        self.spacing = float(spacing)
        self.nominal_interval_s = self.spacing / float(v_nominal)
        self.timeout_steps = max(int(min_timeout_steps),
                                 int(round(timeout_gain * self.nominal_interval_s / dt)))
        self.idx = 0
        self.timer = 0
        self.events = []
        self.total_advance = 0      # 累计推进次数（用于判断走了几圈）

    @property
    def prev_wp(self):
        return self.wps[self.idx]

    @property
    def target_wp(self):
        if self.closed:
            return self.wps[(self.idx + 1) % self.n]
        return self.wps[min(self.idx + 1, self.n - 1)]

    def dist_to_target(self, pos):
        tx, ty = self.target_wp
        return math.hypot(pos[0] - tx, pos[1] - ty)

    def advance(self, step, reason):
        old = self.idx
        if self.closed:
            self.idx = (self.idx + 1) % self.n
        else:
            self.idx = min(self.idx + 1, self.n - 1)
        self.events.append((step, old, self.idx, reason, self.timer))
        self.timer = 0
        self.total_advance += 1

    def update(self, pos, step):
        switched, reason = False, ''
        adv = 0
        while (self.dist_to_target(pos) <= self.r_accept) and (adv < self.max_advance):
            self.advance(step, 'circle')
            switched, reason = True, 'circle'
            adv += 1
            if not self.closed and self.idx >= self.n - 1:
                break
        if not switched:
            self.timer += 1
            if self.timer >= self.timeout_steps:
                self.advance(step, 'timeout')
                switched, reason = True, 'timeout'
        return switched, reason


_pts = np.asarray(path_points, dtype=float)
_path_len = float(np.sum(np.linalg.norm(np.diff(_pts, axis=0), axis=1)))
V_NOMINAL = _path_len / (len(path_points) * T)

wp_mgr = WaypointManager(waypoints, T, WP_SPACING, V_NOMINAL, R_ACCEPT,
                         TIMEOUT_GAIN, TIMEOUT_MIN_STEPS, MAX_ADVANCE, closed=True)

print("=" * 76, flush=True)
print("航点管理器参数", flush=True)
print("=" * 76, flush=True)
print(f"  参考点个数     : {wp_mgr.n}", flush=True)
print(f"  平均参考点间距 : {wp_mgr.spacing:.4f} m", flush=True)
print(f"  额定航速       : {V_NOMINAL:.4f} m/s", flush=True)
print(f"  额定切换间隔   : {wp_mgr.nominal_interval_s:.3f} s "
      f"({wp_mgr.nominal_interval_s / T:.1f} 步)", flush=True)
print(f"  接纳圆半径     : {R_ACCEPT:.3f} m", flush=True)
print(f"  超时步数       : {wp_mgr.timeout_steps} 步 "
      f"({wp_mgr.timeout_steps * T:.2f} s)", flush=True)

# ==================== 权重 ====================
Q_low  = [0.0005, 0.001, 0.001, 100, 100, 0.1]
Q_high = [0.0005, 0.001, 0.001, 1000, 1000, 0.2]
R_low  = [0.05, 0.05]
R_high = [0.002, 0.002]
F_low  = [0.001, 0.002, 0.002, 100, 100, 0.1]
F_high = [0.001, 0.002, 0.002, 5000, 5000, 0.5]

# ==================== 符号与模型 ====================
u = ca.SX.sym('u'); v = ca.SX.sym('v'); r = ca.SX.sym('r')
x_pos = ca.SX.sym('x'); y_pos = ca.SX.sym('y'); psi = ca.SX.sym('psi')
Tp = ca.SX.sym('Tp'); Ts = ca.SX.sym('Ts')
state = ca.vertcat(u, v, r, x_pos, y_pos, psi)
control = ca.vertcat(Tp, Ts)

rhs = ca.vertcat(
    1.428555 * v * r - 0.296273 * u - 0.050424 * v + 0.048005 * r
    + 0.040889 * Tp + 0.020753 * Ts + 0.064785,
    -0.642374 * u * r - 0.117058 * u - 1.051617 * v + 0.018381 * r
    - 0.008968 * Tp + 0.005212 * Ts + 0.012565,
    -4.503596 * u * v + 1.871949 * u + 18.340192 * v - 0.692702 * r
    - 0.184732 * Tp + 0.149389 * Ts - 0.504434,
    u * ca.cos(psi) - v * ca.sin(psi),
    u * ca.sin(psi) + v * ca.cos(psi),
    r
)
f = ca.Function('f', [state, control], [rhs])

# ==================== 构建 NLP ====================
print("[1/3] 构建 NLP 问题 ...", flush=True)
_t0 = time.time()
U = ca.SX.sym('U', 2, N)
P_state = ca.SX.sym('P_state', 12)
P_weight = ca.SX.sym('P_weight', 14)
P = ca.vertcat(P_state, P_weight)
X = ca.SX.sym('X', 6, N + 1)
X[:, 0] = P_state[0:6]
for k in range(N):
    X[:, k + 1] = X[:, k] + T * f(X[:, k], U[:, k])

obj = 0
Qk = ca.diagcat(*[P_weight[i] for i in range(6)])
Rk = ca.diagcat(P_weight[6], P_weight[7])
Qf = ca.diagcat(*[P_weight[8 + i] for i in range(6)])
for k in range(N):
    e = X[:, k] - P_state[6:12]
    obj += e.T @ Qk @ e + U[:, k].T @ Rk @ U[:, k]
e = X[:, N] - P_state[6:12]
obj += e.T @ Qf @ e

g = []
for k in range(N + 1):
    g += [X[0, k], X[1, k], X[2, k]]
g = ca.vertcat(*g)

nlp = {'x': ca.reshape(U, 2 * N, 1), 'f': obj, 'g': g, 'p': P}
opts = {
    'ipopt.print_level': 0,
    'print_time': 0,
    'ipopt.max_iter': 30,                 # 上限，防单步卡死
    'ipopt.tol': 1e-6,
    'ipopt.acceptable_tol': 1e-5,
    'ipopt.acceptable_iter': 3,
    'ipopt.warm_start_init_point': 'yes',
    'ipopt.hessian_approximation': 'limited-memory',   # 更快更稳
}
solver = ca.nlpsol('solver', 'ipopt', nlp, opts)
print(f"      完成，用时 {time.time() - _t0:.2f} s", flush=True)

lbx = [-motor_power] * (2 * N)
ubx = [motor_power] * (2 * N)
lbg = [-5, -2, -0.3 * M_PI] * (N + 1)
ubg = [5, 2, 0.3 * M_PI] * (N + 1)

# ==================== 初始状态 ====================
x0 = [0, 0, 0, path_points[0][0] - 0.1, path_points[0][1] - 0.1, 0]
state_history = [list(x0)]
u0 = np.zeros(2 * N).tolist()
control_history = []
lateral_errors, heading_errors = [], []
wp_index_history, switch_steps, switch_reasons = [], [], []

look_ahead = 0.3
last_good_u = [0.0, 0.0]      # 求解失败时的 fallback


def normalize_angle_diff(delta):
    return math.atan2(math.sin(delta), math.cos(delta))


def compute_los_heading(current_pos, prev_wp, target_wp, speed, delta_los):
    cx, cy = current_pos
    xk, yk = prev_wp
    xk1, yk1 = target_wp
    pi_h = math.atan2(yk1 - yk, xk1 - xk)
    Rm = np.array([[math.cos(pi_h),  math.sin(pi_h)],
                   [-math.sin(pi_h), math.cos(pi_h)]])
    along_track, cross_track = Rm @ np.array([cx - xk, cy - yk])
    psi_d = pi_h - math.atan2(cross_track, delta_los)
    dy_e = -abs(speed) * cross_track / math.sqrt(delta_los ** 2 + cross_track ** 2)
    r_d = -dy_e / (delta_los * (1 + (cross_track / delta_los) ** 2))
    return psi_d, cross_track, along_track, r_d


# ==================== 主循环 ====================
print(f"[2/3] 开始仿真，共 {sim_steps} 步 ...", flush=True)
# ---- 走满 LAPS 圈所需的推进次数（closed 路径：推进 n 次 = 一圈）----
TARGET_ADVANCE = int(round(LAPS * wp_mgr.n))
print(f"  目标圈数 LAPS       : {LAPS}  -> 需推进 {TARGET_ADVANCE} 个参考点", flush=True)
print("=" * 76, flush=True)

_t_sim = time.time()
n_fail = 0
for sim in range(sim_steps):
    # ---- 走满目标圈数 -> 提前结束（保证轨迹只画一遍，不重叠）----
    if wp_mgr.total_advance >= TARGET_ADVANCE:
        print(f"      已走满 {LAPS} 圈（推进 {wp_mgr.total_advance} 个参考点），停止仿真", flush=True)
        break

    cx, cy = x0[3], x0[4]
    current_psi = x0[5]
    current_speed = math.hypot(x0[0], x0[1])

    switched, reason = wp_mgr.update((cx, cy), sim)
    if switched:
        switch_steps.append(sim)
        switch_reasons.append(reason)
    wp_index_history.append(wp_mgr.idx)

    prev_wp = wp_mgr.prev_wp
    target_x, target_y = wp_mgr.target_wp
    psi_d, cross_track, along_track, r_d = compute_los_heading(
        (cx, cy), prev_wp, (target_x, target_y), current_speed, look_ahead)

    lateral_errors.append(abs(cross_track))
    heading_errors.append(abs(normalize_angle_diff(current_psi - psi_d)))

    dpsi = normalize_angle_diff(psi_d - x0[5])
    xs = [0, 0, 0, target_x, target_y, x0[5] + dpsi]

    de = math.hypot(target_x - cx, target_y - cy)
    dc = max(0.0, min(1.0, de / 2.0))
    if Adaptive_flag:
        Qc = [Q_low[i] + dc * (Q_high[i] - Q_low[i]) for i in range(6)]
        Rc = [R_low[i] + dc * (R_high[i] - R_low[i]) for i in range(2)]
        Fc = [F_low[i] + dc * (F_high[i] - F_low[i]) for i in range(6)]
    else:
        Qc, Rc, Fc = Q_low, R_low, F_low

    p = x0 + xs + Qc + Rc + Fc

    # ---- 求解：带失败保护 ----
    try:
        res = solver(x0=u0, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg, p=p)
        ok = solver.stats().get('success', False)
        u_opt = res['x'].full().flatten().tolist()
        if (not ok) or (not np.all(np.isfinite(u_opt))):
            raise RuntimeError(solver.stats().get('return_status', 'failed'))
        u0_step = u_opt[:2]
        last_good_u = u0_step
        u0 = u_opt[2:] + u_opt[-2:]
    except Exception:
        n_fail += 1
        u0_step = last_good_u          # fallback：沿用上一拍有效推力

    xn = np.array(x0, dtype=float) + T * np.array(
        f(x0, u0_step).full().flatten(), dtype=float)
    xn[3] += np.random.normal(0, position_noise_std)
    xn[4] += np.random.normal(0, position_noise_std)
    xn[5] = math.atan2(math.sin(xn[5]), math.cos(xn[5]))

    x0 = xn.tolist()
    control_history.append(u0_step)
    state_history.append(list(x0))

    # ---- 实时进度（每 5% 打印一次，flush 保证立即可见）----
    if (sim + 1) % max(1, sim_steps // 20) == 0 or sim == 0:
        el = time.time() - _t_sim
        prog = wp_mgr.total_advance / max(TARGET_ADVANCE, 1)
        eta = el / max(prog, 1e-6) * (1 - prog)
        print(f"      步 {sim + 1:>4}  圈进度 {prog*100:5.1f}%  "
              f"({wp_mgr.total_advance}/{TARGET_ADVANCE} 参考点)  "
              f"已用 {el:5.1f}s  预计还剩 {eta:5.1f}s  "
              f"横迹误差={lateral_errors[-1]:.4f} m", flush=True)

print(f"      仿真完成，用时 {time.time() - _t_sim:.2f} s"
      f"（求解失败/降级 {n_fail} 次）", flush=True)

# ==================== 统计 ====================
traj = np.array(state_history[1:])
traj_u, traj_v, traj_r = traj[:, 0], traj[:, 1], traj[:, 2]
traj_x, traj_y, traj_psi = traj[:, 3], traj[:, 4], traj[:, 5]
PWML = [c[0] for c in control_history]
PWMR = [c[1] for c in control_history]

print("=" * 76, flush=True)
print("仿真结果", flush=True)
print("=" * 76, flush=True)
_n_actual = len(control_history)
print(f"  总步数              : {_n_actual}  ({_n_actual * T:.1f} s)", flush=True)
print(f"  完成圈数            : {wp_mgr.total_advance / wp_mgr.n:.2f} 圈", flush=True)
print(f"  参考点切换次数      : {len(switch_steps)}"
      f"  (进圈 {switch_reasons.count('circle')} / "
      f"超时 {switch_reasons.count('timeout')})", flush=True)
print(f"  横向误差 Mean/Std   : {np.mean(lateral_errors):.4f} / "
      f"{np.std(lateral_errors):.4f} m", flush=True)
print(f"  航向误差 Mean/Std   : {np.mean(heading_errors):.4f} / "
      f"{np.std(heading_errors):.4f} rad", flush=True)
print(f"  求解失败次数        : {n_fail}", flush=True)

# ==================== 绘图（与原代码一致：plt.figure + 最后统一 plt.show）====================
if DO_PLOT:
    print("[3/3] 生成图片窗口 ...", flush=True)
    ref_x = [p[0] for p in path_points]
    ref_y = [p[1] for p in path_points]

    # ---------- 图 1：轨迹 + 航点 + 接纳圆 + 切换点 + 方向箭头 ----------
    plt.figure(figsize=(10, 6))
    plt.plot(ref_x, ref_y, 'r--', lw=1.8, label=LB['ref'])
    plt.plot(traj_x, traj_y, 'b-', lw=2.0, label=LB['traj'])

    # 接纳圆：只画淡轮廓，避免糊成一片
    ax = plt.gca()
    for i, pp in enumerate(waypoints):
        ax.add_patch(Circle(pp, R_ACCEPT, color='gray', alpha=0.10, lw=0.5,
                            fill=True, zorder=1))
    ax.add_patch(Circle(waypoints[0], R_ACCEPT, color='gray', alpha=0.10, lw=0.5,
                        label=LB['circle'] % R_ACCEPT))

    # 航点：小而醒目
    plt.scatter([pp[0] for pp in waypoints], [pp[1] for pp in waypoints],
                c='k', s=22, marker='^', zorder=5, label=LB['wp'])

    # 轨迹方向箭头：每隔若干步画一个，指明航行方向
    n_arrow = 10
    step_a = max(1, len(traj_x) // n_arrow)
    for i in range(step_a, len(traj_x) - 1, step_a):
        dx = traj_x[i + 1] - traj_x[i]
        dy = traj_y[i + 1] - traj_y[i]
        plt.annotate('', xy=(traj_x[i] + dx, traj_y[i] + dy),
                     xytext=(traj_x[i], traj_y[i]),
                     arrowprops=dict(arrowstyle='->', color='darkorange',
                                     lw=1.8, shrinkA=0, shrinkB=0), zorder=7)

    # 切换点
    if switch_steps:
        cs = np.array(switch_reasons); ss = np.array(switch_steps)
        mc, mt = cs == 'circle', cs == 'timeout'
        if mc.any():
            plt.scatter(traj_x[ss[mc]], traj_y[ss[mc]], c='lime', s=50, marker='o',
                        edgecolors='k', linewidths=.5, zorder=6, label=LB['sw_in'])
        if mt.any():
            plt.scatter(traj_x[ss[mt]], traj_y[ss[mt]], c='red', s=65, marker='X',
                        edgecolors='k', linewidths=.5, zorder=6, label=LB['sw_to'])

    plt.scatter([traj_x[0]], [traj_y[0]], c='g', s=110, marker='s',
                edgecolors='k', linewidths=.8, zorder=8, label=LB['start'])
    plt.scatter([traj_x[-1]], [traj_y[-1]], c='orange', s=110, marker='D',
                edgecolors='k', linewidths=.8, zorder=8, label=LB['end'])
    plt.xlabel(LB['x']); plt.ylabel(LB['y'])
    plt.title(LB['title1'] + '  (%.2f 圈)' % (wp_mgr.total_advance / wp_mgr.n))
    plt.legend(loc='upper right', fontsize=8); plt.grid(True, alpha=.4)
    plt.axis('equal')

    # ---------- 图 2：横迹误差 / 航向误差 / 航点索引 ----------
    plt.figure(figsize=(12, 6))
    plt.subplot(3, 1, 1)
    plt.plot(lateral_errors, label=LB['e1'], color='b')
    plt.axhline(y=np.mean(lateral_errors), color='b', linestyle='--',
                label='Mean: %.4f m' % np.mean(lateral_errors))
    plt.legend(fontsize=8); plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(heading_errors, label=LB['e2'], color='r')
    plt.axhline(y=np.mean(heading_errors), color='r', linestyle='--',
                label='Mean: %.4f rad' % np.mean(heading_errors))
    plt.legend(fontsize=8); plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(wp_index_history, color='purple', lw=1.6, label=LB['e3'])
    if switch_steps:
        cs = np.array(switch_reasons); ss = np.array(switch_steps)
        ih = np.array(wp_index_history)
        mc, mt = cs == 'circle', cs == 'timeout'
        if mc.any():
            plt.scatter(ss[mc], ih[ss[mc]], c='lime', s=45, edgecolors='k',
                        linewidths=.5, zorder=5, label=LB['iu'])
        if mt.any():
            plt.scatter(ss[mt], ih[ss[mt]], c='red', s=55, marker='X',
                        zorder=5, label=LB['it'])
    plt.xlabel(LB['step']); plt.ylabel('Waypoint Index')
    plt.legend(fontsize=8); plt.grid(True)
    plt.suptitle(LB['err_t'], fontsize=12)
    plt.tight_layout()

    # ---------- 图 3：状态变量 u / v / r / psi ----------
    plt.figure(figsize=(12, 6))
    for i, (dat, nm) in enumerate(zip([traj_u, traj_v, traj_r, traj_psi],
                                      ['u (m/s)', 'v (m/s)', 'r (rad/s)', 'psi (rad)']), 1):
        plt.subplot(4, 1, i)
        plt.plot(dat, lw=1.3, label=nm)
        plt.legend(fontsize=8); plt.grid(True)
    plt.xlabel(LB['step'])
    plt.suptitle(LB['st'], fontsize=12)
    plt.tight_layout()

    # ---------- 图 4：推进器输出 ----------
    plt.figure(figsize=(12, 6))
    plt.subplot(2, 1, 1)
    plt.plot(PWML, label=LB['tp'], color='blue')
    plt.legend(loc='upper center', fontsize=8)
    plt.title(LB['pr']); plt.xlabel(LB['step']); plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(PWMR, label=LB['ts'], color='blue')
    plt.legend(loc='upper center', fontsize=8)
    plt.xlabel(LB['step']); plt.grid(True)
    plt.tight_layout()

    print("      共 4 张图，正在显示（关闭全部窗口后程序结束）...", flush=True)
    plt.show()

print("全部完成。", flush=True)
