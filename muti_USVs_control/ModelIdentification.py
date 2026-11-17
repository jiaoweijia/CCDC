"""
模型辨识模块（供主界面"模型辨识"页调用）
========================================
算法与 example/demo2.py 完全一致（30 参数 ARX 模型）：
- 每通道 10 个参数（同通道滞后导数构成 AR 项）：
    du = a1*v*r + a2*u + a3*v + a4*r + a5*Tp + a6*Ts + a7
         + a8*du' + a9*dv' + a10*dr'
    dv = b1*u*r + b2*u + b3*v + b4*r + b5*Tp + b6*Ts + b7
         + b8*du' + b9*dv' + b10*dr'
    dr = c1*u*v + c2*u + c3*v + c4*r + c5*Tp + c6*Ts + c7
         + c8*du' + c9*dv' + c10*dr'
  （'表示上一时刻导数，ARX 滞后项）
- 流程：
    1. 推进器延迟网格搜索（0-10 步，du/dr 单步归一化残差）
    2. 逐通道 Ridge 线性最小二乘求初值（列归一化，结构约束体现在列选取）
    3. SLSQP / trust-constr 单步拟合（OLS 选项直接用 Ridge 解）
    4. L-BFGS-B 多步开环预测误差精调（tanh 软限幅 + AR 极点惩罚）
    5. 前向欧拉 ARX 开环仿真（含运动学），输出轨迹与 u/v/r/psi/du/dv/dr 对比
- 滤波器（Filter_Sel）：savgol / ekf / lowpass / none，差别体现在 u/v/r 平滑
  与 du/dv/dr 求导方式（对应 demo2 第56-58、70-72行）；savgol 选项与 demo2 完全一致
"""
import numpy as np
import pandas as pd
from scipy.optimize import minimize, Bounds, LinearConstraint
from scipy.signal import savgol_filter, butter, sosfiltfilt

REQUIRED_COLS = ['DateTime', 'x', 'y', 'Heading', 'PWM_L', 'PWM_R']

# ========================================
# 参数物理边界（与 demo2.py 一致）
#   阻尼项 a2/b3/c4 只允许 <= 0；交叉滞后项固定 0；
#   同通道滞后项只允许负反馈 (-5,-0.05)；b5/b6 固定 0（差动艇无侧向推力）
# ========================================
BIG = 50.0
_B_ZERO = (0.0, 0.0)
_B_AR = (-5.0, -0.05)
_B_DAMP = (-3.0, 0.0)
_B_CPL = (-4.0, 4.0)
_B_CPL2 = (-2.0, 2.0)
# 推力增益数量级约束：du 通道 a5/a6 为正增益；S0-S11 开环激励数据实测巡航仅 ~0.3 m/s，
# 对应增益 ~3.4e-4（10^-4 量级），下界取 1e-4 防止塌缩到 0。
# dr 通道 c5/c6 为 A×10^-3 量级（差速转向，c5 负 / c6 正）
_B_DU_THR = (1e-4, 9.9e-2)
_B_DR_THR_TP = (-9.9e-3, -1e-4)
_B_DR_THR_TS = (1e-4, 9.9e-3)
PARAM_BOUNDS = (
    [_B_CPL, _B_DAMP, _B_CPL2, _B_CPL2, _B_DU_THR, _B_DU_THR, (-BIG, BIG)]
    + [_B_AR, _B_ZERO, _B_ZERO]
    + [_B_CPL, (-BIG, BIG), _B_DAMP, _B_CPL2, _B_ZERO, _B_ZERO, (-BIG, BIG), _B_ZERO, _B_AR, _B_ZERO]
    + [_B_CPL, _B_CPL2, _B_CPL2, _B_DAMP, _B_DR_THR_TP, _B_DR_THR_TS, (-BIG, BIG)]
    + [_B_ZERO, _B_ZERO, _B_AR]
)

# ========================================
# du 通道推力增益等式约束 a5 == a6（左右推进器对纵向贡献相同）
#   方程仍保留 a5*Tp + a6*Ts 两个独立参数（不合并写法），约束通过四层实现：
#   Ridge 初值合并 (Tp+Ts) 列求解后均分、SLSQP 等式约束、
#   trust-constr LinearConstraint、多步 L-BFGS-B 二次惩罚 + 最终平均投影
# ========================================
_W_EQ_DU = 1e3
_A_DU_EQ = np.zeros((1, 30))
_A_DU_EQ[0, 4], _A_DU_EQ[0, 5] = 1.0, -1.0
_EQ_DU_SLSQP = {'type': 'eq', 'fun': lambda p: p[4] - p[5], 'jac': lambda p: _A_DU_EQ[0]}


# ========================================
# 数据读取
# ========================================
def load_excel_data(path, start_row, n_rows):
    """读取Excel指定行区间，返回预处理所需的基础数据"""
    df = pd.read_excel(path).iloc[start_row:start_row + n_rows]
    missing = [c for c in REQUIRED_COLS if c not in df.columns]
    if missing:
        raise ValueError(f"Excel缺少必要列: {missing}")
    use_cols = REQUIRED_COLS + [c for c in ('u', 'v') if c in df.columns]
    df = df.dropna(subset=use_cols).reset_index(drop=True)
    if len(df) < 20:
        raise ValueError(f"有效数据仅 {len(df)} 行（<20），请调整开始行/数据行")

    timestamp = df['DateTime'].to_numpy(dtype=float)
    dt = 0.1    # 固定采样周期

    return {
        'N': len(df),
        'dt': dt,
        't': timestamp - timestamp[0],
        'x': df['x'].to_numpy(dtype=float),
        'y': df['y'].to_numpy(dtype=float),
        'psi_deg': df['Heading'].to_numpy(dtype=float),
        'Tp': df['PWM_L'].to_numpy(dtype=float) - 1500.0,
        'Ts': df['PWM_R'].to_numpy(dtype=float) - 1500.0,
    }


# ========================================
# 滤波器
# ========================================
def _rts_kf_1d(z, dt, q, r):
    """常速模型卡尔曼滤波 + RTS平滑（用于 ekf 滤波选项），z为标量序列"""
    N = len(z)
    F = np.array([[1.0, dt], [0.0, 1.0]])
    H = np.array([[1.0, 0.0]])
    Q = q * np.array([[dt ** 3 / 3, dt ** 2 / 2], [dt ** 2 / 2, dt]])

    x = np.array([z[0], 0.0])
    P = np.diag([r, 1.0])
    x_filt = np.zeros((N, 2))
    P_filt = np.zeros((N, 2, 2))
    x_pred = np.zeros((N, 2))
    P_pred = np.zeros((N, 2, 2))

    for k in range(N):
        if k > 0:
            x = F @ x
            P = F @ P @ F.T + Q
        x_pred[k], P_pred[k] = x, P
        innov = z[k] - (H @ x)[0]
        S = (H @ P @ H.T)[0, 0] + r
        K = (P @ H.T / S).flatten()
        x = x + K * innov
        P = (np.eye(2) - K[:, None] @ H) @ P
        x_filt[k], P_filt[k] = x, P

    x_smooth = x_filt.copy()
    for k in range(N - 2, -1, -1):
        P_pred_inv = np.linalg.inv(P_pred[k + 1])
        C = P_filt[k] @ F.T @ P_pred_inv
        x_smooth[k] = x_filt[k] + C @ (x_smooth[k + 1] - x_pred[k + 1])
    return x_smooth[:, 0]


def _wrap_pi(angle):
    """角度折回 (-pi, pi]"""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def prepare_states(data, filter_name):
    """
    状态与导数准备。psi 折回 ±pi，r 由解包裹角度差分，u/v 由位置差分按机体系换算。
    滤波器选项的差别体现在 u/v/r 平滑与 du/dv/dr 求导方式（对应 demo2 第56-58、70-72行）：
      savgol : 与 demo2 完全一致——状态 savgol(5,1)，导数 savgol(9,2,deriv=1)
      ekf    : 常速KF+RTS 平滑位置/艏向后换算状态，导数对平滑状态直接差分
      lowpass: 3阶 Butterworth 零相位低通平滑状态，导数对平滑状态直接差分
      none   : 原始状态，导数直接差分
    返回 x, y, psi(±pi), u, v, r, du, dv, dr（长度均为 N）
    """
    dt = data['dt']
    x, y = data['x'].copy(), data['y'].copy()
    psi = _wrap_pi(np.deg2rad(data['psi_deg']))
    r = np.gradient(np.unwrap(psi), dt)
    vx = np.gradient(x, dt)
    vy = np.gradient(y, dt)
    u = vx * np.cos(psi) + vy * np.sin(psi)
    v = -vx * np.sin(psi) + vy * np.cos(psi)
  #原始数据滤波
    if filter_name == 'ekf':
        # 常速KF+RTS平滑位置与艏向（q 过小会过度平滑滞后），再换算 u,v
        q_pos, r_pos = 0.05, 0.01 ** 2
        q_psi, r_psi = 0.1, np.deg2rad(0.5) ** 2
        x = _rts_kf_1d(x, dt, q_pos, r_pos)
        y = _rts_kf_1d(y, dt, q_pos, r_pos)
        psi_s = _rts_kf_1d(np.unwrap(psi), dt, q_psi, r_psi)
        vx = np.gradient(x, dt)
        vy = np.gradient(y, dt)
        u = vx * np.cos(psi_s) + vy * np.sin(psi_s)
        v = -vx * np.sin(psi_s) + vy * np.cos(psi_s)
        r = np.gradient(psi_s, dt)
        psi = _wrap_pi(psi_s)
        # KF 已去噪，导数直接差分
        du = np.gradient(u, dt)
        dv = np.gradient(v, dt)
        dr = np.gradient(r, dt)
    elif filter_name == 'savgol':
        # demo2 第56-58行：状态平滑 window=5, polyorder=1
        u = savgol_filter(u, window_length=5, polyorder=1)
        v = savgol_filter(v, window_length=5, polyorder=1)
        r = savgol_filter(r, window_length=5, polyorder=1)
        # demo2 第70-72行：导数用滑窗9点、2阶多项式、一阶导数
        _swin = 9
        du = savgol_filter(u, _swin, 2, deriv=1, delta=dt)
        dv = savgol_filter(v, _swin, 2, deriv=1, delta=dt)
        dr = savgol_filter(r, _swin, 2, deriv=1, delta=dt)
    elif filter_name == 'lowpass':
        fs = 1.0 / dt
        wn = 1.5    # 截止频率 Hz
        if wn < fs / 2:
            sos = butter(3, wn, btype='low', fs=fs, output='sos')
            try:
                u = sosfiltfilt(sos, u)
                v = sosfiltfilt(sos, v)
                r = sosfiltfilt(sos, r)
            except Exception:
                pass    # 信号过短滤波失败时退回原始数据
        du = np.gradient(u, dt)
        dv = np.gradient(v, dt)
        dr = np.gradient(r, dt)
    else:   # none：原始数据，导数直接差分
        du = np.gradient(u, dt)
        dv = np.gradient(v, dt)
        dr = np.gradient(r, dt)

    return x, y, psi, u, v, r, du, dv, dr


# ========================================
# Ridge 线性最小二乘（列归一化 + lambda 正则）
# ========================================
def ridge_solve(Phi, y, lam=1e-6):
    nrm = np.linalg.norm(Phi, axis=0)
    nrm[nrm == 0] = 1.0
    Pn = Phi / nrm
    th = np.linalg.solve(Pn.T @ Pn + lam * np.eye(Pn.shape[1]), Pn.T @ y)
    return th / nrm


# ========================================
# 30 参数 ARX 模型
# ========================================
def model_equations(params, X, U, dXp):
    (a1, a2, a3, a4, a5, a6, a7, a8, a9, a10,
     b1, b2, b3, b4, b5, b6, b7, b8, b9, b10,
     c1, c2, c3, c4, c5, c6, c7, c8, c9, c10) = params
    u_vec, v_vec, r_vec = X[:, 0], X[:, 1], X[:, 2]
    Ts_vec, Tp_vec = U[:, 0], U[:, 1]
    du_p, dv_p, dr_p = dXp[:, 0], dXp[:, 1], dXp[:, 2]

    du_model = a1 * v_vec * r_vec + a2 * u_vec + a3 * v_vec + a4 * r_vec + a5 * Tp_vec + a6 * Ts_vec + a7 \
               + a8 * du_p + a9 * dv_p + a10 * dr_p
    dv_model = b1 * u_vec * r_vec + b2 * u_vec + b3 * v_vec + b4 * r_vec + b5 * Tp_vec + b6 * Ts_vec + b7 \
               + b8 * du_p + b9 * dv_p + b10 * dr_p
    dr_model = c1 * u_vec * v_vec + c2 * u_vec + c3 * v_vec + c4 * r_vec + c5 * Tp_vec + c6 * Ts_vec + c7 \
               + c8 * du_p + c9 * dv_p + c10 * dr_p

    return np.column_stack((du_model, dv_model, dr_model))


def loss_function(params, X, U, dX, dXp):
    pred = model_equations(params, X, U, dXp)
    error = pred - dX
    return np.sum(error ** 2)


def arx_derivs(params, u_, v_, r_, du_p, dv_p, dr_p, Ts_, Tp_):
    """标量状态下的 ARX 导数（多步开环递推用）"""
    (a1, a2, a3, a4, a5, a6, a7, a8, a9, a10,
     b1, b2, b3, b4, b5, b6, b7, b8, b9, b10,
     c1, c2, c3, c4, c5, c6, c7, c8, c9, c10) = params
    du = a1 * v_ * r_ + a2 * u_ + a3 * v_ + a4 * r_ + a5 * Tp_ + a6 * Ts_ + a7 \
         + a8 * du_p + a9 * dv_p + a10 * dr_p
    dv = b1 * u_ * r_ + b2 * u_ + b3 * v_ + b4 * r_ + b5 * Tp_ + b6 * Ts_ + b7 \
         + b8 * du_p + b9 * dv_p + b10 * dr_p
    dr = c1 * u_ * v_ + c2 * u_ + c3 * v_ + c4 * r_ + c5 * Tp_ + c6 * Ts_ + c7 \
         + c8 * du_p + c9 * dv_p + c10 * dr_p
    return du, dv, dr


def format_equations(params):
    (a1, a2, a3, a4, a5, a6, a7, a8, a9, a10,
     b1, b2, b3, b4, b5, b6, b7, b8, b9, b10,
     c1, c2, c3, c4, c5, c6, c7, c8, c9, c10) = params
    f = lambda v: f"{v:.6f}"
    eqs = [
        f"du = {f(a1)}*v*r + {f(a2)}*u + {f(a3)}*v + {f(a4)}*r + {f(a5)}*Tp + {f(a6)}*Ts + {f(a7)}"
        f" + {f(a8)}*du' + {f(a9)}*dv' + {f(a10)}*dr'",
        f"dv = {f(b1)}*u*r + {f(b2)}*u + {f(b3)}*v + {f(b4)}*r + {f(b5)}*Tp + {f(b6)}*Ts + {f(b7)}"
        f" + {f(b8)}*du' + {f(b9)}*dv' + {f(b10)}*dr'",
        f"dr = {f(c1)}*u*v + {f(c2)}*u + {f(c3)}*v + {f(c4)}*r + {f(c5)}*Tp + {f(c6)}*Ts + {f(c7)}"
        f" + {f(c8)}*du' + {f(c9)}*dv' + {f(c10)}*dr'",
    ]
    return [eq.replace('+ -', '- ') for eq in eqs]


# ========================================
# ARX 开环仿真（前向欧拉 + 运动学，与 demo2 一致）
# ========================================
def simulate(params, X0, U_all, dX0, x0, y0, psi0, dt, N_samples):
    (a1, a2, a3, a4, a5, a6, a7, a8, a9, a10,
     b1, b2, b3, b4, b5, b6, b7, b8, b9, b10,
     c1, c2, c3, c4, c5, c6, c7, c8, c9, c10) = params

    u_sim = np.zeros(N_samples)
    v_sim = np.zeros(N_samples)
    r_sim = np.zeros(N_samples)
    x_sim = np.zeros(N_samples)
    y_sim = np.zeros(N_samples)
    psi_sim = np.zeros(N_samples)

    u_sim[0], v_sim[0], r_sim[0] = X0
    x_sim[0], y_sim[0], psi_sim[0] = x0, y0, psi0

    du_est = np.zeros(N_samples)
    dv_est = np.zeros(N_samples)
    dr_est = np.zeros(N_samples)

    # ARX 滞后导数初值：用 k=0 的测量导数播种，之后迭代模型预测值
    du_prev, dv_prev, dr_prev = dX0
    du_est[0], dv_est[0], dr_est[0] = du_prev, dv_prev, dr_prev

    for k in range(1, N_samples):
        Ts = U_all[k - 1, 0]
        Tp = U_all[k - 1, 1]

        u = u_sim[k - 1]
        v = v_sim[k - 1]
        r = r_sim[k - 1]
        psi = psi_sim[k - 1]

        du = a1 * v * r + a2 * u + a3 * v + a4 * r + a5 * Tp + a6 * Ts + a7 \
             + a8 * du_prev + a9 * dv_prev + a10 * dr_prev
        dv = b1 * u * r + b2 * u + b3 * v + b4 * r + b5 * Tp + b6 * Ts + b7 \
             + b8 * du_prev + b9 * dv_prev + b10 * dr_prev
        dr = c1 * u * v + c2 * u + c3 * v + c4 * r + c5 * Tp + c6 * Ts + c7 \
             + c8 * du_prev + c9 * dv_prev + c10 * dr_prev

        # 开环迭代：滞后项使用模型预测值（而非测量值）
        du_prev, dv_prev, dr_prev = du, dv, dr

        # 状态限幅，防止开环仿真发散到 nan/inf
        u_sim[k] = np.clip(u + dt * du, -3.0, 3.0)
        v_sim[k] = np.clip(v + dt * dv, -1.0, 1.0)
        r_sim[k] = np.clip(r + dt * dr, -5.0, 5.0)

        x_sim[k] = x_sim[k - 1] + dt * (u * np.cos(psi) - v * np.sin(psi))
        y_sim[k] = y_sim[k - 1] + dt * (u * np.sin(psi) + v * np.cos(psi))
        psi_sim[k] = _wrap_pi(psi_sim[k - 1] + dt * r_sim[k - 1])

        du_est[k] = du
        dv_est[k] = dv
        dr_est[k] = dr

    return x_sim, y_sim, psi_sim, u_sim, v_sim, r_sim, du_est, dv_est, dr_est


# ========================================
# 主流程
# ========================================
def run_identification(path, start_row, n_rows, filter_name, optimizer_name, h_multi=100):
    # ---------- 1. 数据与状态准备（u/v/r 与 du/dv/dr 由所选滤波器决定） ----------
    data = load_excel_data(path, start_row, n_rows)
    x, y, psi, u, v, r, du, dv, dr = prepare_states(data, filter_name)
    Ts, Tp, dt = data['Ts'], data['Tp'], data['dt']
    N_samples = data['N']

    # 状态/控制（各 N-1 行，对应 k=0..N-2）
    X = np.column_stack((u[:-1], v[:-1], r[:-1]))
    dX = np.column_stack((du[1:], dv[1:], dr[1:]))
    # ARX 滞后项：k-1 时刻的导数
    dX_prev = np.column_stack((du[:-1], dv[:-1], dr[:-1]))

    # ---------- 2. 推进器延迟补偿 ----------
    # 执行器延迟使推力项与导数失去相关性，回归会把解释权塞给状态项（闭环共线性）。
    # 简化模型（无 AR 项、无归一化求解）按 du/dr 归一化残差搜索 delay=0..10 步。
    _ones_col = np.ones(len(X))
    _Phi_du0 = np.column_stack((X[:, 1] * X[:, 2], X[:, 0], X[:, 1], X[:, 2], _ones_col))
    _Phi_dr0 = np.column_stack((X[:, 0] * X[:, 1], X[:, 0], X[:, 1], X[:, 2], _ones_col))

    def _search_loss(U_try):
        tot = 0.0
        for _Phi0, _ch in ((_Phi_du0, 0), (_Phi_dr0, 2)):
            Phi = np.column_stack((_Phi0, U_try[:, 1], U_try[:, 0]))
            th, *_ = np.linalg.lstsq(Phi, dX[:, _ch], rcond=None)
            resid = Phi @ th - dX[:, _ch]
            # 按通道方差归一化，避免量纲大的通道主导延迟选择
            tot += float(np.sum(resid ** 2) / np.var(dX[:, _ch]))
        return tot

    _best = (np.inf, 0)
    Tm_full = np.column_stack((Ts, Tp))   # 列顺序：[Ts, Tp]，长度 N
    for _dly in range(0, 11):
        if _dly:
            U_try = np.vstack([np.repeat(Tm_full[:1], _dly, axis=0), Tm_full])[:N_samples - 1]
        else:
            U_try = Tm_full[:N_samples - 1]
        _L = _search_loss(U_try)
        if _L < _best[0]:
            _best = (_L, _dly)

    dly_best = _best[1]
    delay_msg = f"延迟={dly_best}步({dly_best * dt:.2f}s), 搜索损失={_best[0]:.4g}"
    if dly_best:
        U = np.vstack([np.repeat(Tm_full[:1], dly_best, axis=0), Tm_full])[:N_samples - 1]
    else:
        U = Tm_full[:N_samples - 1]
    U_scaled = U

    _ones = np.ones_like(X[:, 0])
    _nl = [X[:, 1] * X[:, 2], X[:, 0] * X[:, 2], X[:, 0] * X[:, 1]]  # v*r, u*r, u*v
    _ar_own = [dX_prev[:, 0], dX_prev[:, 1], dX_prev[:, 2]]

    theta0 = np.zeros(30)
    for _ch in range(3):
        if _ch == 0:    # du：推力合并 (Tp+Ts) 单列求解保证 a5==a6，解出后均分到 a5/a6
            _Phi = np.column_stack((_nl[_ch], X[:, 0], X[:, 1], X[:, 2],
                                    U[:, 1] + U[:, 0], _ones, _ar_own[_ch]))
            _th = ridge_solve(_Phi, dX[:, _ch])
            theta0[0:4] = _th[:4]
            theta0[4] = theta0[5] = _th[4]
            theta0[6], theta0[7] = _th[5], np.clip(_th[6], -5.0, -0.05)
        elif _ch == 1:  # dv：不含推力项
            _Phi = np.column_stack((_nl[_ch], X[:, 0], X[:, 1], X[:, 2], _ones, _ar_own[_ch]))
            _th = ridge_solve(_Phi, dX[:, _ch])
            theta0[10:14], theta0[16], theta0[18] = _th[:4], _th[4], np.clip(_th[5], -5.0, -0.05)
        else:
            _Phi = np.column_stack((_nl[_ch], X[:, 0], X[:, 1], X[:, 2],
                                    U[:, 1], U[:, 0], _ones, _ar_own[_ch]))
            _th = ridge_solve(_Phi, dX[:, _ch])
            theta0[20:27], theta0[29] = _th[:7], np.clip(_th[7], -5.0, -0.05)

    # 统一裁剪进物理边界
    initial_params = np.array([np.clip(theta0[i], b[0], b[1])
                               for i, b in enumerate(PARAM_BOUNDS)])

    # ---------- 4. 单步拟合优化 ----------
    # 未收敛不中断：回退 SLSQP → 再退 Ridge 初值，后续仍有多步 L-BFGS-B 精调兜底
    if optimizer_name == 'OLS':
        # 模型对参数线性，直接使用 Ridge 闭式解（已裁剪进边界）不迭代
        estimated_params = initial_params.copy()
        message = 'OLS (Ridge closed-form least squares)'
    else:
        if optimizer_name == 'trust-constr':
            lb = np.array([b[0] for b in PARAM_BOUNDS])
            ub = np.array([b[1] for b in PARAM_BOUNDS])
            # gtol 过严(1e-8)时数值梯度难以满足，会假报"优化失败"，放宽到 1e-6
            result = minimize(loss_function, initial_params, args=(X, U_scaled, dX, dX_prev),
                              method='trust-constr', bounds=Bounds(lb, ub),
                              constraints=LinearConstraint(_A_DU_EQ, 0.0, 0.0),
                              options={'maxiter': 200, 'gtol': 1e-6, 'xtol': 1e-8, 'disp': False})
        else:   # SLSQP（demo2 方法）
            result = minimize(loss_function, initial_params, args=(X, U_scaled, dX, dX_prev),
                              method='SLSQP', bounds=PARAM_BOUNDS, constraints=[_EQ_DU_SLSQP],
                              options={'maxiter': 400, 'disp': False})
        if result.success:
            estimated_params = result.x
            message = str(result.message)
        elif optimizer_name != 'SLSQP':
            # trust-constr 失败时回退 SLSQP 再试一次
            result2 = minimize(loss_function, initial_params, args=(X, U_scaled, dX, dX_prev),
                               method='SLSQP', bounds=PARAM_BOUNDS, constraints=[_EQ_DU_SLSQP],
                               options={'maxiter': 400, 'disp': False})
            if result2.success:
                estimated_params = result2.x
                message = f"{optimizer_name} 未收敛({result.message})，已回退 SLSQP"
            else:
                estimated_params = initial_params.copy()
                message = f"{optimizer_name}/SLSQP 均未收敛，用 Ridge 初值直接进入多步精调"
        else:
            estimated_params = initial_params.copy()
            message = f"SLSQP 未收敛({result.message})，用 Ridge 初值直接进入多步精调"
    final_loss = float(loss_function(estimated_params, X, U_scaled, dX, dX_prev))

    # ---------- 5. 多步预测误差精调（长窗口开环，约束长时漂移与稳定性） ----------
    # h_multi 由 UI 输入（默认 100），数据过短时自动压到 N/2 保证窗口放得下
    H_multi = min(max(10, int(h_multi)), N_samples // 2)
    stride_multi = max(5, H_multi // 8)
    pole_bound = 0.99
    w_pole = 10

    def multi_step_loss(params):
        starts = range(0, N_samples - H_multi, stride_multi)
        err_sum = 0.0
        for k0 in starts:
            u_p, v_p, r_p = u[k0], v[k0], r[k0]
            du_p, dv_p, dr_p = du[k0], dv[k0], dr[k0]
            psi_p = psi[k0]
            for k in range(k0 + 1, k0 + H_multi + 1):
                Ts_k, Tp_k = U_scaled[k - 1]
                du_p, dv_p, dr_p = arx_derivs(params, u_p, v_p, r_p,
                                              du_p, dv_p, dr_p, Ts_k, Tp_k)
                # 软限幅（tanh）：有界且处处可导，避免硬限幅造成梯度死区
                u_p = 3.0 * np.tanh((u_p + dt * du_p) / 3.0)
                v_p = 1.0 * np.tanh((v_p + dt * dv_p) / 1.0)
                r_p = 5.0 * np.tanh((r_p + dt * dr_p) / 5.0)
                psi_p = psi_p + dt * r_p
                dpsi = _wrap_pi(psi_p - psi[k])
                err_sum += ((u_p - u[k]) ** 2 + (v_p - v[k]) ** 2
                            + (r_p - r[k]) ** 2 + dpsi ** 2)
        loss = err_sum / (len(starts) * H_multi * 4)

        # AR 极点越界惩罚：把离散极点压回单位圆内
        A_ar = np.array([[params[7], params[8], params[9]],
                         [params[17], params[18], params[19]],
                         [params[27], params[28], params[29]]])
        poles = np.abs(np.linalg.eigvals(np.eye(3) + dt * A_ar))
        loss += w_pole * np.sum(np.maximum(poles - pole_bound, 0.0) ** 2)
        # a5==a6 二次惩罚（L-BFGS-B 不支持等式约束）
        loss += _W_EQ_DU * (params[4] - params[5]) ** 2
        return loss

    ms_loss_init = multi_step_loss(estimated_params)
    res_multi = minimize(multi_step_loss, estimated_params, method='L-BFGS-B',
                         bounds=PARAM_BOUNDS, options={'maxiter': 150})
    estimated_params = res_multi.x
    # a5/a6 最终平均投影：消除惩罚法的残余漂移，保证约束严格成立
    _a5_raw, _a6_raw = estimated_params[4], estimated_params[5]
    _a56 = 0.5 * (_a5_raw + _a6_raw)
    estimated_params[4] = estimated_params[5] = _a56
    multi_loss_final = float(multi_step_loss(estimated_params))

    # 精调后 AR 环节离散稳定性体检
    A_ar = np.array([[estimated_params[7], estimated_params[8], estimated_params[9]],
                     [estimated_params[17], estimated_params[18], estimated_params[19]],
                     [estimated_params[27], estimated_params[28], estimated_params[29]]])
    poles = np.abs(np.linalg.eigvals(np.eye(3) + dt * A_ar))
    pole_msg = f"极点={np.round(poles, 4)}, 最大={poles.max():.4f}"
    if poles.max() >= 1:
        pole_msg += " [警告: 极点>=1, 开环仿真可能发散]"

    # ---------- 6. 全段开环仿真对比 ----------
    X0 = [u[0], v[0], r[0]]
    dX0 = [du[0], dv[0], dr[0]]
    x0, y0, psi0 = x[0], y[0], psi[0]

    x_sim, y_sim, psi_sim, u_sim, v_sim, r_sim, du_model, dv_model, dr_model = simulate(
        estimated_params, X0, U_scaled, dX0, x0, y0, psi0, dt, N_samples
    )

    rmse = {
        'u': float(np.sqrt(np.mean((u_sim - u) ** 2))),
        'v': float(np.sqrt(np.mean((v_sim - v) ** 2))),
        'r': float(np.sqrt(np.mean((r_sim - r) ** 2))),
        'psi': float(np.sqrt(np.mean((np.unwrap(psi_sim) - np.unwrap(psi)) ** 2))),
        'traj': float(np.mean(np.sqrt((x_sim - x) ** 2 + (y_sim - y) ** 2))),
        'du': float(np.sqrt(np.mean((du_model[1:] - du[1:]) ** 2))),
        'dv': float(np.sqrt(np.mean((dv_model[1:] - dv[1:]) ** 2))),
        'dr': float(np.sqrt(np.mean((dr_model[1:] - dr[1:]) ** 2))),
    }

    # 状态信息（对应 demo2 的 print 输出，供 UI 左侧状态栏显示）
    status_lines = [
        f"延迟搜索: delay={dly_best} 步 ({dly_best * dt:.2f} s), 损失 {_best[0]:.4g}",
        f"单步优化[{optimizer_name}]: {message}",
        f"多步精调(H={H_multi}): 多步损失 {ms_loss_init:.6g} -> {multi_loss_final:.6g}",
        f"du 推力增益约束: a5=a6={_a56:.6g}（多步精调漂移 {abs(_a5_raw - _a6_raw):.3g}，已投影）",
        f"AR 环节离散极点模长: {np.round(poles, 4)}, 最大 {poles.max():.4f}",
    ]
    if poles.max() >= 1:
        status_lines.append("警告: 离散极点 >= 1, 开环仿真可能发散")

    return {
        't': np.arange(N_samples) * dt,
        'dt': dt,
        'x': x, 'y': y, 'psi': psi,
        'u': u, 'v': v, 'r': r,
        'x_sim': x_sim, 'y_sim': y_sim, 'psi_sim': psi_sim,
        'u_sim': u_sim, 'v_sim': v_sim, 'r_sim': r_sim,
        'du': du, 'dv': dv, 'dr': dr,
        'du_model': du_model, 'dv_model': dv_model, 'dr_model': dr_model,
        'params': estimated_params,
        'equations': format_equations(estimated_params),
        'loss': multi_loss_final,
        'message': f"{message}; {delay_msg}; 单步损失={final_loss:.6g}; 多步损失 {ms_loss_init:.6g}->{multi_loss_final:.6g}; {pole_msg}",
        'filter': filter_name,
        'optimizer': optimizer_name,
        'rmse': rmse,
        'status_lines': status_lines,
    }
