import numpy as np
from scipy.optimize import minimize
from scipy.signal import savgol_filter
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import pandas as pd
from sklearn.preprocessing import StandardScaler
# ========================================
# 1. 加载 CSV 数据
# ========================================
Start_row = 10
row = 250
# 读取Excel文件
# df = pd.read_excel('boat1_2_circle.xlsx').iloc[Start_row:Start_row+row]
# df = pd.read_excel('data/boat1_2_sin.xlsx').iloc[Start_row:Start_row+row]
# df = pd.read_excel('data/data_20260521_155805.xlsx').iloc[Start_row:Start_row+row]
df = pd.read_excel('data/usv2_data.xlsx').iloc[Start_row:Start_row+row]
timestamp = df['DateTime'].values
x = df['x'].values
y = df['y'].values
psi = df['Heading'].values
Ts = df['PWM_R'].values-1500
Tp = df['PWM_L'].values-1500

# ========================================
# 2. 角度预处理函数（改进版：加入弧度转换）
# ========================================

def unwrap_angle(psi):
    return (psi * np.pi / 180 + np.pi) % (2 * np.pi) - np.pi

def calculate_angular_velocity(psi, dt):
    # psi_centered = unwrap_angle(psi)
    psi_unwrapped = np.unwrap(psi)
    r = np.gradient(psi_unwrapped, dt)
    return r

# ========================================
# 3. 参数设置与数据准备（加入平滑滤波）
# ========================================

N_samples = len(x)
# dt_values = np.diff(np.array([pd.to_datetime(t).timestamp() for t in timestamp]))
# dt_values = np.append(dt_values, dt_values[-1])
dt = 0.1

dx_dt = np.gradient(x, dt)
dy_dt = np.gradient(y, dt)
psi = unwrap_angle(psi)
r = calculate_angular_velocity(psi, dt)

u = dx_dt * np.cos(psi) + dy_dt * np.sin(psi)
v = -dx_dt * np.sin(psi) + dy_dt * np.cos(psi)

# 平滑滤波
u = savgol_filter(u, window_length=5, polyorder=1)
v = savgol_filter(v, window_length=5, polyorder=1)
r = savgol_filter(r, window_length=5, polyorder=1)

# 构造状态变量和控制输入
X = np.column_stack((u[:-1], v[:-1], r[:-1]))
U = np.column_stack((Ts[:-1], Tp[:-1]))

# 控制输入归一化
# U_scaler = StandardScaler()
# U_scaled = U_scaler.fit_transform(U)
U_scaled = U
# 导数计算：savgol 
_swin = 9  #滑窗长度9点，拟合2次多项式，返回一阶导数 最小二乘拟合
du = savgol_filter(u, _swin, 2, deriv=1, delta=dt)
dv = savgol_filter(v, _swin, 2, deriv=1, delta=dt)
dr = savgol_filter(r, _swin, 2, deriv=1, delta=dt)
dX = np.column_stack((du[1:], dv[1:], dr[1:]))

# ARX 滞后项：k-1 时刻的导数（与 X、dX 逐行对齐，各 N-1 行）
dX_prev = np.column_stack((du[:-1], dv[:-1], dr[:-1]))

# ========================================
# 推进器延迟补偿
# 执行器延迟会让推力项与导数失去相关性，
# 回归便把解释权全塞给状态项（闭环共线性），开环仿真即漂移。
# 以 du/dr 单步回归损失最小选延迟，再映射输入。
# ========================================
_ones_col = np.ones(len(X))
_Phi_du0 = np.column_stack((X[:, 1] * X[:, 2], X[:, 0], X[:, 1], X[:, 2], _ones_col)) #回归矩阵
_Phi_dr0 = np.column_stack((X[:, 0] * X[:, 1], X[:, 0], X[:, 1], X[:, 2], _ones_col))

def _search_loss(U_try): #待优化序列TP TS  简单快速处理找最优 ，未做归一化，没导数项（导数项帮不上忙）
    tot = 0.0 #总残差
    for _Phi0, _ch in ((_Phi_du0, 0), (_Phi_dr0, 2)): #_ch = 0 目标输出取du _ch = 2 目标输出取dr
        Phi = np.column_stack((_Phi0, U_try[:, 1], U_try[:, 0]))#5列状态基地后加入推力
        #dX[:, _ch]取出要拟合的目标向量
        th, *_ = np.linalg.lstsq(Phi, dX[:, _ch], rcond=None)#最小二乘求解器`‖A·θ − b‖²`（求θ为7个） 最小 th为所求参数向量 dX为目标向量
        # tot += float(np.sum((Phi @ th - dX[:, _ch]) ** 2))
        resid = Phi @ th - dX[:, _ch]
        tot += float(np.sum(resid ** 2) / np.var(dX[:, _ch]))
        # tot = np.sum((Phi @ th - dX[:, _ch]) ** 2)
    return tot

_best = (np.inf, 0) #初始化最优解二元组（损失，延迟）损失初值为无穷，第一个候选值必更优
Tm_full = np.column_stack((Ts, Tp)) 
for _dly in range(0, 11): #找出使"单步回归残差"最小的那个，作为后续参数辨识的输入对齐依据 执行器响应有滞后
    if _dly:
        U_try = np.vstack([np.repeat(Tm_full[:1], _dly, axis=0), Tm_full])[:N_samples - 1]
    else:
        U_try = Tm_full[:N_samples - 1]#249 × 2
    _L = _search_loss(U_try)
    if _L < _best[0]:
        _best = (_L, _dly)

_dly_best = _best[1]  #求解最优对齐怎么对齐
print(f"延迟搜索: delay={_dly_best} 步 ({_dly_best * dt:.2f} s), 损失 {_best[0]:.4g}")
if _dly_best:
    U = np.vstack([np.repeat(Tm_full[:1], _dly_best, axis=0), Tm_full])[:N_samples - 1]
else:
    U = Tm_full[:N_samples - 1]
U_scaled = U

# ========================================
# 4. 模型方程定义
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

# ========================================
# 5. 损失函数（目标函数）
# ========================================

def loss_function(params, X, U, dX, dXp):
    pred = model_equations(params, X, U, dXp)
    error = pred - dX
    return np.sum(error ** 2)

# ========================================
# 6. 初始参数猜测
# ========================================
# 参数边界（物理约束）：
#  - 阻尼项 a2/b3/c4 只允许 <= 0（耗散方向，防止辨识出正反馈）
#  - 交叉滞后项固定为 0（主要在拟合导数噪声）
#  - 同通道滞后项只允许负反馈（稳定方向）
#  - b5/b6 固定为 0（差动艇侧向无推进力）
BIG = 50.0
_b_zero = (0.0, 0.0)  #参数固定为0 交叉导数项，侧向推力向
_b_ar = (-5.0, -0.05)  #同通道导数项边界  上一时刻导数对当前导数的惯性 必须负的，否则漂移
_b_damp = (-3.0, 0.0)    # 三个方向沿自己状态的阻尼项
_b_cpl = (-4.0, 4.0)     # 状态耦合项（v*r / u*r / u*v）
_b_cpl2 = (-2.0, 2.0)    # 跨状态耦合项边界
param_bounds = (
    [_b_cpl, _b_damp, _b_cpl2, _b_cpl2] + [(-BIG, BIG)] * 3 + [_b_ar, _b_zero, _b_zero]
    + [_b_cpl, (-BIG, BIG), _b_damp, _b_cpl2, _b_zero, _b_zero, (-BIG, BIG), _b_zero, _b_ar, _b_zero]
    + [_b_cpl, _b_cpl2, _b_cpl2, _b_damp] + [(-BIG, BIG)] * 3 + [_b_zero, _b_zero, _b_ar]
)

# ========================================
# 6. 初始参数：Ridge 线性最小二乘
#    模型对参数线性，直接解回归得到初值（数据集无关），
#    结构约束直接体现在回归列选取中
# ========================================

def ridge_solve(Phi, y, lam=1e-6):  #给定回归矩阵和目标  归一化+λ正则
    nrm = np.linalg.norm(Phi, axis=0)
    nrm[nrm == 0] = 1.0
    Pn = Phi / nrm #除以特征值长度
    th = np.linalg.solve(Pn.T @ Pn + lam * np.eye(Pn.shape[1]), Pn.T @ y)
    return th / nrm

_ones = np.ones_like(X[:, 0])
_nl = [X[:, 1] * X[:, 2], X[:, 0] * X[:, 2], X[:, 0] * X[:, 1]]  # v*r, u*r, u*v
_ar_own = [dX_prev[:, 0], dX_prev[:, 1], dX_prev[:, 2]]

theta0 = np.zeros(30)
for _ch in range(3):
    if _ch == 1:  # dv：不含推力项
        _Phi = np.column_stack((_nl[_ch], X[:, 0], X[:, 1], X[:, 2], _ones, _ar_own[_ch]))
    else:
        _Phi = np.column_stack((_nl[_ch], X[:, 0], X[:, 1], X[:, 2], U[:, 1], U[:, 0], _ones, _ar_own[_ch]))
    _th = ridge_solve(_Phi, dX[:, _ch])
    if _ch == 0:
        theta0[0:7], theta0[7] = _th[:7], np.clip(_th[7], -5.0, -0.05)
    elif _ch == 1:
        theta0[10:14], theta0[16], theta0[18] = _th[:4], _th[4], np.clip(_th[5], -5.0, -0.05)
    else:
        theta0[20:27], theta0[29] = _th[:7], np.clip(_th[7], -5.0, -0.05)

# 统一裁剪进边界
initial_params = np.array([np.clip(theta0[i], b[0], b[1]) for i, b in enumerate(param_bounds)])

# ========================================
# 7. 使用 SLSQP 进行参数优化
# ========================================

result = minimize(loss_function, initial_params, args=(X, U_scaled, dX, dX_prev), method='SLSQP', bounds=param_bounds, options={'disp': True})

# ========================================
# 7.5 多步预测误差精调（抑制 ARX 噪声偏差，约束开环稳定性）
# 长窗口开环仿真误差为目标：不稳定/漂移参数会使误差爆炸而被自动排除
# ========================================
H_multi = min(100, N_samples // 2)    # 多步预测步长（约半个数据长度，考核长时误差累积）
stride_multi = max(5, H_multi // 8)   # 起点间隔（密集覆盖全段）
pole_bound = 0.99   # 允许的最大离散极点模长
w_pole = 10         # 极点越界惩罚权重

def arx_derivs(params, u_, v_, r_, du_p, dv_p, dr_p, Ts_, Tp_):
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

def multi_step_loss(params):
    starts = range(0, N_samples - H_multi, stride_multi)#起点 终点 步长
    err_sum = 0.0
    for k0 in starts:
        u_p, v_p, r_p = u[k0], v[k0], r[k0]
        du_p, dv_p, dr_p = du[k0], dv[k0], dr[k0]
        psi_p = psi[k0]
        for k in range(k0 + 1, k0 + H_multi + 1):
            Ts_k, Tp_k = U_scaled[k-1]
            du_p, dv_p, dr_p = arx_derivs(params, u_p, v_p, r_p, du_p, dv_p, dr_p, Ts_k, Tp_k)
            # 软限幅（tanh）：有界且处处可导，避免硬限幅造成梯度死区
            u_p = 3.0 * np.tanh((u_p + dt * du_p) / 3.0)  #tanh光滑处处可导
            v_p = 1.0 * np.tanh((v_p + dt * dv_p) / 1.0)
            r_p = 5.0 * np.tanh((r_p + dt * dr_p) / 5.0)
            psi_p = psi_p + dt * r_p
            dpsi = (psi_p - psi[k] + np.pi) % (2 * np.pi) - np.pi  #计算航向误差，角度折回正负派
            err_sum += (u_p - u[k]) ** 2 + (v_p - v[k]) ** 2 + (r_p - r[k]) ** 2 + dpsi ** 2
    loss = err_sum / (len(starts) * H_multi * 4)

    # AR 极点越界惩罚：把离散极点压回单位圆内 同一个模型自己 vs 带了一点初始扰动的自己  导数状态对自身的延迟反馈环 其他参数有固定意义
    A_ar = np.array([[params[7], params[8], params[9]],
                     [params[17], params[18], params[19]],
                     [params[27], params[28], params[29]]])
    poles = np.abs(np.linalg.eigvals(np.eye(3) + dt * A_ar))
    loss += w_pole * np.sum(np.maximum(poles - pole_bound, 0.0) ** 2)
    return loss

if result.success:
    estimated_params = result.x
    print("参数优化成功！")

    # 多步精调（L-BFGS-B 数值梯度）
    ms_loss_init = multi_step_loss(estimated_params)
    res_multi = minimize(multi_step_loss, estimated_params, method='L-BFGS-B', #算法用有限差分自动算数值梯度
                         bounds=param_bounds, options={'maxiter': 150}) #限制迭代次数，计算时间过长
    print(f"多步精调: 多步损失 {ms_loss_init:.6g} -> {res_multi.fun:.6g}")
    estimated_params = res_multi.x

    # AR 环节离散稳定性检查（精调后）
    A_ar = np.array([[estimated_params[7], estimated_params[8], estimated_params[9]],
                     [estimated_params[17], estimated_params[18], estimated_params[19]],
                     [estimated_params[27], estimated_params[28], estimated_params[29]]])
    poles = np.abs(np.linalg.eigvals(np.eye(3) + dt * A_ar))
    print(f"AR 环节离散极点模长: {np.round(poles, 4)}, 最大 {poles.max():.4f}")
    if poles.max() >= 1:
        print("警告: 离散极点 >= 1, 开环仿真可能发散")
else:
    print("参数优化失败。")

# ========================================
# 8. 修复后的仿真函数（含运动学）
# ========================================

def simulate_with_params_and_kinematics(params, X0, U_all, dX0, x0, y0, psi0, dt, N_samples):
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
        Ts = U_all[k-1, 0]
        Tp = U_all[k-1, 1]

        u = u_sim[k-1]
        v = v_sim[k-1]
        r = r_sim[k-1]
        psi = psi_sim[k-1]

        du = a1 * v * r + a2 * u + a3 * v + a4 * r + a5 * Tp + a6 * Ts + a7 \
             + a8 * du_prev + a9 * dv_prev + a10 * dr_prev
        dv = b1 * u * r + b2 * u + b3 * v + b4 * r + b5 * Tp + b6 * Ts + b7 \
             + b8 * du_prev + b9 * dv_prev + b10 * dr_prev
        dr = c1 * u * v + c2 * u + c3 * v + c4 * r + c5 * Tp + c6 * Ts + c7 \
             + c8 * du_prev + c9 * dv_prev + c10 * dr_prev

        # 开环迭代：滞后项使用模型预测值（而非测量值）
        du_prev, dv_prev, dr_prev = du, dv, dr

        # 状态限幅，防止开环仿真发散到 nan/inf
        u_sim[k] = np.clip(u_sim[k-1] + dt * du, -3.0, 3.0)
        v_sim[k] = np.clip(v_sim[k-1] + dt * dv, -1.0, 1.0)
        r_sim[k] = np.clip(r_sim[k-1] + dt * dr, -5.0, 5.0)

        x_sim[k] = x_sim[k-1] + dt * (u * np.cos(psi) - v * np.sin(psi))
        y_sim[k] = y_sim[k-1] + dt * (u * np.sin(psi) + v * np.cos(psi))
        psi_sim[k] = psi_sim[k-1] + dt * r_sim[k-1]
        psi_sim[k] = (psi_sim[k] + np.pi) % (2 * np.pi) - np.pi

        du_est[k] = du
        dv_est[k] = dv
        dr_est[k] = dr

    return x_sim, y_sim, psi_sim, u_sim, v_sim, r_sim, du_est, dv_est, dr_est

# ========================================
# 9. 结果分析与绘图
# ========================================

if result.success:
    # 使用多步精调后的 estimated_params，不要覆盖为 result.x
    X0 = [u[0], v[0], r[0]]
    dX0 = [du[0], dv[0], dr[0]]
    x0, y0, psi0 = x[0], y[0], psi[0]
    U_all = U_scaled

    x_est, y_est, psi_est, u_est, v_est, r_est, du_est, dv_est, dr_est = simulate_with_params_and_kinematics(
        estimated_params, X0, U_all, dX0, x0, y0, psi0, dt, N_samples
    )

    time = np.arange(N_samples) * dt

    (a1, a2, a3, a4, a5, a6, a7, a8, a9, a10,
     b1, b2, b3, b4, b5, b6, b7, b8, b9, b10,
     c1, c2, c3, c4, c5, c6, c7, c8, c9, c10) = estimated_params

    # 打印模型方程（ARX：du'/dv'/dr' 为上一时刻导数）
    print("\n辨识的模型方程:")
    print(f"{a1:.6f}*v*r + {a2:.6f}*u + {a3:.6f}*v + {a4:.6f}*r + {a5:.6f}*Tp + {a6:.6f}*Ts + {a7:.6f}"
          f" + {a8:.6f}*du' + {a9:.6f}*dv' + {a10:.6f}*dr',")
    print(f"{b1:.6f}*u*r + {b2:.6f}*u + {b3:.6f}*v + {b4:.6f}*r + {b5:.6f}*Tp + {b6:.6f}*Ts + {b7:.6f}"
          f" + {b8:.6f}*du' + {b9:.6f}*dv' + {b10:.6f}*dr',")
    print(f"{c1:.6f}*u*v + {c2:.6f}*u + {c3:.6f}*v + {c4:.6f}*r + {c5:.6f}*Tp + {c6:.6f}*Ts + {c7:.6f}"
          f" + {c8:.6f}*du' + {c9:.6f}*dv' + {c10:.6f}*dr',")
    # 可视化状态变量导数
    plt.figure(0)
    plt.subplot(3, 1, 1)
    plt.plot(time[:-1], du[:-1], label='True du')
    plt.plot(time[:-1], du_est[1:], '--', label='Estimated du')
    plt.legend()
    plt.title('du: True vs Estimated')

    plt.subplot(3, 1, 2)
    plt.plot(time[:-1], dv[:-1], label='True dv')
    plt.plot(time[:-1], dv_est[1:], '--', label='Estimated dv')
    plt.legend()
    plt.title('dv: True vs Estimated')

    plt.subplot(3, 1, 3)
    plt.plot(time[:-1], dr[:-1], label='True dr')
    plt.plot(time[:-1], dr_est[1:], '--', label='Estimated dr')
    plt.legend()
    plt.title('dr: True vs Estimated')
    plt.tight_layout()

    # 可视化状态变量
    plt.figure(1)
    plt.subplot(4, 1, 1)
    plt.plot(time[:-1], u[:-1], label='True u')
    plt.plot(time[:-1], u_est[:-1], '--', label='Estimated u')
    plt.legend()
    plt.title('u: True vs Estimated')

    plt.subplot(4, 1, 2)
    plt.plot(time[:-1], v[:-1], label='True v')
    plt.plot(time[:-1], v_est[:-1], '--', label='Estimated v')
    plt.legend()
    plt.title('v: True vs Estimated')

    plt.subplot(4, 1, 3)
    plt.plot(time[:-1], r[:-1], label='True r')
    plt.plot(time[:-1], r_est[:-1], '--', label='Estimated r')
    plt.legend()
    plt.title('r: True vs Estimated')

    plt.subplot(4, 1, 4)
    plt.plot(time[:-1], psi[:-1], label='True psi')
    plt.plot(time[:-1], psi_est[:-1], '--', label='Estimated psi')
    plt.legend()
    plt.title('psi: True vs Estimated')
    plt.tight_layout()

    # 轨迹对比
    plt.figure(2)
    plt.plot(x, y, label='True Trajectory')
    plt.plot(x_est, y_est, '--', label='Estimated Trajectory')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.title('Trajectory Comparison')
    plt.grid(True)
    plt.axis('equal')
    plt.tight_layout()
    # ========================================
    # 新增：7. 推进器输出差值对比 (plt.figure(3))
    # ========================================

    plt.figure(3)

    # Ts（右推进器）
    plt.subplot(2, 1, 1)
    plt.plot(time, Ts, label='Right Propeller (Ts)', color='blue')
    plt.legend(loc='upper center')
    plt.title('Propeller Outputs')
    plt.xlabel('Time (s)')
    plt.ylabel('PWM Value (Ts)')
    plt.grid(True)

    # Tp（左推进器）
    plt.subplot(2, 1, 2)
    plt.plot(time, Tp, label='Left Propeller (Tp)', color='blue')
    plt.legend(loc='upper center')
    plt.xlabel('Time (s)')
    plt.ylabel('PWM Value (Tp)')
    plt.grid(True)

    plt.tight_layout()
    plt.show()