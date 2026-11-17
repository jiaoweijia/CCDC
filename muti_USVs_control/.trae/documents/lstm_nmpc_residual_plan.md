# LSTM-NMPC 残差补偿融合方案

## 一、需求理解

将 LSTM 预测模型与 NMPC 控制器结合，采用**残差补偿**方式：
- 保留现有 21 参数线性动力学模型作为基础
- 用 LSTM 学习线性模型的预测误差（残差）
- NMPC 使用"线性模型 + LSTM 残差补偿"进行状态预测
- 在 `demo2.py` 中验证融合效果

## 二、技术方案

### 2.1 融合架构

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   历史状态序列   │────▶│   LSTM 残差模型  │────▶│   残差补偿输出   │
│  [u,v,r,Tp,Ts]  │     │  (8维输入,3维输出) │     │  [e_du,e_dv,e_dr]│
└─────────────────┘     └─────────────────┘     └─────────────────┘
         │                                               │
         │                                               ▼
         │                                       ┌─────────────────┐
         │                                       │  线性模型预测    │
         └──────────────────────────────────────▶│  + 残差补偿     │
                                                 │  = 混合预测输出  │
                                                 └─────────────────┘
                                                          │
                                                          ▼
                                                 ┌─────────────────┐
                                                 │   NMPC 优化求解  │
                                                 │  (CasADi/IPOPT)  │
                                                 └─────────────────┘
```

### 2.2 LSTM 模型设计

- **输入特征（8 维/步）**: `[u, v, r, du_lin, dv_lin, dr_lin, Tp, Ts]`
- **序列长度**: L=15 步（1.5 秒 @ dt=0.1s）
- **输出（3 维）**: 残差导数 `[e_du, e_dv, e_dr]`
- **网络结构**:
  ```
  Input(L, 8) → LSTM(hidden=64, num_layers=1) → 取最后一步隐状态
              → Linear(64→32) + Tanh → Linear(32→3)
  ```

### 2.3 训练数据构造

1. 用已辨识的 21 参数计算线性模型导数预测 `dX_lin`
2. 残差标签: `res = dX_true - dX_lin`
3. 滑窗构造样本：输入 `Feat[t-L:t]`，标签 `res[t]`
4. 归一化：输入特征用 `StandardScaler`
5. 数据划分：按时间顺序 70% / 15% / 15%（train/val/test）

### 2.4 验证方案

在 `demo2.py` 中实现三种对比：

| 对比项 | 内容 | 指标 |
|--------|------|------|
| A. 单步导数预测 | 真实 vs 线性模型 vs 线性+LSTM | RMSE, MAE, R² |
| B. 自由 rollout | 轨迹对比 | 终端位置误差, 轨迹 RMSE |
| C. 多步预测 | 预测步数 vs RMSE | NMPC 关心的 N 步累计误差 |

## 三、实现步骤

### 步骤 1: 创建 LSTM 残差模块
- 新建 `example/lstm_residual.py`
- 实现 `LSTMResidualNet` 类（PyTorch nn.Module）
- 实现 `build_sequences()` 构造训练样本
- 实现 `train_lstm()` 训练函数（含 early stopping）
- 实现 `simulate_hybrid()` 混合预测函数

### 步骤 2: 修改 demo2.py
- 在现有线性辨识代码后追加 LSTM 训练代码
- 读取多船数据（usv1+usv2+usv3）构造训练集
- 调用 `lstm_residual` 模块进行训练
- 生成对比图表

### 步骤 3: 验证与调优
- 运行 `demo2.py` 验证 LSTM 残差补偿效果
- 调整超参数（hidden_size, seq_len, lr 等）
- 保存最佳模型到 `example/checkpoints/lstm_residual.pt`

## 四、关键文件

| 文件 | 操作 | 说明 |
|------|------|------|
| `example/lstm_residual.py` | 新建 | LSTM 残差模型核心模块 |
| `example/demo2.py` | 修改 | 追加 LSTM 训练与验证代码 |
| `example/checkpoints/` | 新建目录 | 保存训练好的模型 |

## 五、依赖

- PyTorch 2.13.0（已安装）
- numpy, pandas, scikit-learn, matplotlib（已有）

## 六、验证方式

1. 运行 `python example/demo2.py`
2. 观察输出：
   - 参数辨识结果
   - LSTM 训练过程（loss 曲线）
   - 对比图表（单步预测、轨迹、多步预测）
   - 精度指标表格
3. 检查 `example/checkpoints/lstm_residual.pt` 是否生成

## 七、后续扩展（第二阶段）

验证通过后，可将 LSTM 残差注入 `USVController.py` 的 NMPC：
- 扩展 `P_state` 参数向量，追加 N 步残差序列
- 修改预测方程为 `X[:, k+1] = X[:, k] + T * (f(...) + res[k])`
- 在 `update()` 中先用 LSTM 预测残差，再求解 NMPC
