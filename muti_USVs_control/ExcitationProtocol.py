"""
S0-S11 开环激励协议（用于 30 参数 ARX 模型辨识的数据采集）
=========================================================
特点：
- 不使用任何控制率/制导率，也不追踪虚拟领航点；只按协议给左右电机下发原始 PWM。
- PWM 为绝对值，1500 为中位（停桨）。模型辨识中：Tp = PWM_L - 1500（左桨）、
  Ts = PWM_R - 1500（右桨）。
- 电机有死区：正向 PWM < 1630、反向 PWM > 1400 时不产生推力（死区带 1401~1629）。
  因此协议里所有非零档位都刻意避开死区（正向 >=1650、反向 <=1300）。默认死区基准
  1630/1400，与 TrajectoryPlotWidget（NMPC 下发路径）一致；可用 validate() 校验。
- 本模块只负责“时序 + 目标 PWM 计算”，是纯 Python（不依赖 Qt），便于无头测试；
  实际下发命令、记录数据、急停由 MainWindow 侧的 QTimer 驱动完成。

协议分段的设计意图（对应参数辨识）：
  S0  静止基准          零位/基线校验
  S1  双桨同步阶跃 +300   a5=a6 推力增益、a2 加速阻尼、a8
  S2  双桨降档 +100      第二工作点，锁定 a2（与 S1 联立）
  S3  自由滑行           推力=0，干净辨识 a2 / a7 / a8
  S4  只推左桨 +200       c5（差速转向增益）、b1 / b4
  S5  只推右桨 +200       c6，并与 S4 做左右对称性校验
  S6  反差动脉冲          c5/c6 鲁棒性、反向（若电机支持）
  S7  回转滑行           推力=0，干净辨识 c4 / c10
  S8  双桨正弦推力调制    u 单独变化，分离 c2 / c3
  S9  zigzag 差动换向     r 换向使 u·r / v·r / u·v 与 u 解耦，辨识 a1 / b1 / c1 及 v 项
  S10 突停 + 满差动       耦合项瞬态补强
  S11 收尾滑行           a7 终值校验
"""
import math

PWM_MID = 1500          # 中位（停桨）
PWM_MIN = 1000          # 下发下限
PWM_MAX = 2000          # 下发上限
PWM_DEAD_POS = 1630     # 正转死区基准（PWM 低于此值正转不转）——与 TrajectoryPlotWidget 默认一致
PWM_DEAD_NEG = 1400     # 反转死区基准（PWM 高于此值反转不转）


def _const(name, dur, offset_l, offset_r):
    """固定档位段：offset 为相对 1500 的偏置（+300 -> PWM 1800）"""
    return {'name': name, 'dur': dur,
            'pwm': lambda t, l=PWM_MID + offset_l, r=PWM_MID + offset_r: (l, r)}


def _clamp(v):
    return int(max(PWM_MIN, min(PWM_MAX, v)))


# 协议表：每段含 name / dur(秒) / pwm(t_local) -> (pwm_l, pwm_r)
EXCITATION_PROTOCOL = [
    _const('S0 静止基准',            5,    0,    0),
    _const('S1 双桨同步阶跃+300',     10,  300,  300),
    _const('S2 双桨降档+150',         6,  150,  150),   # +150 -> PWM 1650，避开正转死区
    _const('S3 自由滑行到停',         12,    0,    0),
    _const('S4 只推左桨+200',         8,  200,    0),
    _const('S5 只推右桨+200',          8,    0,  200),
    _const('S6 反差动脉冲',            4,  200, -200),
    _const('S7 回转滑行',              6,    0,    0),
    # 基线 +250、振幅 100 -> PWM 1650~1850，最低点仍高于正转死区 1630
    {'name': 'S8 双桨正弦推力调制', 'dur': 6.0,
     'pwm': lambda t: (_clamp(PWM_MID + 250 + 100 * math.sin(2 * math.pi * t / 4.0)),) * 2},
    # 高低桨 1900/1300 交替，两侧都在死区之外（正向 >1630、反向 <=1400）
    {'name': 'S9 zigzag 差动换向', 'dur': 15.0,
     'pwm': lambda t: ((PWM_MID + 400, PWM_MID - 200) if int(t // 3) % 2 == 0
                       else (PWM_MID - 200, PWM_MID + 400))},
    {'name': 'S10 突停+满差动', 'dur': 4.0,
     'pwm': lambda t: (_clamp(PWM_MID + 200), _clamp(PWM_MID - 200)) if t < 3.0
                      else (_clamp(PWM_MID), _clamp(PWM_MID))},
    _const('S11 收尾滑行到全停',      5.0,    0,    0),
]


class ExcitationProtocol:
    """S0-S11 状态机：按协议表推进，逐拍返回当前应下发的 PWM。"""

    def __init__(self, protocol=None, base_positive=PWM_DEAD_POS, base_nagitive=PWM_DEAD_NEG):
        self.protocol = list(protocol) if protocol is not None else list(EXCITATION_PROTOCOL)
        self.total_duration = sum(seg['dur'] for seg in self.protocol)
        self.base_positive = base_positive
        self.base_nagitive = base_nagitive
        self._start = None
        self._abort = False

    def start(self, now):
        """记录起始时刻（now 建议传 time.monotonic()）"""
        self._start = now
        self._abort = False

    def abort(self):
        self._abort = True

    @property
    def aborted(self):
        return self._abort

    def _sample(self, elapsed):
        """按“段内已过去的时间”求 (段名, pwm_l, pwm_r)"""
        acc = 0.0
        for seg in self.protocol:
            if elapsed < acc + seg['dur']:
                pwm_l, pwm_r = seg['pwm'](elapsed - acc)
                return (seg['name'], int(pwm_l), int(pwm_r))
            acc += seg['dur']
        return ('已完成', PWM_MID, PWM_MID)

    def tick(self, now):
        """按当前时刻返回 (段名, pwm_l, pwm_r, finished)。

        finished=True 表示协议跑完或已中止；此时返回停桨值 (1500, 1500)。
        """
        if self._abort or self._start is None:
            return ('已中止', PWM_MID, PWM_MID, True)
        elapsed = now - self._start
        if elapsed >= self.total_duration:
            return ('已完成', PWM_MID, PWM_MID, True)
        name, pwm_l, pwm_r = self._sample(elapsed)
        return (name, pwm_l, pwm_r, False)

    def validate(self, sample_dt=0.1):
        """按 sample_dt 采样整条协议，返回落入电机死区带
        (base_nagitive, base_positive) 且非停桨(1500) 的样本列表。

        空列表 = 所有非零指令都能真正驱动电机；非空则说明会有“无效段”
        （日志记录了偏置但电机不转），会污染辨识数据，应调整档位。
        """
        lo, hi = self.base_nagitive, self.base_positive
        bad = []
        t = 0.0
        while t < self.total_duration:
            name, pwm_l, pwm_r = self._sample(t)
            for v in (pwm_l, pwm_r):
                if v != PWM_MID and lo < v < hi:
                    bad.append((round(t, 2), name, pwm_l, pwm_r))
                    break
            t += sample_dt
        return bad
