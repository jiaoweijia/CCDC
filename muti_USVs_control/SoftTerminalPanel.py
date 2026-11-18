"""Qt controls for soft predicted-energy contraction (not a certificate)."""
from collections import deque
from dataclasses import dataclass, field

import numpy as np
from PySide6.QtCore import QSignalBlocker
from PySide6.QtWidgets import (QHBoxLayout, QLabel, QComboBox, QCheckBox,
                               QDoubleSpinBox, QSpinBox)
from SoftTerminalMPC import preview_from_points

TURN_LOOKAHEAD_S = .5
TURN_EXIT_RATIO = .70
TURN_EXIT_FRAMES = 3


def reference_curvature(points):
    """Unsigned curvature (1/m) of the circles through consecutive triplets."""
    first = points[1:-1] - points[:-2]
    second = points[2:] - points[1:-1]
    lengths = np.column_stack([np.linalg.norm(v, axis=1) for v in (first, second, first+second)])
    valid = np.all(lengths > 1e-9, axis=1)
    cross = first[:, 0]*second[:, 1] - first[:, 1]*second[:, 0]
    curvature = np.full(len(first), np.nan)
    curvature[valid] = 2*np.abs(cross[valid])/np.prod(lengths[valid], axis=1)
    return curvature


@dataclass
class TurnSwitchState:
    identity: str
    last_t: float
    in_turn: bool = False
    exit_frames: int = 0
    recent: deque = field(default_factory=lambda: deque(maxlen=120))


class SoftTerminalPanelMixin:
    def _setup_soft_terminal_controls(self):
        row = QHBoxLayout()
        self.terminal_mode_selector = QComboBox()
        self.terminal_mode_selector.addItem('自适应终端＋软下降（弯道自动切换）', 'adaptive')
        self.terminal_mode_selector.addItem('固定终端权重（σ=1）', 'fixed')
        self.terminal_mode_selector.addItem('原启发式终端（对照）', 'legacy')
        row.addWidget(self.terminal_mode_selector)
        self.terminal_soft_check = QCheckBox('启用软下降')
        self.terminal_soft_check.setChecked(True); row.addWidget(self.terminal_soft_check)
        row.addWidget(QLabel('入弯曲率'))
        self.turn_curvature_spin = QDoubleSpinBox()
        self.turn_curvature_spin.setRange(.05, 100.)
        self.turn_curvature_spin.setSingleStep(.1)
        self.turn_curvature_spin.setSuffix(' m⁻¹')
        self.turn_curvature_spin.setValue(1.50)
        self.turn_curvature_spin.setToolTip('前看0.5秒。出弯门槛为入弯曲率的70%，连续满足3拍后切回自适应。')
        row.addWidget(self.turn_curvature_spin)
        self.layout.addLayout(row)
        row = QHBoxLayout(); row.addWidget(QLabel('未来H步下降目标'))
        self.terminal_rate_spin = QDoubleSpinBox()
        self.terminal_rate_spin.setRange(.001, 99.); self.terminal_rate_spin.setDecimals(3)
        self.terminal_rate_spin.setSuffix(' %'); self.terminal_rate_spin.setValue(10.)
        self.terminal_rate_spin.setToolTip('预测H步后的误差能量目标，不是实船每拍保证下降率。')
        row.addWidget(self.terminal_rate_spin); row.addWidget(QLabel('H'))
        self.terminal_steps_spin = QSpinBox(); self.terminal_steps_spin.setRange(0, 10)
        self.terminal_steps_spin.setSpecialValueText('N'); self.terminal_steps_spin.setValue(0)
        self.terminal_steps_spin.setToolTip('0显示N，表示使用预测末端；其余为检查步数。')
        row.addWidget(self.terminal_steps_spin); row.addWidget(QLabel('松弛惩罚λ'))
        self.terminal_penalty_spin = QDoubleSpinBox()
        self.terminal_penalty_spin.setRange(.01, 10000.); self.terminal_penalty_spin.setValue(10.)
        self.terminal_penalty_spin.setToolTip('归一化目标中的松弛平方惩罚；越大越重视下降要求。')
        row.addWidget(self.terminal_penalty_spin); self.layout.addLayout(row)
        self.terminal_status_label = QLabel('软下降：等待运行；仅预测检查，无稳定性认证')
        self.terminal_status_label.setWordWrap(True); self.layout.addWidget(self.terminal_status_label)
        self._terminal_messages = {}
        self._corner_states = {}
        for spin in (self.terminal_rate_spin, self.terminal_steps_spin, self.terminal_penalty_spin):
            spin.editingFinished.connect(self._apply_soft_terminal_controls)
        self.terminal_mode_selector.currentIndexChanged.connect(self._apply_soft_terminal_controls)
        self.terminal_soft_check.toggled.connect(self._apply_soft_terminal_controls)
        self.turn_curvature_spin.editingFinished.connect(self._apply_soft_terminal_controls)
        self._terminal_selected_control = getattr(self.usv_manager, 'control_law', 'NMPC')
        self._apply_soft_terminal_controls()

    def _terminal_is_active(self):
        return self._terminal_selected_control == 'NMPC'

    def terminal_controls_config(self):
        values = self._terminal_options()
        values['turn_curvature'] = self.turn_curvature_spin.value()
        return {key: str(value) for key, value in values.items()}

    def restore_terminal_controls(self, values):
        widgets = (self.terminal_mode_selector, self.terminal_soft_check,
                   self.terminal_rate_spin, self.terminal_steps_spin,
                   self.terminal_penalty_spin, self.turn_curvature_spin)
        blockers = [QSignalBlocker(widget) for widget in widgets]
        try:
            index = self.terminal_mode_selector.findData(values.get('mode', 'adaptive'))
            if index >= 0:
                self.terminal_mode_selector.setCurrentIndex(index)
            self.terminal_soft_check.setChecked(str(values.get('enabled', 'True')).lower() == 'true')
            self.terminal_rate_spin.setValue(100*float(values.get('rate', .10)))
            self.terminal_steps_spin.setValue(int(values.get('steps', 0)))
            self.terminal_penalty_spin.setValue(float(values.get('penalty', 10.)))
            self.turn_curvature_spin.setValue(float(values.get('turn_curvature', 1.50)))
        finally:
            del blockers
        self._apply_soft_terminal_controls()

    def _terminal_options(self):
        return dict(mode=self.terminal_mode_selector.currentData(),
                    enabled=self.terminal_soft_check.isChecked(),
                    rate=self.terminal_rate_spin.value()/100,
                    steps=self.terminal_steps_spin.value(),
                    penalty=self.terminal_penalty_spin.value())

    def _sync_soft_terminal(self, system, effective_mode=None):
        if system is not None:
            options = self._terminal_options()
            if effective_mode is not None:
                options['mode'] = effective_mode
            if not self._terminal_is_active() or system.current_control != 'NMPC':
                options.update(mode='legacy', enabled=False)
            system.mpc.configure_terminal(**options)

    def _apply_soft_terminal_controls(self, *_):
        active = self._terminal_is_active()
        self.terminal_mode_selector.setEnabled(active)
        legacy = self.terminal_mode_selector.currentData() == 'legacy'
        self.turn_curvature_spin.setEnabled(active and self.terminal_mode_selector.currentData() == 'adaptive')
        self.terminal_soft_check.setEnabled(active and not legacy)
        enabled = active and not legacy and self.terminal_soft_check.isChecked()
        for widget in (self.terminal_rate_spin, self.terminal_steps_spin, self.terminal_penalty_spin):
            widget.setEnabled(enabled)
        try:
            if self.usv_manager is not None:
                systems = list(self.usv_manager.control_systems.values())
                if systems:
                    self.terminal_steps_spin.setMaximum(min(s.mpc.N for s in systems))
                for system in systems:
                    self._sync_soft_terminal(system)
            self._terminal_messages.clear()
            self._corner_states.clear()
            self.terminal_status_label.setText('设置已应用；等待下一拍预测（无稳定性认证）' if active else '当前控制律：'+self._terminal_selected_control+'；终端机制未启用（仅NMPC生效）')
        except ValueError as exc:
            self.terminal_status_label.setText('设置未应用：'+str(exc))

    def _ellipse_preview(self, usv_id, system):
        n = system.mpc.N; dt = system.mpc.T
        original_x, original_y, interval = self.usv_offsets[usv_id]
        points = []
        for j in range(-1, n+3):
            t = self.current_t+j*self.t_step
            xb, yb = original_x, original_y
            if (self.exchange_control_enabled and self.exchange_time > 0
                    and t >= self.exchange_time_threshold
                    and usv_id in self.exchange_biases and usv_id in self.exchange_biases_from):
                start = np.asarray(self.exchange_biases_from[usv_id], float)
                end = np.asarray(self.exchange_biases[usv_id], float)
                fraction = 1. if self.exchange_duration_threshold <= 0 else np.clip(
                    (t-self.exchange_time_threshold)/self.exchange_duration_threshold, 0., 1.)
                xb, yb = start+(end-start)*fraction
            x, y = self.calculate_point(t, interval=interval*np.pi/100)
            a = self.calculate_leader_direction(t)
            points.append([x+xb*np.cos(a)-yb*np.sin(a), y+xb*np.sin(a)+yb*np.cos(a)])
        identity = repr((usv_id, self.traj_type, self.x_coeff, self.y_coeff,
                         self.x_offset, self.y_offset, self.omega_val, self.t_step,
                         tuple(self.usv_offsets[usv_id]), self.exchange_control_enabled,
                         self.exchange_time, self.exchange_duration,
                         self.exchange_biases.get(usv_id), self.exchange_biases_from.get(usv_id)))
        points = np.asarray(points, float)
        preview = preview_from_points(points, dt, identity)
        preview['curvature'] = reference_curvature(points)[:n+1]
        return preview

    def _choose_turn_mode(self, usv_id, preview, dt, boat_position):
        curvature = preview['curvature']
        # Include the node just beyond 0.5 s if the sampling period doesn't divide it.
        lookahead = min(len(curvature), int(np.ceil(TURN_LOOKAHEAD_S/dt))+1)
        ahead = curvature[:lookahead]
        if not np.any(np.isfinite(ahead)):
            return None
        ahead_max = float(np.nanmax(ahead))
        positions = preview['X'][:lookahead, 3:5]
        state = self._corner_states.get(usv_id)
        if state is None or state.identity != preview['identity'] or self.current_t <= state.last_t:
            state = TurnSwitchState(preview['identity'], self.current_t)
            self._corner_states[usv_id] = state
        state.last_t = self.current_t
        samples = np.column_stack((positions, ahead))
        samples = samples[np.isfinite(samples[:, 2])]
        if np.isfinite(curvature[0]):
            state.recent.append((*positions[0], curvature[0]))

        boat_curvature = None
        if boat_position is not None and np.all(np.isfinite(boat_position)):
            if state.recent:
                samples = np.vstack((state.recent, samples))
            nearest = np.argmin(np.sum((samples[:, :2]-boat_position)**2, axis=1))
            boat_curvature = float(samples[nearest, 2])
        observed_curvature = max(ahead_max, boat_curvature) if boat_curvature is not None else ahead_max
        enter = self.turn_curvature_spin.value()
        exit_threshold = TURN_EXIT_RATIO*enter
        reason = 'straight'
        if state.in_turn:
            state.exit_frames = state.exit_frames+1 if observed_curvature <= exit_threshold else 0
            if state.exit_frames >= TURN_EXIT_FRAMES:
                state.in_turn = False
                state.exit_frames = 0
                reason = 'exit'
            else:
                reason = 'hold_turn'
        elif observed_curvature >= enter:
            state.in_turn = True
            reason = 'enter'
        return dict(curvature_ahead=ahead_max, curvature_at_boat=boat_curvature,
                    turn_switch_active=state.in_turn, turn_switch_reason=reason)

    def _control_with_terminal(self, usv_id, *args, **kwargs):
        system = self.usv_manager.get_control_system(usv_id)
        selected_mode = self.terminal_mode_selector.currentData()
        effective_mode = selected_mode
        turn = None
        if (system is not None and system.current_control == 'NMPC'
                and self.tracking_mode == 'TTracking'
                and not kwargs.get('obstacles')
                and selected_mode != 'legacy'):
            try:
                kwargs['reference_preview'] = self._ellipse_preview(usv_id, system)
            except ValueError:
                # Static/degenerate geometry is explicitly displayed as frozen guidance.
                kwargs['reference_preview'] = None
            preview = kwargs['reference_preview']
            if selected_mode == 'adaptive' and preview is not None:
                state = kwargs.get('usv_state', args[2] if len(args) >= 3 else None)
                if state is None:
                    state = system.get_state()
                measured = np.asarray(state, float).reshape(-1)
                position = measured[3:5] if measured.size >= 5 else None
                turn = self._choose_turn_mode(usv_id, preview, system.mpc.T, position)
                if turn is not None and turn['turn_switch_active']:
                    effective_mode = 'legacy'
                    kwargs['reference_preview'] = None
        if turn is None:
            self._corner_states.pop(usv_id, None)
        self._sync_soft_terminal(system, effective_mode)
        result = self.usv_manager.control_usv(usv_id, *args, **kwargs)
        if result is not None and result.get('terminal'):
            d = result['terminal']
            d.update(selected_mode=selected_mode, effective_mode=effective_mode)
            if turn is not None:
                d.update(turn)
            if d.get('mode') == 'legacy':
                message = (f"{usv_id}：弯道自动切换 → 原启发式终端" if turn is not None and turn['turn_switch_active']
                           else f"{usv_id}：原启发式终端，无软下降约束")
            elif not d.get('enabled'):
                message = f"{usv_id}：软下降关闭，σ={d.get('sigma', 1):.3g}"
            else:
                flag = '目标达到' if d['target_met'] else '目标已放松'
                fallback = '；备用计划' if d['mode'] == 'soft_recovery' else ''
                source = '；固定制导目标' if d['reference_source'].startswith('frozen') else ''
                message = (f"{usv_id}：未来{d['check_steps']}步目标{100*d['rate_target']:.3g}%，"
                    f"V {d['energy_current']:.3g}→{d['energy_predicted']:.3g}，"
                    f"s={d['slack']:.3g}，σ={d['sigma']:.3g}；{flag}{fallback}{source}")
            if turn is not None:
                if turn['turn_switch_reason'] == 'exit':
                    message = '出弯，恢复自适应｜'+message
                message += f"｜前方曲率 {turn['curvature_ahead']:.2f} m⁻¹"
                if turn['curvature_at_boat'] is not None:
                    message += f"，船位曲率 {turn['curvature_at_boat']:.2f} m⁻¹"
            self._terminal_messages[usv_id] = message
            messages = [self._terminal_messages[k] for k in self.active_usvs if k in self._terminal_messages]
            self.terminal_status_label.setText('\n'.join(messages or [message])+'\n仅预测检查，无稳定性认证')
        return result
