"""Qt controls for soft predicted-energy contraction (not a certificate)."""
import numpy as np
from PySide6.QtCore import QSignalBlocker
from PySide6.QtWidgets import (QHBoxLayout, QLabel, QComboBox, QCheckBox,
                               QDoubleSpinBox, QSpinBox)
from SoftTerminalMPC import preview_from_points, curvature_from_points


class SoftTerminalPanelMixin:
    def _setup_soft_terminal_controls(self):
        row = QHBoxLayout()
        self.terminal_mode_selector = QComboBox()
        self.terminal_mode_selector.addItem('自适应终端＋软下降（位置优先）', 'adaptive')
        self.terminal_mode_selector.addItem('固定终端权重（σ=1）', 'fixed')
        self.terminal_mode_selector.addItem('原启发式终端（对照）', 'legacy')
        row.addWidget(self.terminal_mode_selector)
        self.terminal_soft_check = QCheckBox('启用软下降')
        self.terminal_soft_check.setChecked(True); row.addWidget(self.terminal_soft_check)
        self.layout.addLayout(row)
        row = QHBoxLayout(); row.addWidget(QLabel('未来H步下降目标'))
        self.terminal_rate_spin = QDoubleSpinBox()
        self.terminal_rate_spin.setRange(.001, 99.); self.terminal_rate_spin.setDecimals(3)
        self.terminal_rate_spin.setSuffix(' %'); self.terminal_rate_spin.setValue(10.)
        self.terminal_rate_spin.setToolTip('位置优先模式同时设置预测H步后位置误差平方和综合误差能量的下降目标。')
        row.addWidget(self.terminal_rate_spin); row.addWidget(QLabel('H'))
        self.terminal_steps_spin = QSpinBox(); self.terminal_steps_spin.setRange(0, 10)
        self.terminal_steps_spin.setSpecialValueText('N'); self.terminal_steps_spin.setValue(0)
        self.terminal_steps_spin.setToolTip('0显示N，表示使用预测末端；其余为检查步数。')
        row.addWidget(self.terminal_steps_spin); row.addWidget(QLabel('松弛惩罚λ'))
        self.terminal_penalty_spin = QDoubleSpinBox()
        self.terminal_penalty_spin.setRange(.01, 10000.); self.terminal_penalty_spin.setValue(10.)
        self.terminal_penalty_spin.setToolTip('归一化目标中的松弛平方惩罚；越大越重视下降要求。')
        row.addWidget(self.terminal_penalty_spin); self.layout.addLayout(row)
        row = QHBoxLayout()
        self.terminal_curvature_check = QCheckBox('按曲率切回原启发式')
        self.terminal_curvature_check.setChecked(True)
        row.addWidget(self.terminal_curvature_check)
        row.addWidget(QLabel('入弯 κ ≥'))
        self.terminal_curvature_enter_spin = QDoubleSpinBox()
        self.terminal_curvature_enter_spin.setDecimals(3)
        self.terminal_curvature_enter_spin.setRange(.001, 1000.)
        self.terminal_curvature_enter_spin.setValue(.85)
        self.terminal_curvature_enter_spin.setSuffix(' 1/m')
        row.addWidget(self.terminal_curvature_enter_spin)
        row.addWidget(QLabel('出弯 κ ≤'))
        self.terminal_curvature_exit_spin = QDoubleSpinBox()
        self.terminal_curvature_exit_spin.setDecimals(3)
        self.terminal_curvature_exit_spin.setRange(0., .849)
        self.terminal_curvature_exit_spin.setValue(.70)
        self.terminal_curvature_exit_spin.setSuffix(' 1/m')
        row.addWidget(self.terminal_curvature_exit_spin)
        self.layout.addLayout(row)
        self.terminal_curvature_check.setToolTip('仅自适应模式：入弯停用软下降，出弯自动恢复；两门槛之间保持当前模式。')
        self.terminal_curvature_enter_spin.valueChanged.connect(
            lambda value: self.terminal_curvature_exit_spin.setMaximum(max(0., value-.001)))
        self.terminal_status_label = QLabel('软下降：等待运行；仅预测检查，无稳定性认证')
        self.terminal_status_label.setWordWrap(True); self.layout.addWidget(self.terminal_status_label)
        self._terminal_messages = {}
        self._terminal_reference_history = {}
        for spin in (self.terminal_rate_spin, self.terminal_steps_spin, self.terminal_penalty_spin,
                     self.terminal_curvature_enter_spin, self.terminal_curvature_exit_spin):
            spin.editingFinished.connect(self._apply_soft_terminal_controls)
        self.terminal_mode_selector.currentIndexChanged.connect(self._apply_soft_terminal_controls)
        self.terminal_soft_check.toggled.connect(self._apply_soft_terminal_controls)
        self.terminal_curvature_check.toggled.connect(self._apply_soft_terminal_controls)
        self._terminal_selected_control = getattr(self.usv_manager, 'control_law', 'NMPC')
        self._apply_soft_terminal_controls()

    def _terminal_is_active(self):
        return self._terminal_selected_control == 'NMPC'

    def terminal_controls_config(self):
        options = self._terminal_options()
        return {key: str(value) for key, value in options.items()}

    def restore_terminal_controls(self, values):
        widgets = (self.terminal_mode_selector, self.terminal_soft_check,
                   self.terminal_rate_spin, self.terminal_steps_spin, self.terminal_penalty_spin,
                   self.terminal_curvature_check, self.terminal_curvature_enter_spin,
                   self.terminal_curvature_exit_spin)
        blockers = [QSignalBlocker(widget) for widget in widgets]
        try:
            index = self.terminal_mode_selector.findData(values.get('mode', 'adaptive'))
            if index >= 0:
                self.terminal_mode_selector.setCurrentIndex(index)
            self.terminal_soft_check.setChecked(str(values.get('enabled', 'True')).lower() == 'true')
            self.terminal_rate_spin.setValue(100*float(values.get('rate', .10)))
            self.terminal_steps_spin.setValue(int(values.get('steps', 0)))
            self.terminal_penalty_spin.setValue(float(values.get('penalty', 10.)))
            self.terminal_curvature_check.setChecked(str(values.get('curvature_switch', 'True')).lower() == 'true')
            self.terminal_curvature_enter_spin.setValue(float(values.get('curvature_enter', .85)))
            self.terminal_curvature_exit_spin.setMaximum(self.terminal_curvature_enter_spin.value()-.001)
            self.terminal_curvature_exit_spin.setValue(float(values.get('curvature_exit', .70)))
        finally:
            del blockers
        self._apply_soft_terminal_controls()

    def _terminal_options(self):
        return dict(mode=self.terminal_mode_selector.currentData(),
                    enabled=self.terminal_soft_check.isChecked(),
                    rate=self.terminal_rate_spin.value()/100,
                    steps=self.terminal_steps_spin.value(),
                    penalty=self.terminal_penalty_spin.value(),
                    curvature_switch=self.terminal_curvature_check.isChecked(),
                    curvature_enter=self.terminal_curvature_enter_spin.value(),
                    curvature_exit=self.terminal_curvature_exit_spin.value())

    def _sync_soft_terminal(self, system):
        if system is not None:
            if self._terminal_is_active() and system.current_control == 'NMPC':
                system.mpc.configure_terminal(**self._terminal_options())
            else:
                # Inactive algorithms never enter the soft-terminal optimizer.
                system.mpc.configure_terminal(mode='legacy', enabled=False)

    def _apply_soft_terminal_controls(self, *_):
        active = self._terminal_is_active()
        legacy = self.terminal_mode_selector.currentData() == 'legacy'
        self.terminal_mode_selector.setEnabled(active)
        self.terminal_soft_check.setEnabled(active and not legacy)
        enabled = active and not legacy and self.terminal_soft_check.isChecked()
        for widget in (self.terminal_rate_spin, self.terminal_steps_spin, self.terminal_penalty_spin):
            widget.setEnabled(enabled)
        adaptive = active and self.terminal_mode_selector.currentData() == 'adaptive'
        self.terminal_curvature_check.setEnabled(adaptive)
        for widget in (self.terminal_curvature_enter_spin, self.terminal_curvature_exit_spin):
            widget.setEnabled(adaptive and self.terminal_curvature_check.isChecked())
        try:
            if self.usv_manager is not None:
                systems = list(self.usv_manager.control_systems.values())
                if systems:
                    self.terminal_steps_spin.setMaximum(min(s.mpc.N for s in systems))
                for system in systems:
                    self._sync_soft_terminal(system)
            self._terminal_messages.clear()
            self._terminal_reference_history.clear()
            if active:
                self.terminal_status_label.setText('设置已应用；等待下一拍预测（无稳定性认证）')
            else:
                self.terminal_status_label.setText('当前控制律：'+self._terminal_selected_control+'；终端机制未启用（仅NMPC生效）')
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
        return preview_from_points(points, dt, identity)

    def _control_with_terminal(self, usv_id, *args, **kwargs):
        system = self.usv_manager.get_control_system(usv_id)
        self._sync_soft_terminal(system)
        if system is not None and system.current_control == 'NMPC' and len(args) >= 2:
            current, following = args[:2]
            if current is not None and following is not None:
                current = np.asarray(current, float)
                previous = self._terminal_reference_history.get(usv_id)
                if previous is not None:
                    kwargs['reference_curvature'] = curvature_from_points([previous, current, following])
                if previous is None or np.linalg.norm(current-previous) > 1e-9:
                    self._terminal_reference_history[usv_id] = current.copy()
        if (system is not None and system.current_control == 'NMPC'
                and self.tracking_mode == 'TTracking'
                and not kwargs.get('obstacles')
                and self.terminal_mode_selector.currentData() != 'legacy'):
            try:
                kwargs['reference_preview'] = self._ellipse_preview(usv_id, system)
            except ValueError:
                # Static/degenerate geometry is explicitly displayed as frozen guidance.
                kwargs['reference_preview'] = None
        result = self.usv_manager.control_usv(usv_id, *args, **kwargs)
        if result is not None and result.get('terminal'):
            d = result['terminal']
            if d.get('mode') == 'legacy':
                message = str(usv_id)+'：原启发式终端，无软下降约束'
                if d.get('curvature_fallback'):
                    message += '（入弯保持；出弯自动恢复自适应）'
            elif not d.get('enabled'):
                message = f"{usv_id}：软下降关闭，σ={d.get('sigma', 1):.3g}"
            else:
                flag = '目标达到' if d['target_met'] else '目标已放松'
                fallback = '；备用计划' if d['mode'] == 'soft_recovery' else ''
                source = '；固定制导目标' if d['reference_source'].startswith('frozen') else ''
                message = (f"{usv_id}：未来{d['check_steps']}步目标{100*d['rate_target']:.3g}%，"
                    f"V {d['energy_current']:.3g}→{d['energy_predicted']:.3g}，"
                    f"s={d['slack']:.3g}，σ={d['sigma']:.3g}；{flag}{fallback}{source}")
                if d.get('position_priority'):
                    position_flag = '达标' if d['position_target_met'] else '未达标'
                    message = (f"{usv_id}：未来{d['check_steps']}步，位置 "
                               f"{100*d['position_current_m']:.1f}→{100*d['position_predicted_m']:.1f} cm"
                               f"（目标≤{100*d['position_target_m']:.1f} cm，{position_flag}）\n"
                               f"综合V {d['energy_current']:.3g}→{d['energy_predicted']:.3g}，"
                               f"s={d['slack']:.3g}，σ={d['sigma']:.3g}；能量{flag}{fallback}{source}")
            if d.get('curvature') is not None:
                message += f"；κ={d['curvature']:.3f} 1/m"
            self._terminal_messages[usv_id] = message
            messages = [self._terminal_messages[k] for k in self.active_usvs if k in self._terminal_messages]
            self.terminal_status_label.setText('\n'.join(messages or [message])+'\n仅预测检查，无稳定性认证')
        return result
