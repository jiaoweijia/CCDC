"""Curvature switching against the original controller and the real Qt control path."""
import configparser
import math
import os
from pathlib import Path
import tempfile
import unittest

import test_adapt as base
import numpy as np
from SoftTerminalMPC import curvature_from_points, preview_from_points


class CurvatureTests(unittest.TestCase):
    def test_geometric_curvature(self):
        for radius in (.5, 1., 2.):
            for spacing in (.01, .1):
                angles = np.array([-spacing, 0., spacing])
                points = radius*np.column_stack([np.cos(angles), np.sin(angles)])
                for geometry in (points, points[::-1], points+[200., -300.]):
                    self.assertAlmostEqual(curvature_from_points(geometry), 1/radius, places=7)
        self.assertEqual(curvature_from_points([[0, 0], [1, 0], [2, 0]]), 0.)
        self.assertEqual(curvature_from_points([[0, 0], [0, 0], [0, 0]]), 0.)
        angles = .02*np.arange(-1, 13)
        preview = preview_from_points(2*np.column_stack([np.cos(angles), np.sin(angles)]), .1)
        self.assertAlmostEqual(preview['curvature'], .5)

    def test_hysteresis_with_continuous_transition_and_cached_solver(self):
        c, preview, state, target = base.case()
        c.configure_terminal(curvature_switch=True)
        last = np.zeros(2)
        cached = None
        for curvature, latched in ((.85, True), (.8, True), (.7, False), (.8, False), (-.85, True)):
            preview['curvature'] = curvature
            actual = np.asarray(c.update(state, target, .1, preview))
            self.assertLessEqual(np.max(abs(actual-last)), c.terminal_settings['motor_step']+1e-6)
            self.assertEqual(c.terminal_diagnostics['curvature_fallback'], latched)
            if cached is not None:
                self.assertIs(c._soft_engine.solver, cached)
            cached = c._soft_engine.solver
            last = actual
        for _ in range(10):
            c.update(state, target, .1, preview)
        self.assertEqual(c.terminal_diagnostics['mode'], 'legacy')
        self.assertFalse(c.terminal_diagnostics['soft_constraint_checked'])
        c.reset()
        self.assertIs(c._soft_engine.solver, cached)
        self.assertFalse(c._curvature_legacy)
        self.assertIsNone(c._soft_engine.previous_energy)

    def test_switch_off_and_fixed_mode_ignore_high_curvature(self):
        c, preview, state, target = base.case()
        preview['curvature'] = 10.
        for options in ({'curvature_switch': False}, {'mode': 'fixed', 'curvature_switch': True}):
            c.configure_terminal(**options)
            c.update(state, target, .1, preview)
            self.assertFalse(c.terminal_diagnostics['curvature_fallback'])
            self.assertTrue(c.terminal_diagnostics['soft_constraint_checked'])
        c.configure_terminal(mode='legacy')
        c.update(state, target, .1, preview)
        self.assertEqual(c.terminal_diagnostics['mode'], 'legacy')
        self.assertFalse(c.terminal_diagnostics['curvature_fallback'])

    def test_invalid_thresholds_and_curvature(self):
        c, preview, state, target = base.case()
        for options in ({'curvature_exit': .85}, {'curvature_exit': -1},
                        {'curvature_enter': float('nan')}, {'curvature_switch': 'yes'}):
            with self.assertRaises(ValueError):
                c.configure_terminal(**options)
        preview['curvature'] = float('nan')
        with self.assertRaises(ValueError):
            c.update(state, target, .1, preview)

    def test_missing_geometry_does_not_keep_turn_latch(self):
        c, _, state, target = base.case()
        c.configure_terminal(curvature_switch=True)
        c.update(state, target, .1, reference_curvature=1.)
        self.assertTrue(c.terminal_diagnostics['curvature_fallback'])
        c.update(state, target, .1)
        self.assertFalse(c.terminal_diagnostics['curvature_fallback'])
        self.assertIsNone(c.terminal_diagnostics['curvature'])

    def test_qt_curve_regions_and_configuration(self):
        manager = base.USVManager()
        manager.set_law('LOS', 'NMPC')
        widget = base.TrajectoryPlotWidget(usv_manager=manager)
        widget.resize(1180, 800)
        widget.show()
        try:
            widget.set_usvs({'USV1': (0, 0, 0)})
            widget.enable_dynamics(True)
            widget.set_control_law('NMPC')
            widget.set_tracking_mode('TTracking')
            widget.update_params('椭圆', 1.4, 1.2, .1, .05, .01, 'infinite', 'simulation', .3)
            system = manager.get_control_system('USV1')
            system.update_params(mpc_q=1000, mpc_r=.002, mpc_t=8000, mpca=.4)
            widget.terminal_curvature_enter_spin.setValue(.85)
            widget.terminal_curvature_exit_spin.setValue(.7)
            widget.terminal_curvature_enter_spin.editingFinished.emit()
            parser = configparser.ConfigParser()
            parser['NMPC_TERMINAL'] = widget.terminal_controls_config()
            widget.terminal_curvature_enter_spin.setValue(2.)
            widget.restore_terminal_controls(parser['NMPC_TERMINAL'])
            self.assertEqual(widget.terminal_curvature_enter_spin.value(), .85)
            self.assertEqual(widget.terminal_curvature_exit_spin.value(), .7)
            widget.start_drawing()
            widget.animation_timer.stop()
            for t, expected_turn in ((0., False), (math.pi/(2*widget.omega_val), True), (0., False)):
                widget.current_t = t
                # Each sampled region starts from a consistent, near-reference measured state.
                # A geometric jump around an ellipse is not a physical simulation step.
                preview = widget._ellipse_preview('USV1', system)
                system.reset(np.r_[.1, 0., 0., preview['X'][0, 3:6]])
                widget.update_animation()
                diagnostic = system.mpc.terminal_diagnostics
                self.assertEqual(diagnostic.get('curvature_fallback'), expected_turn, (t, system.get_state(), diagnostic, widget.terminal_status_label.text()))
                if expected_turn:
                    self.assertGreater(diagnostic['legacy_blend'], 0.)
                    self.assertLess(diagnostic['legacy_blend'], 1.)
                else:
                    self.assertTrue(diagnostic['soft_constraint_checked'])
            base.QT.processEvents()
            widget.grab().save(str(Path(os.environ.get('USV_TEST_OUTPUT', tempfile.gettempdir()))/'adapt_curvature_panel.png'))
            for control in ('PID', 'Backstepping', 'ADRC'):
                widget.set_control_law(control)
                self.assertFalse(widget.terminal_curvature_check.isEnabled())
                self.assertFalse(widget.terminal_curvature_enter_spin.isEnabled())
                self.assertEqual(system.mpc.terminal_settings['mode'], 'legacy')
            widget.set_control_law('NMPC')
            self.assertTrue(widget.terminal_curvature_check.isEnabled())
            self.assertEqual(widget.terminal_curvature_exit_spin.value(), .7)
        finally:
            widget.animation_timer.stop()
            widget.close()


if __name__ == '__main__':
    unittest.main(verbosity=2)
