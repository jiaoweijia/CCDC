"""Continuous closed-loop and new panel-setting regression tests."""
import unittest
import test_adapt as base
from benchmark_adapt1 import run

@unittest.skip("Historical adapt1 terminal replaced by the requested Nov9 implementation")
class Adapt1Tests(unittest.TestCase):
    def test_continuous_model_lap_has_bounded_commands(self):
        report=run(base.MPCController)
        self.assertIsNone(report['failure'], report)
        self.assertEqual(report['frames'],260)
        self.assertGreaterEqual(report['switches'],4)
        self.assertLessEqual(report['max_increment'],15.+1e-6)

    def test_transition_controls_round_trip_and_disabling(self):
        manager=base.USVManager();manager.set_law('LOS','NMPC')
        panel=base.TrajectoryPlotWidget(usv_manager=manager)
        try:
            panel.terminal_transition_spin.setValue(1.2)
            panel.terminal_motor_step_spin.setValue(12.)
            values=panel.terminal_controls_config()
            panel.terminal_transition_spin.setValue(.5)
            panel.terminal_motor_step_spin.setValue(50.)
            panel.restore_terminal_controls(values)
            self.assertEqual(panel.terminal_transition_spin.value(),1.2)
            self.assertEqual(panel.terminal_motor_step_spin.value(),12.)
            panel.set_control_law('PID')
            self.assertFalse(panel.terminal_transition_spin.isEnabled())
            self.assertFalse(panel.terminal_motor_step_spin.isEnabled())
        finally:
            panel.animation_timer.stop();panel.close()

if __name__=='__main__': unittest.main(verbosity=2)
