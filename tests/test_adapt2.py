"""Regression tests for the exact Nov9 terminal restored as adapt2."""
import hashlib
from types import SimpleNamespace
import unittest
import numpy as np
import test_adapt as base
from SoftTerminalPanel import reference_curvature,TurnSwitchState

class Adapt2Tests(unittest.TestCase):
    def panel(self):
        manager=base.USVManager();manager.set_law('LOS','NMPC')
        panel=base.TrajectoryPlotWidget(usv_manager=manager)
        self.addCleanup(panel.close);self.addCleanup(panel.animation_timer.stop)
        return panel

    def test_engine_matches_historical_installed_sha256(self):
        data=(base.ROOT.parent/'muti_USVs_control/SoftTerminalMPC.py').read_bytes()
        self.assertEqual(hashlib.sha256(data).hexdigest(),'045ae4f76f7ca7b1ea68ce78009a1d6f0e5f5a9cd064a88255d1616e44559d36')

    def test_geometry_lookahead_and_three_frame_exit(self):
        angles=np.arange(10)*.1
        points=2*np.column_stack([np.cos(angles),np.sin(angles)])
        np.testing.assert_allclose(reference_curvature(points),.5,atol=1e-12)
        panel=self.panel();refs=np.zeros((11,9));refs[:,3]=np.arange(11)*.1
        preview=dict(identity='test',X=refs,curvature=np.r_[np.full(5,.3),1.6,np.full(5,.3)])
        panel.current_t=0.
        self.assertTrue(panel._choose_turn_mode('boat',preview,.1,None)['turn_switch_active'])
        preview['curvature'][:]=1.
        for frame in range(1,4):
            panel.current_t=frame*.1
            result=panel._choose_turn_mode('boat',preview,.1,None)
            self.assertEqual(result['turn_switch_active'],frame<3)
        self.assertEqual(result['turn_switch_reason'],'exit')

    def test_lagging_boat_holds_turn(self):
        panel=self.panel();refs=np.zeros((11,9));refs[:,3]=1.+np.arange(11)*.1
        preview=dict(identity='lag',X=refs,curvature=np.full(11,.2))
        state=TurnSwitchState('lag',0.,in_turn=True);state.recent.append((0.,0.,2.))
        panel._corner_states['boat']=state
        for frame in range(1,5):
            panel.current_t=frame*.1
            result=panel._choose_turn_mode('boat',preview,.1,np.zeros(2))
            self.assertTrue(result['turn_switch_active']);self.assertEqual(result['curvature_at_boat'],2.)

    def test_complete_ellipse_switches_twice_each_direction(self):
        panel=self.panel();panel.usv_offsets={'boat':(0.,0.,0.)}
        panel.update_params('椭圆',1.4,1.2,.1,.05,.01,'infinite','simulation',.3)
        system=SimpleNamespace(mpc=SimpleNamespace(N=10,T=.1))
        previous=False;enter=exit=0
        for t in np.linspace(0.,2*np.pi/panel.omega_val,600):
            panel.current_t=t;preview=panel._ellipse_preview('boat',system)
            result=panel._choose_turn_mode('boat',preview,.1,preview['X'][0,3:5])
            current=result['turn_switch_active'];enter+=current and not previous;exit+=previous and not current;previous=current
        self.assertEqual((enter,exit),(2,2))

    def test_config_and_non_nmpc_scope(self):
        panel=self.panel();self.assertEqual(panel.turn_curvature_spin.value(),1.5)
        panel.turn_curvature_spin.setValue(1.8);config=panel.terminal_controls_config()
        panel.turn_curvature_spin.setValue(1.);panel.restore_terminal_controls(config)
        self.assertEqual(panel.turn_curvature_spin.value(),1.8)
        for mode in ('PID','Backstepping','ADRC'):
            panel.set_control_law(mode);self.assertFalse(panel.turn_curvature_spin.isEnabled())
        panel.set_control_law('NMPC');self.assertTrue(panel.turn_curvature_spin.isEnabled())

    def test_solver_cache_survives_effective_mode_switch(self):
        c,preview,state,target=base.case();c.update(state,target,.1,preview);solver=c._soft_engine.solver
        c.configure_terminal(mode='legacy');c.update(state,target,.1)
        c.configure_terminal(mode='adaptive');c.update(state,target,.1,preview)
        self.assertIs(c._soft_engine.solver,solver)

if __name__=='__main__':unittest.main(verbosity=2)
