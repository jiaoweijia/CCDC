"""Actual ARX/Qt regression checks; no live sockets or hardware instances."""
from pathlib import Path
import types
import subprocess
import os
import sys
import unittest
import tempfile
import configparser
import time
import json
os.environ['QT_QPA_PLATFORM']='offscreen'
ROOT=Path(__file__).resolve().parent
sys.dont_write_bytecode=True
sys.path.insert(0,os.environ.get('USV_TEST_APP',str(ROOT.parent/'muti_USVs_control')))
import numpy as np
from PySide6.QtWidgets import QApplication
from PySide6.QtGui import QFont,QFontDatabase
from USVController import MPCController,USVControlSystem
from USVManager import USVManager
from TrajectoryPlotWidget import TrajectoryPlotWidget
from SoftTerminalMPC import preview_from_points,error_numeric
from DataProcessor import DataProcessor
from openpyxl import load_workbook
ORIGINAL = types.ModuleType('before_adapt')
original_source = subprocess.check_output(['git', 'show', '79c0dd1:muti_USVs_control/USVController.py'], cwd=ROOT.parent)
exec(compile(original_source, 'before_adapt.py', 'exec'), ORIGINAL.__dict__)
QT=QApplication.instance() or QApplication([])
font_id=QFontDatabase.addApplicationFont('C:/Windows/Fonts/msyh.ttc')
if font_id>=0:QT.setFont(QFont(QFontDatabase.applicationFontFamilies(font_id)[0],9))

def case(**settings):
 c=MPCController(min_output=0)
 c.Q=[1.,.01,.01,1000.,1000.,1.];c.R=[.002,.002]
 c.configure_terminal(curvature_switch=False, **settings)
 p=np.array([[1.4*np.sin(.03*j)+.1,.6*np.cos(.03*j)+.05] for j in range(-1,13)])
 return c,preview_from_points(p,.1),np.array([0.,0.,0.,-.6,.1,.4]),np.array([.4,0.,0.,*p[1],1.5])

class RestoreTests(unittest.TestCase):
 def test_other_algorithms_identical_and_ignore_terminal(self):
  state=np.array([.12,.01,.05,.25,-.1,3.12]);ref=(.6,.2);nxt=(.61,.21)
  for mode in ['PID','Backstepping','ADRC']:
   before=ORIGINAL.USVControlSystem('test',guidance_law='PP',control_law=mode)
   after=USVControlSystem('test',guidance_law='PP',control_law=mode)
   after.mpc.configure_terminal(mode='adaptive',rate=.99,penalty=10000.)
   def forbid(*args,**kwargs):raise AssertionError('Non-NMPC called the NMPC optimizer')
   after.mpc.update=forbid
   for _ in range(3):
    expected=before.control('test',ref,nxt,state.copy())
    actual=after.control('test',ref,nxt,state.copy(),reference_preview={'invalid':'must be ignored'})
    np.testing.assert_allclose([actual['pwml'],actual['pwmr']],[expected['pwml'],expected['pwmr']],atol=1e-12)
    self.assertEqual(actual['terminal'],{})
    self.assertIsNone(after.mpc._soft_engine)

 def test_legacy_matches_current_original(self):
  old=ORIGINAL.MPCController(min_output=0);c,preview,x,target=case(mode='legacy')
  old.Q=c.Q.copy();old.R=c.R.copy()
  np.testing.assert_allclose(c.update(x,target,.1,preview),old.update(x,target,.1),atol=1e-8)
  self.assertEqual(c.terminal_diagnostics['mode'],'legacy')

 def test_actual_H_slack_and_independent_model(self):
  c,preview,x,target=case(rate=.2,steps=3)
  command=c.update(x,target,.1,preview);d=c.terminal_diagnostics;engine=c._soft_engine
  self.assertTrue(engine.plan['valid']);self.assertFalse(d['position_priority'])
  self.assertEqual(d['check_steps'],3);self.assertAlmostEqual(d['energy_target'],.8*d['energy_current'])
  state=np.r_[x,np.zeros(3)]
  states=[state.copy()]
  for u,expected in zip(engine.plan['U'],engine.plan['X'][1:]):
   dn=np.array(c._arx_step(state[:3],state[6:],u))
   a=state[5];v=state[:3];dp=[np.cos(a)*v[0]-np.sin(a)*v[1],np.sin(a)*v[0]+np.cos(a)*v[1]]
   state=np.r_[v+.1*dn,state[3:5]+.1*np.array(dp),a+.1*v[2],dn]
   np.testing.assert_allclose(state,expected,atol=1e-12);states.append(state.copy())
  reference=preview['X'][3].copy();reference[0]=target[0];reference[5]=target[5]
  e=error_numeric(states[3],reference);V=float(np.sum(e*e*np.array([.1,.1,.1,1.,1.,.1,0.,0.,0.])/np.array(c.terminal_settings['state_scales'])**2))
  self.assertAlmostEqual(V,d['energy_predicted'],places=10)
  self.assertAlmostEqual(d['slack'],max(0.,V-d['energy_target']),places=10)
  self.assertLessEqual(d['slack'],d['slack_optimized']+1e-6)
  self.assertLessEqual(d['max_motor_increment'],c.terminal_settings['motor_step']+1e-6)
  self.assertTrue(np.all(np.asarray(command)>=0));self.assertTrue(np.all(np.asarray(command)<=500))
  self.assertFalse(d['nominal_plan_checked'])

 def test_sigma_adapts_and_fixed_does_not(self):
  c,preview,x,target=case()
  c.update(x,target,.1,preview)
  c.update(x,target,.1,preview)
  self.assertGreater(c.terminal_diagnostics['sigma'],1.)
  self.assertLessEqual(c.terminal_diagnostics['sigma'],c.terminal_settings['sigma_max'])
  c.configure_terminal(mode='fixed');c.update(x,target,.1,preview)
  self.assertEqual(c.terminal_diagnostics['sigma'],1.)
  c.configure_terminal(enabled=False);c.update(x,target,.1,preview)
  self.assertIsNone(c.terminal_diagnostics['slack']);self.assertFalse(c.terminal_diagnostics['soft_constraint_checked'])

 def test_penalty_has_real_effect(self):
  results=[]
  for penalty in [1.,100.]:
   c,preview,x,target=case(mode='fixed',penalty=penalty,rate=.99)
   c.update(x,target,.1,preview)
   results.append((c._soft_engine.plan['U'].copy(),c.terminal_diagnostics['slack']))
  self.assertGreater(np.linalg.norm(results[0][0]-results[1][0]),.001)
  self.assertLess(results[1][1],results[0][1])

 def test_solver_failure_uses_checked_fallback(self):
  c,preview,x,target=case();c.update(x,target,.1,preview)
  class Broken:
   def __call__(self,**kwargs):raise RuntimeError('test solver failure')
  c._soft_engine.solver=Broken();c.update(x,target,.1,preview)
  self.assertEqual(c.terminal_diagnostics['mode'],'soft_recovery');self.assertTrue(c._soft_engine.plan['valid'])
  self.assertIn('test solver failure',c.terminal_diagnostics['reason'])

 def test_invalid_parameters_and_clone(self):
  c,preview,x,target=case()
  for cfg in [dict(rate=0),dict(rate=1),dict(steps=11),dict(steps=1.5),dict(penalty=-1)]:
   with self.assertRaises(ValueError):c.configure_terminal(**cfg)
  system=USVControlSystem('test');system.mpc.configure_terminal(rate=.25,steps=4,penalty=30.)
  self.assertEqual(system.clone_for_simulation().mpc.terminal_settings,system.mpc.terminal_settings)

 def test_gui_switching_preview_and_real_path(self):
  m=USVManager();m.set_law('PP','PID')
  w=TrajectoryPlotWidget(usv_manager=m);w.resize(1180,800);w.show();QT.processEvents()
  try:
   w.set_usvs({'USV1':(0,0,0)});w.enable_dynamics(True)
   for mode in ['PID','Backstepping','ADRC']:
    w.set_control_law(mode)
    self.assertFalse(w.terminal_mode_selector.isEnabled());self.assertFalse(w.terminal_soft_check.isEnabled())
    self.assertFalse(w.terminal_rate_spin.isEnabled());self.assertIn('仅NMPC',w.terminal_status_label.text())
    self.assertEqual(m.get_control_system('USV1').mpc.terminal_settings['mode'],'legacy')
   w.set_control_law('NMPC');self.assertTrue(w.terminal_rate_spin.isEnabled())
   w.terminal_curvature_check.setChecked(False)
   w.set_tracking_mode('TTracking');w.update_params('椭圆',1.4,.6,.1,.05,.01,'infinite','simulation',.5)
   system=m.get_control_system('USV1');system.update_params(mpc_q=1000,mpc_r=.002)
   w.start_drawing();w.animation_timer.stop()
   times=[]
   for _ in range(4):
    before=w.current_t;start=time.perf_counter();w.update_animation();times.append((time.perf_counter()-start)*1000)
    self.assertGreater(w.current_t,before,w.terminal_status_label.text())
    self.assertTrue(system.mpc.terminal_diagnostics['soft_constraint_checked'])
   w.terminal_rate_spin.setValue(20);w.terminal_steps_spin.setValue(4);w.terminal_penalty_spin.setValue(30)
   w.terminal_penalty_spin.editingFinished.emit();w.update_animation()
   self.assertEqual(system.mpc.terminal_diagnostics['check_steps'],4)
   self.assertAlmostEqual(system.mpc.terminal_diagnostics['rate_target'],.2)
   self.assertEqual(system.mpc.terminal_diagnostics['penalty'],30)
   config=w.terminal_controls_config();parser=configparser.ConfigParser();parser['NMPC_TERMINAL']=config
   w.terminal_rate_spin.setValue(5);w.restore_terminal_controls(parser['NMPC_TERMINAL'])
   self.assertEqual(w.terminal_rate_spin.value(),20)
   w.set_control_law('ADRC');w.set_control_law('NMPC')
   self.assertEqual(w.terminal_rate_spin.value(),20);self.assertTrue(w.terminal_soft_check.isChecked())
   for mode in ['fixed','legacy','adaptive']:
    w.terminal_mode_selector.setCurrentIndex(w.terminal_mode_selector.findData(mode));w.update_animation()
    if mode=='legacy':self.assertEqual(system.mpc.terminal_diagnostics['mode'],'legacy')
    else:self.assertEqual(system.mpc.terminal_diagnostics['terminal_mode'],mode)
   measured=system.get_state().copy()
   class Capture:
    def get_usv_state(self,name):return measured.copy()
   class Commands:
    sent=[]
    def send_control_command(self,*values):self.sent.append(values)
   w.motion_capture=Capture();w.command_manager=Commands();w.dynamics_enabled=False
   w.update_animation();self.assertEqual(len(w.command_manager.sent),1)
   np.testing.assert_allclose(system.get_state(),measured)
   self.assertEqual(system.mpc.terminal_diagnostics['reference_source'],'geometric_chord_unverified')
   # A hard failure is visible and stops the timer; all outputs here are mocked.
   failures=[];w.terminal_control_failed.connect(failures.append)
   real_frame=w._update_animation_frame
   def fail():raise RuntimeError('injected no feasible plan')
   w._update_animation_frame=fail;w.is_drawing=True;w.animation_timer.start(100)
   w.update_animation()
   self.assertFalse(w.is_drawing);self.assertFalse(w.animation_timer.isActive())
   self.assertEqual(failures,['injected no feasible plan'])
   w._update_animation_frame=real_frame
   # Restore a real diagnostic for the review screenshot.
   w._control_with_terminal('USV1',(.1,.65),(.11,.65),measured.copy(),obstacles=[])
   QT.processEvents();w.grab().save(str(Path(os.environ.get('USV_TEST_OUTPUT', tempfile.gettempdir()))/'adapt_panel_preview.png'))
   (Path(os.environ.get('USV_TEST_OUTPUT', tempfile.gettempdir()))/'gui_timings.json').write_text(json.dumps({'frame_ms':times,'hardware_started':False}),encoding='utf-8')
  finally:w.animation_timer.stop();w.close()

 def test_export_does_not_change_other_algorithm_headers(self):
  original=os.getcwd()
  try:
   for terminal in [{},{'mode':'soft_predicted_descent','slack':.25,'energy_predicted':2.,'sigma':1.2}]:
    with tempfile.TemporaryDirectory(dir=ROOT) as tmp:
     os.chdir(tmp);Path('data').mkdir()
     dp=DataProcessor();dp.start_experiment('test')
     record={'x':0.,'y':0.,'rx':1.,'ry':1.,'heading':0.,'pwml':2.,'pwmr':3.,'u':0.,'v':0.,'terminal':terminal}
     dp.collect_data('USV1',record);self.assertTrue(dp.save_experiment_data())
     book=load_workbook(next(Path('data').glob('*.xlsx')));values=list(book.active.values)
     if not terminal:self.assertEqual(len(values[0]),10)
     else:self.assertEqual(values[1][values[0].index('terminal_slack')],.25)
     book.close();os.chdir(original)
  finally:os.chdir(original)



if __name__ == '__main__':
    unittest.main(verbosity=2)
