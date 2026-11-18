"""Reproducible model-only closed-loop comparison; never opens hardware."""
import sys, types, subprocess, json, time
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_adapt as base
from SoftTerminalMPC import preview_from_points

def run(cls):
    c=cls(min_output=0)
    c.Q=[1.,.01,.01,1000.,1000.,1.]; c.R=[.002,.002]
    c.F_high=[.08,.00016,.00016,4000.,4000.,.4]
    guidance=base.ORIGINAL.LOSGuidance()
    x=np.array([.35,0.,0.,0.,.6,0.,0.,0.,0.]); prev=np.zeros(2)
    errors=[]; increments=[]; times=[]; gates=[]; failure=None
    for k in range(260):
        phase=.25*(.1*k+.1*np.arange(-1,13))
        points=np.column_stack([1.4*np.sin(phase),.6*np.cos(phase)])
        preview=preview_from_points(points,.1,'ellipse')
        heading,speed=guidance.calculate_guidance(x[3:5],x[5],points[1],points[2])
        target=np.array([speed,0.,0.,*points[1],heading])
        started=time.perf_counter()
        try: command=np.asarray(c.update(x[:6],target,.1,preview))
        except Exception as exc:
            failure={'frame':k,'reason':str(exc)}; break
        times.append(1000*(time.perf_counter()-started))
        increments.append(float(np.max(abs(command-prev)))); prev=command
        errors.append(float(np.linalg.norm(x[3:5]-points[1])))
        gates.append(c.terminal_diagnostics.get('curvature_fallback'))
        x=x+.1*np.asarray(c.f(x,command)).ravel()
    return dict(frames=len(errors),mean_error_m=float(np.mean(errors)),
                max_increment=max(increments),p95_ms=float(np.percentile(times,95)),
                max_ms=max(times),switches=sum(a!=b for a,b in zip(gates,gates[1:])),failure=failure)

def compare():
    saved=sys.modules['SoftTerminalMPC']
    oldmod=types.ModuleType('SoftTerminalMPC')
    root=Path(__file__).resolve().parent.parent
    exec(compile(subprocess.check_output(['git','show','adapt:muti_USVs_control/SoftTerminalMPC.py'],cwd=root),'adapt_engine','exec'),oldmod.__dict__)
    sys.modules['SoftTerminalMPC']=oldmod
    old=types.ModuleType('adapt_controller')
    try:
        exec(compile(subprocess.check_output(['git','show','adapt:muti_USVs_control/USVController.py'],cwd=root),'adapt_controller','exec'),old.__dict__)
    finally: sys.modules['SoftTerminalMPC']=saved
    return {'adapt':run(old.MPCController),'adapt1':run(base.MPCController)}

if __name__=='__main__':
    print(json.dumps(compare(),indent=2))
