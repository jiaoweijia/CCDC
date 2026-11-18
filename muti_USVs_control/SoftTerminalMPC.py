"""Soft predicted-energy contraction. No terminal-set or closed-loop certificate.

The original six-state stage cost is kept. The contraction metric is fixed and
independent of terminal sigma, Qf and stage tuning. A nonnegative slack is solved
jointly with the model inputs, then independently checked using the ARX rollout.
"""
import numpy as np
import casadi as ca


DEFAULTS = dict(mode='adaptive', enabled=True, rate=.10, steps=0, penalty=10.,
                cost_scale=0., energy_scale=1., sigma_max=20., adaptation_gain=.2,
                state_scales=(1., .5, 1., 1., 1., 1., 1., 1., 1.))


def validate_settings(settings, horizon):
    s = dict(settings)
    if s['mode'] not in ('adaptive', 'fixed', 'legacy'):
        raise ValueError('Unknown terminal mode')
    if not isinstance(s['enabled'], (bool, np.bool_)):
        raise ValueError('enabled must be boolean')
    for key in ('rate', 'penalty', 'cost_scale', 'energy_scale', 'sigma_max', 'adaptation_gain'):
        if not np.isfinite(s[key]):
            raise ValueError('Nonfinite terminal setting: '+key)
    if not 0 < s['rate'] < 1 or s['penalty'] <= 0 or s['cost_scale'] < 0 or s['energy_scale'] <= 0:
        raise ValueError('Invalid rate/penalty/normalization')
    if not 1 <= s['sigma_max'] <= 100 or not 0 <= s['adaptation_gain'] <= 2:
        raise ValueError('Invalid adaptive weight bounds')
    if isinstance(s['steps'], bool) or int(s['steps']) != s['steps'] or not 0 <= s['steps'] <= horizon:
        raise ValueError('Contraction steps must be 0 (N) or an integer in [1,N]')
    scales = np.asarray(s['state_scales'], float)
    if scales.shape != (9,) or not np.all(np.isfinite(scales)) or np.any(scales <= 0):
        raise ValueError('Nine finite positive state scales required')
    s['steps'] = int(s['steps']); s['state_scales'] = tuple(scales)
    return s


def error_numeric(x, ref):
    e = np.asarray(x, float)-np.asarray(ref, float)
    e[5] = np.arctan2(np.sin(e[5]), np.cos(e[5]))
    return e


def error_symbolic(x, ref):
    e = x-ref
    return ca.vertcat(e[:5], ca.atan2(ca.sin(e[5]), ca.cos(e[5])), e[6:])


def preview_from_points(points, dt, identity='ellipse'):
    """points[-1,0,...,N+2] -> reference[0,...,N], with previous derivatives.

    This is a geometric chord preview, not a dynamically feasible certificate.
    """
    p = np.asarray(points, float)
    if p.ndim != 2 or p.shape[1] != 2 or len(p) < 5 or not np.all(np.isfinite(p)) or dt <= 0:
        raise ValueError('Invalid reference points')
    vel = np.diff(p, axis=0)/dt
    if np.any(np.linalg.norm(vel, axis=1) < 1e-10):
        raise ValueError('Stationary geometry needs an explicit state reference')
    psi = np.unwrap(np.arctan2(vel[:, 1], vel[:, 0]))
    yaw = np.diff(psi)/dt
    nu = np.column_stack([np.linalg.norm(vel[:-1], axis=1), np.zeros(len(yaw)), yaw])
    memory = np.diff(nu, axis=0)/dt
    X = np.column_stack([nu[1:], p[1:-2], psi[1:-1], memory])
    return dict(X=X, identity=str(identity), source='geometric_chord_unverified')


class SoftTerminalEngine:
    def __init__(self, owner):
        self.c = owner
        self.solver_key = None
        self.reset()

    def reset(self):
        self.sigma = 1.
        self.previous_energy = None
        self.previous_identity = None
        self.plan = None
        self.cost_scale = None
        self.signature = None

    def step(self, x, m):
        return np.asarray(x, float)+self.c.T*np.asarray(self.c.f(x, m), float).ravel()

    def rollout(self, x, U):
        X = [np.asarray(x, float)]
        for u in U:
            X.append(self.step(X[-1], u))
        return np.asarray(X)

    def _build(self, H):
        n = self.c.N
        # Keep the compiled solver when a terminal mode switch only resets history.
        key = (n, H, self.c.T, self.c.f)
        if self.solver_key == key:
            return
        U = ca.SX.sym('motors', 2, n); slack = ca.SX.sym('slack')
        p = ca.SX.sym('parameters', 9+6+9*(n+1)+6+2+9+9+4)
        at = 0
        def take(size):
            nonlocal at
            v = p[at:at+size]; at += size; return v
        x = take(9); desired = take(6); refs = ca.reshape(take(9*(n+1)), 9, n+1)
        q = take(6); r = take(2); pf = take(9); metric = take(9)
        target, js, vs, penalty = [take(1)[0] for _ in range(4)]
        assert at == p.numel()
        X = [x]; J = 0
        for i in range(n):
            # Preserve the original stage cost, including its frozen guidance target.
            e = X[-1][:6]-desired
            J += ca.dot(q, e*e)+ca.dot(r, U[:, i]*U[:, i])
            X.append(X[-1]+self.c.T*self.c.f(X[-1], U[:, i]))
        terminal = error_symbolic(X[-1], refs[:, -1])
        J += ca.dot(pf, terminal*terminal)
        eH = error_symbolic(X[H], refs[:, H])
        VH = ca.dot(metric, eH*eH)
        g = ca.vertcat(*[xx[:3] for xx in X], (VH-target-slack)/vs)
        self.solver = ca.nlpsol('soft_terminal', 'ipopt',
            dict(x=ca.vertcat(ca.vec(U), slack), p=p, f=J/js+penalty*(slack/vs)**2, g=g),
            {'ipopt.print_level': 0, 'print_time': False, 'ipopt.max_iter': 100,
             'ipopt.tol': 1e-8, 'ipopt.constr_viol_tol': 1e-8,
             'ipopt.bound_relax_factor': 0.})
        self.solver_key = key

    def update(self, current, desired, dt, preview=None):
        c = self.c; cfg = validate_settings(c.terminal_settings, c.N)
        try:
            return self._update(current, desired, dt, preview, cfg)
        except (ValueError, RuntimeError) as exc:
            c.terminal_diagnostics = dict(mode='failed', reason=str(exc),
                nominal_plan_checked=False, soft_constraint_checked=False,
                certificate_scope='no stability certificate')
            raise

    def _update(self, current, desired, dt, preview, cfg):
        c = self.c; n = c.N
        if not np.isclose(dt, c.T, rtol=0, atol=1e-12):
            raise ValueError('Sampling time must match ARX model')
        current = np.asarray(current, float).reshape(-1); desired = np.asarray(desired, float)
        if current.shape not in ((6,), (9,)) or desired.shape != (6,) or not np.all(np.isfinite(np.r_[current, desired])):
            raise ValueError('Invalid measured/desired state')
        if len(current) == 9:
            x = current.copy(); source = 'provided_nine_state'
        else:
            memory = np.zeros(3) if c._prev_uvr is None else np.asarray(c._arx_step(c._prev_uvr, c._prev_derivs, c._prev_ctrl))
            memory = np.clip(memory, [-5., -3., -10.], [5., 3., 10.])
            x = np.r_[current, memory]; source = 'model_memory_estimate'
        if preview is None:
            # Compatibility for LOS/obstacles/waypoints: no fictitious moving preview.
            refs = np.tile(np.r_[desired, np.zeros(3)], (n+1, 1))
            identity = 'frozen_guidance'; refsource = 'frozen_guidance_unverified'
        else:
            refs = np.asarray(preview['X'], float)
            identity = str(preview.get('identity', 'explicit'))
            refsource = str(preview.get('source', 'explicit_unverified'))
            if refs.shape != (n+1, 9) or not np.all(np.isfinite(refs)):
                raise ValueError('Reference preview must be finite (N+1,9)')
        q = np.asarray(c.Q, float); r = np.asarray(c.R, float)
        base = np.r_[np.maximum(c.F_low, c.F_high), [.01]*3]
        if q.shape != (6,) or r.shape != (2,) or not np.all(np.isfinite(np.r_[q, r, base])) or min(np.r_[q, r, base]) <= 0:
            raise ValueError('Positive finite stage/terminal weights required')
        H = cfg['steps'] or n; metric = 1/np.asarray(cfg['state_scales'])**2
        signature = (repr(cfg), tuple(q), tuple(r), tuple(base), identity, c.T, n)
        if signature != self.signature:
            self.reset(); self.signature = signature
        e0 = error_numeric(x, refs[0]); V0 = float(np.dot(metric, e0*e0))
        if cfg['mode'] == 'fixed':
            self.sigma = 1.
        elif self.previous_energy is not None and self.previous_energy > 1e-10:
            per_step_target = (1-cfg['rate'])**(1/H)
            ratio = V0/self.previous_energy
            self.sigma = float(np.clip(self.sigma*np.exp(cfg['adaptation_gain']*np.clip(ratio-per_step_target, -1, 1)), 1, cfg['sigma_max']))
        pf = self.sigma*base
        target = (1-cfg['rate'])*V0
        lower = np.asarray(c.lbx[:2*n], float).reshape(n, 2)
        upper = np.asarray(c.ubx[:2*n], float).reshape(n, 2)
        if not np.all(np.isfinite(np.r_[lower.ravel(), upper.ravel()])) or np.any(lower >= upper):
            raise ValueError('Invalid actuator bounds')
        warm = np.clip(np.asarray(c.u0, float).reshape(n, 2), lower, upper)
        limits = np.array([2., .5, np.pi/2])

        def inspect(U):
            X = self.rollout(x, U)
            errors = np.array([error_numeric(a, b) for a, b in zip(X, refs)])
            VH = float(np.dot(metric, errors[H]**2))
            required = max(0., VH-target)
            stage_error = X[:-1, :6]-desired
            stages = np.sum(stage_error**2*q, axis=1)+np.sum(U**2*r, axis=1)
            terminal = float(np.dot(pf, errors[-1]**2))
            J = float(sum(stages)+terminal)
            valid = bool(np.all(np.isfinite(X)) and np.isfinite(J) and np.isfinite(VH)
                         and np.all(U >= lower-1e-9) and np.all(U <= upper+1e-9)
                         and np.all(np.abs(X[:, :3]) <= limits+1e-6))
            return dict(U=U, X=X, VH=VH, required=required, cost=J,
                        terminal_cost=terminal, valid=valid)

        seed = inspect(warm)
        if self.cost_scale is None:
            self.cost_scale = cfg['cost_scale'] or max(1., seed['cost'])
        js = self.cost_scale; vs = cfg['energy_scale']; enabled = cfg['enabled']
        self._build(H)
        p = np.r_[x, desired, refs.ravel(), q, r, pf, metric, target, js, vs, cfg['penalty'] if enabled else 0.]
        failure = None; stats = {}; plan = None; slack_opt = None
        try:
            res = self.solver(x0=np.r_[warm.ravel(), seed['required'] if enabled else 0.], p=p,
                lbx=np.r_[lower.ravel(), 0.], ubx=np.r_[upper.ravel(), np.inf if enabled else 0.],
                lbg=np.r_[np.tile(-limits, n+1), -np.inf],
                ubg=np.r_[np.tile(limits, n+1), 0. if enabled else np.inf])
            raw = np.asarray(res['x'], float).ravel(); stats = self.solver.stats()
            if not np.all(np.isfinite(raw)):
                raise RuntimeError('Nonfinite solver result')
            Uraw = raw[:2*n].reshape(n, 2); bounded = np.clip(Uraw, lower, upper)
            if np.max(abs(bounded-Uraw)) > 1e-7:
                raise RuntimeError('Solver actuator bound violation')
            trial = inspect(bounded); slack_opt = max(0., float(raw[-1]))
            if raw[-1] < -1e-8 or not trial['valid'] or (enabled and trial['required'] > slack_opt+1e-6*vs):
                raise RuntimeError('Independent prediction/soft-constraint verification failed')
            plan = trial
        except (ValueError, RuntimeError) as exc:
            failure = str(exc)
        if plan is None:
            # Re-evaluate a bounded warm plan from the current measured state;
            # unlimited soft slack can relax ONLY the energy request, not velocities.
            if not seed['valid']:
                raise RuntimeError('NMPC has no checked hard-feasible plan: '+str(failure))
            plan = seed; slack_opt = seed['required'] if enabled else None
        U = plan['U']; c.u0 = np.vstack([U[1:], U[-1]]).ravel().tolist()
        c._prev_uvr = x[:3].copy(); c._prev_derivs = x[6:].copy(); c._prev_ctrl = U[0].copy()
        self.previous_energy = V0; self.previous_identity = identity; self.plan = plan
        c.Qf = pf[:6].tolist(); c.terminal_matrix = np.diag(pf)
        residual = max(np.max(abs(self.step(refs[i], np.zeros(2))-refs[i+1])) for i in range(n))
        # This residual uses zero input, not an inverse-dynamics feasibility test.
        c.terminal_diagnostics = dict(mode=('soft_recovery' if failure else ('soft_predicted_descent' if stats.get('success') else 'soft_feasible_suboptimal')),
            terminal_mode=cfg['mode'], enabled=enabled, sigma=self.sigma,
            rate_target=cfg['rate'], check_steps=H, penalty=cfg['penalty'],
            energy_current=V0, energy_target=target, energy_predicted=plan['VH'],
            slack=plan['required'] if enabled else None, slack_optimized=slack_opt,
            target_met=bool(plan['VH'] <= target+1e-7*vs) if enabled else None,
            predicted_drop=V0-plan['VH'], cost=plan['cost'], terminal_cost=plan['terminal_cost'],
            objective_normalized=plan['cost']/js+(cfg['penalty']*(plan['required']/vs)**2 if enabled else 0.),
            cost_scale=js, energy_scale=vs, state_source=source, reference_source=refsource,
            zero_input_reference_residual=float(residual), solver_success=bool(stats.get('success', False)),
            solver_status=str(stats.get('return_status', 'fallback')), reason=failure,
            soft_constraint_checked=bool(enabled), nominal_plan_checked=False,
            certificate_scope='predicted soft energy check only; no recursive or real-boat stability guarantee')
        return tuple(float(v) for v in U[0])
