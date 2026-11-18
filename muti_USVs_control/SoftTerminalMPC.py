"""Soft predicted-energy contraction. No terminal-set or closed-loop certificate.

Moving-reference tracking with bounded terminal adaptation and motor increments.
Plans are independently checked by ARX rollout.
"""
import numpy as np
import casadi as ca
from time import perf_counter


DEFAULTS = dict(mode='adaptive', enabled=True, rate=.10, steps=0, penalty=10.,
                curvature_switch=True, curvature_enter=.85, curvature_exit=.70,
                cost_scale=0., energy_scale=1., sigma_max=3., adaptation_gain=.05,
                transition_time=.8, motor_step=15., increment_weight=.1,
                state_scales=(1., .5, 1., 1., 1., 1., 1., 1., 1.))

def validate_settings(settings, horizon):
    s = dict(settings)
    if s['mode'] not in ('adaptive', 'fixed', 'legacy'):
        raise ValueError('Unknown terminal mode')
    if not isinstance(s['enabled'], (bool, np.bool_)):
        raise ValueError('enabled must be boolean')
    if not isinstance(s['curvature_switch'], (bool, np.bool_)):
        raise ValueError('curvature_switch must be boolean')
    if (not np.isfinite(s['curvature_enter']) or not np.isfinite(s['curvature_exit'])
            or not 0 <= s['curvature_exit'] < s['curvature_enter']):
        raise ValueError('Curvature thresholds require 0 <= exit < enter (1/m)')
    for key in ('rate', 'penalty', 'cost_scale', 'energy_scale', 'sigma_max', 'adaptation_gain', 'transition_time', 'motor_step', 'increment_weight'):
        if not np.isfinite(s[key]):
            raise ValueError('Nonfinite terminal setting: '+key)
    if not 0 < s['rate'] < 1 or s['penalty'] <= 0 or s['cost_scale'] < 0 or s['energy_scale'] <= 0:
        raise ValueError('Invalid rate/penalty/normalization')
    if not 1 <= s['sigma_max'] <= 100 or not 0 <= s['adaptation_gain'] <= 2:
        raise ValueError('Invalid adaptive weight bounds')
    if isinstance(s['steps'], bool) or int(s['steps']) != s['steps'] or not 0 <= s['steps'] <= horizon:
        raise ValueError('Contraction steps must be 0 (N) or an integer in [1,N]')
    if s['transition_time'] <= 0 or s['motor_step'] <= 0 or s['increment_weight'] < 0:
        raise ValueError('Invalid transition or motor increment settings')
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


def curvature_from_points(points):
    """Unsigned circumcircle curvature of three adjacent reference positions, 1/m."""
    a, b, c = np.asarray(points, float)
    ab, bc, ac = b-a, c-b, c-a
    denominator = np.linalg.norm(ab)*np.linalg.norm(bc)*np.linalg.norm(ac)
    if denominator < 1e-15:
        return 0.
    return float(2*abs(ab[0]*bc[1]-ab[1]*bc[0])/denominator)


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
    return dict(X=X, identity=str(identity), source='geometric_chord_unverified',
                curvature=curvature_from_points(p[:3]))


class SoftTerminalEngine:
    def __init__(self, owner):
        self.c = owner
        self.reset()

    def reset(self):
        self.sigma = 1.
        self.previous_energy = None
        self.previous_identity = None
        self.plan = None
        self.cost_scale = None
        self.signature = None
        # Keep symbolic solver cache across resets/settings/curvature changes.
        if not hasattr(self, "solver_key"):
            self.solver_key = None

    def step(self, x, m):
        return np.asarray(x, float)+self.c.T*np.asarray(self.c.f(x, m), float).ravel()

    def rollout(self, x, U):
        X = [np.asarray(x, float)]
        for u in U:
            X.append(self.step(X[-1], u))
        return np.asarray(X)

    def _build(self, H):
        n = self.c.N
        key = (n, H)
        if self.solver_key == key:
            return
        U = ca.SX.sym('motors', 2, n)
        slack = ca.SX.sym('slack')
        p = ca.SX.sym('parameters', 9+9*(n+1)+6+2+9+9+8)
        at = 0
        def take(size):
            nonlocal at
            result = p[at:at+size]; at += size
            return result
        x = take(9); refs = ca.reshape(take(9*(n+1)), 9, n+1)
        q = take(6); r = take(2); pf = take(9); metric = take(9)
        target, js, vs, penalty = [take(1)[0] for _ in range(4)]
        previous = take(2); max_step = take(1)[0]; increment_weight = take(1)[0]
        assert at == p.numel()
        X = [x]; J = 0; increments = []
        for i in range(n):
            e = error_symbolic(X[-1], refs[:, i])[:6]
            delta = U[:, i] - (previous if i == 0 else U[:, i-1])
            increments.append(delta/max_step)
            J += ca.dot(q, e*e) + ca.dot(r, U[:, i]*U[:, i])
            J += increment_weight*ca.dot(r, delta*delta)
            X.append(X[-1]+self.c.T*self.c.f(X[-1], U[:, i]))
        terminal = error_symbolic(X[-1], refs[:, -1])
        J += ca.dot(pf, terminal*terminal)
        eH = error_symbolic(X[H], refs[:, H])
        VH = ca.dot(metric, eH*eH)
        g = ca.vertcat(*[xx[:3] for xx in X], (VH-target-slack)/vs, *increments)
        self.solver = ca.nlpsol('soft_terminal', 'ipopt',
            dict(x=ca.vertcat(ca.vec(U), slack), p=p,
                 f=J/js+penalty*(slack/vs)**2, g=g),
            {'ipopt.print_level': 0, 'print_time': False, 'ipopt.max_iter': 70,
             'ipopt.tol': 1e-7, 'ipopt.constr_viol_tol': 1e-8,
             'ipopt.bound_relax_factor': 0.})
        self.solver_key = key

    def update(self, current, desired, dt, preview=None, legacy_blend=0.):
        c = self.c; cfg = validate_settings(c.terminal_settings, c.N)
        try:
            return self._update(current, desired, dt, preview, cfg, legacy_blend)
        except (ValueError, RuntimeError) as exc:
            c.terminal_diagnostics = dict(mode='failed', reason=str(exc),
                nominal_plan_checked=False, soft_constraint_checked=False,
                certificate_scope='no stability certificate')
            raise

    def _update(self, current, desired, dt, preview, cfg, legacy_blend):
        started_at = perf_counter()
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
        # Keep the original LOS/PP heading and speed commands; use geometry for
        # future position and curvature, not to bypass the guidance layer.
        refs = refs.copy()
        refs[:, 0] = desired[0]
        refs[:, 5] = desired[5]
        q = np.asarray(c.Q, float); r = np.asarray(c.R, float)
        distance = np.linalg.norm(error_numeric(x[:6], desired))
        fraction = 1-np.exp(-c.a*distance)
        heuristic = np.asarray(c.F_low)+fraction*(np.asarray(c.F_high)-c.F_low)
        base = np.r_[heuristic, np.zeros(3)]
        if q.shape != (6,) or r.shape != (2,) or not np.all(np.isfinite(np.r_[q, r, base])) or min(np.r_[q, r, base[:6]]) <= 0:
            raise ValueError('Positive finite stage/terminal weights required')
        H = cfg['steps'] or n
        # Geometric reference derivatives are not ARX-feasible targets.
        metric = np.array([.1, .1, .1, 1., 1., .1, 0., 0., 0.])/np.asarray(cfg['state_scales'])**2
        signature = (repr(cfg), tuple(q), tuple(r), identity, c.T, n)
        if signature != self.signature:
            self.reset(); self.signature = signature
        e0 = error_numeric(x, refs[0]); V0 = float(np.dot(metric, e0*e0))
        blend = float(np.clip(legacy_blend, 0., 1.))
        enabled = cfg['enabled'] and blend < 1.-1e-9
        position_now = float(np.linalg.norm(e0[3:5]))
        position_target = np.sqrt(1-cfg['rate'])*position_now
        adaptation_energy = V0
        if cfg['mode'] == 'fixed':
            self.sigma = 1.
        elif self.previous_energy is not None:
            # Deadband near the path, filtered adaptation and leakage toward one.
            ratio = V0/max(self.previous_energy, .01)
            progress = np.clip(ratio-(1-cfg['rate'])**(1/H), -.2, .2)
            change = cfg['adaptation_gain']*progress if position_now > .05 else 0.
            self.sigma = float(np.clip(1+(self.sigma-1)*.98+change, 1., cfg['sigma_max']))
        pf = (1+(1-blend)*(self.sigma-1))*base
        target = (1-cfg['rate']*(1-blend))*V0
        lower = np.asarray(c.lbx[:2*n], float).reshape(n, 2)
        upper = np.asarray(c.ubx[:2*n], float).reshape(n, 2)
        if not np.all(np.isfinite(np.r_[lower.ravel(), upper.ravel()])) or np.any(lower >= upper):
            raise ValueError('Invalid actuator bounds')
        previous = np.zeros(2) if c._prev_ctrl is None else np.asarray(c._prev_ctrl, float)
        max_step = cfg['motor_step']
        warm = np.asarray(c.u0, float).reshape(n, 2).copy()
        last = previous.copy()
        for i in range(n):
            warm[i] = np.clip(warm[i], np.maximum(lower[i], last-max_step), np.minimum(upper[i], last+max_step))
            last = warm[i]
        limits = np.array([2., .5, np.pi/2])

        def inspect(U):
            X = self.rollout(x, U)
            errors = np.array([error_numeric(a, b) for a, b in zip(X, refs)])
            VH = float(np.dot(metric, errors[H]**2))
            required = max(0., VH-target)
            stage_error = errors[:-1, :6]
            deltas = np.diff(np.vstack([previous, U]), axis=0)
            stages = np.sum(stage_error**2*q, axis=1)+np.sum(U**2*r, axis=1)
            terminal = float(np.dot(pf, errors[-1]**2))
            J = float(sum(stages)+terminal+cfg['increment_weight']*np.sum(deltas*deltas*r))
            valid = bool(np.all(np.isfinite(X)) and np.isfinite(J) and np.isfinite(VH)
                         and np.all(U >= lower-1e-9) and np.all(U <= upper+1e-9)
                         and np.all(np.abs(X[:, :3]) <= limits+1e-6)
                         and np.all(np.abs(deltas) <= max_step+1e-6))
            return dict(U=U, X=X, VH=VH, required=required, cost=J,
                        terminal_cost=terminal, valid=valid,
                        position_m=float(np.linalg.norm(errors[H, 3:5])),
                        heading_deg=float(abs(np.degrees(errors[H, 5]))))

        seed = inspect(warm)
        # Recompute normalization smoothly from current weights/reference, not a stale first frame.
        js = cfg['cost_scale'] or max(1., seed['cost'])
        vs = max(cfg['energy_scale'], V0, 1.)
        self._build(H)
        p = np.r_[x, refs.ravel(), q, r, pf, metric, target, js, vs,
                  cfg['penalty']*(1-blend) if enabled else 0., previous,
                  max_step, cfg['increment_weight']]

        def solve(seed_plan):
            result = self.solver(
                x0=np.r_[seed_plan['U'].ravel(), seed_plan['required'] if enabled else 0.],
                p=p, lbx=np.r_[lower.ravel(), 0.],
                ubx=np.r_[upper.ravel(), np.inf if enabled else 0.],
                lbg=np.r_[np.tile(-limits, n+1), -np.inf, -np.ones(2*n)],
                ubg=np.r_[np.tile(limits, n+1), 0. if enabled else np.inf, np.ones(2*n)])
            raw = np.asarray(result['x'], float).ravel()
            if not np.all(np.isfinite(raw)):
                raise RuntimeError('Nonfinite solver result')
            Uraw = raw[:2*n].reshape(n, 2)
            bounded = np.clip(Uraw, lower, upper)
            trial = inspect(bounded); slack_opt = max(0., float(raw[-1]))
            if (np.max(abs(bounded-Uraw)) > 1e-7 or raw[-1] < -1e-8
                    or not trial['valid'] or (enabled and trial['required'] > slack_opt+1e-6*vs)):
                raise RuntimeError('Independent rollout/soft-constraint/increment verification failed')
            return trial, slack_opt, self.solver.stats()

        backup = seed
        failure = None
        stats = {}
        try:
            plan, slack_opt, stats = solve(backup)
        except (ValueError, RuntimeError) as exc:
            failure = str(exc)
            if not backup['valid']:
                raise RuntimeError('NMPC has no checked hard-feasible plan: '+str(failure))
            plan = backup
            slack_opt = plan['required'] if enabled else None
        U = plan['U']; c.u0 = np.vstack([U[1:], U[-1]]).ravel().tolist()
        c._prev_uvr = x[:3].copy(); c._prev_derivs = x[6:].copy(); c._prev_ctrl = U[0].copy()
        self.previous_energy = adaptation_energy; self.previous_identity = identity; self.plan = plan
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
            solve_time_ms=1000*(perf_counter()-started_at),
            certificate_scope='predicted soft energy check only; no recursive or real-boat stability guarantee')
        c.terminal_diagnostics.update(
            position_priority=False, position_current_m=position_now,
            position_target_m=float(position_target), position_predicted_m=plan['position_m'],
            legacy_blend=blend, motor_step=max_step,
            max_motor_increment=float(np.max(np.abs(np.diff(np.vstack([previous, U]), axis=0)))))
        return tuple(float(v) for v in U[0])
