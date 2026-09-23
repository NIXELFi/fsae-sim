"""Bump steer and bump camber from the SDM26 OptimumK hardpoints.

Small-displacement solve of the double wishbone + tie rod: rotate the lower
arm, find the upper arm angle that keeps the ball-joint spacing, then the
upright spin about its kingpin axis that keeps the tie-rod length; read the
wheel's spin-axis direction. Reproduces OptimumK's camber gains (0.826 /
0.678 deg/in) to 1-3 %, which is the check on the solver.

Result (2026-09-22): toe-in per inch of bump 0.2139 front, 0.0402 rear.
"""
import json, os, numpy as np
from scipy.optimize import fsolve
path = os.path.expanduser(os.environ.get('SDM26_TEAM_DATA', '~/sdm26-assetto-corsa/data/sdm26_team_data.json'))
d = json.load(open(path, encoding='utf-8'))['hardpoints']
def P(ax, k): return np.array(d[ax][k], float)
def rot(p, a, b, th):
    k = (b - a); k = k / np.linalg.norm(k); v = p - a
    return a + v*np.cos(th) + np.cross(k, v)*np.sin(th) + k*np.dot(k, v)*(1 - np.cos(th))
def solve(axle):
    UF, UA, LF, LA = P(axle,'CHAS_UppFor'), P(axle,'CHAS_UppAft'), P(axle,'CHAS_LowFor'), P(axle,'CHAS_LowAft')
    UB, LB, WC = P(axle,'UPRI_UppPnt'), P(axle,'UPRI_LowPnt'), P(axle,'wheel_centre')
    TC, TU = P(axle,'CHAS_TiePnt'), P(axle,'UPRI_TiePnt')
    WA = WC + np.array([0, 1.0, 0])
    def state(thL):
        LBn = rot(LB, LF, LA, thL); dUL = np.linalg.norm(UB - LB)
        thU = fsolve(lambda th: np.linalg.norm(rot(UB, UF, UA, th[0]) - LBn) - dUL, [thL])[0]; UBn = rot(UB, UF, UA, thU)
        Ltie = np.linalg.norm(TU - TC)
        a0 = (LB - UB)/np.linalg.norm(LB - UB); a1 = (LBn - UBn)/np.linalg.norm(LBn - UBn)
        v = np.cross(a0, a1); s = np.linalg.norm(v); c = np.dot(a0, a1)
        if s < 1e-12: Rm = np.eye(3)
        else:
            vx = np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]]); Rm = np.eye(3) + vx + vx@vx*((1-c)/s**2)
        T = lambda p, phi: rot(UBn + Rm@(p - UB), UBn, LBn, phi)
        phi = fsolve(lambda ph: np.linalg.norm(T(TU, ph[0]) - TC) - Ltie, [0.0])[0]
        w = T(WC, phi); ax = T(WA, phi) - w
        # Left wheel, y outboard: axis x > 0 = rolling direction toward -y = toe-in.
        return w[2], np.degrees(np.arctan2(ax[0], ax[1])), np.degrees(np.arctan2(ax[2], ax[1]))
    z0, t0, c0 = state(0.0); rows = []
    for th in np.radians(np.linspace(-2, 2, 9)):
        z, t, c = state(th); rows.append((z - z0, t - t0, c - c0))
    r = np.array(rows)
    return np.polyfit(r[:,0], r[:,1], 1)[0], np.polyfit(r[:,0], r[:,2], 1)[0]
for ax in ['front', 'rear']:
    bs, cg = solve(ax)
    print(f"{ax}: toe-in {bs:+.4f} deg/in bump, camber {-cg:+.4f} deg/in (negative = more negative in bump)")
