#!/usr/bin/env python3
"""Offline LQR design for the lower body (Python equivalent of the authors'
MATLAB call  K = lqr(A, B, Q, R)).

    python3 compute_lqr.py --Q 1 1 1 1 1 1 --R 1 1
    python3 compute_lqr.py --lump-arm --payload 0.72

The printed K can be pasted into config/controllers.yaml (gain_source: custom).
The same model is implemented in include/wheel_legged_arm_balance/wip_model.hpp.
"""
import argparse
import math

import numpy as np
from scipy.linalg import solve_continuous_are

P = dict(m_c=7.5, m_w=0.35, l=0.5914, I_yy=0.0347, I_zz=0.0632, I_wa=0.0018, I_wd=0.000929,
         I_ma=1.04e-7, I_md=0.0, gamma=1.0, g=9.81, r_w=0.1016, b=0.215, c=0.25)
ARM = dict(l1=0.3, l2=0.3, lg1=0.3, lg2=0.3, m1=0.6, m2=1.13, h=0.23,
           q1=math.pi / 3, q2=-2 * math.pi / 3)


def lump_arm(p, payload):
    a = ARM
    zs = p['l'] + a['h']
    xe = a['l1'] * math.cos(a['q1'])
    xt = xe + a['l2'] * math.cos(a['q1'] + a['q2'])
    masses = [(p['m_c'], 0.0, p['l']), (a['m1'], xe, zs), (a['m2'] + payload, xt, zs)]
    m = sum(mi for mi, _, _ in masses)
    xc = sum(mi * x for mi, x, _ in masses) / m
    zc = sum(mi * z for mi, _, z in masses) / m
    q = dict(p)
    q['I_yy'] = p['I_yy'] + sum(mi * ((x - xc) ** 2 + (z - zc) ** 2) for mi, x, z in masses)
    q['m_c'] = m
    q['l'] = math.hypot(xc, zc)
    return q


def linearize(p):
    Iw = p['I_wa'] + p['I_ma'] * p['gamma'] ** 2
    Mx = p['m_c'] + 2 * p['m_w'] + 2 * Iw / p['r_w'] ** 2
    M = np.array([[Mx, p['m_c'] * p['l']], [p['m_c'] * p['l'], p['I_yy'] + p['m_c'] * p['l'] ** 2]])
    Mi = np.linalg.inv(M)
    c, r = p['c'], p['r_w']
    sg = Mi @ np.array([0.0, p['m_c'] * p['g'] * p['l']])
    sd = Mi @ np.array([[-2 * c / r ** 2, 2 * c / r], [2 * c / r, -2 * c]])
    se = Mi @ np.array([1 / r, -1.0])
    Ja = p['I_zz'] + 2 * p['m_w'] * p['b'] ** 2 + 2 * Iw * p['b'] ** 2 / r ** 2 + 2 * (p['I_wd'] + p['I_md'])
    A = np.zeros((6, 6))
    B = np.zeros((6, 2))
    A[0, 3] = A[1, 4] = A[2, 5] = 1
    A[3, 2], A[5, 2] = sg
    A[3, 3], A[3, 5], A[5, 3], A[5, 5] = sd[0, 0], sd[0, 1], sd[1, 0], sd[1, 1]
    A[4, 4] = -2 * c * p['b'] ** 2 / r ** 2 / Ja
    B[3, :] = se[0]
    B[5, :] = se[1]
    B[4, 0], B[4, 1] = p['b'] / r / Ja, -p['b'] / r / Ja
    return A, B


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--Q', type=float, nargs=6, default=[1, 1, 1, 1, 1, 1])
    ap.add_argument('--R', type=float, nargs=2, default=[1, 1])
    ap.add_argument('--lump-arm', action='store_true')
    ap.add_argument('--payload', type=float, default=0.36)
    args = ap.parse_args()
    p = lump_arm(P, args.payload) if args.lump_arm else P
    A, B = linearize(p)
    Q, R = np.diag(args.Q), np.diag(args.R)
    K = np.linalg.solve(R, B.T @ solve_continuous_are(A, B, Q, R))
    np.set_printoptions(precision=4, suppress=True, linewidth=120)
    print('X = [p alpha beta v alpha_dot beta_dot], U = [tau_r tau_l]')
    print('A =\n', A, '\nB =\n', B)
    print('K =\n', K)
    print('closed-loop poles:', np.linalg.eigvals(A - B @ K))
    print('yaml: K: [' + ', '.join(f'{k:.4f}' for k in K.ravel()) + ']')


if __name__ == '__main__':
    main()
