#!/usr/bin/env python3
"""Analyse experiment logs.

  push mode : builds the equivalent of Table 2 (time-integrated CMP error,
              mean +- std over the repetitions, % change active vs fixed arm,
              N/A if the robot fell) and plots CMP_X / CMP_Y vs time (Fig. 6).
  loop mode : plots target path, robot path and CMP path (Fig. 8) and prints
              the mean inward CMP offset while turning.

  python3 analyze_results.py push ~/wlbr_results/push_campaign
  python3 analyze_results.py loop ~/wlbr_results/loop_fixed_fast.csv ~/wlbr_results/loop_active_fast.csv
"""
import argparse
import csv
import glob
import json
import math
import os
import re
from collections import defaultdict

import numpy as np

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
except ImportError:      # plotting is optional
    plt = None

RUN_RE = re.compile(r'push_(?P<mode>fixed|active)_f(?P<force>[\d.]+)_m(?P<mass>[\d.]+)_r(?P<rep>\d+)')


def load_csv(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) if r[k] not in ('', 'nan') else math.nan for r in rows])
            for k in rows[0]} if rows else {}


def push_table(folder):
    groups = defaultdict(list)
    for js in sorted(glob.glob(os.path.join(folder, 'push_*.json'))):
        m = RUN_RE.search(os.path.basename(js))
        if not m:
            continue
        with open(js) as f:
            s = json.load(f)
        groups[(float(m['mass']), float(m['force']), m['mode'])].append(s)
    if not groups:
        print('no push_*.json found in', folder)
        return
    lines = ['| can mass [kg] | force [N] | fixed arm e_x [m.s] | active arm e_x [m.s] | change |',
             '|---|---|---|---|---|']
    for mass in sorted({k[0] for k in groups}):
        for force in sorted({k[1] for k in groups if k[0] == mass}):
            cells, means = [], {}
            for mode in ('fixed', 'active'):
                runs = groups.get((mass, force, mode), [])
                if not runs:
                    cells.append('-')
                    continue
                if any(r['fell'] for r in runs):
                    n_fell = sum(r['fell'] for r in runs)
                    cells.append(f'N/A (fell {n_fell}/{len(runs)})')
                    continue
                e = np.array([r['cmp_error_integral_x'] for r in runs])
                means[mode] = e.mean()
                cells.append(f'{e.mean():.4f} +- {e.std():.4f}')
            change = (f'{100 * (means["active"] - means["fixed"]) / means["fixed"]:+.2f} %'
                      if len(means) == 2 else '-')
            lines.append(f'| {mass} | {force} | {cells[0]} | {cells[1]} | {change} |')
    table = '\n'.join(lines)
    print(table)
    with open(os.path.join(folder, 'table2.md'), 'w') as f:
        f.write(table + '\n')
    if plt is None:
        return
    # Fig. 6 style plot for every force / mass (first repetition)
    for mass in sorted({k[0] for k in groups}):
        for force in sorted({k[1] for k in groups if k[0] == mass}):
            fig, ax = plt.subplots(figsize=(8, 3.5))
            for mode, col in (('fixed', 'tab:red'), ('active', 'tab:cyan')):
                files = sorted(glob.glob(os.path.join(folder, f'push_{mode}_f{force:g}_m{mass:g}_r*.csv')))
                if not files:
                    continue
                d = load_csv(files[0])
                lbl = 'Without' if mode == 'fixed' else 'With'
                ax.plot(d['t'], d['cmp_ex'], color=col, label=f'CMP_X ({lbl} Arm Compensation)')
                ax.plot(d['t'], d['cmp_ey'], color=col, ls=':', label=f'CMP_Y ({lbl} Arm Compensation)')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('CMP (m)')
            ax.set_title(f'{force:g} N push, can {mass:g} kg')
            ax.grid(alpha=0.3)
            ax.legend(fontsize=7)
            fig.tight_layout()
            out = os.path.join(folder, f'fig6_f{force:g}_m{mass:g}.png')
            fig.savefig(out, dpi=130)
            plt.close(fig)
            print('saved', out)


def inward_cmp_offset(d, t_min=5.0, min_rate=0.1):
    """Mean signed lateral CMP offset from the wheel-axis centre towards the
    centre of curvature while turning (positive = inside the curve)."""
    t = d['t']
    yaw = np.unwrap(d['base_yaw'])
    rate = np.gradient(yaw, t)
    dx, dy = d['cmp_x'] - d['base_x'], d['cmp_y'] - d['base_y']
    lat = -np.sin(yaw) * dx + np.cos(yaw) * dy          # +Y of the heading frame (left)
    m = (t > t_min) & (np.abs(rate) > min_rate) & np.isfinite(lat)
    return float(np.mean(np.sign(rate[m]) * lat[m])) if m.any() else math.nan


def loop_plot(files, out):
    for path in files:
        d = load_csv(path)
        print(f'{os.path.basename(path)}: mean inward CMP offset while turning = '
              f'{100 * inward_cmp_offset(d):+.2f} cm, mean |CMP-CoM| lateral = '
              f'{100 * np.nanmean(np.abs(d["cmp_ey"][d["t"] > 5])):.2f} cm')
    if plt is None:
        print('matplotlib not available')
        return
    fig, axes = plt.subplots(1, len(files), figsize=(6 * len(files), 4.5), squeeze=False)
    for ax, path in zip(axes[0], files):
        d = load_csv(path)
        ax.plot(d['base_x'], d['base_y'], 'k-', lw=1.0, label='robot path')
        ax.plot(d['cmp_x'], d['cmp_y'], color='tab:pink', lw=0.8, label='CMP')
        tgt = path.replace('.csv', '_target.csv')
        if os.path.exists(tgt):
            t = load_csv(tgt)
            ax.plot(t['x'], t['y'], 'g--', lw=1.2, label='target')
        ax.set_aspect('equal')
        ax.set_title(os.path.basename(path))
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print('saved', out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('mode', choices=['push', 'loop'])
    ap.add_argument('paths', nargs='+')
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    if a.mode == 'push':
        push_table(a.paths[0])
    else:
        loop_plot(a.paths, a.out or os.path.join(os.path.dirname(a.paths[0]), 'fig8_loops.png'))


if __name__ == '__main__':
    main()
