#!/usr/bin/env python3
"""Run the translational-disturbance campaign of Sec. V-A / Table 2.

For each can mass, push force and arm mode (fixed / active) the
push_experiment launch file is executed `--repeats` times (headless), then
analyze_results.py builds the table and the Fig. 6 plots.

  ros2 run wheel_legged_arm_balance run_push_campaign.py \
      --forces 27 29 31 33 --masses 0.36 0.72 --repeats 5
"""
import argparse
import os
import signal
import subprocess
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--forces', type=float, nargs='+', default=[27, 29, 31, 33])
    ap.add_argument('--masses', type=float, nargs='+', default=[0.36, 0.72])
    ap.add_argument('--modes', nargs='+', default=['fixed', 'active'])
    ap.add_argument('--repeats', type=int, default=5)
    ap.add_argument('--push-duration', type=float, default=1.0)
    ap.add_argument('--jitter', type=float, default=0.05, help='random push timing jitter [s]')
    ap.add_argument('--output-dir', default=os.path.expanduser('~/wlbr_results/push_campaign'))
    ap.add_argument('--timeout', type=float, default=120.0)
    a = ap.parse_args()
    os.makedirs(a.output_dir, exist_ok=True)

    total = len(a.forces) * len(a.masses) * len(a.modes) * a.repeats
    k = 0
    for mass in a.masses:
        for force in a.forces:
            for mode in a.modes:
                for rep in range(a.repeats):
                    k += 1
                    name = f'push_{mode}_f{force:g}_m{mass:g}_r{rep}'
                    if os.path.exists(os.path.join(a.output_dir, name + '.json')):
                        print(f'[{k}/{total}] {name} exists, skipping')
                        continue
                    print(f'[{k}/{total}] {name}', flush=True)
                    cmd = ['ros2', 'launch', 'wheel_legged_arm_balance', 'push_experiment.launch.py',
                           f'force:={force}', f'payload_mass:={mass}', f'arm_mode:={mode}',
                           f'push_duration:={a.push_duration}', f'start_jitter:={a.jitter}',
                           f'output_dir:={a.output_dir}', f'run_name:={name}']
                    p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
                                         start_new_session=True)
                    try:
                        p.wait(timeout=a.timeout)
                    except subprocess.TimeoutExpired:
                        print('  timeout, killing')
                        os.killpg(p.pid, signal.SIGINT)
                        time.sleep(5)
                        os.killpg(p.pid, signal.SIGKILL)
                    # make sure no gzserver is left behind before the next run
                    subprocess.run(['pkill', '-x', 'gzserver'], check=False)
                    time.sleep(2)
    here = os.path.dirname(os.path.abspath(__file__))
    subprocess.run([sys.executable, os.path.join(here, 'analyze_results.py'), 'push', a.output_dir])


if __name__ == '__main__':
    main()
