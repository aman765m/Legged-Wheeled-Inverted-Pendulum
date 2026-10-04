#!/usr/bin/env python3
"""Record an experiment to CSV and summarise the CMP balance metric.

Columns: time, CMP error in the heading frame (ex, ey), CMP and CoM in the
world, base pose, pitch, arm end-effector, FSM state and robot acceleration.
At the end (sim time `duration`) a JSON summary is written with the
time-integrated |CMP error| (m.s, as in Table 2) and whether the robot fell.
The node then exits, which shuts the launch file down.
"""
import csv
import json
import math
import os

import rclpy
from geometry_msgs.msg import PointStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

COLUMNS = ['t', 'cmp_ex', 'cmp_ey', 'cmp_x', 'cmp_y', 'com_x', 'com_y', 'com_z',
           'base_x', 'base_y', 'base_yaw', 'pitch', 'beta', 'v', 'tau_r', 'tau_l',
           'ee_x', 'ee_y', 'ee_ref_x', 'ee_ref_y', 'fsm_state', 'acc_x', 'twa_yaw_rate']


def declare_number(node, name, default):
    """Declare a numeric parameter that accepts both `20` and `20.0` from the command line."""
    node.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
    return float(node.get_parameter(name).value)


class ExperimentLogger(Node):
    def __init__(self):
        super().__init__('experiment_logger')
        self.declare_parameter('output_dir', os.path.expanduser('~/wlbr_results'))
        self.declare_parameter('run_name', 'run')
        self.duration = declare_number(self, 'duration', 20.0)
        self.metric_start = declare_number(self, 'metric_start', 5.0)
        self.event_time = declare_number(self, 'event_time', 10.0)
        self.fall_pitch = declare_number(self, 'fall_pitch', 0.8)
        self.dt_sample = 1.0 / declare_number(self, 'sample_rate', 200.0)
        out = os.path.expanduser(self.get_parameter('output_dir').value)
        os.makedirs(out, exist_ok=True)
        name = self.get_parameter('run_name').value
        self.csv_path = os.path.join(out, f'{name}.csv')
        self.json_path = os.path.join(out, f'{name}.json')
        self.f = open(self.csv_path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(COLUMNS)
        self.row = {c: float('nan') for c in COLUMNS}
        self.last_t = None
        self.last_sample = -1e9
        self.int_x = self.int_y = 0.0
        self.int_x_event = 0.0
        self.max_ex = 0.0
        self.fell = False
        self.fall_time = None
        self.done = False

        self.create_subscription(Vector3Stamped, '/cmp_estimator/cmp_error', self.on_err, 50)
        self.create_subscription(PointStamped, '/cmp_estimator/cmp', self.on_cmp, 50)
        self.create_subscription(PointStamped, '/cmp_estimator/com', self.on_com, 50)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_odom, 50)
        self.create_subscription(Float64MultiArray, '/lqr_balance_controller/state', self.on_lqr, 50)
        self.create_subscription(Float64MultiArray, '/rac_arm_controller/state', self.on_arm, 50)
        self.create_subscription(Float64MultiArray, '/arm_fsm/state', self.on_fsm, 50)

    def on_cmp(self, m):
        self.row['cmp_x'], self.row['cmp_y'] = m.point.x, m.point.y

    def on_com(self, m):
        self.row['com_x'], self.row['com_y'], self.row['com_z'] = m.point.x, m.point.y, m.point.z

    def on_odom(self, m):
        p = m.pose.pose
        q = p.orientation
        self.row['base_x'], self.row['base_y'] = p.position.x, p.position.y
        self.row['base_yaw'] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        self.row['pitch'] = pitch
        if abs(pitch) > self.fall_pitch and not self.fell:
            self.fell = True
            self.fall_time = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            self.get_logger().warn(f'robot fell at t={self.fall_time:.2f} s')

    def on_lqr(self, m):
        d = m.data
        if len(d) >= 8:
            self.row['beta'], self.row['v'] = d[2], d[3]
            self.row['tau_r'], self.row['tau_l'] = d[6], d[7]

    def on_arm(self, m):
        d = m.data
        if len(d) >= 6:
            self.row['ee_x'], self.row['ee_y'] = d[0], d[1]
            self.row['ee_ref_x'], self.row['ee_ref_y'] = d[4], d[5]

    def on_fsm(self, m):
        d = m.data
        if len(d) >= 4:
            self.row['fsm_state'], self.row['acc_x'], self.row['twa_yaw_rate'] = d[0], d[2], d[3]

    def on_err(self, m):
        if self.done:
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        ex, ey = m.vector.x, m.vector.y
        if self.last_t is not None and t > self.last_t:
            dt = t - self.last_t
            if t >= self.metric_start:
                self.int_x += abs(ex) * dt
                self.int_y += abs(ey) * dt
                self.max_ex = max(self.max_ex, abs(ex))
            if t >= self.event_time:
                self.int_x_event += abs(ex) * dt
        self.last_t = t
        self.row['t'], self.row['cmp_ex'], self.row['cmp_ey'] = t, ex, ey
        if t - self.last_sample >= self.dt_sample:
            self.last_sample = t
            self.w.writerow([self.row[c] for c in COLUMNS])
        if t >= self.duration or (self.fell and t > self.fall_time + 1.0):
            self.finish()

    def finish(self):
        self.done = True
        self.f.close()
        summary = {
            'csv': self.csv_path,
            'fell': self.fell,
            'fall_time': self.fall_time,
            'cmp_error_integral_x': self.int_x,
            'cmp_error_integral_y': self.int_y,
            'cmp_error_integral_x_after_event': self.int_x_event,
            'max_abs_cmp_error_x': self.max_ex,
            'metric_start': self.metric_start,
            'duration': self.duration,
        }
        with open(self.json_path, 'w') as f:
            json.dump(summary, f, indent=2)
        self.get_logger().info(f'experiment finished: {json.dumps(summary)}')
        raise SystemExit


def main():
    rclpy.init()
    node = ExperimentLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if not node.done:
            node.f.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
