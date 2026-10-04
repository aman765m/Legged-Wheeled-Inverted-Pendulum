#!/usr/bin/env python3
"""Infinity-loop (figure-eight) trajectory tracking (Sec. V-B, Fig. 8).

A lemniscate of Bernoulli (half width `size`), rotated so that it starts at
the robot's initial pose with tangent +X, is tracked with a pure-pursuit law
using the ground-truth odometry. The commanded forward speed is
`max_linear_velocity` and the yaw rate is saturated at `max_angular_velocity`,
so at high speed the robot drifts outwards on the curves, as in the paper.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def declare_number(node, name, default):
    """Declare a numeric parameter that accepts both `1` and `1.0` from the command line."""
    node.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
    return float(node.get_parameter(name).value)


class InfinityLoopTracker(Node):
    def __init__(self):
        super().__init__('infinity_loop_tracker')
        a = declare_number(self, 'size', 4.0)                     # lemniscate half width a [m]
        self.v = declare_number(self, 'max_linear_velocity', 0.3)
        self.w_max = declare_number(self, 'max_angular_velocity', 0.3)
        self.L = declare_number(self, 'lookahead', 1.0)
        self.laps = declare_number(self, 'laps', 1.0)
        self.t_start = declare_number(self, 'start_time', 5.0)
        rate = declare_number(self, 'rate', 50.0)
        self.declare_parameter('cmd_topic', '/lqr_balance_controller/cmd_vel')
        self.declare_parameter('target_csv', '')
        # robot: anchor the path at the robot's first pose; world: path starts at the
        # world origin heading +X (matches worlds/figure8_track.world)
        self.declare_parameter('path_anchor', 'robot')

        # lemniscate of Bernoulli, starting at the crossing point (s = pi/2);
        # min. radius of curvature = a / 3
        s = np.linspace(0, 2 * math.pi, 6000, endpoint=False) + math.pi / 2
        den = 1.0 + np.sin(s) ** 2
        pts = np.stack([a * np.cos(s) / den, a * np.sin(s) * np.cos(s) / den], axis=1)
        tangent = pts[1] - pts[0]
        rot = -math.atan2(tangent[1], tangent[0])           # initial tangent -> +X
        R = np.array([[math.cos(rot), -math.sin(rot)], [math.sin(rot), math.cos(rot)]])
        self.base_path = (pts - pts[0]) @ R.T
        seg = np.linalg.norm(np.diff(np.vstack([self.base_path, self.base_path[:1]]), axis=0), axis=1)
        self.loop_len = float(seg.sum())
        self.path = None
        self.idx = 0
        self.travelled = 0.0
        self.pose = None
        self.finished = False
        self.finish_time = 0.0

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_topic').value, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '~/target_path', latched)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_odom, 10)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f'infinity loop: a={a} m, length={self.loop_len:.2f} m, v={self.v}, w_max={self.w_max}')

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def publish_path(self):
        path = Path()
        path.header.frame_id = 'world'
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in np.vstack([self.path, self.path[:1]]):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.finished:
            if now - self.finish_time > 3.0:
                raise SystemExit
            return
        if self.pose is None:
            return
        x, y, th = self.pose
        if self.path is None:
            if self.get_parameter('path_anchor').value == 'world':
                self.path = self.base_path.copy()
            else:
                # anchor the path at the robot's pose
                c, s = math.cos(th), math.sin(th)
                self.path = self.base_path @ np.array([[c, s], [-s, c]]) + np.array([x, y])
            self.publish_path()
            out = self.get_parameter('target_csv').value
            if out:
                np.savetxt(out, self.path, delimiter=',', header='x,y', comments='')
        if now < self.t_start:
            return
        n = len(self.path)
        # advance the closest-point index monotonically (search a window ahead)
        window = [(self.idx + k) % n for k in range(0, 300)]
        d = [math.hypot(self.path[i][0] - x, self.path[i][1] - y) for i in window]
        k_best = int(np.argmin(d))
        if k_best > 0:
            pts = self.path[[window[j] for j in range(k_best + 1)]]
            self.travelled += np.linalg.norm(np.diff(pts, axis=0), axis=1).sum()
        self.idx = window[k_best]
        if self.travelled >= self.laps * self.loop_len - 0.05:
            self.finished = True
            self.finish_time = now
            self.cmd_pub.publish(Twist())
            self.get_logger().info('infinity loop finished')
            return
        # lookahead point
        j, acc = self.idx, 0.0
        while acc < self.L:
            nj = (j + 1) % n
            acc += math.hypot(*(self.path[nj] - self.path[j]))
            j = nj
        tx, ty = self.path[j]
        alpha = math.atan2(ty - y, tx - x) - th
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        kappa = 2.0 * math.sin(alpha) / self.L
        cmd = Twist()
        cmd.linear.x = self.v
        cmd.angular.z = max(-self.w_max, min(self.w_max, self.v * kappa))
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = InfinityLoopTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
