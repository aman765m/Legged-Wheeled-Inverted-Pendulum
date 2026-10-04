#!/usr/bin/env python3
"""Apply a translational push to the robot body (Sec. V-A of the paper).

By default a force of `force` N is applied along -X of the body
(the "-X_C direction" of the paper, i.e. a push on the front of the robot)
at sim time `start_time` for `duration` seconds, via Gazebo's
/apply_link_wrench service (gazebo_ros_force_system).
"""
import random

import rclpy
from gazebo_msgs.srv import ApplyLinkWrench
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


def declare_number(node, name, default):
    """Declare a numeric parameter that accepts both `27` and `27.0` from the command line."""
    node.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
    return float(node.get_parameter(name).value)


class PushDisturbance(Node):
    def __init__(self):
        super().__init__('push_disturbance')
        self.force = declare_number(self, 'force', 27.0)
        self.duration = declare_number(self, 'duration', 1.0)
        start = declare_number(self, 'start_time', 10.0)
        jitter = declare_number(self, 'start_jitter', 0.0)   # random +- jitter [s] for repeated trials
        self.declare_parameter('direction', [-1.0, 0.0, 0.0])
        self.declare_parameter('link', 'wheel_legged_robot::base_link')
        self.declare_parameter('body_frame', True)
        self.cli = self.create_client(ApplyLinkWrench, '/apply_link_wrench')
        self.t_start = start + random.uniform(-jitter, jitter)
        self.done = False
        self.timer = self.create_timer(0.005, self.tick)

    def tick(self):
        if self.done:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now < self.t_start:
            return
        if not self.cli.service_is_ready():
            self.get_logger().warn('/apply_link_wrench not ready yet', throttle_duration_sec=2.0)
            return
        self.done = True
        f = self.force
        d = self.get_parameter('direction').value
        link = self.get_parameter('link').value
        req = ApplyLinkWrench.Request()
        req.link_name = link
        req.reference_frame = link if self.get_parameter('body_frame').value else ''
        req.wrench.force.x, req.wrench.force.y, req.wrench.force.z = (f * d[0], f * d[1], f * d[2])
        # gazebo_ros_force_system (Humble) does not treat 0 as "now": stamp the current sim time
        req.start_time = self.get_clock().now().to_msg()
        dur = self.duration
        req.duration.sec = int(dur)
        req.duration.nanosec = int((dur - int(dur)) * 1e9)
        self.get_logger().info(f'applying push of {f:.1f} N for {dur:.2f} s at t={now:.3f} s')
        self.cli.call_async(req).add_done_callback(
            lambda fut: self.get_logger().info(f'push applied: {fut.result().success}'))


def main():
    rclpy.init()
    node = PushDisturbance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
