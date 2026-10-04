#!/usr/bin/env python3
"""Activate the controllers and un-pause Gazebo at the same instant.

The robot is an inverted pendulum: if physics ran while the controllers are
still being spawned, it would fall. Gazebo is therefore started paused, the
controllers are loaded+configured (spawner --inactive), and this node then
requests their activation and un-pauses the simulation, so that the balance
controller acts from the very first physics step.
"""
import time

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.node import Node
from std_srvs.srv import Empty


class SimStarter(Node):
    def __init__(self):
        super().__init__('sim_starter')
        self.declare_parameter('controllers', ['joint_state_broadcaster', 'imu_sensor_broadcaster',
                                               'lqr_balance_controller', 'rac_arm_controller'])
        self.declare_parameter('controller_manager', '/controller_manager')
        self.declare_parameter('timeout', 120.0)
        self.names = list(self.get_parameter('controllers').value)
        cm = self.get_parameter('controller_manager').value
        self.list_cli = self.create_client(ListControllers, f'{cm}/list_controllers')
        self.switch_cli = self.create_client(SwitchController, f'{cm}/switch_controller')
        self.unpause_cli = self.create_client(Empty, '/unpause_physics')

    def call(self, cli, req, timeout=10.0):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def run(self):
        deadline = time.time() + self.get_parameter('timeout').value
        for cli in (self.list_cli, self.switch_cli, self.unpause_cli):
            while not cli.wait_for_service(timeout_sec=1.0):
                if time.time() > deadline:
                    self.get_logger().error(f'service {cli.srv_name} not available')
                    return False
        # wait until every controller is loaded and configured
        while time.time() < deadline:
            res = self.call(self.list_cli, ListControllers.Request())
            states = {c.name: c.state for c in res.controller} if res else {}
            if all(states.get(n) in ('inactive', 'active') for n in self.names):
                break
            time.sleep(0.2)
        else:
            self.get_logger().error('controllers were not configured in time')
            return False

        req = SwitchController.Request()
        req.activate_controllers = [n for n in self.names if states.get(n) != 'active']
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout.sec = 10
        fut = self.switch_cli.call_async(req)
        time.sleep(0.2)   # make sure the switch request is queued before physics resumes
        self.call(self.unpause_cli, Empty.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        ok = fut.result() is not None and fut.result().ok
        if ok:
            self.get_logger().info(f'activated {req.activate_controllers} and started physics')
        else:
            self.get_logger().error('controller activation failed')
        return ok


def main():
    rclpy.init()
    node = SimStarter()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
