#!/usr/bin/env python3
"""Put the robot back at its spawn pose without restarting the simulation.

Calls Gazebo's /reset_world (poses and velocities only; the sim clock keeps
running, unlike /reset_simulation which rewinds time and breaks the
controllers), then clears the integrated references of the balance
controller and the CMP error integral / paths of the CMP estimator.

  ros2 run wheel_legged_arm_balance reset_sim.py
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty, Trigger


def main():
    rclpy.init()
    node = Node('reset_sim')
    reset_pub = node.create_publisher(EmptyMsg, '/lqr_balance_controller/reset', 10)
    world = node.create_client(Empty, '/reset_world')
    cmp = node.create_client(Trigger, '/cmp_estimator/reset')
    if not world.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('/reset_world not available - is the simulation running?')
    else:
        fut = world.call_async(Empty.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        # the subscription must be matched before publishing
        for _ in range(20):
            if reset_pub.get_subscription_count() > 0:
                break
            rclpy.spin_once(node, timeout_sec=0.05)
        reset_pub.publish(EmptyMsg())
        if cmp.wait_for_service(timeout_sec=1.0):
            fut = cmp.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
        rclpy.spin_once(node, timeout_sec=0.2)
        node.get_logger().info('robot reset to its spawn pose')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
