#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "wheel_legged_arm_balance/arm_dynamics.hpp"

namespace wheel_legged_arm_balance
{

/**
 * Resolved acceleration control of the 2-link arm (Sec. II-B, Eq. 17, Fig. 4):
 *
 *   tau = M(q) J^-1 [ r_dd_d + Kv (r_d_d - r_d) + Kp (r_d - r) - J_dot q_dot ] + h(q, q_dot)
 *
 * The Cartesian reference (frame Sigma_A) is received on ~/reference as a
 * trajectory_msgs/JointTrajectoryPoint whose positions / velocities /
 * accelerations hold [x, y] of the end-effector.
 */
class RacArmController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Reference
  {
    Eigen::Vector2d r{0.3, 0.0};
    Eigen::Vector2d rd{0.0, 0.0};
    Eigen::Vector2d rdd{0.0, 0.0};
  };

  std::vector<std::string> joints_;
  TwoLinkArm arm_;
  Eigen::Vector2d kp_{400.0, 400.0};
  Eigen::Vector2d kv_{40.0, 40.0};
  double effort_limit_{30.0};
  double damping_{1e-3};
  double max_reach_{0.58};
  double min_reach_{0.05};
  size_t pos_idx_[2]{0, 2}, vel_idx_[2]{1, 3}, cmd_idx_[2]{0, 1};

  realtime_tools::RealtimeBuffer<std::shared_ptr<Reference>> ref_buffer_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr ref_sub_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> state_pub_raw_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> state_pub_;
};

}  // namespace wheel_legged_arm_balance
