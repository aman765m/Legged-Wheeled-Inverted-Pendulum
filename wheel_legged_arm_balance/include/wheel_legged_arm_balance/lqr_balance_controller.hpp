#pragma once

#include <Eigen/Geometry>

#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "wheel_legged_arm_balance/arm_dynamics.hpp"
#include "wheel_legged_arm_balance/wip_model.hpp"

namespace wheel_legged_arm_balance
{

/**
 * Lower-body balance controller of the wheel-legged robot (Sec. II-A).
 *
 * Reads the wheel joint states and the IMU, builds the state vector
 * X = [p, alpha, beta, v, alpha_dot, beta_dot] and applies
 * U = ref - K (X - X_ref) to the wheel torques (Eq. 12), where "ref" is a
 * feed-forward of the wheel viscous friction. Velocity references come from
 * ~/cmd_vel (Twist).
 *
 * beta is the pitch of the line joining the wheel axis and the CoM (the angle
 * between Z_R and Z_C in Fig. 2). When arm joints are configured, the CoM
 * includes the arm and payload, so a static arm offset does not make the
 * robot drift.
 */
class LqrBalanceController : public controller_interface::ControllerInterface
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
  struct Command
  {
    double v{0.0};
    double w{0.0};
    rclcpp::Time stamp;
  };

  double state(const std::string & name) const;
  Eigen::Vector3d comInWheelFrame(double q1, double q2) const;

  // parameters
  std::string wheel_right_, wheel_left_, imu_name_;
  std::vector<std::string> arm_joints_;
  WipParameters model_;
  ArmParameters arm_;
  double payload_mass_{0.36};
  double shoulder_height_{0.23};
  double torque_limit_{40.0};
  double max_position_error_{1.0};
  double max_yaw_error_{0.5};
  double cmd_timeout_{0.5};
  double max_v_{1.0}, max_w_{1.5}, max_acc_{1.0}, max_wacc_{2.0};
  bool com_compensation_{true};
  bool friction_ff_{true};
  Matrix26d K_;
  Eigen::Quaterniond imu_ref_{Eigen::Quaterniond::Identity()};

  // interface lookup
  std::unordered_map<std::string, size_t> state_index_;
  size_t cmd_right_{0}, cmd_left_{1};

  // runtime state
  double p_{0.0}, p_ref_{0.0}, alpha_ref_{0.0}, v_ref_{0.0}, w_ref_{0.0};
  double yaw_prev_{0.0}, yaw_unwrapped_{0.0};
  bool first_update_{true};

  realtime_tools::RealtimeBuffer<std::shared_ptr<Command>> cmd_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  // ~/reset: zero the integrated position / yaw references (e.g. after /reset_world)
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
  std::atomic<bool> reset_requested_{false};
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> state_pub_raw_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> state_pub_;
};

}  // namespace wheel_legged_arm_balance
