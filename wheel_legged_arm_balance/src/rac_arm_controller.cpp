#include "wheel_legged_arm_balance/rac_arm_controller.hpp"

#include <algorithm>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"

namespace wheel_legged_arm_balance
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;

CallbackReturn RacArmController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>(
      "joints", std::vector<std::string>{"arm_joint_1", "arm_joint_2"});
    const ArmParameters a;
    auto_declare<double>("arm.l1", a.l1);
    auto_declare<double>("arm.l2", a.l2);
    auto_declare<double>("arm.lg1", a.lg1);
    auto_declare<double>("arm.lg2", a.lg2);
    auto_declare<double>("arm.m1", a.m1);
    auto_declare<double>("arm.m2", a.m2);
    auto_declare<double>("arm.I1", a.I1);
    auto_declare<double>("arm.I2", a.I2);
    auto_declare<double>("payload_mass", 0.36);
    auto_declare<double>("payload_inertia", 0.0002);
    auto_declare<std::vector<double>>("Kp", {400.0, 400.0});
    auto_declare<std::vector<double>>("Kv", {40.0, 40.0});
    auto_declare<double>("effort_limit", 30.0);
    auto_declare<double>("jacobian_damping", 1e-3);
    auto_declare<double>("max_reach", 0.58);
    auto_declare<double>("min_reach", 0.05);
  } catch (const std::exception & e) {
    fprintf(stderr, "RacArmController init exception: %s\n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration RacArmController::command_interface_configuration() const
{
  InterfaceConfiguration cfg{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & j : joints_) {
    cfg.names.push_back(j + "/effort");
  }
  return cfg;
}

InterfaceConfiguration RacArmController::state_interface_configuration() const
{
  InterfaceConfiguration cfg{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & j : joints_) {
    cfg.names.push_back(j + "/position");
    cfg.names.push_back(j + "/velocity");
  }
  return cfg;
}

CallbackReturn RacArmController::on_configure(const rclcpp_lifecycle::State &)
{
  auto n = get_node();
  joints_ = n->get_parameter("joints").as_string_array();
  if (joints_.size() != 2) {
    RCLCPP_ERROR(n->get_logger(), "'joints' must contain exactly 2 joints");
    return CallbackReturn::ERROR;
  }
  auto gp = [&](const std::string & name) {return n->get_parameter(name).as_double();};
  ArmParameters a;
  a.l1 = gp("arm.l1");
  a.l2 = gp("arm.l2");
  a.lg1 = gp("arm.lg1");
  a.lg2 = gp("arm.lg2");
  a.m1 = gp("arm.m1");
  a.m2 = gp("arm.m2");
  a.I1 = gp("arm.I1");
  a.I2 = gp("arm.I2");
  arm_.setParameters(a.withPayload(gp("payload_mass"), gp("payload_inertia")));

  const auto kp = n->get_parameter("Kp").as_double_array();
  const auto kv = n->get_parameter("Kv").as_double_array();
  if (kp.size() != 2 || kv.size() != 2) {
    RCLCPP_ERROR(n->get_logger(), "Kp and Kv need 2 values");
    return CallbackReturn::ERROR;
  }
  kp_ << kp[0], kp[1];
  kv_ << kv[0], kv[1];
  effort_limit_ = gp("effort_limit");
  damping_ = gp("jacobian_damping");
  max_reach_ = gp("max_reach");
  min_reach_ = gp("min_reach");
  RCLCPP_INFO(n->get_logger(), "RAC arm: m2_eff=%.3f lg2_eff=%.3f I2_eff=%.4f Kp=[%.0f %.0f] Kv=[%.0f %.0f]",
    arm_.parameters().m2, arm_.parameters().lg2, arm_.parameters().I2,
    kp_(0), kp_(1), kv_(0), kv_(1));

  ref_sub_ = n->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
    "~/reference", rclcpp::SystemDefaultsQoS(),
    [this](const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg) {
      if (msg->positions.size() < 2) {
        return;
      }
      auto r = std::make_shared<Reference>();
      r->r << msg->positions[0], msg->positions[1];
      if (msg->velocities.size() >= 2) {r->rd << msg->velocities[0], msg->velocities[1];}
      if (msg->accelerations.size() >= 2) {r->rdd << msg->accelerations[0], msg->accelerations[1];}
      ref_buffer_.writeFromNonRT(r);
    });
  state_pub_raw_ = n->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/state", rclcpp::SystemDefaultsQoS());
  state_pub_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
    state_pub_raw_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn RacArmController::on_activate(const rclcpp_lifecycle::State &)
{
  for (size_t k = 0; k < 2; ++k) {
    for (size_t i = 0; i < state_interfaces_.size(); ++i) {
      if (state_interfaces_[i].get_name() == joints_[k] + "/position") {pos_idx_[k] = i;}
      if (state_interfaces_[i].get_name() == joints_[k] + "/velocity") {vel_idx_[k] = i;}
    }
    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
      if (command_interfaces_[i].get_name() == joints_[k] + "/effort") {cmd_idx_[k] = i;}
    }
  }
  // hold the current end-effector position until a reference arrives
  const Eigen::Vector2d q(state_interfaces_[pos_idx_[0]].get_value(),
    state_interfaces_[pos_idx_[1]].get_value());
  auto r = std::make_shared<Reference>();
  r->r = arm_.forwardKinematics(q);
  ref_buffer_.writeFromNonRT(r);
  return CallbackReturn::SUCCESS;
}

CallbackReturn RacArmController::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & c : command_interfaces_) {
    c.set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type RacArmController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const Eigen::Vector2d q(state_interfaces_[pos_idx_[0]].get_value(),
    state_interfaces_[pos_idx_[1]].get_value());
  const Eigen::Vector2d qd(state_interfaces_[vel_idx_[0]].get_value(),
    state_interfaces_[vel_idx_[1]].get_value());

  auto ref_ptr = ref_buffer_.readFromRT();
  if (!ref_ptr || !*ref_ptr) {
    command_interfaces_[cmd_idx_[0]].set_value(0.0);
    command_interfaces_[cmd_idx_[1]].set_value(0.0);
    return controller_interface::return_type::OK;
  }
  Reference ref = **ref_ptr;
  // keep the reference inside the reachable annulus
  const double reach = ref.r.norm();
  if (reach > max_reach_) {ref.r *= max_reach_ / reach;}
  if (reach < min_reach_ && reach > 1e-9) {ref.r *= min_reach_ / reach;}

  const Eigen::Vector2d r = arm_.forwardKinematics(q);
  const Eigen::Matrix2d J = arm_.jacobian(q);
  const Eigen::Vector2d rd = J * qd;

  // Eq. (17)
  const Eigen::Vector2d a = ref.rdd + kv_.cwiseProduct(ref.rd - rd) +
    kp_.cwiseProduct(ref.r - r) - arm_.jacobianDotTimesQdot(q, qd);
  // damped least-squares inverse to stay well defined near the stretched posture
  const Eigen::Matrix2d Jinv =
    J.transpose() * (J * J.transpose() + damping_ * Eigen::Matrix2d::Identity()).inverse();
  const Eigen::Vector2d qdd = Jinv * a;
  Eigen::Vector2d tau = arm_.massMatrix(q) * qdd + arm_.coriolis(q, qd);
  tau = tau.cwiseMax(-effort_limit_).cwiseMin(effort_limit_);

  command_interfaces_[cmd_idx_[0]].set_value(tau(0));
  command_interfaces_[cmd_idx_[1]].set_value(tau(1));

  if (state_pub_ && state_pub_->trylock()) {
    state_pub_->msg_.data = {r(0), r(1), rd(0), rd(1), ref.r(0), ref.r(1), ref.rdd(0), ref.rdd(1),
      tau(0), tau(1)};
    state_pub_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

}  // namespace wheel_legged_arm_balance

PLUGINLIB_EXPORT_CLASS(
  wheel_legged_arm_balance::RacArmController, controller_interface::ControllerInterface)
