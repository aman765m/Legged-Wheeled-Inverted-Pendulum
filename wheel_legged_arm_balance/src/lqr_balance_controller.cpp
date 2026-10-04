#include "wheel_legged_arm_balance/lqr_balance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "pluginlib/class_list_macros.hpp"

namespace wheel_legged_arm_balance
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;

namespace
{
const char * const kImuSuffix[] = {
  "orientation.x", "orientation.y", "orientation.z", "orientation.w",
  "angular_velocity.x", "angular_velocity.y", "angular_velocity.z"};
}

CallbackReturn LqrBalanceController::on_init()
{
  try {
    auto_declare<std::string>("wheel_right_joint", "wheel_right_joint");
    auto_declare<std::string>("wheel_left_joint", "wheel_left_joint");
    auto_declare<std::string>("imu_name", "imu_sensor");
    // Gazebo Classic reports the IMU orientation relative to the spawn pose:
    // R_true = R_ref * R_imu with R_ref given here (roll, pitch, yaw)
    auto_declare<std::vector<double>>("imu_reference_rpy", {0.0, 0.0, 0.0});
    auto_declare<std::vector<std::string>>(
      "arm_joints", std::vector<std::string>{"arm_joint_1", "arm_joint_2"});

    const WipParameters d;
    auto_declare<double>("model.m_c", d.m_c);
    auto_declare<double>("model.m_w", d.m_w);
    auto_declare<double>("model.l", d.l);
    auto_declare<double>("model.I_xx", d.I_xx);
    auto_declare<double>("model.I_yy", d.I_yy);
    auto_declare<double>("model.I_zz", d.I_zz);
    auto_declare<double>("model.I_wa", d.I_wa);
    auto_declare<double>("model.I_wd", d.I_wd);
    auto_declare<double>("model.I_ma", d.I_ma);
    auto_declare<double>("model.I_md", d.I_md);
    auto_declare<double>("model.gamma", d.gamma);
    auto_declare<double>("model.g", d.g);
    auto_declare<double>("model.r_w", d.r_w);
    auto_declare<double>("model.b", d.b);
    auto_declare<double>("model.c", d.c);
    auto_declare<bool>("model.lump_arm", false);

    const ArmParameters a;
    auto_declare<double>("arm.l1", a.l1);
    auto_declare<double>("arm.l2", a.l2);
    auto_declare<double>("arm.lg1", a.lg1);
    auto_declare<double>("arm.lg2", a.lg2);
    auto_declare<double>("arm.m1", a.m1);
    auto_declare<double>("arm.m2", a.m2);
    auto_declare<double>("arm.I1", a.I1);
    auto_declare<double>("arm.I2", a.I2);
    auto_declare<double>("arm.shoulder_height", 0.23);
    auto_declare<double>("arm.home_q1", M_PI / 3.0);
    auto_declare<double>("arm.home_q2", -2.0 * M_PI / 3.0);
    auto_declare<double>("payload_mass", 0.36);
    auto_declare<bool>("com_compensation", true);

    auto_declare<std::string>("gain_source", "paper");   // paper | computed | custom
    auto_declare<std::vector<double>>("Q", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    auto_declare<std::vector<double>>("R", {1.0, 1.0});
    auto_declare<std::vector<double>>("K", std::vector<double>{});

    auto_declare<double>("torque_limit", 40.0);
    // "ref." of Eq. (12): feed-forward of the wheel viscous friction for the
    // commanded forward / yaw velocity (c * relative wheel speed)
    auto_declare<bool>("friction_feedforward", true);
    auto_declare<double>("max_position_error", 1.0);
    auto_declare<double>("max_yaw_error", 0.5);
    auto_declare<double>("cmd_vel_timeout", 0.5);
    auto_declare<double>("max_linear_velocity", 1.0);
    auto_declare<double>("max_angular_velocity", 1.5);
    auto_declare<double>("max_linear_acceleration", 1.0);
    auto_declare<double>("max_angular_acceleration", 2.0);
  } catch (const std::exception & e) {
    fprintf(stderr, "LqrBalanceController init exception: %s\n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration LqrBalanceController::command_interface_configuration() const
{
  return {interface_configuration_type::INDIVIDUAL,
    {wheel_right_ + "/effort", wheel_left_ + "/effort"}};
}

InterfaceConfiguration LqrBalanceController::state_interface_configuration() const
{
  InterfaceConfiguration cfg{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & j : {wheel_right_, wheel_left_}) {
    cfg.names.push_back(j + "/position");
    cfg.names.push_back(j + "/velocity");
  }
  for (const auto * s : kImuSuffix) {
    cfg.names.push_back(imu_name_ + "/" + s);
  }
  for (const auto & j : arm_joints_) {
    cfg.names.push_back(j + "/position");
  }
  return cfg;
}

Eigen::Vector3d LqrBalanceController::comInWheelFrame(double q1, double q2) const
{
  // Returns (x, z, total_mass) of the CoM of body + arm + payload relative to
  // the wheel axis, expressed in the body frame. Wheels are excluded
  // (their CoM is on the axis).
  const double zs = model_.l + shoulder_height_;
  const double x1 = arm_.lg1 * std::cos(q1);
  const double x2 = arm_.l1 * std::cos(q1) + arm_.lg2 * std::cos(q1 + q2);
  const double xp = arm_.l1 * std::cos(q1) + arm_.l2 * std::cos(q1 + q2);
  const double m = model_.m_c + arm_.m1 + arm_.m2 + payload_mass_;
  const double x = (arm_.m1 * x1 + arm_.m2 * x2 + payload_mass_ * xp) / m;
  const double z = (model_.m_c * model_.l + (arm_.m1 + arm_.m2 + payload_mass_) * zs) / m;
  return {x, z, m};
}

CallbackReturn LqrBalanceController::on_configure(const rclcpp_lifecycle::State &)
{
  auto n = get_node();
  wheel_right_ = n->get_parameter("wheel_right_joint").as_string();
  wheel_left_ = n->get_parameter("wheel_left_joint").as_string();
  imu_name_ = n->get_parameter("imu_name").as_string();
  arm_joints_ = n->get_parameter("arm_joints").as_string_array();
  const auto rpy = n->get_parameter("imu_reference_rpy").as_double_array();
  if (rpy.size() != 3) {
    RCLCPP_ERROR(n->get_logger(), "imu_reference_rpy needs 3 values");
    return CallbackReturn::ERROR;
  }
  imu_ref_ = Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX());
  if (!arm_joints_.empty() && arm_joints_.size() != 2) {
    RCLCPP_ERROR(n->get_logger(), "arm_joints must be empty or contain exactly 2 joints");
    return CallbackReturn::ERROR;
  }

  auto gp = [&](const std::string & name) {return n->get_parameter(name).as_double();};
  model_.m_c = gp("model.m_c");
  model_.m_w = gp("model.m_w");
  model_.l = gp("model.l");
  model_.I_xx = gp("model.I_xx");
  model_.I_yy = gp("model.I_yy");
  model_.I_zz = gp("model.I_zz");
  model_.I_wa = gp("model.I_wa");
  model_.I_wd = gp("model.I_wd");
  model_.I_ma = gp("model.I_ma");
  model_.I_md = gp("model.I_md");
  model_.gamma = gp("model.gamma");
  model_.g = gp("model.g");
  model_.r_w = gp("model.r_w");
  model_.b = gp("model.b");
  model_.c = gp("model.c");
  arm_.l1 = gp("arm.l1");
  arm_.l2 = gp("arm.l2");
  arm_.lg1 = gp("arm.lg1");
  arm_.lg2 = gp("arm.lg2");
  arm_.m1 = gp("arm.m1");
  arm_.m2 = gp("arm.m2");
  arm_.I1 = gp("arm.I1");
  arm_.I2 = gp("arm.I2");
  shoulder_height_ = gp("arm.shoulder_height");
  payload_mass_ = gp("payload_mass");
  com_compensation_ = n->get_parameter("com_compensation").as_bool() && !arm_joints_.empty();
  torque_limit_ = gp("torque_limit");
  friction_ff_ = n->get_parameter("friction_feedforward").as_bool();
  max_position_error_ = gp("max_position_error");
  max_yaw_error_ = gp("max_yaw_error");
  cmd_timeout_ = gp("cmd_vel_timeout");
  max_v_ = gp("max_linear_velocity");
  max_w_ = gp("max_angular_velocity");
  max_acc_ = gp("max_linear_acceleration");
  max_wacc_ = gp("max_angular_acceleration");

  // ---- model used for the gain computation ----
  WipParameters design = model_;
  if (n->get_parameter("model.lump_arm").as_bool()) {
    // lump arm + payload (home posture) into the body
    const double q1 = gp("arm.home_q1"), q2 = gp("arm.home_q2");
    const Eigen::Vector3d com = comInWheelFrame(q1, q2);
    const double zs = model_.l + shoulder_height_;
    const double xe = arm_.l1 * std::cos(q1);
    const double xt = xe + arm_.l2 * std::cos(q1 + q2);
    auto pm = [&](double m, double x, double z) {
        return m * ((x - com(0)) * (x - com(0)) + (z - com(1)) * (z - com(1)));
      };
    design.I_yy = model_.I_yy + pm(model_.m_c, 0.0, model_.l) + pm(arm_.m1, xe, zs) +
      pm(arm_.m2 + payload_mass_, xt, zs);
    design.m_c = com(2);
    design.l = std::hypot(com(0), com(1));
  }

  const std::string source = n->get_parameter("gain_source").as_string();
  if (source == "paper") {
    K_ = paperLqrGain();
  } else if (source == "computed") {
    const auto q = n->get_parameter("Q").as_double_array();
    const auto r = n->get_parameter("R").as_double_array();
    if (q.size() != 6 || r.size() != 2) {
      RCLCPP_ERROR(n->get_logger(), "Q must have 6 and R 2 diagonal entries");
      return CallbackReturn::ERROR;
    }
    Matrix6d A, Q = Matrix6d::Zero();
    Matrix62d B;
    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    for (int i = 0; i < 6; ++i) {Q(i, i) = q[i];}
    R(0, 0) = r[0];
    R(1, 1) = r[1];
    linearizeWip(design, A, B);
    try {
      K_ = solveLqr<6, 2>(A, B, Q, R);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(n->get_logger(), "%s", e.what());
      return CallbackReturn::ERROR;
    }
    std::ostringstream ss;
    ss << "A =\n" << A << "\nB =\n" << B;
    RCLCPP_INFO(n->get_logger(), "Linearised model (m_c=%.3f, l=%.4f, I_yy=%.4f):\n%s",
      design.m_c, design.l, design.I_yy, ss.str().c_str());
  } else if (source == "custom") {
    const auto k = n->get_parameter("K").as_double_array();
    if (k.size() != 12) {
      RCLCPP_ERROR(n->get_logger(), "K must contain 12 values (row-major 2x6)");
      return CallbackReturn::ERROR;
    }
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 6; ++j) {K_(i, j) = k[i * 6 + j];}
    }
  } else {
    RCLCPP_ERROR(n->get_logger(), "gain_source must be one of paper|computed|custom");
    return CallbackReturn::ERROR;
  }
  std::ostringstream ks;
  ks << K_;
  RCLCPP_INFO(n->get_logger(), "LQR gain (%s), X=[p a beta v a_dot beta_dot]:\n%s",
    source.c_str(), ks.str().c_str());

  cmd_sub_ = n->create_subscription<geometry_msgs::msg::Twist>(
    "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      auto c = std::make_shared<Command>();
      c->v = msg->linear.x;
      c->w = msg->angular.z;
      c->stamp = get_node()->now();
      cmd_buffer_.writeFromNonRT(c);
    });
  reset_sub_ = n->create_subscription<std_msgs::msg::Empty>(
    "~/reset", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Empty::SharedPtr) {reset_requested_ = true;});
  state_pub_raw_ = n->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/state", rclcpp::SystemDefaultsQoS());
  state_pub_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
    state_pub_raw_);
  state_pub_->msg_.data.resize(14, 0.0);
  return CallbackReturn::SUCCESS;
}

double LqrBalanceController::state(const std::string & name) const
{
  return state_interfaces_[state_index_.at(name)].get_value();
}

CallbackReturn LqrBalanceController::on_activate(const rclcpp_lifecycle::State &)
{
  state_index_.clear();
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    state_index_[state_interfaces_[i].get_name()] = i;
  }
  for (const auto & name : state_interface_configuration().names) {
    if (state_index_.count(name) == 0) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing state interface %s", name.c_str());
      return CallbackReturn::ERROR;
    }
  }
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    if (command_interfaces_[i].get_name() == wheel_right_ + "/effort") {cmd_right_ = i;}
    if (command_interfaces_[i].get_name() == wheel_left_ + "/effort") {cmd_left_ = i;}
  }
  p_ = p_ref_ = v_ref_ = w_ref_ = 0.0;
  first_update_ = true;
  cmd_buffer_.writeFromNonRT(nullptr);
  return CallbackReturn::SUCCESS;
}

CallbackReturn LqrBalanceController::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & c : command_interfaces_) {
    c.set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type LqrBalanceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const double dt = std::clamp(period.seconds(), 0.0, 0.02);
  const std::string & imu = imu_name_;

  // ---- IMU: ZYX Euler angles and their rates ----
  const Eigen::Quaterniond q_imu(state(imu + "/orientation.w"), state(imu + "/orientation.x"),
    state(imu + "/orientation.y"), state(imu + "/orientation.z"));
  const Eigen::Quaterniond q = imu_ref_ * q_imu;
  const double qx = q.x(), qy = q.y(), qz = q.z(), qw = q.w();
  const double wx = state(imu + "/angular_velocity.x");
  const double wy = state(imu + "/angular_velocity.y");
  const double wz = state(imu + "/angular_velocity.z");
  (void)wx;
  const double roll = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
  const double pitch = std::asin(std::clamp(2.0 * (qw * qy - qz * qx), -1.0, 1.0));
  const double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
  const double pitch_rate = wy * std::cos(roll) - wz * std::sin(roll);
  const double yaw_rate = (wy * std::sin(roll) + wz * std::cos(roll)) / std::cos(pitch);

  if (reset_requested_.exchange(false)) {
    p_ = p_ref_ = v_ref_ = w_ref_ = 0.0;
    first_update_ = true;
  }
  if (first_update_) {
    yaw_prev_ = yaw;
    yaw_unwrapped_ = yaw;
    alpha_ref_ = yaw;
    first_update_ = false;
  }
  yaw_unwrapped_ += wrapAngle(yaw - yaw_prev_);
  yaw_prev_ = yaw;

  // ---- beta: pitch of the wheel-axis -> CoM line ----
  double beta = pitch;
  if (com_compensation_) {
    const Eigen::Vector3d com = comInWheelFrame(
      state(arm_joints_[0] + "/position"), state(arm_joints_[1] + "/position"));
    beta += std::atan2(com(0), com(1));
  }

  // ---- forward velocity from wheel odometry (wheel speed is relative to the body) ----
  const double wr = state(wheel_right_ + "/velocity");
  const double wl = state(wheel_left_ + "/velocity");
  const double v = model_.r_w * (0.5 * (wr + wl) + pitch_rate);
  p_ += v * dt;

  // ---- references (rate limited) ----
  double v_cmd = 0.0, w_cmd = 0.0;
  auto cmd = cmd_buffer_.readFromRT();
  if (cmd && *cmd && (time - (*cmd)->stamp).seconds() < cmd_timeout_) {
    v_cmd = std::clamp((*cmd)->v, -max_v_, max_v_);
    w_cmd = std::clamp((*cmd)->w, -max_w_, max_w_);
  }
  v_ref_ += std::clamp(v_cmd - v_ref_, -max_acc_ * dt, max_acc_ * dt);
  w_ref_ += std::clamp(w_cmd - w_ref_, -max_wacc_ * dt, max_wacc_ * dt);
  p_ref_ += v_ref_ * dt;
  alpha_ref_ += w_ref_ * dt;

  // keep the integrated references within reach (avoids violent recovery after big pushes)
  double e_p = std::clamp(p_ - p_ref_, -max_position_error_, max_position_error_);
  p_ref_ = p_ - e_p;
  double e_a = std::clamp(yaw_unwrapped_ - alpha_ref_, -max_yaw_error_, max_yaw_error_);
  alpha_ref_ = yaw_unwrapped_ - e_a;

  // ---- Eq. (12): U = ref - K (X - X_ref) ----
  Eigen::Matrix<double, 6, 1> e;
  e << e_p, e_a, beta, v - v_ref_, yaw_rate - w_ref_, pitch_rate;
  Eigen::Vector2d u = -K_ * e;
  if (friction_ff_) {
    const double wv = v_ref_ / model_.r_w, ww = model_.b * w_ref_ / model_.r_w;
    u(0) += model_.c * (wv + ww);
    u(1) += model_.c * (wv - ww);
  }
  u = u.cwiseMax(-torque_limit_).cwiseMin(torque_limit_);
  if (std::abs(pitch) > 1.2) {   // fallen: stop the wheels
    u.setZero();
  }
  command_interfaces_[cmd_right_].set_value(u(0));
  command_interfaces_[cmd_left_].set_value(u(1));

  if (state_pub_ && state_pub_->trylock()) {
    auto & d = state_pub_->msg_.data;
    d = {p_, yaw_unwrapped_, beta, v, yaw_rate, pitch_rate, u(0), u(1),
      p_ref_, alpha_ref_, v_ref_, w_ref_, pitch, roll};
    state_pub_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

}  // namespace wheel_legged_arm_balance

PLUGINLIB_EXPORT_CLASS(
  wheel_legged_arm_balance::LqrBalanceController, controller_interface::ControllerInterface)
