// Finite-state machine for active arm usage (Sec. III, Fig. 5 of Raza et al. 2021).
//
//  1. If the translational acceleration of the robot |a_x| >= a_th, the
//     end-effector accelerates in the opposite direction:  A_x_ee_dd = -K_x a_x.
//  2. Otherwise the time-weighted average of the reference yaw velocity over
//     the last 0.5 s decides the target posture:
//        (alpha_dot_ref)_TWA == 0 -> Home  (0.3, 0)
//        alpha_dot_ref < 0       -> Right (0.15, -0.4)   (clockwise turn)
//        alpha_dot_ref > 0       -> Left  (0.15,  0.4)   (counter-clockwise turn)
//
// Additions not specified in the paper: the push state has hysteresis
// (it ends when |a_x| < a_th_exit_ratio * a_th) and is followed by a HOLD
// state of `hold_time` seconds during which the end-effector stays where the
// push response left it, before returning Home.
//
// The node turns this decision into a consistent Cartesian reference
// (r_d, r_d_dot, r_d_ddot) for the resolved acceleration arm controller.
// mode:=fixed keeps the arm at Home (the "fixed arm" baseline of the paper).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace wheel_legged_arm_balance
{

class ArmFsmNode : public rclcpp::Node
{
public:
  enum State { HOME = 0, PUSH = 1, RIGHT = 2, LEFT = 3, HOLD = 4 };

  ArmFsmNode()
  : Node("arm_fsm")
  {
    mode_ = declare_parameter<std::string>("mode", "active");
    a_th_ = declare_parameter<double>("a_th", 1.9);
    k_x_ = declare_parameter<double>("K_x", 6.0);
    exit_ratio_ = declare_parameter<double>("a_th_exit_ratio", 0.5);
    hold_time_ = declare_parameter<double>("hold_time", 1.0);
    home_ = vec(declare_parameter<std::vector<double>>("home", {0.3, 0.0}));
    right_ = vec(declare_parameter<std::vector<double>>("right", {0.15, -0.4}));
    left_ = vec(declare_parameter<std::vector<double>>("left", {0.15, 0.4}));
    twa_window_ = declare_parameter<double>("twa_window", 0.5);
    yaw_eps_ = declare_parameter<double>("yaw_rate_epsilon", 0.05);
    wn_ = declare_parameter<double>("return_natural_frequency", 3.0);
    zeta_ = declare_parameter<double>("return_damping_ratio", 1.0);
    wn_turn_ = declare_parameter<double>("turn_natural_frequency", 1.5);
    accel_cutoff_ = declare_parameter<double>("accel_cutoff_hz", 10.0);
    rate_ = declare_parameter<double>("rate", 500.0);
    x_min_ = declare_parameter<double>("workspace.x_min", 0.05);
    x_max_ = declare_parameter<double>("workspace.x_max", 0.58);
    reach_max_ = declare_parameter<double>("workspace.max_reach", 0.58);
    v_max_ = declare_parameter<double>("max_ee_velocity", 2.5);
    acc_max_ = declare_parameter<double>("max_ee_acceleration", 25.0);
    cmd_timeout_ = declare_parameter<double>("cmd_vel_timeout", 0.5);
    const double gravity = declare_parameter<double>("gravity", 9.81);
    g_ = Eigen::Vector3d(0.0, 0.0, -gravity);
    // Gazebo Classic reports the IMU orientation relative to the spawn pose
    const auto rpy = declare_parameter<std::vector<double>>("imu_reference_rpy", {0.0, 0.0, 0.0});
    if (rpy.size() == 3) {
      imu_ref_ = Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX());
    }

    r_d_ = home_;
    rd_d_.setZero();
    rdd_d_.setZero();

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr m) {onImu(*m);});
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr m) {
        w_ref_ = m->angular.z;
        last_cmd_ = now();
      });
    ref_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>("arm_reference", 10);
    state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/state", 10);

    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_), [this]() {step();});
    RCLCPP_INFO(get_logger(), "Arm FSM mode=%s a_th=%.2f K_x=%.2f", mode_.c_str(), a_th_, k_x_);
  }

private:
  static Eigen::Vector2d vec(const std::vector<double> & v)
  {
    return {v.size() > 0 ? v[0] : 0.0, v.size() > 1 ? v[1] : 0.0};
  }

  void onImu(const sensor_msgs::msg::Imu & m)
  {
    const Eigen::Quaterniond q = imu_ref_ *
      Eigen::Quaterniond(m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z);
    const Eigen::Vector3d f(m.linear_acceleration.x, m.linear_acceleration.y,
      m.linear_acceleration.z);
    // accelerometer measures specific force f = R^T (a - g)  ->  a = R f + g
    const Eigen::Vector3d a_world = q.toRotationMatrix() * f + g_;
    const Eigen::Vector3d xb = q.toRotationMatrix().col(0);
    const double yaw = std::atan2(xb.y(), xb.x());
    const double ax = a_world.x() * std::cos(yaw) + a_world.y() * std::sin(yaw);

    const rclcpp::Time t(m.header.stamp);
    if (have_imu_) {
      const double dt = std::clamp((t - last_imu_).seconds(), 0.0, 0.05);
      const double alpha = 1.0 - std::exp(-2.0 * M_PI * accel_cutoff_ * dt);
      a_x_filt_ += alpha * (ax - a_x_filt_);
    } else {
      a_x_filt_ = ax;
    }
    a_x_raw_ = ax;
    last_imu_ = t;
    have_imu_ = true;
  }

  double timeWeightedYawRate(const rclcpp::Time & t)
  {
    double w = 0.0;
    if (last_cmd_.nanoseconds() > 0 && (t - last_cmd_).seconds() < cmd_timeout_) {
      w = w_ref_;
    }
    yaw_hist_.emplace_back(t.seconds(), w);
    while (!yaw_hist_.empty() && t.seconds() - yaw_hist_.front().first > twa_window_) {
      yaw_hist_.pop_front();
    }
    // linearly increasing weights: the newest samples count most
    double num = 0.0, den = 0.0;
    const double t0 = t.seconds() - twa_window_;
    for (const auto & [ts, wv] : yaw_hist_) {
      const double weight = std::max(ts - t0, 1e-6);
      num += weight * wv;
      den += weight;
    }
    return den > 0.0 ? num / den : 0.0;
  }

  void step()
  {
    const rclcpp::Time t = now();
    double dt = last_step_.nanoseconds() > 0 ? (t - last_step_).seconds() : 0.0;
    last_step_ = t;
    if (dt <= 0.0 || dt > 0.1) {
      dt = 1.0 / rate_;
    }
    const double twa = timeWeightedYawRate(t);

    if (mode_ == "fixed") {
      state_ = HOME;
      r_d_ = home_;
      rd_d_.setZero();
      rdd_d_.setZero();
    } else {
      Eigen::Vector2d target = home_;
      const double a_abs = std::abs(a_x_filt_);
      // hysteresis: a push stays active until |a| drops below exit_ratio * a_th
      const bool pushing = have_imu_ &&
        (a_abs >= a_th_ || (state_ == PUSH && a_abs >= exit_ratio_ * a_th_));
      if (pushing) {
        state_ = PUSH;
        hold_until_ = t + rclcpp::Duration::from_seconds(hold_time_);
      } else if (t < hold_until_) {
        // keep the end-effector where the push response left it while the
        // robot recovers, then go back to the FSM targets
        state_ = HOLD;
      } else if (std::abs(twa) <= yaw_eps_) {
        state_ = HOME;
      } else if (twa < 0.0) {
        state_ = RIGHT;
        target = right_;
      } else {
        state_ = LEFT;
        target = left_;
      }

      if (state_ == PUSH) {
        // accelerate the end-effector opposite to the robot's acceleration
        rdd_d_ << -k_x_ * a_x_filt_, 0.0;
        // mild damping of the lateral component so it stays still
        rdd_d_(1) = -2.0 * wn_ * rd_d_(1);
      } else if (state_ == HOLD) {
        rdd_d_ = -2.0 * wn_ * rd_d_;
      } else {
        // critically damped second-order transition to the target posture
        // (slower for the turn postures to limit the lateral reaction forces)
        const double w = (state_ == RIGHT || state_ == LEFT) ? wn_turn_ : wn_;
        rdd_d_ = w * w * (target - r_d_) - 2.0 * zeta_ * w * rd_d_;
      }
      if (rdd_d_.norm() > acc_max_) {rdd_d_ *= acc_max_ / rdd_d_.norm();}
      rd_d_ += rdd_d_ * dt;
      if (rd_d_.norm() > v_max_) {rd_d_ *= v_max_ / rd_d_.norm();}
      r_d_ += rd_d_ * dt;
      clampWorkspace();
    }

    trajectory_msgs::msg::JointTrajectoryPoint ref;
    ref.positions = {r_d_(0), r_d_(1)};
    ref.velocities = {rd_d_(0), rd_d_(1)};
    ref.accelerations = {rdd_d_(0), rdd_d_(1)};
    ref.time_from_start = rclcpp::Duration::from_seconds(0.0);
    ref_pub_->publish(ref);

    std_msgs::msg::Float64MultiArray s;
    s.data = {static_cast<double>(state_), a_x_raw_, a_x_filt_, twa,
      r_d_(0), r_d_(1), rd_d_(0), rd_d_(1), rdd_d_(0), rdd_d_(1)};
    state_pub_->publish(s);
  }

  void clampWorkspace()
  {
    if (r_d_(0) < x_min_) {
      r_d_(0) = x_min_;
      rd_d_(0) = std::max(rd_d_(0), 0.0);
    }
    if (r_d_(0) > x_max_) {
      r_d_(0) = x_max_;
      rd_d_(0) = std::min(rd_d_(0), 0.0);
    }
    const double n = r_d_.norm();
    if (n > reach_max_) {
      const Eigen::Vector2d u = r_d_ / n;
      r_d_ = u * reach_max_;
      const double radial = rd_d_.dot(u);
      if (radial > 0.0) {rd_d_ -= radial * u;}
    }
  }

  std::string mode_;
  double wn_turn_{1.5};
  double a_th_, k_x_, exit_ratio_, hold_time_, twa_window_, yaw_eps_, wn_, zeta_, accel_cutoff_,
    rate_;
  double x_min_, x_max_, reach_max_, v_max_, acc_max_, cmd_timeout_;
  Eigen::Vector2d home_, right_, left_;
  Eigen::Vector3d g_;
  Eigen::Quaterniond imu_ref_{Eigen::Quaterniond::Identity()};

  Eigen::Vector2d r_d_, rd_d_, rdd_d_;
  State state_{HOME};
  double a_x_filt_{0.0}, a_x_raw_{0.0}, w_ref_{0.0};
  bool have_imu_{false};
  rclcpp::Time last_imu_{0, 0, RCL_ROS_TIME}, last_cmd_{0, 0, RCL_ROS_TIME},
  last_step_{0, 0, RCL_ROS_TIME}, hold_until_{0, 0, RCL_ROS_TIME};
  std::deque<std::pair<double, double>> yaw_hist_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr ref_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace wheel_legged_arm_balance

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wheel_legged_arm_balance::ArmFsmNode>());
  rclcpp::shutdown();
  return 0;
}
