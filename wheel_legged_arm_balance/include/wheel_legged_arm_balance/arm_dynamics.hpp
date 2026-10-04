// Planar two-link arm model of Sec. II-B (Eqs. 14-17) of Raza et al. 2021.
// The arm moves in the horizontal plane of frame Sigma_A (X forward, Y left),
// so gravity does not act on the joint torques.
#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace wheel_legged_arm_balance
{

struct ArmParameters
{
  double l1{0.3};
  double l2{0.3};
  double lg1{0.3};
  double lg2{0.3};
  double m1{0.6};
  double m2{1.13};
  double I1{0.018};
  double I2{0.0339};

  /// Lump a point-like payload held at the tip of link 2 into link 2.
  ArmParameters withPayload(double mass, double inertia) const
  {
    ArmParameters a = *this;
    if (mass <= 0.0) {
      return a;
    }
    const double m = m2 + mass;
    const double lg = (m2 * lg2 + mass * l2) / m;
    a.I2 = I2 + m2 * (lg2 - lg) * (lg2 - lg) + mass * (l2 - lg) * (l2 - lg) + inertia;
    a.m2 = m;
    a.lg2 = lg;
    return a;
  }
};

class TwoLinkArm
{
public:
  explicit TwoLinkArm(const ArmParameters & p = ArmParameters()) : p_(p) {}

  void setParameters(const ArmParameters & p) {p_ = p;}
  const ArmParameters & parameters() const {return p_;}

  Eigen::Vector2d forwardKinematics(const Eigen::Vector2d & q) const
  {
    return {p_.l1 * std::cos(q(0)) + p_.l2 * std::cos(q(0) + q(1)),
      p_.l1 * std::sin(q(0)) + p_.l2 * std::sin(q(0) + q(1))};
  }

  Eigen::Matrix2d jacobian(const Eigen::Vector2d & q) const
  {
    const double s1 = std::sin(q(0)), c1 = std::cos(q(0));
    const double s12 = std::sin(q(0) + q(1)), c12 = std::cos(q(0) + q(1));
    Eigen::Matrix2d J;
    J << -p_.l1 * s1 - p_.l2 * s12, -p_.l2 * s12,
      p_.l1 * c1 + p_.l2 * c12, p_.l2 * c12;
    return J;
  }

  /// J_dot * q_dot
  Eigen::Vector2d jacobianDotTimesQdot(const Eigen::Vector2d & q, const Eigen::Vector2d & qd) const
  {
    const double w1 = qd(0), w12 = qd(0) + qd(1);
    return {-p_.l1 * std::cos(q(0)) * w1 * w1 - p_.l2 * std::cos(q(0) + q(1)) * w12 * w12,
      -p_.l1 * std::sin(q(0)) * w1 * w1 - p_.l2 * std::sin(q(0) + q(1)) * w12 * w12};
  }

  /// Inertia matrix M(theta)
  Eigen::Matrix2d massMatrix(const Eigen::Vector2d & q) const
  {
    const double c2 = std::cos(q(1));
    const double M11 = p_.I1 + p_.I2 + p_.m1 * p_.lg1 * p_.lg1 +
      p_.m2 * (p_.l1 * p_.l1 + p_.lg2 * p_.lg2 + 2.0 * p_.l1 * p_.lg2 * c2);
    const double M12 = p_.I2 + p_.m2 * (p_.lg2 * p_.lg2 + p_.l1 * p_.lg2 * c2);
    const double M22 = p_.I2 + p_.m2 * p_.lg2 * p_.lg2;
    Eigen::Matrix2d M;
    M << M11, M12, M12, M22;
    return M;
  }

  /// Coriolis and centrifugal vector h(theta, theta_dot)
  Eigen::Vector2d coriolis(const Eigen::Vector2d & q, const Eigen::Vector2d & qd) const
  {
    const double k = p_.m2 * p_.l1 * p_.lg2 * std::sin(q(1));
    return {-k * qd(1) * (2.0 * qd(0) + qd(1)), k * qd(0) * qd(0)};
  }

  /// Analytic inverse kinematics; elbow_sign selects the branch (sign of theta_2).
  bool inverseKinematics(const Eigen::Vector2d & r, double elbow_sign, Eigen::Vector2d & q) const
  {
    const double d2 = r.squaredNorm();
    double c2 = (d2 - p_.l1 * p_.l1 - p_.l2 * p_.l2) / (2.0 * p_.l1 * p_.l2);
    if (c2 > 1.0 || c2 < -1.0) {
      return false;
    }
    const double q2 = (elbow_sign >= 0.0 ? 1.0 : -1.0) * std::acos(c2);
    const double q1 = std::atan2(r(1), r(0)) -
      std::atan2(p_.l2 * std::sin(q2), p_.l1 + p_.l2 * std::cos(q2));
    q << q1, q2;
    return true;
  }

private:
  ArmParameters p_;
};

}  // namespace wheel_legged_arm_balance
