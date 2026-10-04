// Linearised lower-body model of the wheel-legged robot (Sec. II-A of
// Raza et al., IEEE Access 2021) and an LQR gain solver (equivalent of the
// MATLAB command  K = lqr(A, B, Q, R)  used by the authors).
//
// State     X = [p, alpha, beta, v, alpha_dot, beta_dot]^T
// Input     U = [tau_r, tau_l]^T      (wheel joint torques)
// Control   U = -K (X - X_ref)        (Eq. 12)
#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

namespace wheel_legged_arm_balance
{

struct WipParameters
{
  double m_c{7.5};       // body mass
  double m_w{0.35};      // wheel mass
  double l{0.5914};      // CoM height above the wheel axis
  double I_xx{0.0878};
  double I_yy{0.0347};
  double I_zz{0.0632};
  double I_wa{0.0018};   // wheel inertia about its axis
  double I_wd{0.000929}; // wheel inertia about its diameter
  double I_ma{1.04e-7};  // motor rotor inertia about its axis
  double I_md{0.0};      // motor rotor inertia about its diameter
  double gamma{1.0};     // gear ratio
  double g{9.81};
  double r_w{0.1016};    // wheel radius
  double b{0.215};       // half track width
  double c{0.25};        // viscous friction c_r = c_l
};

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix62d = Eigen::Matrix<double, 6, 2>;
using Matrix26d = Eigen::Matrix<double, 2, 6>;

/// Linearisation of the reduced-order equations of motion (Eq. 10) about beta = 0.
inline void linearizeWip(const WipParameters & p, Matrix6d & A, Matrix62d & B)
{
  // Sagittal plane: M [v_dot, beta_ddot]^T = G beta + D [v, beta_dot]^T + E (tau_r + tau_l)
  const double wheel_axis_inertia = p.I_wa + p.I_ma * p.gamma * p.gamma;
  const double Mx = p.m_c + 2.0 * p.m_w + 2.0 * wheel_axis_inertia / (p.r_w * p.r_w);
  const double Mxb = p.m_c * p.l;
  const double Mb = p.I_yy + p.m_c * p.l * p.l;
  Eigen::Matrix2d M;
  M << Mx, Mxb, Mxb, Mb;
  const Eigen::Matrix2d Mi = M.inverse();

  Eigen::Vector2d G(0.0, p.m_c * p.g * p.l);
  Eigen::Matrix2d D;
  // Rayleigh dissipation (Eq. 4) acts on the wheel speed relative to the body
  D << -2.0 * p.c / (p.r_w * p.r_w), 2.0 * p.c / p.r_w,
    2.0 * p.c / p.r_w, -2.0 * p.c;
  Eigen::Vector2d E(1.0 / p.r_w, -1.0);

  const Eigen::Vector2d sg = Mi * G;
  const Eigen::Matrix2d sd = Mi * D;
  const Eigen::Vector2d se = Mi * E;

  // Yaw: J_alpha alpha_ddot = b/r_w (tau_r - tau_l) - 2 c b^2 / r_w^2 alpha_dot
  const double J_alpha = p.I_zz + 2.0 * p.m_w * p.b * p.b +
    2.0 * wheel_axis_inertia * p.b * p.b / (p.r_w * p.r_w) + 2.0 * (p.I_wd + p.I_md);

  A.setZero();
  B.setZero();
  A(0, 3) = 1.0;
  A(1, 4) = 1.0;
  A(2, 5) = 1.0;
  A(3, 2) = sg(0);
  A(5, 2) = sg(1);
  A(3, 3) = sd(0, 0);
  A(3, 5) = sd(0, 1);
  A(5, 3) = sd(1, 0);
  A(5, 5) = sd(1, 1);
  A(4, 4) = -2.0 * p.c * p.b * p.b / (p.r_w * p.r_w) / J_alpha;
  B(3, 0) = B(3, 1) = se(0);
  B(5, 0) = B(5, 1) = se(1);
  B(4, 0) = p.b / p.r_w / J_alpha;
  B(4, 1) = -p.b / p.r_w / J_alpha;
}

/// Solve the continuous algebraic Riccati equation (Eq. 13) with the matrix sign
/// function method and return K = R^-1 B^T P.
template<int N, int M>
Eigen::Matrix<double, M, N> solveLqr(
  const Eigen::Matrix<double, N, N> & A, const Eigen::Matrix<double, N, M> & B,
  const Eigen::Matrix<double, N, N> & Q, const Eigen::Matrix<double, M, M> & R)
{
  using MatN = Eigen::Matrix<double, N, N>;
  using Mat2N = Eigen::Matrix<double, 2 * N, 2 * N>;
  const MatN G = B * R.ldlt().solve(B.transpose());
  Mat2N Z;
  Z << A, -G, -Q, -A.transpose();
  for (int it = 0; it < 200; ++it) {
    const double det = std::abs(Z.determinant());
    if (!(det > 0.0) || !std::isfinite(det)) {
      throw std::runtime_error("LQR: Hamiltonian became singular (system not stabilisable?)");
    }
    const double c = std::pow(det, 1.0 / (2.0 * N));
    const Mat2N Zn = 0.5 * (Z / c + c * Z.inverse());
    const double diff = (Zn - Z).cwiseAbs().colwise().sum().maxCoeff();
    const double nrm = Z.cwiseAbs().colwise().sum().maxCoeff();
    Z = Zn;
    if (diff < 1e-12 * nrm) {
      break;
    }
  }
  Eigen::Matrix<double, 2 * N, N> lhs;
  Eigen::Matrix<double, 2 * N, N> rhs;
  lhs << Z.topRightCorner(N, N), Z.bottomRightCorner(N, N) + MatN::Identity();
  rhs << -(Z.topLeftCorner(N, N) + MatN::Identity()), -Z.bottomLeftCorner(N, N);
  MatN P = lhs.colPivHouseholderQr().solve(rhs);
  P = 0.5 * (P + P.transpose());
  return R.ldlt().solve(B.transpose() * P);
}

/// Feedback gain reported in the paper (Sec. II-A).
inline Matrix26d paperLqrGain()
{
  Matrix26d K;
  K << -0.7071, 0.7071, -16.2331, -3.1751, 0.2016, -3.5082,
    -0.7071, -0.7071, -16.2331, -3.1751, -0.2016, -3.5082;
  return K;
}

inline double wrapAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

}  // namespace wheel_legged_arm_balance
