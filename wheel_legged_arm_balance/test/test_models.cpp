#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>

#include "wheel_legged_arm_balance/arm_dynamics.hpp"
#include "wheel_legged_arm_balance/wip_model.hpp"

using namespace wheel_legged_arm_balance;

TEST(WipModel, LqrMatchesReferenceSolution)
{
  Matrix6d A;
  Matrix62d B;
  linearizeWip(WipParameters(), A, B);
  const Matrix6d Q = Matrix6d::Identity();
  const Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
  const Matrix26d K = solveLqr<6, 2>(A, B, Q, R);
  // reference values from scipy.linalg.solve_continuous_are on the same model
  Matrix26d K_ref;
  K_ref << -0.7071, 0.7071, -17.8044, -5.4623, 0.3753, -3.6422,
    -0.7071, -0.7071, -17.8044, -5.4623, -0.3753, -3.6422;
  EXPECT_LT((K - K_ref).cwiseAbs().maxCoeff(), 2e-3);
}

TEST(WipModel, PaperGainStabilisesModel)
{
  Matrix6d A;
  Matrix62d B;
  linearizeWip(WipParameters(), A, B);
  const Matrix6d Acl = A - B * paperLqrGain();
  Eigen::EigenSolver<Matrix6d> es(Acl);
  for (int i = 0; i < 6; ++i) {
    EXPECT_LT(es.eigenvalues()(i).real(), 0.0);
  }
}

TEST(TwoLinkArm, JacobianMatchesFiniteDifference)
{
  TwoLinkArm arm;
  const Eigen::Vector2d q(0.7, -1.9), qd(1.3, -0.4);
  const double h = 1e-7;
  Eigen::Matrix2d Jn;
  for (int i = 0; i < 2; ++i) {
    Eigen::Vector2d dq = Eigen::Vector2d::Zero();
    dq(i) = h;
    Jn.col(i) = (arm.forwardKinematics(q + dq) - arm.forwardKinematics(q - dq)) / (2 * h);
  }
  EXPECT_LT((Jn - arm.jacobian(q)).cwiseAbs().maxCoeff(), 1e-6);
  // J_dot q_dot via d/dt (J q_dot) along q(t) = q + t q_dot
  const Eigen::Vector2d jdn =
    (arm.jacobian(q + h * qd) * qd - arm.jacobian(q - h * qd) * qd) / (2 * h);
  EXPECT_LT((jdn - arm.jacobianDotTimesQdot(q, qd)).cwiseAbs().maxCoeff(), 1e-5);
}

TEST(TwoLinkArm, HomePostureAndInverseKinematics)
{
  TwoLinkArm arm;
  const Eigen::Vector2d home(M_PI / 3, -2 * M_PI / 3);
  const Eigen::Vector2d r = arm.forwardKinematics(home);
  EXPECT_NEAR(r(0), 0.3, 1e-9);
  EXPECT_NEAR(r(1), 0.0, 1e-9);
  Eigen::Vector2d q;
  ASSERT_TRUE(arm.inverseKinematics(Eigen::Vector2d(0.15, -0.4), -1.0, q));
  EXPECT_LT((arm.forwardKinematics(q) - Eigen::Vector2d(0.15, -0.4)).norm(), 1e-9);
}

TEST(TwoLinkArm, MassMatrixIsSymmetricPositiveDefinite)
{
  TwoLinkArm arm(ArmParameters().withPayload(0.72, 0.001));
  for (double q2 = -3.0; q2 <= 3.0; q2 += 0.5) {
    const Eigen::Matrix2d M = arm.massMatrix(Eigen::Vector2d(0.0, q2));
    EXPECT_DOUBLE_EQ(M(0, 1), M(1, 0));
    EXPECT_GT(M.determinant(), 0.0);
    EXPECT_GT(M(0, 0), 0.0);
  }
}

TEST(TwoLinkArm, PayloadLumping)
{
  const ArmParameters p = ArmParameters().withPayload(0.36, 0.0);
  EXPECT_NEAR(p.m2, 1.49, 1e-12);
  EXPECT_NEAR(p.lg2, 0.3, 1e-12);   // payload at the tip, CoM of link 2 also at the tip
}
