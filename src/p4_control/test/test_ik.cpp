#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include "p4_control/ik.hpp"

namespace
{

// Runs FK on a set of angles and hands back the pose. FK was verified
// against MuJoCo on three poses, so it is an independent check of IK.
KDL::Frame ForwardOf(const std::array<double, p4_control::kNumIkJoints> & q)
{
  p4_control::IKSolver solver(URDF_PATH);
  KDL::JntArray jq(p4_control::kNumIkJoints);
  for (std::size_t i = 0; i < p4_control::kNumIkJoints; ++i) {
    jq(i) = q[i];
  }
  return solver.Forward(jq);
}

}  // namespace

TEST(IKSolver, ConstructsFromRealUrdf)
{
  EXPECT_NO_THROW({ p4_control::IKSolver solver(URDF_PATH); });
}

TEST(IKSolver, ThrowsOnMissingUrdf)
{
  EXPECT_THROW(
    p4_control::IKSolver solver("/nonexistent/path/to/nothing.urdf"), std::runtime_error);
}

TEST(IKSolver, SolvedAnglesReachTheTargetPosition)
{
  p4_control::IKSolver solver(URDF_PATH);

  // A pose the arm demonstrably reaches: taken from the FK cross-check,
  // joints at 0.3 -0.5 0.8 0.2 1.1 put the gripper here.
  const KDL::Vector target_p(0.215553, -0.054850, 0.164623);
  const KDL::Rotation target_m = KDL::Rotation::RPY(-1.00910, -0.45077, 1.00322);
  const KDL::Frame target(target_m, target_p);

  KDL::JntArray seed(p4_control::kNumIkJoints);  // zeros
  std::array<double, p4_control::kNumIkJoints> q_out{};

  ASSERT_TRUE(solver.Solve(seed, target, q_out));

  const KDL::Frame reached = ForwardOf(q_out);

  EXPECT_NEAR(reached.p.x(), target.p.x(), 1e-4);
  EXPECT_NEAR(reached.p.y(), target.p.y(), 1e-4);
  EXPECT_NEAR(reached.p.z(), target.p.z(), 1e-4);
}

TEST(IKSolver, YawIsNotConstrained)
{
  p4_control::IKSolver solver(URDF_PATH);

  const KDL::Vector p(0.215553, -0.054850, 0.164623);
  double r, pitch, y;
  KDL::Rotation::RPY(-1.00910, -0.45077, 1.00322).GetRPY(r, pitch, y);

  // Same position and same roll/pitch, but yaw rotated well away. If yaw is
  // genuinely unweighted, the solver still reaches position and roll/pitch.
  const KDL::Frame target(KDL::Rotation::RPY(r, pitch, y + 1.0), p);

  KDL::JntArray seed(p4_control::kNumIkJoints);
  std::array<double, p4_control::kNumIkJoints> q_out{};

  ASSERT_TRUE(solver.Solve(seed, target, q_out));

  const KDL::Frame reached = ForwardOf(q_out);

  EXPECT_NEAR(reached.p.x(), p.x(), 1e-4);
  EXPECT_NEAR(reached.p.y(), p.y(), 1e-4);
  EXPECT_NEAR(reached.p.z(), p.z(), 1e-4);

  double rr, rp, ry;
  reached.M.GetRPY(rr, rp, ry);
  EXPECT_NEAR(rr, r, 1e-3);
  EXPECT_NEAR(rp, pitch, 1e-3);

  // And yaw lands where the mechanism puts it, not where we asked.
  EXPECT_GT(std::abs(ry - (y + 1.0)), 0.1);
}

TEST(IKSolver, FailsOnUnreachableTarget)
{
  p4_control::IKSolver solver(URDF_PATH);

  // Two metres out. The arm is roughly 0.3 m long.
  const KDL::Frame target(KDL::Rotation::Identity(), KDL::Vector(2.0, 0.0, 0.0));

  KDL::JntArray seed(p4_control::kNumIkJoints);
  std::array<double, p4_control::kNumIkJoints> q_out{};

  EXPECT_FALSE(solver.Solve(seed, target, q_out));
}
