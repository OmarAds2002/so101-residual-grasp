#include <gtest/gtest.h>

#include <cmath>
#include <kdl/frames.hpp>
#include <stdexcept>

#include "p4_control/trajectory.hpp"

TEST(Trajectory, ReturnsExactEndpoints)
{
  const KDL::Frame start(KDL::Rotation::RPY(0.1, -0.2, 0.3), KDL::Vector(0.1, 0.2, 0.3));

  const KDL::Frame goal(KDL::Rotation::RPY(-0.4, 0.5, -0.6), KDL::Vector(0.7, -0.2, 0.5));

  const double duration = 2.0;
  const p4_control::Trajectory trajectory(start, goal, duration);

  const auto initial = trajectory.At(0.0);
  EXPECT_FALSE(initial.finished);
  EXPECT_DOUBLE_EQ(initial.pose.p.x(), start.p.x());
  EXPECT_DOUBLE_EQ(initial.pose.p.y(), start.p.y());
  EXPECT_DOUBLE_EQ(initial.pose.p.z(), start.p.z());
  for (int i = 0; i < 9; ++i) {
    EXPECT_DOUBLE_EQ(initial.pose.M.data[i], start.M.data[i]);
  }

  const auto end = trajectory.At(duration);
  EXPECT_TRUE(end.finished);
  EXPECT_DOUBLE_EQ(end.pose.p.x(), goal.p.x());
  EXPECT_DOUBLE_EQ(end.pose.p.y(), goal.p.y());
  EXPECT_DOUBLE_EQ(end.pose.p.z(), goal.p.z());
  for (int i = 0; i < 9; ++i) {
    EXPECT_NEAR(end.pose.M.data[i], goal.M.data[i], 1e-12);
  }

  // Past the end, the plan holds at the goal rather than extrapolating.
  const auto after = trajectory.At(duration + 1.0);
  EXPECT_TRUE(after.finished);
  EXPECT_DOUBLE_EQ(after.pose.p.x(), goal.p.x());
  EXPECT_DOUBLE_EQ(after.pose.p.y(), goal.p.y());
  EXPECT_DOUBLE_EQ(after.pose.p.z(), goal.p.z());
}

TEST(Trajectory, PositionFollowsStraightLine)
{
  const KDL::Frame start(KDL::Rotation::Identity(), KDL::Vector(0.0, 0.0, 0.0));

  const KDL::Frame goal(KDL::Rotation::Identity(), KDL::Vector(2.0, 4.0, 6.0));

  const p4_control::Trajectory trajectory(start, goal, 2.0);

  // Smoothstep at half the duration gives f = 0.5.
  const auto midpoint = trajectory.At(1.0);

  EXPECT_NEAR(midpoint.pose.p.x(), 1.0, 1e-12);
  EXPECT_NEAR(midpoint.pose.p.y(), 2.0, 1e-12);
  EXPECT_NEAR(midpoint.pose.p.z(), 3.0, 1e-12);
  EXPECT_FALSE(midpoint.finished);
}

TEST(Trajectory, OrientationUsesSphericalInterpolation)
{
  const KDL::Frame start(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));

  // Two axes, deliberately. With rotation about a single axis, interpolating
  // roll, pitch and yaw separately gives the same answer as rotating about
  // that axis, and this test could not tell them apart.
  const KDL::Frame goal(KDL::Rotation::RPY(0.9, 0.8, 0.7), KDL::Vector(1.0, 0.0, 0.0));

  const p4_control::Trajectory trajectory(start, goal, 2.0);
  const auto midpoint = trajectory.At(1.0);

  // The claim: start to midpoint is half the rotation of start to goal,
  // about the same axis. Separate-rpy interpolation breaks the axis part.
  KDL::Vector axis_full, axis_half;
  const double angle_full = (start.M.Inverse() * goal.M).GetRotAngle(axis_full);
  const double angle_half = (start.M.Inverse() * midpoint.pose.M).GetRotAngle(axis_half);

  EXPECT_NEAR(angle_half, angle_full / 2.0, 1e-9);
  EXPECT_NEAR(KDL::dot(axis_half, axis_full), 1.0, 1e-9);

  EXPECT_NEAR(midpoint.pose.p.x(), 0.5, 1e-12);
  EXPECT_NEAR(midpoint.pose.p.y(), 0.0, 1e-12);
  EXPECT_NEAR(midpoint.pose.p.z(), 0.0, 1e-12);
}

TEST(Trajectory, ThrowsOnZeroDuration)
{
  const KDL::Frame start(KDL::Rotation::Identity(), KDL::Vector(0.0, 0.0, 0.0));

  const KDL::Frame goal(KDL::Rotation::Identity(), KDL::Vector(1.0, 1.0, 1.0));

  EXPECT_THROW(p4_control::Trajectory trajectory(start, goal, 0.0), std::invalid_argument);
}
