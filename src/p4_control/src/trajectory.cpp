#include "p4_control/trajectory.hpp"

#include <algorithm>
#include <kdl/frames.hpp>
#include <stdexcept>

namespace p4_control
{

Trajectory::Trajectory(const KDL::Frame & start, const KDL::Frame & goal, double duration_s)
: start_(start), goal_(goal), duration_s_(duration_s)
{
  // A zero or negative duration would divide by zero and hand NaN to a servo
  // command. Caller bug; fail loudly here rather than quietly at cycle 1.
  if (duration_s_ <= 0.0) {
    throw std::invalid_argument("Trajectory: duration must be positive");
  }
}

Setpoint Trajectory::At(double elapsed_s) const
{
  // Judged on the real time, before clamping.
  const bool finished = (elapsed_s >= duration_s_);

  // Never extrapolate past either end of the plan.
  const double t = std::clamp(elapsed_s, 0.0, duration_s_);

  const double frac = t / duration_s_;

  // Smoothstep: zero slope at both ends, so the motion ramps in and out
  // instead of demanding full speed from the first setpoint.
  const double f = frac;

  // Position: straight line in gripper space.
  const KDL::Vector pos = start_.p + f * (goal_.p - start_.p);

  // Orientation: rotate about the single axis that takes start to goal.
  // Interpolating roll, pitch and yaw separately gives the right endpoints
  // and a wandering middle.
  KDL::Vector axis;
  const double total = (start_.M.Inverse() * goal_.M).GetRotAngle(axis);
  const KDL::Rotation rot = start_.M * KDL::Rotation::Rot(axis, f * total);

  Setpoint setpoint;
  setpoint.pose = KDL::Frame(rot, pos);
  setpoint.finished = finished;
  return setpoint;
}

}  // namespace p4_control
