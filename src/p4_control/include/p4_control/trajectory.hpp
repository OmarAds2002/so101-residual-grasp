#pragma once

#include <kdl/frames.hpp>

namespace p4_control
{

struct Setpoint
{
  KDL::Frame pose;
  bool finished;  // the plan's clock has run out; hand off to the caller
};

// A straight line in gripper space from start to goal, with speed ramped in
// and out so the first setpoint does not demand instant full speed. Fixed at
// construction: the plan does not change once the motion begins.
class Trajectory
{
public:
  Trajectory(const KDL::Frame & start, const KDL::Frame & goal, double duration_s);

  // Where the gripper should be at this many seconds into the motion.
  // Clamped at both ends; never extrapolates past the plan.
  Setpoint At(double elapsed_s) const;

private:
  KDL::Frame start_;
  KDL::Frame goal_;
  double duration_s_;
  double fraction_;  // [0, 1] of the plan completed, ramped in and out
  double f_;         // fraction_ remapped to a cubic for smooth start and stop
};

}  // namespace p4_control
