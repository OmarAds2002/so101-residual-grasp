#pragma once

#include <array>
#include <cstddef>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <string>

namespace p4_control
{

// The SO-101 has six servos, but only five position the gripper; the sixth
// opens and closes the jaw. The IK chain is base_link -> gripper_link and
// holds those five. The caller supplies the jaw command and assembles all
// six in p4_common's joint_order before write().
static constexpr std::size_t kNumIkJoints = 5;

// Five joints cannot reach an arbitrary six-number pose, so one task-space
// freedom is given up: rotation about base-frame z, which for a top-down
// grasp is spin about the gripper's approach axis. A parallel jaw closing
// on a cube does not care about it. This holds only while the approach is
// straight down.
class IKSolver
{
public:
  explicit IKSolver(const std::string & urdf_path);

  // Returns true if the solver converged. On false, joint_angles_rad_out
  // is not meaningful and the caller must hold its previous command.
  bool Solve(
    const KDL::JntArray & q_seed, const KDL::Frame & desired_pose,
    std::array<double, kNumIkJoints> & joint_angles_rad_out);
  KDL::Frame Forward(const KDL::JntArray & q);

private:
  static KDL::Chain BuildChain(const std::string & urdf_path);

  KDL::Chain chain_;
  KDL::ChainFkSolverPos_recursive fk_solver_;
  KDL::ChainIkSolverPos_LMA ik_solver_;

  // Scratch space for the solver's answer. A member, not a local, so that
  // Solve() allocates nothing. Makes Solve() non-const and single-threaded.
  KDL::JntArray q_scratch_;
};

}  // namespace p4_control
