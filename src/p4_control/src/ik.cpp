#include "p4_control/ik.hpp"

#include <Eigen/Dense>
#include <kdl_parser/kdl_parser.hpp>
#include <stdexcept>
#include <string>

namespace p4_control
{

namespace
{

// Five joints, six task-space freedoms. Yaw (rotation about base-frame z)
// is weighted to zero -- see the comment on IKSolver in ik.hpp.
Eigen::Matrix<double, 6, 1> MakeWeights()
{
  Eigen::Matrix<double, 6, 1> L;
  L << 1, 1, 1,  // x, y, z
    1, 1, 0;     // roll, pitch, yaw
  return L;
}

}  // namespace

KDL::Chain IKSolver::BuildChain(const std::string & urdf_path)
{
  KDL::Tree tree;
  if (!kdl_parser::treeFromFile(urdf_path, tree)) {
    throw std::runtime_error("IKSolver: failed to parse URDF at " + urdf_path);
  }

  KDL::Chain chain;
  if (!tree.getChain("base_link", "gripper_link", chain)) {
    throw std::runtime_error("IKSolver: no chain from base_link to gripper_link in " + urdf_path);
  }

  if (chain.getNrOfJoints() != kNumIkJoints) {
    throw std::runtime_error(
      "IKSolver: chain has " + std::to_string(chain.getNrOfJoints()) + " joints, expected " +
      std::to_string(kNumIkJoints));
  }

  return chain;
}

IKSolver::IKSolver(const std::string & urdf_path)
: chain_(BuildChain(urdf_path)),
  fk_solver_(chain_),
  ik_solver_(chain_, MakeWeights()),
  q_scratch_(kNumIkJoints)
{
}

KDL::Frame IKSolver::Forward(const KDL::JntArray & q)
{
  KDL::Frame pose;
  fk_solver_.JntToCart(q, pose);
  return pose;
}

bool IKSolver::Solve(
  const KDL::JntArray & q_seed, const KDL::Frame & desired_pose,
  std::array<double, kNumIkJoints> & joint_angles_rad_out)
{
  const int rc = ik_solver_.CartToJnt(q_seed, desired_pose, q_scratch_);
  if (rc != 0) {
    return false;  // did not converge; q_scratch_ is not meaningful
  }

  for (std::size_t i = 0; i < kNumIkJoints; ++i) {
    joint_angles_rad_out[i] = q_scratch_(i);
  }
  return true;
}

}  // namespace p4_control
