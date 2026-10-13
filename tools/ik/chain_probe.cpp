#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <iostream>
#include <kdl/chainfksolverpos_recursive.hpp>

int main(int argc, char** argv) {
    if (argc != 6) {
        std::cerr << "usage: chain_probe q0 q1 q2 q3 q4  (radians)\n";
        return -1;
    }
    KDL::Tree tree;
    if (!kdl_parser::treeFromFile("src/p4_common/urdf/so101_new_calib.urdf", tree)) {
        std::cerr << "Failed to load URDF!" << std::endl;
        return -1;
    }
    KDL::Chain chain;
    tree.getChain("base_link", "gripper_link", chain);  // returns bool
    if (chain.getNrOfJoints() != 5) {
        std::cerr << "Chain does not have 5 joints!" << std::endl;
        return -1;
    }
    std::cout << "Chain has " << chain.getNrOfSegments() << " segments." << std::endl;
    for (int i = 0; i < chain.getNrOfJoints(); ++i) {
        std::cout << "Joint " << i << ": " << chain.getSegment(i).getJoint().getName() << std::endl;
        std::cout << "Type " << i << ": " << chain.getSegment(i).getJoint().getType() << std::endl;
    }
    std::cout << "Chain has 5 joints!" << std::endl;
    KDL::ChainFkSolverPos_recursive fk(chain);
    KDL::Frame pose;
    KDL::JntArray q(chain.getNrOfJoints());
    for (unsigned i = 0; i < 5; ++i) q(i) = std::stod(argv[i + 1]);
    if(fk.JntToCart(q, pose) != 0) {   // q is a JntArray, returns int, 0 = ok
        std::cerr << "Failed to compute forward kinematics!" << std::endl;
        return -1;
    }
    std::cout << "FK at zeros: x=" << pose.p.x()
          << " y=" << pose.p.y()
          << " z=" << pose.p.z() << std::endl;

    double r, p_, y;
    pose.M.GetRPY(r, p_, y);
    std::cout << "FK at zeros: rpy=" << r << " " << p_ << " " << y << std::endl;
    // KDL::ChainIkSolverPos_LMA ik_solver(chain, L, eps, maxiter, eps_joints);

}
