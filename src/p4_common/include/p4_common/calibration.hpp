#pragma once

#include <array>
#include <string>

#include "p4_common/joint_order.hpp"

namespace p4_common
{

struct JointCalibration
{
  int offset;    // tick reading when the joint is at zero radians
  int servo_id;  // address on the bus
  int sign;      // +1 or -1: does the servo count up in the URDF's positive direction
};

// Reads the calibration file. Throws std::runtime_error naming the joint
// if anything is missing or out of range.
std::array<JointCalibration, kNumJoints> LoadCalibration(const std::string & path);

}  // namespace p4_common
