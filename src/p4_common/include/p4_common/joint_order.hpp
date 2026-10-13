#pragma once
#include <array>
#include <cstddef>
#include <string_view>
namespace p4_common
{
enum class Joints
{
  shoulder_pan,
  shoulder_lift,
  elbow_flex,
  wrist_flex,
  wrist_roll,
  gripper,
  NUM_JOINTS
};
constexpr std::size_t kNumJoints = static_cast<std::size_t>(Joints::NUM_JOINTS);
constexpr auto joint_order = std::to_array<std::string_view>(
  {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"});

static_assert(joint_order.size() == kNumJoints, "error message");
}  // namespace p4_common
