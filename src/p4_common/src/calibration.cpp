#include "p4_common/calibration.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_map>
namespace p4_common
{

namespace
{

constexpr int kMinTick = 0;
constexpr int kMaxTick = 4095;
constexpr int kMinServoId = 1;
constexpr int kMaxServoId = 253;

int RequireInt(const YAML::Node & node, const char * field, const std::string & joint)
{
  if (!node[field]) {
    throw std::runtime_error(
      "calibration for joint '" + joint + "' is missing field: " + field);
  }
  try {
    return node[field].as<int>();
  } catch (const std::exception &) {
    throw std::runtime_error(
      "calibration for joint '" + joint + "', field '" + field + "' is not a whole number");
  }
}

}  // namespace
std::array<JointCalibration, kNumJoints> LoadCalibration(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    throw std::runtime_error("could not read calibration file '" + path + "': " + e.what());
  }

  if (!root["servos"] || !root["servos"].IsMap()) {
    throw std::runtime_error(
            "calibration file '" + path + "' has no 'servos' map at the top level");
  }
  const YAML::Node servos = root["servos"];

  std::array<JointCalibration, kNumJoints> result{};
  std::unordered_map<int, std::size_t> seen_ids;   // servo_id -> joint index

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    const std::string joint(joint_order[i]);

    if (!servos[joint]) {
      throw std::runtime_error("calibration file is missing joint: " + joint);
    }
    const YAML::Node entry = servos[joint];
    if (!entry.IsMap()) {
      throw std::runtime_error("calibration entry for joint '" + joint + "' is not a map");
    }

    JointCalibration cal{};
    cal.offset = RequireInt(entry, "offset", joint);
    cal.servo_id = RequireInt(entry, "servo_id", joint);
    cal.sign = RequireInt(entry, "sign", joint);

    if (cal.offset < kMinTick || cal.offset > kMaxTick) {
      throw std::runtime_error(
              "joint '" + joint + "' has offset " + std::to_string(cal.offset) +
              ", outside the tick range 0-4095");
    }
    if (cal.servo_id < kMinServoId || cal.servo_id > kMaxServoId) {
      throw std::runtime_error(
              "joint '" + joint + "' has servo_id " + std::to_string(cal.servo_id) +
              ", outside the legal range 1-253");
    }
    if (cal.sign != 1 && cal.sign != -1) {
      throw std::runtime_error(
              "joint '" + joint + "' has sign " + std::to_string(cal.sign) + ", must be 1 or -1");
    }

    // emplace reports whether the id was new. If not, it->second is the
    // joint that claimed it first.
    const auto [it, inserted] = seen_ids.emplace(cal.servo_id, i);
    if (!inserted) {
      throw std::runtime_error(
              "calibration file has duplicate servo_id " + std::to_string(cal.servo_id) +
              " for joints '" + std::string(joint_order[it->second]) +
              "' and '" + joint + "'");
    }

    result[i] = cal;
  }

  if (servos.size() != kNumJoints) {
    throw std::runtime_error(
            "calibration file '" + path + "' has " + std::to_string(servos.size()) +
            " entries, expected " + std::to_string(kNumJoints) +
            " - check for a misspelled or extra joint name");
  }

  return result;
}

}  // namespace p4_common