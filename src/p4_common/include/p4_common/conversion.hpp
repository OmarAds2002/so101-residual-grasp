#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>

#include "p4_common/calibration.hpp"
#include "p4_common/joint_order.hpp"

namespace p4_common
{

inline constexpr int kTicksPerRevolution = 4096;
inline constexpr double kRadPerTick = 2.0 * std::numbers::pi / kTicksPerRevolution;

// One joint. Returns fractional ticks on purpose — rounding happens
// where bytes are packed, not here.
inline double RadToTicks(double rad, const JointCalibration & cal) noexcept
{
  return cal.sign * (rad / kRadPerTick) + cal.offset;
}

inline double TicksToRad(double ticks, const JointCalibration & cal) noexcept
{
  return cal.sign * (ticks - cal.offset) * kRadPerTick;
}

// All six. What read() and write() call.
inline std::array<double, kNumJoints> RadToTicksAll(
  const std::array<double, kNumJoints> & rad,
  const std::array<JointCalibration, kNumJoints> & cal) noexcept
{
  std::array<double, kNumJoints> ticks{};
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    ticks[i] = RadToTicks(rad[i], cal[i]);
  }
  return ticks;
}

inline std::array<double, kNumJoints> TicksToRadAll(
  const std::array<double, kNumJoints> & ticks,
  const std::array<JointCalibration, kNumJoints> & cal) noexcept
{
  std::array<double, kNumJoints> rad{};
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    rad[i] = TicksToRad(ticks[i], cal[i]);
  }
  return rad;
}

}  // namespace p4_common
