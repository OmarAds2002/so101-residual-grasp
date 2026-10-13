#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

#include "p4_common/calibration.hpp"
#include "p4_common/conversion.hpp"
#include "p4_common/joint_order.hpp"
#include "p4_control/ik.hpp"
#include "p4_control/trajectory.hpp"
#include "p4_hardware/feetech_bus.hpp"

namespace
{

constexpr const char * kUrdfPath = URDF_PATH;
constexpr const char * kCalibrationPath = CALIBRATION_PATH;

constexpr std::size_t kIk = p4_control::kNumIkJoints;     // 5
constexpr std::size_t kServos = p4_hardware::kNumServos;  // 6

constexpr int kMaxConsecutiveFailures = 10;   // 50 ms of not following

// ---- the three knobs. One edit per run. ----
constexpr double kDistanceM = 0.08;
constexpr double kSpeedMps = 0.25;
constexpr double kSettleS = 2.0;   // time after the trajectory ends to keep logging
constexpr const char * kOutPath = "tools/ik/results/ik_8cm_25cms.csv";

struct LogRow
{
  double t;
  double sp_x, sp_y, sp_z;
  std::array<double, kIk> measured_rad;
  std::array<double, kIk> commanded_rad;
  bool converged;
  long cycle_us;
};

std::uint16_t ToTicks(double rad, const p4_common::JointCalibration & cal)
{
  // The conversion returns an unrounded double on purpose. This is where
  // bytes get packed, so this is where rounding belongs.
  const double raw = p4_common::RadToTicks(rad, cal);
  const double clamped = std::clamp(raw, 0.0, 4095.0);
  return static_cast<std::uint16_t>(std::lround(clamped));
}

bool WriteCsv(const std::string & path, const std::vector<LogRow> & log)
{
  std::ofstream out(path);
  if (!out) {
    return false;   // missing directory, bad permissions -- say so, don't lie
  }

  out << "t,sp_x,sp_y,sp_z";
  for (std::size_t i = 0; i < kIk; ++i) { out << ",meas_" << i; }
  for (std::size_t i = 0; i < kIk; ++i) { out << ",cmd_" << i; }
  out << ",converged,cycle_us\n";

  out.precision(9);
  for (const auto & r : log) {
    out << r.t << ',' << r.sp_x << ',' << r.sp_y << ',' << r.sp_z;
    for (double v : r.measured_rad) { out << ',' << v; }
    for (double v : r.commanded_rad) { out << ',' << v; }
    out << ',' << (r.converged ? 1 : 0) << ',' << r.cycle_us << '\n';
  }

  return out.good();
}

}  // namespace

int main()
{
  // ---- setup: slow, allocates, may throw. Once. ----
  p4_control::IKSolver solver(kUrdfPath);
  const auto calibration = p4_common::LoadCalibration(kCalibrationPath);
  p4_hardware::FeetechBus bus("/dev/ttyACM0", 1000000);

  std::array<std::uint16_t, kServos> ticks{};
  if (!bus.ReadPositions(ticks)) {
    std::cerr << "initial read failed; is the arm powered?\n";
    return -1;
  }

  // calibration is indexed by joint_order, and the IK chain is the first
  // five of that same order. The alignment is load-bearing.
  KDL::JntArray q_measured(kIk);
  for (std::size_t i = 0; i < kIk; ++i) {
    q_measured(i) = p4_common::TicksToRad(ticks[i], calibration[i]);
  }

  // The jaw is not part of the chain. Hold it where it already is so
  // nothing surprising happens at the gripper.
  const std::uint16_t jaw_ticks = ticks[5];

  const KDL::Frame start = solver.Forward(q_measured);
  const KDL::Frame goal(start.M, start.p + KDL::Vector(kDistanceM, 0.0, 0.0));

  const double duration_s = kDistanceM / kSpeedMps;
  const p4_control::Trajectory traj(start, goal, duration_s);

  std::cout << "start  x=" << start.p.x() << " y=" << start.p.y()
            << " z=" << start.p.z() << '\n'
            << "goal   x=" << goal.p.x() << " y=" << goal.p.y()
            << " z=" << goal.p.z() << '\n'
            << kDistanceM * 100.0 << " cm at " << kSpeedMps * 100.0
            << " cm/s over " << duration_s << " s\n"
            << "-> " << kOutPath << '\n'
            << "press enter to run, ctrl-c to abort\n";
  std::cin.get();

  std::vector<LogRow> log;
  log.reserve(1000);

  // ---- the loop: fixed rate, no allocation, no file I/O. ----
  using clock = std::chrono::steady_clock;
  const auto period = std::chrono::microseconds(5000);   // 200 Hz
  const auto t0 = clock::now();
  auto next = t0;

  std::array<double, kIk> q_cmd{};
  bool have_command = false;
  int consecutive_failures = 0;

  while (true) {
    const auto cycle_start = clock::now();
    const double elapsed = std::chrono::duration<double>(cycle_start - t0).count();

    const bool read_ok = bus.ReadPositions(ticks);
    if (read_ok) {
      for (std::size_t i = 0; i < kIk; ++i) {
        q_measured(i) = p4_common::TicksToRad(ticks[i], calibration[i]);
      }
    }

    const auto setpoint = traj.At(elapsed);

    std::array<double, kIk> q_solved{};
    const bool converged = solver.Solve(q_measured, setpoint.pose, q_solved);

    if (converged) {
      q_cmd = q_solved;
      have_command = true;
      consecutive_failures = 0;
    } else {
      ++consecutive_failures;          // hold q_cmd unchanged
      if (consecutive_failures > kMaxConsecutiveFailures) {
        std::cerr << "too many non-convergent cycles; stopping\n";
        break;
      }
    }

    if (have_command) {
      std::array<std::uint16_t, kServos> out{};
      for (std::size_t i = 0; i < kIk; ++i) {
        out[i] = ToTicks(q_cmd[i], calibration[i]);
      }
      out[5] = jaw_ticks;
      bus.WritePositions(out);
    }

    LogRow row{};
    row.t = elapsed;
    row.sp_x = setpoint.pose.p.x();
    row.sp_y = setpoint.pose.p.y();
    row.sp_z = setpoint.pose.p.z();
    for (std::size_t i = 0; i < kIk; ++i) {
      row.measured_rad[i] = q_measured(i);
      row.commanded_rad[i] = q_cmd[i];
    }
    row.converged = converged;
    row.cycle_us = std::chrono::duration_cast<std::chrono::microseconds>(
      clock::now() - cycle_start).count();
    log.push_back(row);

    if (elapsed > duration_s + kSettleS) { break; }

    next += period;
    std::this_thread::sleep_until(next);
  }

  if (!WriteCsv(kOutPath, log)) {
    std::cerr << "FAILED to write " << kOutPath
              << " -- does tools/ik/results/ exist?\n";
    return -1;
  }
  std::cout << "wrote " << log.size() << " rows to " << kOutPath << '\n';
}
