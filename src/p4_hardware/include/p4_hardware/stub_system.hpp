#pragma once

#include <array>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "p4_common/calibration.hpp"
#include "p4_common/joint_order.hpp"

namespace p4_hardware
{

// A hardware backend with no hardware. read() echoes back whatever was
// last commanded, so the arm behaves like a perfect, instant servo.
// Lets the full stack come up with nothing plugged in.
class StubSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::array<double, p4_common::kNumJoints> positions_{};
  std::array<double, p4_common::kNumJoints> commands_{};
  std::array<p4_common::JointCalibration, p4_common::kNumJoints> calibration_{};
};

}  // namespace p4_hardware
