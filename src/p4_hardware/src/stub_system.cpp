#include "p4_hardware/stub_system.hpp"

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

namespace p4_hardware
{

hardware_interface::CallbackReturn StubSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const hardware_interface::HardwareInfo & info = params.hardware_info;

  // The URDF must declare exactly our six joints, in canonical order.
  // Catching a mismatch here beats a wrong index at cycle 400.
  if (info.joints.size() != p4_common::kNumJoints) {
    RCLCPP_ERROR(
      rclcpp::get_logger("StubSystem"), "URDF declares %zu joints, expected %zu",
      info.joints.size(), p4_common::kNumJoints);
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (std::size_t i = 0; i < p4_common::kNumJoints; ++i) {
    const std::string expected(p4_common::joint_order[i]);
    if (info.joints[i].name != expected) {
      RCLCPP_ERROR(
        rclcpp::get_logger("StubSystem"), "joint %zu is '%s', expected '%s'", i,
        info.joints[i].name.c_str(), expected.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  const auto it = info.hardware_parameters.find("calibration_path");
  if (it == info.hardware_parameters.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("StubSystem"), "missing hardware parameter: calibration_path");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // The stub never converts to ticks, but loading here means a broken
  // calibration file fails with no arm plugged in.
  try {
    calibration_ = p4_common::LoadCalibration(it->second);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(rclcpp::get_logger("StubSystem"), "%s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> StubSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < p4_common::kNumJoints; ++i) {
    interfaces.emplace_back(
      std::string(p4_common::joint_order[i]), hardware_interface::HW_IF_POSITION, &positions_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> StubSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (std::size_t i = 0; i < p4_common::kNumJoints; ++i) {
    interfaces.emplace_back(
      std::string(p4_common::joint_order[i]), hardware_interface::HW_IF_POSITION, &commands_[i]);
  }
  return interfaces;
}

hardware_interface::return_type StubSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Perfect servo: measured state is whatever was last commanded.
  positions_ = commands_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type StubSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // read() does the echo. Nothing to send.
  return hardware_interface::return_type::OK;
}

}  // namespace p4_hardware

PLUGINLIB_EXPORT_CLASS(p4_hardware::StubSystem, hardware_interface::SystemInterface)
