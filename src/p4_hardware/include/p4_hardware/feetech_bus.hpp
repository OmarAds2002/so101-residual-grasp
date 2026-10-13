#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>

#include "p4_hardware/feetech_protocol.hpp"
#include "p4_hardware/serial_port.hpp"
namespace p4_hardware
{
static constexpr std::size_t kNumServos = 6;
class FeetechBus
{
public:
  FeetechBus(const std::string & device_path, int baud_rate);
  bool ReadPositions(std::array<std::uint16_t, kNumServos> & ticks_out);
  bool WritePositions(const std::array<std::uint16_t, kNumServos> & ticks);
  FeetechBus(const FeetechBus &) = delete;
  FeetechBus & operator=(const FeetechBus &) = delete;

private:
  SerialPort port_;
  ResponseReader reader_;
  std::array<std::uint8_t, 64> buf_;
};

}  // namespace p4_hardware
