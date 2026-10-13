#include <array>
#include <cstdint>
#include <iostream>

#include "p4_hardware/feetech_bus.hpp"

int main()
{
  try {
    p4_hardware::FeetechBus bus("/dev/ttyACM0", 1000000);
    std::array<std::uint16_t, p4_hardware::kNumServos> ticks{};

    if (!bus.ReadPositions(ticks)) {
      std::cerr << "read failed\n";
      return -1;
    }

    for (std::size_t i = 0; i < ticks.size(); ++i) {
      std::cout << "servo " << i + 1 << ": " << ticks[i] << '\n';
    }
  } catch (const std::runtime_error & e) {
    std::cerr << "Error: " << e.what() << "\n";
    return -1;
  }
  return 0;
}
