#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

#include "p4_hardware/feetech_protocol.hpp"
#include "p4_hardware/serial_port.hpp"

int main()
{
  const std::string device_path = "/dev/ttyACM0";
  const int baud_rate = 1000000;

  try {
    p4_hardware::SerialPort port(device_path, baud_rate);
    std::cout << "Opened " << device_path << " at " << baud_rate << " baud.\n";

    for (std::uint8_t id = 1; id <= 6; ++id) {
      const auto packet = p4_hardware::BuildPing(id);

      if (!port.Write(packet.data(), packet.size())) {
        std::cerr << "id " << int(id) << ": write failed\n";
        continue;
      }

      p4_hardware::ResponseReader reader;
      std::array<std::uint8_t, 64> buf{};
      bool answered = false;

      // Phase 1 measured a sync read at max ~1.03 ms. 20 ms is generous
      // for a single ping and still short enough to notice a dead servo.
      const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(20);

      while (std::chrono::steady_clock::now() < deadline && !answered) {
        const std::size_t n = port.Read(buf.data(), buf.size(), 2);
        if (n == 0) {
          continue;                      // nothing yet, keep waiting
        }

        reader.Append(std::span<const std::uint8_t>(buf.data(), n));

        // A read is not a packet: every reply in the S04 capture arrived
        // split across two reads. Keep parsing until the buffer is dry.
        while (true) {
          const auto result = reader.TryParseOne();
          if (result.status == p4_hardware::ResponseStatus::Ok){
            std::cout << "id " << int(id) << ": replied\n";
            answered = true;
            break;
          }
          if (result.status == p4_hardware::ResponseStatus::Incomplete) {
            break;                       // not an error; need more bytes
          }
          std::cerr << "id " << int(id) << ": parse error\n";
          break;
        }
      }

      if (!answered) {
        std::cerr << "id " << int(id) << ": no reply\n";
      }
    }
  } catch (const std::runtime_error & e) {
    std::cerr << "Error: " << e.what() << "\n";
    return -1;
  }

  return 0;
}
