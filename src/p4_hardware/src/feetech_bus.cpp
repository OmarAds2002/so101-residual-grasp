#include "p4_hardware/feetech_bus.hpp"

#include <array>
#include <chrono>
#include <span>

namespace p4_hardware
{

namespace
{
// This arm's servo IDs, base to tip, matching p4_common's joint_order.
// The calibration file names each ID explicitly so a replaced servo is a
// config edit; this class assumes the factory numbering holds.
constexpr std::array<std::uint8_t, kNumServos> kServoIds{1, 2, 3, 4, 5, 6};
}  // namespace

FeetechBus::FeetechBus(const std::string & device_path, int baud_rate)
: port_(device_path, baud_rate)
{
}

bool FeetechBus::ReadPositions(std::array<std::uint16_t, kNumServos> & ticks_out)
{
  // Anything half-arrived from a previous call is junk now: a reply that
  // missed its deadline will never be useful, and leaving it in the buffer
  // would corrupt the parse of this read.
  reader_.Clear();

  const auto packet = BuildSyncRead(kServoIds);
  if (!port_.Write(packet.data(), packet.size())) {
    return false;
  }

  // Phase 1: sync read p50 0.87 ms, max 1.03 ms, in a 5 ms period. 3 ms is
  // far above any plausible good read, including a scheduler hiccup, and
  // still inside the period. A timeout here means something is wrong, not
  // that the machine was briefly busy.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(3);

  std::array<bool, kNumServos> seen{};
  std::size_t replies = 0;

  while (std::chrono::steady_clock::now() < deadline && replies < kNumServos) {
    const std::size_t n = port_.Read(buf_.data(), buf_.size(), 2);
    if (n == 0) {
      continue;  // nothing yet; poll again
    }

    reader_.Append(std::span<const std::uint8_t>(buf_.data(), n));

    // A read is not a packet. Every reply in the S04 capture arrived split
    // across two reads, so drain the reader before going back to the port.
    bool drained = false;
    while (!drained) {
      const auto result = reader_.TryParseOne();

      switch (result.status) {
        case ResponseStatus::Ok: {
          const std::uint8_t id = result.response.servo_id;
          if (id < 1 || id > kNumServos) {
            return false;  // a reply from something that isn't ours
          }
          if (result.response.payload_size != 2) {
            return false;  // a position is two bytes
          }

          // Little-endian: low byte first. STS/SMS with protocol_end = 0.
          const std::uint16_t ticks =
            static_cast<std::uint16_t>(result.response.payload[0]) |
            static_cast<std::uint16_t>(static_cast<std::uint16_t>(result.response.payload[1]) << 8);

          const std::size_t slot = static_cast<std::size_t>(id) - 1;
          ticks_out[slot] = ticks;

          // Count each servo once. A duplicate reply must not make up for
          // a servo that never answered.
          if (!seen[slot]) {
            seen[slot] = true;
            ++replies;
          }
          break;
        }

        case ResponseStatus::Incomplete:
          drained = true;  // not an error; need more bytes
          break;

        default:
          return false;  // bad checksum or overflow
      }
    }
  }

  return replies == kNumServos;
}

bool FeetechBus::WritePositions(const std::array<std::uint16_t, kNumServos> & ticks)
{
  const auto packet = BuildSyncWrite(kServoIds, ticks);
  return port_.Write(packet.data(), packet.size());
}

}  // namespace p4_hardware
