#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace p4_hardware
{

  inline constexpr std::uint8_t kHeaderByte = 0xFF;
  inline constexpr std::size_t kHeaderSize = 2;

  inline constexpr std::uint8_t kInstPing = 0x01;
  inline constexpr std::uint8_t kInstRead = 0x02;
  inline constexpr std::uint8_t kInstWrite = 0x03;
  inline constexpr std::uint8_t kInstSyncRead = 0x82;
  inline constexpr std::uint8_t kInstSyncWrite = 0x83;

  inline constexpr std::uint8_t kBroadcastId = 0xFE;

  inline constexpr std::uint8_t kAddrGoalPosition = 42;
  inline constexpr std::uint8_t kAddrPresentPosition = 56;

  inline constexpr std::size_t kMaxReplySize = 8; // header..checksum
  inline constexpr std::size_t kMaxRepliesPerRead = 6;
  inline constexpr std::size_t kBufferSize = kMaxReplySize * kMaxRepliesPerRead * 2;

  // Feetech checksum: sum the bytes, keep the low 8 bits, invert.
  //
  // Takes the bytes from the ID through the last parameter -- header
  // excluded, checksum itself excluded. The caller decides which span
  // that is, because on the parsing side we don't yet know the bytes
  // form a valid packet.
  inline std::uint8_t CalculateChecksum(std::span<const std::uint8_t> data) noexcept
  {
    std::uint8_t sum = 0;
    for (const std::uint8_t b : data)
    {
      sum += b;
    }
    return static_cast<std::uint8_t>(~sum);
  }

  inline std::array<std::uint8_t, 6> BuildPing(std::uint8_t servo_id) noexcept
  {
    // Length counts the instruction and the checksum. Ping has no parameters.
    constexpr std::uint8_t kPingLength = 2;

    std::array<std::uint8_t, 6> packet{
        kHeaderByte, kHeaderByte, servo_id, kPingLength, kInstPing, 0};

    packet[5] = CalculateChecksum(std::span<const std::uint8_t>(packet).subspan(2, 3));
    return packet;
  }

  inline std::array<std::uint8_t, 14> BuildSyncRead(
      const std::array<std::uint8_t, 6> &servo_ids) noexcept
  {
    constexpr std::uint8_t kBytesPerServo = 2;

    // instruction + address + bytes-per-servo + six IDs + checksum
    constexpr std::uint8_t kSyncReadLength = 1 + 1 + 1 + 6 + 1;

    std::array<std::uint8_t, 14> packet{
        kHeaderByte, kHeaderByte, kBroadcastId, kSyncReadLength,
        kInstSyncRead, kAddrPresentPosition, kBytesPerServo,
        0, 0, 0, 0, 0, 0, // six IDs
        0};               // checksum

    for (std::size_t i = 0; i < servo_ids.size(); ++i)
    {
      packet[7 + i] = servo_ids[i];
    }

    // Sum from the ID through the last parameter: indices 2..12, eleven bytes.
    packet.back() = CalculateChecksum(
        std::span<const std::uint8_t>(packet).subspan(kHeaderSize, 11));

    return packet;
  }

  inline std::array<std::uint8_t, 26> BuildSyncWrite(
      const std::array<std::uint8_t, 6> &servo_ids,
      const std::array<std::uint16_t, 6> &goals) noexcept
  {
    constexpr std::uint8_t kBytesPerServo = 2;

    // instruction + address + bytes-per-servo + six (id + 2 data) + checksum
    constexpr std::uint8_t kSyncWriteLength = 1 + 1 + 1 + (6 * 3) + 1;

    std::array<std::uint8_t, 26> packet{};
    packet[0] = kHeaderByte;
    packet[1] = kHeaderByte;
    packet[2] = kBroadcastId;
    packet[3] = kSyncWriteLength;
    packet[4] = kInstSyncWrite;
    packet[5] = kAddrGoalPosition;
    packet[6] = kBytesPerServo;

    for (std::size_t i = 0; i < servo_ids.size(); ++i)
    {
      const std::size_t at = 7 + (i * 3);
      packet[at] = servo_ids[i];
      packet[at + 1] = static_cast<std::uint8_t>(goals[i] & 0xFF);        // low
      packet[at + 2] = static_cast<std::uint8_t>((goals[i] >> 8) & 0xFF); // high
    }

    // Sum from the ID through the last data byte: indices 2..24, 23 bytes.
    packet.back() = CalculateChecksum(
        std::span<const std::uint8_t>(packet).subspan(kHeaderSize, 23));

    return packet;
  }

  enum class ResponseStatus
  {
    Ok,
    Incomplete,
    BadChecksum,
    BufferOverflow
  };
  struct Response
  {
    std::uint8_t servo_id = 0;
    std::uint8_t error = 0; // servo status bits
    std::array<std::uint8_t, 4> payload{};
    std::size_t payload_size = 0;
  };

  struct ParseResult
  {
    ResponseStatus status = ResponseStatus::Incomplete;
    Response response{};
  };

  class ResponseReader
  {
  public:
    // Add newly-read bytes. Overflow means we failed to frame a packet in
    // far more bytes than the longest one -- whatever is in here is junk,
    // so drop it all and say so rather than guess which half to keep.
    ResponseStatus Append(std::span<const std::uint8_t> bytes)
    {
      if (count_ + bytes.size() > buffer_.size())
      {
        count_ = 0;
        return ResponseStatus::BufferOverflow;
      }
      for (const std::uint8_t b : bytes)
      {
        buffer_[count_++] = b;
      }
      return ResponseStatus::Ok;
    }

    // Pull one complete reply off the front, if there is one.
    ParseResult TryParseOne()
    {
      ParseResult result;

      // Drop anything before the first ff ff.
      const std::size_t start = FindHeader();
      if (start > 0)
      {
        Consume(start);
      }

      // Need at least header, id, length to know how long this reply is.
      if (count_ < 4)
      {
        result.status = ResponseStatus::Incomplete;
        return result;
      }

      const std::uint8_t length = buffer_[3];
      const std::size_t total = kHeaderSize + 2 + length; // ff ff | id len | body

      if (count_ < total)
      {
        result.status = ResponseStatus::Incomplete;
        return result;
      }

      // Sum from the id up to but not including the checksum.
      const std::uint8_t expected = CalculateChecksum(
          std::span<const std::uint8_t>(buffer_).subspan(kHeaderSize, length + 1));

      const std::uint8_t actual = buffer_[total - 1];
      if (expected != actual)
      {
        Consume(total); // drop the bad reply, keep going
        result.status = ResponseStatus::BadChecksum;
        return result;
      }

      result.response.servo_id = buffer_[2];
      result.response.error = buffer_[4];

      // length = error byte + payload + checksum, so payload is length - 2.
      result.response.payload_size = static_cast<std::size_t>(length) - 2;
      for (std::size_t i = 0; i < result.response.payload_size; ++i)
      {
        result.response.payload[i] = buffer_[5 + i];
      }

      Consume(total);
      result.status = ResponseStatus::Ok;
      return result;
    }

  private:
    std::size_t FindHeader() const
    {
      for (std::size_t i = 0; i + 1 < count_; ++i)
      {
        if (buffer_[i] == kHeaderByte && buffer_[i + 1] == kHeaderByte)
        {
          return i;
        }
      }
      return count_;
    }

    // Remove n bytes from the front, shuffle the rest down.
    void Consume(std::size_t n)
    {
      if (n >= count_)
      {
        count_ = 0;
        return;
      }
      for (std::size_t i = 0; i + n < count_; ++i)
      {
        buffer_[i] = buffer_[i + n];
      }
      count_ -= n;
    }

    std::array<std::uint8_t, kBufferSize> buffer_{};
    std::size_t count_ = 0;
  };
} // namespace p4_hardware