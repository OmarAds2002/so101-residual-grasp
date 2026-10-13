#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "p4_hardware/feetech_protocol.hpp"

// Bytes captured from the vendor SDK -- tools/bringup/results/04_packet_capture.md.
// The expected checksums are what the servos actually accepted, so the two
// sides of each check come from different sources.

TEST(FeetechChecksum, Ping)
{
  // ff ff 01 02 01 fb
  const std::vector<std::uint8_t> body = {0x01, 0x02, 0x01};
  EXPECT_EQ(p4_hardware::CalculateChecksum(body), 0xfb);
}

TEST(FeetechChecksum, SyncReadWrapsPastOneByte)
{
  // ff ff fe 0a 82 38 02 01 02 03 04 05 06 26
  const std::vector<std::uint8_t> body = {0xfe, 0x0a, 0x82, 0x38, 0x02, 0x01,
                                          0x02, 0x03, 0x04, 0x05, 0x06};
  EXPECT_EQ(p4_hardware::CalculateChecksum(body), 0x26);
}

TEST(FeetechChecksum, SyncWrite)
{
  // ff ff fe 16 83 2a 02 01 5c 07 02 10 09 03 3c 08
  //          04 b5 0a 05 c0 0b 06 f9 07 dd
  const std::vector<std::uint8_t> body = {0xfe, 0x16, 0x83, 0x2a, 0x02, 0x01, 0x5c, 0x07,
                                          0x02, 0x10, 0x09, 0x03, 0x3c, 0x08, 0x04, 0xb5,
                                          0x0a, 0x05, 0xc0, 0x0b, 0x06, 0xf9, 0x07};
  EXPECT_EQ(p4_hardware::CalculateChecksum(body), 0xdd);
}

TEST(FeetechChecksum, ReplyFromServo)
{
  // ff ff 01 04 00 5c 07 97 -- sync read reply, ID 1, position 1884
  const std::vector<std::uint8_t> body = {0x01, 0x04, 0x00, 0x5c, 0x07};
  EXPECT_EQ(p4_hardware::CalculateChecksum(body), 0x97);
}

TEST(FeetechBuild, Ping)
{
  // ff ff 01 02 01 fb
  const std::array<std::uint8_t, 6> expected = {0xff, 0xff, 0x01, 0x02, 0x01, 0xfb};
  EXPECT_EQ(p4_hardware::BuildPing(1), expected);
}

TEST(FeetechBuild, SyncReadPositions)
{
  // ff ff fe 0a 82 38 02 01 02 03 04 05 06 26
  const std::array<std::uint8_t, 6> ids = {1, 2, 3, 4, 5, 6};
  const std::array<std::uint8_t, 14> expected = {0xff, 0xff, 0xfe, 0x0a, 0x82, 0x38, 0x02,
                                                 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x26};
  EXPECT_EQ(p4_hardware::BuildSyncRead(ids), expected);
}

TEST(FeetechBuild, SyncWriteGoals)
{
  const std::array<std::uint8_t, 6> ids = {1, 2, 3, 4, 5, 6};
  const std::array<std::uint16_t, 6> goals = {1884, 2320, 2108, 2741, 3008, 2041};

  const std::array<std::uint8_t, 26> expected = {
    0xff, 0xff, 0xfe, 0x16, 0x83, 0x2a, 0x02, 0x01, 0x5c, 0x07, 0x02, 0x10, 0x09,
    0x03, 0x3c, 0x08, 0x04, 0xb5, 0x0a, 0x05, 0xc0, 0x0b, 0x06, 0xf9, 0x07, 0xdd};
  EXPECT_EQ(p4_hardware::BuildSyncWrite(ids, goals), expected);
}

TEST(ResponseReader, ReassemblesSplitReply)
{
  p4_hardware::ResponseReader reader;

  const std::array<std::uint8_t, 6> first = {0xff, 0xff, 0x01, 0x04, 0x00, 0x5c};
  const std::array<std::uint8_t, 2> second = {0x07, 0x97};

  reader.Append(first);
  EXPECT_EQ(reader.TryParseOne().status, p4_hardware::ResponseStatus::Incomplete);

  reader.Append(second);
  const auto result = reader.TryParseOne();
  EXPECT_EQ(result.status, p4_hardware::ResponseStatus::Ok);
  EXPECT_EQ(result.response.servo_id, 1);
  EXPECT_EQ(result.response.error, 0);
  EXPECT_EQ(result.response.payload[0], 0x5c);
  EXPECT_EQ(result.response.payload[1], 0x07);
}
TEST(ResponseReader, ReassemblesSixSyncReadReplies)
{
  p4_hardware::ResponseReader reader;

  // Exactly as captured in tools/bringup/results/04_packet_capture.md --
  // every reply arrived split across two reads.
  const std::array<std::uint8_t, 6> r1a = {0xff, 0xff, 0x01, 0x04, 0x00, 0x5c};
  const std::array<std::uint8_t, 2> r1b = {0x07, 0x97};
  const std::array<std::uint8_t, 6> r2a = {0xff, 0xff, 0x02, 0x04, 0x00, 0x10};
  const std::array<std::uint8_t, 2> r2b = {0x09, 0xe0};
  const std::array<std::uint8_t, 6> r3a = {0xff, 0xff, 0x03, 0x04, 0x00, 0x3c};
  const std::array<std::uint8_t, 2> r3b = {0x08, 0xb4};
  const std::array<std::uint8_t, 6> r4a = {0xff, 0xff, 0x04, 0x04, 0x00, 0xb5};
  const std::array<std::uint8_t, 2> r4b = {0x0a, 0x38};
  const std::array<std::uint8_t, 6> r5a = {0xff, 0xff, 0x05, 0x04, 0x00, 0xc0};
  const std::array<std::uint8_t, 2> r5b = {0x0b, 0x2b};
  const std::array<std::uint8_t, 6> r6a = {0xff, 0xff, 0x06, 0x04, 0x00, 0xf9};
  const std::array<std::uint8_t, 2> r6b = {0x07, 0xf5};

  reader.Append(r1a);
  reader.Append(r1b);
  reader.Append(r2a);
  reader.Append(r2b);
  reader.Append(r3a);
  reader.Append(r3b);
  reader.Append(r4a);
  reader.Append(r4b);
  reader.Append(r5a);
  reader.Append(r5b);
  reader.Append(r6a);
  reader.Append(r6b);

  // id, low byte, high byte -- the positions the SDK reported were
  // 1884, 2320, 2108, 2741, 3008, 2041.
  const std::array<std::array<std::uint8_t, 3>, 6> expected = {{
    {0x01, 0x5c, 0x07},
    {0x02, 0x10, 0x09},
    {0x03, 0x3c, 0x08},
    {0x04, 0xb5, 0x0a},
    {0x05, 0xc0, 0x0b},
    {0x06, 0xf9, 0x07},
  }};

  for (const auto & e : expected) {
    const auto result = reader.TryParseOne();
    EXPECT_EQ(result.status, p4_hardware::ResponseStatus::Ok);
    EXPECT_EQ(result.response.servo_id, e[0]);
    EXPECT_EQ(result.response.error, 0);
    EXPECT_EQ(result.response.payload_size, 2u);
    EXPECT_EQ(result.response.payload[0], e[1]);
    EXPECT_EQ(result.response.payload[1], e[2]);
  }

  // Buffer is drained -- nothing left to parse.
  EXPECT_EQ(reader.TryParseOne().status, p4_hardware::ResponseStatus::Incomplete);
}
