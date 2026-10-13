#include <gtest/gtest.h>

#include <stdexcept>

#include "p4_hardware/serial_port.hpp"

TEST(SerialPort, ThrowsOnMissingDevice)
{
  EXPECT_THROW(
    p4_hardware::SerialPort port("/dev/definitely_not_a_real_device", 1000000), std::runtime_error);
}
