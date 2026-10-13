#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace p4_hardware
{
class SerialPort
{
public:
  SerialPort(const std::string & device_path, int baud_rate);
  ~SerialPort();

  // Write the bytes to the serial port. Returns true if all bytes were
  // written, false if an error occurred.
  bool Write(const uint8_t * data, size_t n);
  std::size_t Read(uint8_t * buf, size_t n, int timeout_ms);
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

private:
  std::string device_path_;
  int baud_rate_;
  int fd_;
};

}  // namespace p4_hardware
