#include "p4_hardware/serial_port.hpp"

#include <asm/termbits.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace p4_hardware
{

SerialPort::SerialPort(const std::string & device_path, int baud_rate)
: device_path_(device_path), fd_(-1)
{
  fd_ = ::open(device_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw std::runtime_error(
      "SerialPort: cannot open " + device_path + ": " + std::strerror(errno));
  }

  struct termios2 tio{};
  if (::ioctl(fd_, TCGETS2, &tio) < 0) {
    const int e = errno;
    ::close(fd_);
    throw std::runtime_error(
      "SerialPort: TCGETS2 failed on " + device_path + ": " + std::strerror(e));
  }

  // Raw mode: the kernel must not interpret, translate or buffer anything.
  tio.c_iflag = 0;
  tio.c_oflag = 0;
  tio.c_lflag = 0;

  // 8 data bits, no parity, one stop bit. Reads and writes enabled, and the
  // port is not a controlling terminal.
  tio.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS | CBAUD);
  tio.c_cflag |= CS8 | CREAD | CLOCAL | BOTHER;

  // BOTHER means "the speed is in c_ispeed / c_ospeed, not a constant".
  // 1000000 is not one of the standard B-constants, so this is the only way.
  tio.c_ispeed = baud_rate;
  tio.c_ospeed = baud_rate;

  // poll() owns the timeout, so termios should never block or wait.
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;

  if (::ioctl(fd_, TCSETS2, &tio) < 0) {
    const int e = errno;
    ::close(fd_);
    throw std::runtime_error(
      "SerialPort: TCSETS2 failed on " + device_path + ": " + std::strerror(e));
  }
}

SerialPort::~SerialPort()
{
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

bool SerialPort::Write(const uint8_t * data, std::size_t n)
{
  const ssize_t written = ::write(fd_, data, n);
  return written >= 0 && static_cast<std::size_t>(written) == n;
}

std::size_t SerialPort::Read(uint8_t * buf, std::size_t n, int timeout_ms)
{
  struct pollfd pfd{};
  pfd.fd = fd_;
  pfd.events = POLLIN;

  const int ready = ::poll(&pfd, 1, timeout_ms);
  if (ready <= 0) {
    return 0;  // timed out, or poll failed. Not an error: no bytes yet.
  }

  const ssize_t got = ::read(fd_, buf, n);
  if (got <= 0) {
    return 0;
  }
  return static_cast<std::size_t>(got);
}

}  // namespace p4_hardware
