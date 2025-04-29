// serial_port.hpp
#pragma once
#include <fcntl.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <stdexcept>
#include <string>

#include <cstdint>

namespace rm_serial_driver
{

struct SerialConfig
{
  std::string device_name;
  int baudrate;
  bool hardware_flow;
  bool parity;
  int stop_bits;
  int timeout_ms;
};

class SerialPort
{
public:
  explicit SerialPort(const SerialConfig & cfg) : cfg_(cfg), fd_(-1) {}

  ~SerialPort()
  {
    close();
  }

  // 新增：显式关闭接口
  void close()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  void open()
  {
    // 直接尝试打开设备节点
    fd_ = ::open(cfg_.device_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error(
        "SerialPort: cannot open " + cfg_.device_name + ": "
      );
    }
    // 确认打开后确实是字符设备
    struct stat st;
    if (fstat(fd_, &st) != 0 || !S_ISCHR(st.st_mode)) {
      close();
      throw std::runtime_error(
        "SerialPort: target is not a character device: " + cfg_.device_name);
    }
    configurePort();
  }

  ssize_t read(uint8_t * buf, size_t size) { return ::read(fd_, buf, size); }

  ssize_t write(const uint8_t * data, size_t size)
  {
    return ::write(fd_, data, size);
  }

  // 将整数波特率转换为 termios speed_t
  static speed_t baudToSpeed(int baud)
  {
    switch (baud) {
      case 9600:
        return B9600;
      case 19200:
        return B19200;
      case 38400:
        return B38400;
      case 57600:
        return B57600;
      case 115200:
        return B115200;
      case 230400:
        return B230400;
      default:
        throw std::invalid_argument(
          "Unsupported baud rate: " + std::to_string(baud));
    }
  }

private:
  void configurePort()
  {
    struct termios tio;
    if (tcgetattr(fd_, &tio) != 0) {
      throw std::runtime_error("SerialPort: tcgetattr failed");
    }
    cfsetispeed(&tio, baudToSpeed(cfg_.baudrate));
    cfsetospeed(&tio, baudToSpeed(cfg_.baudrate));

    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= (cfg_.parity ? PARENB : 0);
    tio.c_cflag |= (cfg_.stop_bits == 2 ? CSTOPB : 0);
    tio.c_cflag |= (cfg_.hardware_flow ? CRTSCTS : 0);
    tio.c_cflag |= CS8;

    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_oflag &= ~OPOST;

    // 设置超时
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = cfg_.timeout_ms / 100;

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
      throw std::runtime_error("SerialPort: tcsetattr failed");
    }
  }

  SerialConfig cfg_;
  int fd_;
};

}  // namespace rm_serial_driver
