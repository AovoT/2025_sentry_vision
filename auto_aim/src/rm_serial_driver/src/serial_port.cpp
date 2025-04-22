#include "rm_serial_driver/serial_port.hpp"

#include <filesystem>
#include <stdexcept>

namespace rm_serial_driver
{

SerialPort::SerialPort(const SerialConfig & config) : fd_(-1), config_(config)
{
  namespace fs = std::filesystem;
  if (!fs::exists(fs::path{config.device})) {
    throw std::invalid_argument("Serial device path invalid");
  }
}

SerialPort::~SerialPort() { close(); }

bool SerialPort::open()
{
  if (isOpen()) {
    close();
  }

  fd_ = ::open(config_.device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_ < 0) return false;

  fcntl(fd_, F_SETFL, 0);  // 设置阻塞模式

  return configurePort();
}

bool SerialPort::configure(const SerialConfig & config)
{
  config_ = config;
  if (!isOpen()) {
    return true;  // 如果端口未打开，仅保存配置
  }
  return configurePort();  // 如果端口已打开，立即应用新配置
}

bool SerialPort::configurePort()
{
  struct termios tty;
  memset(&tty, 0, sizeof tty);

  if (tcgetattr(fd_, &tty) != 0) return false;

  // 设置波特率
  speed_t speed = baudToSpeed(config_.baudrate);
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  // 配置数据位
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= dataBitsToFlag(config_.data_bits);

  // 配置停止位
  if (config_.stop_bits == 2) {
    tty.c_cflag |= CSTOPB;
  } else {
    tty.c_cflag &= ~CSTOPB;
  }

  // 配置校验位
  if (config_.parity) {
    tty.c_cflag |= PARENB;
  } else {
    tty.c_cflag &= ~PARENB;
  }

  // 配置硬件流控
  if (config_.hardware_flow) {
    tty.c_cflag |= CRTSCTS;
  } else {
    tty.c_cflag &= ~CRTSCTS;
  }

  tty.c_cflag |= CREAD | CLOCAL;

  // 原始输入输出模式
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_oflag &= ~OPOST;

  // 设置超时
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = config_.timeout_ms / 100;  // 转换为100ms单位

  return tcsetattr(fd_, TCSANOW, &tty) == 0;
}

int SerialPort::dataBitsToFlag(int data_bits)
{
  switch (data_bits) {
    case 5:
      return CS5;
    case 6:
      return CS6;
    case 7:
      return CS7;
    case 8:
      return CS8;
    default:
      return CS8;
  }
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

ssize_t SerialPort::write(const uint8_t * data, size_t size)
{
  return ::write(fd_, data, size);
}

ssize_t SerialPort::read(uint8_t * buffer, size_t size)
{
  return ::read(fd_, buffer, size);
}

bool SerialPort::isOpen() const { return fd_ >= 0; }

}  // namespace rm_serial_driver