#pragma once
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <string>

namespace rm_serial_driver
{

struct SerialConfig
{
  std::string device;         // 设备路径
  int baudrate{115200};       // 波特率
  int data_bits{8};           // 数据位 (5-8)
  int stop_bits{1};           // 停止位 (1-2)
  bool parity{false};         // 是否使用奇偶校验
  bool hardware_flow{false};  // 是否使用硬件流控
  int timeout_ms{1000};       // 读取超时时间(毫秒)
};

class SerialPort
{
public:
  SerialPort(const SerialConfig & config);
  ~SerialPort();

  bool open();
  void close();
  bool configure(const SerialConfig & config);
  ssize_t write(const uint8_t * data, size_t size);
  ssize_t read(uint8_t * buffer, size_t size);
  bool isOpen() const;
  const SerialConfig & getConfig() const { return config_; }

private:
  int fd_;
  SerialConfig config_;

  bool configurePort();
  speed_t baudToSpeed(int baudrate);
  int dataBitsToFlag(int data_bits);
};

}  // namespace rm_serial_driver