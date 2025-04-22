// Date: 2025-03-08
// Creator: PraySky
// Description: 数据crc校验
#pragma once
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
class CRC
{
private:
  inline static uint8_t crc8_table[256]{0};
  inline static uint16_t crc16_table[256]{0};

public:
  static void init_table()
  {
    init_crc8_table();
    init_crc16_table();
  };
  static void init_crc8_table()
  {
    for (uint16_t i = 0; i < 256; i++) {
      uint8_t crc = i;
      for (uint8_t j = 0; j < 8; j++) {
        crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
      }
      crc8_table[i] = crc;
    }
  }
  static void init_crc16_table()
  {
    for (uint16_t i = 0; i < 256; i++) {
      uint16_t crc = i << 8;
      for (uint8_t j = 0; j < 8; j++) {
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
      }
      crc16_table[i] = crc;
    }
  }
  [[nodiscard]] static uint8_t crc8(const std::vector<uint8_t> & data, uint8_t crc = 0)
  {
    for (const auto & byte : data) {
      crc = crc8_table[crc ^ byte];
    }
    return crc;
  }

  static uint16_t crc16(const std::vector<uint8_t> & data, uint16_t crc = 0)
  {
    for (const auto & byte : data) {
      crc = crc16_table[(crc >> 8) ^ byte] ^ (crc << 8);
    }
    return crc;
  }

  static bool verify_crc8(const std::vector<uint8_t> & data) { return crc8(data) == 0; }

  static bool verify_crc16(const std::vector<uint8_t> & data) { return crc16(data) == 0; }
};

}  // namespace rm_serial_driver