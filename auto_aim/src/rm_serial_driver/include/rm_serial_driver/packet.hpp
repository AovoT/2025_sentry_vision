// Date: 2025-03-08
// Creator: PraySky
// Description: 视觉串口通信数据包定义
#pragma once
#include <sys/types.h>

#include <cstdint>
#include <cstring>
#include <vector>
#include <rm_serial_driver/crc.hpp>

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header{0x5A};
  uint8_t detect_color : 1;  // 0 - red 1 - blue
  bool reset_tracker : 1;
  uint8_t reserved : 6;
  float roll;
  float pitch;
  float yaw;
  uint8_t crc;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header{0xA5};
  float x;
  float y;
  float z;
  float yaw;
  float vx;
  float vy;
  float vz;
  float v_yaw;
  float r1;
  float r2;
  float dz;
  uint8_t crc;
} __attribute__((packed));
//TODO 后续拓展方向，使用内嵌struct，方便扩展管理和导航的整合通信

// 从结构体中获取字节流（去掉 crc 字段）
// 检查结构体是否包含 crc 成员（特化版）
template<typename T, typename = void>
struct has_crc_member : std::false_type {};

template<typename T>
struct has_crc_member<T, std::void_t<decltype(std::declval<T>().crc)>> : std::true_type {};

template <typename T>
std::vector<uint8_t> getPacketBytesWithoutCRC(const T& packet)
{
    if constexpr (has_crc_member<T>::value) {
        // 如果有 crc 字段，计算去除 crc 后的数据
        const uint8_t* packetBytes = reinterpret_cast<const uint8_t*>(&packet);
        size_t dataSize = sizeof(T) - sizeof(packet.crc);  // 不包含 CRC 字段
        std::vector<uint8_t> data(packetBytes, packetBytes + dataSize);
        return data;
    } else {
        // 没有 crc 字段，返回整个结构体的字节流
        const uint8_t* packetBytes = reinterpret_cast<const uint8_t*>(&packet);
        std::vector<uint8_t> data(packetBytes, packetBytes + sizeof(T));
        return data;
    }
}

[[nodiscard]] inline ReceivePacket unpack(const uint8_t * data, size_t length)
{
  ReceivePacket packet;
  if (length >= sizeof(ReceivePacket)) {
    std::memcpy(&packet, data, sizeof(ReceivePacket));
  }
  return packet;
}

template <typename Packet>
[[nodiscard]] inline std::vector<uint8_t> pack(Packet & packet)
{
  // 编译期检查：必须有 1 或 2 字节的 crc 成员
  static_assert(
    sizeof(Packet::crc) == 1 || sizeof(Packet::crc) == 2, "crc is not 1 byte or 2 bytes");

  // 拷贝前半部分数据（不含 CRC）
  size_t data_size = sizeof(Packet) - sizeof(Packet::crc);
  std::vector<uint8_t> data(data_size + sizeof(Packet::crc));
  std::memcpy(data.data(), &packet, data_size);

  // 动态计算 CRC
  if constexpr (sizeof(Packet::crc) == 1) {
    packet.crc = CRC::crc8(data);
    data[data_size] = packet.crc;
  } else {
    packet.crc = CRC::crc16(data);
    std::memcpy(&data[data_size], &packet.crc, sizeof(packet.crc));
  }

  return data;
}

}  // namespace rm_serial_driver