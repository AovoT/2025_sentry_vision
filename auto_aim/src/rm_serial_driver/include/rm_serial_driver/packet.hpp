// Date: 2025-03-08
// Creator: PraySky
// Description: 视觉串口通信数据包定义
#pragma once
#include <sys/types.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <iomanip>

namespace rm_serial_driver
{
struct HeaderFrame
{
  uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
  uint8_t len;  // 数据段长度
  uint8_t id;   // 数据段id
  uint8_t crc;  // 数据帧头的 CRC8 校验
} __attribute__((packed));

// IMU 数据包
struct ReceiveImuData
{
  HeaderFrame frame_header;  // id = 0x01

  uint32_t time_stamp;

  struct
  {
    float yaw;    // rad
    float pitch;  // rad
    float roll;   // rad

    float yaw_vel;    // rad/s
    float pitch_vel;  // rad/s
    float roll_vel;   // rad/s

  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));
struct ReceiveTargetInfoData
{
  HeaderFrame frame_header;  // id = 0x09

  uint32_t time_stamp;

  struct
  {
    uint8_t detect_color;
    uint8_t reset_tracker;
    float roll;
    float pitch;
    float yaw;
    float aim_x;
    float aim_y;
    float aim_z;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

struct SendVisionData
{
  HeaderFrame frame_header;  // id = 0x03

  uint32_t time_stamp;

  struct
  {
    uint8_t tracking;
    uint8_t id;
    uint8_t armors_num;
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
  } __attribute__((packed)) data;

  uint16_t checksum;
} __attribute__((packed));


inline void print(const ReceiveImuData &imu)
{
  std::ios old_state(nullptr);
  old_state.copyfmt(std::cout);                    // 备份输出格式

  std::cout << std::fixed << std::setprecision(3); // 统一小数位
  std::cout << "---- ReceiveImuData ----\n";
  std::cout << "timestamp : " << imu.time_stamp << " us\n";

  std::cout << "yaw       : "   << imu.data.yaw
            << "  rad   (vel "  << imu.data.yaw_vel   << " rad/s)\n";
  std::cout << "pitch     : "   << imu.data.pitch
            << "  rad   (vel "  << imu.data.pitch_vel << " rad/s)\n";
  std::cout << "roll      : "   << imu.data.roll
            << "  rad   (vel "  << imu.data.roll_vel  << " rad/s)\n";

  std::cout << "crc16     : 0x" << std::hex << std::setw(4) << std::setfill('0')
            << imu.crc << std::dec << '\n';
  std::cout << "------------------------" << std::endl;

  std::cout.copyfmt(old_state);                   // 恢复原格式
}

inline void print(const SendVisionData &pkt)
{
  printf("=== SendVisionData ===\n");
  printf("Header ID     : 0x%02X\n", pkt.frame_header.id);
  printf("Timestamp     : %u\n", pkt.time_stamp);
  printf("Tracking      : %u\n", pkt.data.tracking);
  printf("Target ID     : %u\n", pkt.data.id);
  printf("Armors Num    : %u\n", pkt.data.armors_num);
  printf("Position      : (x: %.2f, y: %.2f, z: %.2f)\n",
         pkt.data.x, pkt.data.y, pkt.data.z);
  printf("Norm:         : %.2f\n", std::sqrt((pkt.data.x* pkt.data.x + pkt.data.y* pkt.data.y + pkt.data.z * pkt.data.z)));
  printf("Yaw           : %.2f deg\n", pkt.data.yaw);
  printf("Velocity      : (vx: %.2f, vy: %.2f, vz: %.2f)\n",
         pkt.data.vx, pkt.data.vy, pkt.data.vz);
  printf("Yaw Velocity  : %.2f deg/s\n", pkt.data.v_yaw);
  printf("Radius        : (r1: %.2f, r2: %.2f)\n", pkt.data.r1, pkt.data.r2);
  printf("Height offset : dz = %.2f\n", pkt.data.dz);
  printf("Checksum      : 0x%04X\n", pkt.checksum);
  printf("=======================\n");
}


}  // namespace rm_serial_driver