#pragma once

#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <unordered_map>
#include <vector>

#include "CRC8_CRC16.hpp"
#include "rm_serial_driver/packet.hpp"  // 定义 HeaderFrame, ReceiveImuData, ReceiveTargetInfoData, SendVisionData

namespace rm_serial_driver
{

// 判断尾部字段名是否为 crc 或 checksum
template <typename T, typename = void>
struct has_crc_field : std::false_type
{
};

template <typename T>
struct has_crc_field<T, std::void_t<decltype(std::declval<T>().crc)>>
: std::true_type
{
};

template <typename T, typename = void>
struct has_checksum_field : std::false_type
{
};

template <typename T>
struct has_checksum_field<T, std::void_t<decltype(std::declval<T>().checksum)>>
: std::true_type
{
};

// pack: 先算帧头 CRC8，再算整包 CRC16 小端追加
template <typename Packet>
std::vector<uint8_t> pack(const Packet & pkt_in)
{
  // 复制一份，以便改写 header 字段
  Packet pkt = pkt_in;

  // —— 1) 设置 header 字段 ——
  // SOF
  pkt.frame_header.sof = 0x5A;
  // payload 长度（字节数）
  pkt.frame_header.len = static_cast<uint8_t>(sizeof(pkt.data));
  // id 由外部 pkt_in.frame_header.id 提供，pack 不改动它

  // 计算 CRC8（仅对 sof, len, id 三字节，seed = 0xFF）
  {
    uint8_t tmp[3] = {
      pkt.frame_header.sof, pkt.frame_header.len, pkt.frame_header.id};
    pkt.frame_header.crc =
      crc8::get_CRC8_check_sum(tmp, /*len=*/3, /*seed=*/0xFF);
  }

  // —— 2) 拼接 header + payload ——
  std::vector<uint8_t> buf;
  buf.reserve(/*header*/ 4 + /*data*/ sizeof(pkt.data) + /*CRC16*/ 2);

  // 手工推入 4 字节 header，避免任何 padding
  buf.push_back(pkt.frame_header.sof);
  buf.push_back(pkt.frame_header.len);
  buf.push_back(pkt.frame_header.id);
  buf.push_back(pkt.frame_header.crc);

  // 推入 payload
  const uint8_t * pd = reinterpret_cast<const uint8_t *>(&pkt.data);
  buf.insert(buf.end(), pd, pd + sizeof(pkt.data));

  // —— 3) 计算 CRC16（对上面所有字节，seed = 0xFFFF）并以小端追加 ——
  uint16_t crc16_val =
    crc16::get_CRC16_check_sum(buf.data(), buf.size(), /*seed=*/0xFFFF);
  buf.push_back(static_cast<uint8_t>(crc16_val & 0xFF));
  buf.push_back(static_cast<uint8_t>((crc16_val >> 8) & 0xFF));

  return buf;
}

template <typename Packet>
bool unpack(const uint8_t * raw, size_t len, Packet & pkt)
{
  if (len < sizeof(HeaderFrame) + 2) return false;  // 至少能放下头+CRC16

  auto * hdr = reinterpret_cast<const HeaderFrame *>(raw);

  /* 1) CRC8 检查三字节帧头 */
  if (crc8::get_CRC8_check_sum(raw, 3, 0xFF) != hdr->crc) return false;

  /* 2) 根据 hdr->len 计算完整包长 */
  const size_t frame_len = sizeof(HeaderFrame) + hdr->len + 2;
  if (len < frame_len) return false;  // 数据还没收全

  /* 3) CRC16 检查 */
  if (!crc16::verify_CRC16_check_sum(raw, frame_len)) return false;  // 假设库有该重载

  /* 4) 拷贝解析 */
  std::memcpy(&pkt, raw, std::min(frame_len, sizeof(Packet)));
  return true;
}

// 串口数据帧解析器
class SerialParser
{
public:
  explicit SerialParser(const rclcpp::Logger & logger) : logger_(logger) {}

  // 注册回调，id 对应 packet.frame_header.id
  template <typename Packet>
  void registerHandler(uint8_t id, std::function<void(const Packet &)> cb)
  {
    handlers_[id] = [cb](const uint8_t * raw, size_t len) {
      Packet pkt{};
      if (unpack<Packet>(raw, len, pkt)) {
        cb(pkt);
        return true;
      }
      return false;
    };
  }

  // 喂入串口读到的数据
  void feed(const uint8_t * data, size_t size)
  {
    buf_.insert(buf_.end(), data, data + size);
    while (rclcpp::ok()) {
      if (buf_.size() < sizeof(HeaderFrame)) break;
      if (buf_.front() != 0x5A) {
        buf_.pop_front();
        RCLCPP_INFO(logger_, "unexpected sof, drop one byte");
        continue;
      }

      uint8_t len = buf_[1];
      size_t full = sizeof(HeaderFrame) + len + 2;
      if (buf_.size() < full) break;

      uint8_t id = buf_[2];
      auto it = handlers_.find(id);
      if (it != handlers_.end()) {
        std::vector<uint8_t> frame(buf_.begin(), buf_.begin() + full);
        it->second(frame.data(), full);
      } else {
        RCLCPP_INFO(logger_, "unknown packet id: %d", id);
      }

      buf_.erase(buf_.begin(), buf_.begin() + full);
    }
  }

private:
  std::deque<uint8_t> buf_;
  rclcpp::Logger logger_;
  std::unordered_map<uint8_t, std::function<bool(const uint8_t *, size_t)>>
    handlers_;
};

}  // namespace rm_serial_driver
