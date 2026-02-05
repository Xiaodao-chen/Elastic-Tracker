#pragma once

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <cstdint>
#include <fstream>
#include <string>
#include <type_traits>

namespace wr_msg {

template <class MsgT>
static void writeMsgRos2(const MsgT& msg, const std::string& filename) {
  rclcpp::Serialization<MsgT> serializer;
  rclcpp::SerializedMessage serialized;
  serializer.serialize_message(&msg, &serialized);

  const auto& rcl = serialized.get_rcl_serialized_message();
  const uint32_t size = static_cast<uint32_t>(rcl.buffer_length);

  std::ofstream ofs(filename, std::ios::out | std::ios::binary);
  ofs.write(reinterpret_cast<const char*>(&size), sizeof(size));
  ofs.write(reinterpret_cast<const char*>(rcl.buffer), static_cast<std::streamsize>(size));
  ofs.close();
}

template <class MsgT>
static bool readMsgRos2(MsgT& msg, const std::string& filename) {
  std::ifstream ifs(filename, std::ios::in | std::ios::binary);
  if (!ifs.good()) {
    return false;
  }

  uint32_t size = 0;
  ifs.read(reinterpret_cast<char*>(&size), sizeof(size));
  if (!ifs.good() || size == 0) {
    return false;
  }

  rclcpp::SerializedMessage serialized(static_cast<size_t>(size));
  auto& rcl = serialized.get_rcl_serialized_message();
  ifs.read(reinterpret_cast<char*>(rcl.buffer), static_cast<std::streamsize>(size));
  if (!ifs.good()) {
    return false;
  }
  rcl.buffer_length = size;

  rclcpp::Serialization<MsgT> serializer;
  serializer.deserialize_message(&serialized, &msg);
  return true;
}

}  // namespace wr_msg

