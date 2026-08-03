#include <chrono>
#include <cstdint>
#include <cstring>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <zmq.hpp>

#include "input_interface/zmq_manager.hpp"

namespace {

constexpr size_t kHeaderSize = 1280;

std::vector<uint8_t> BuildCommand(bool playback_start, bool normal_completion) {
  nlohmann::json header = {
      {"v", 1},
      {"endian", "le"},
      {"count", 1},
      {"fields", nlohmann::json::array({
          {{"name", "start"}, {"dtype", "u8"}, {"shape", {1}}},
          {{"name", "stop"}, {"dtype", "u8"}, {"shape", {1}}},
          {{"name", "planner"}, {"dtype", "u8"}, {"shape", {1}}},
          {{"name", "playback_start"}, {"dtype", "u8"}, {"shape", {1}}},
          {{"name", "normal_completion"}, {"dtype", "u8"}, {"shape", {1}}},
          {{"name", "playback_id"}, {"dtype", "i64"}, {"shape", {1}}},
          {{"name", "terminal_frame_index"}, {"dtype", "i64"}, {"shape", {1}}},
      })},
  };
  const auto header_text = header.dump();
  std::vector<uint8_t> message;
  message.insert(message.end(), {'c', 'o', 'm', 'm', 'a', 'n', 'd'});
  message.insert(message.end(), header_text.begin(), header_text.end());
  message.resize(message.size() + kHeaderSize - header_text.size(), 0);
  message.insert(message.end(), {0, 0, 0,
                                 static_cast<uint8_t>(playback_start),
                                 static_cast<uint8_t>(normal_completion)});
  const int64_t playback_id = 17;
  const int64_t terminal_frame_index = normal_completion ? 42 : -1;
  const auto append_i64 = [&message](int64_t value) {
    const auto offset = message.size();
    message.resize(offset + sizeof(value));
    std::memcpy(message.data() + offset, &value, sizeof(value));
  };
  append_i64(playback_id);
  append_i64(terminal_frame_index);
  return message;
}

}  // namespace

TEST(ZMQPlaybackProtocolTest, DecodesTaggedMarkersAndDefersStaleCompletion) {
  zmq::context_t context(1);
  zmq::socket_t publisher(context, zmq::socket_type::pub);
  publisher.bind("tcp://127.0.0.1:*");
  const auto endpoint = publisher.get(zmq::sockopt::last_endpoint);
  const auto port = std::stoi(endpoint.substr(endpoint.rfind(':') + 1));

  ZMQManager manager("127.0.0.1", port);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  const auto send = [&publisher](const std::vector<uint8_t>& message) {
    for (int i = 0; i < 3; ++i) {
      publisher.send(zmq::buffer(message), zmq::send_flags::none);
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
  };

  send(BuildCommand(true, false));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  manager.update();
  const auto playback_start = manager.ConsumePlaybackStart();
  ASSERT_TRUE(playback_start.has_value());
  EXPECT_EQ(*playback_start, 17);

  send(BuildCommand(false, true));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  manager.update();
  EXPECT_FALSE(manager.ConsumeNormalCompletionIfReady().has_value());
}
