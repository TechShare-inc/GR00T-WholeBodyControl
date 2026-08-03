#include <chrono>
#include <cstdint>
#include <cstring>
#include <array>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <zmq.hpp>

#include "input_interface/input_command.hpp"
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

std::vector<uint8_t> BuildPose(int64_t frame_index) {
  constexpr size_t kHeaderSize = 1280;
  nlohmann::json header = {
      {"v", 1},
      {"endian", "le"},
      {"count", 1},
      {"fields", nlohmann::json::array({
          {{"name", "joint_pos"}, {"dtype", "f32"}, {"shape", {1, 29}}},
          {{"name", "joint_vel"}, {"dtype", "f32"}, {"shape", {1, 29}}},
          {{"name", "body_quat_w"}, {"dtype", "f32"}, {"shape", {1, 4}}},
          {{"name", "frame_index"}, {"dtype", "i64"}, {"shape", {1}}},
          {{"name", "catch_up"}, {"dtype", "u8"}, {"shape", {1}}},
      })},
  };
  const auto header_text = header.dump();
  std::vector<uint8_t> message;
  message.insert(message.end(), {'p', 'o', 's', 'e'});
  message.insert(message.end(), header_text.begin(), header_text.end());
  message.resize(message.size() + kHeaderSize - header_text.size(), 0);

  const auto append_bytes = [&message](const void* data, size_t size) {
    const auto offset = message.size();
    message.resize(offset + size);
    std::memcpy(message.data() + offset, data, size);
  };
  const std::array<float, 29> joint_position{};
  const std::array<float, 29> joint_velocity{};
  const std::array<float, 4> body_quaternion = {1.0F, 0.0F, 0.0F, 0.0F};
  const uint8_t catch_up = 0;
  append_bytes(joint_position.data(), sizeof(joint_position));
  append_bytes(joint_velocity.data(), sizeof(joint_velocity));
  append_bytes(body_quaternion.data(), sizeof(body_quaternion));
  append_bytes(&frame_index, sizeof(frame_index));
  append_bytes(&catch_up, sizeof(catch_up));
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

TEST(ZMQPlaybackProtocolTest, LoopbackMarkersDriveControllerReturnToStandProtocol) {
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

  const auto defaults = PlaybackProtocol::JointArray{};
  const auto measured = [] {
    PlaybackProtocol::JointArray values{};
    values.fill(1.0);
    return values;
  }();
  const auto velocities = PlaybackProtocol::JointArray{};
  PlaybackProtocol protocol(defaults, 1.0, 0.05, 0.1, 0.1);
  const auto t0 = PlaybackProtocol::Clock::time_point{};

  MotionDataReader motion_reader;
  std::shared_ptr<const MotionSequence> current_motion =
      std::make_shared<MotionSequence>();
  int current_frame = 0;
  OperatorState operator_state;
  bool reinitialize_heading = false;
  DataBuffer<HeadingState> heading_state_buffer;
  PlannerState planner_state;
  DataBuffer<MovementState> movement_state_buffer;
  std::mutex current_motion_mutex;
  bool report_temperature = false;

  send(BuildCommand(true, false));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  manager.update();
  const auto playback_start = manager.ConsumePlaybackStart();
  ASSERT_TRUE(playback_start.has_value());
  protocol.start_action(*playback_start);

  // ZMQManager applies the mode-switch marker during update(), then the
  // control loop applies the safety reset and enables pose streaming during
  // handle_input().
  manager.handle_input(
      motion_reader, current_motion, current_frame, operator_state,
      reinitialize_heading, heading_state_buffer, true, planner_state,
      movement_state_buffer, current_motion_mutex, report_temperature);

  send(BuildPose(42));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  manager.update();
  manager.handle_input(
      motion_reader, current_motion, current_frame, operator_state,
      reinitialize_heading, heading_state_buffer, true, planner_state,
      movement_state_buffer, current_motion_mutex, report_temperature);
  const auto accepted = manager.GetLastAcceptedFrameIndex();
  ASSERT_TRUE(accepted.has_value());
  ASSERT_EQ(*accepted, 42);

  send(BuildCommand(false, true));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  manager.update();
  const auto completion = manager.ConsumeNormalCompletionIfReady();
  ASSERT_TRUE(completion.has_value());
  ASSERT_TRUE(protocol.request_return_to_stand(
      completion->playback_id, completion->terminal_frame_index, *accepted, measured, t0));

  EXPECT_EQ(
      protocol.update(measured, velocities, true, true, t0 + std::chrono::seconds(1)).phase,
      PlaybackPhase::RETURN_TO_STAND);
  EXPECT_EQ(
      protocol.update(
          defaults, velocities, true, true, t0 + std::chrono::milliseconds(1100)).phase,
      PlaybackPhase::RETURN_TO_STAND);
  EXPECT_EQ(
      protocol.update(
          defaults, velocities, true, true, t0 + std::chrono::milliseconds(1200)).phase,
      PlaybackPhase::STABLE_STANDING);
  EXPECT_EQ(
      protocol.update(
          defaults, velocities, true, false, t0 + std::chrono::milliseconds(1300)).phase,
      PlaybackPhase::FAULT);
}
