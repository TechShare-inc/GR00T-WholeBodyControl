#include <set>
#include <string>

#include <gtest/gtest.h>
#include <msgpack.hpp>

#include "utils.hpp"
#include "robot_parameters.hpp"
#include "output_interface/zmq_output_handler.hpp"

namespace {

msgpack::object_handle PackAndUnpack(msgpack::sbuffer& sbuf, std::size_t& offset) {
  OutputInterface::ControlStatus status;
  status.instance_id = "wbc-test";
  status.playback_phase = "STABLE_STANDING";
  status.controller_epoch = 3;
  ZMQOutputHandler::PackControlStatus(status, 42, 1234.5, sbuf);
  offset = 0;
  return msgpack::unpack(sbuf.data(), sbuf.size(), offset);
}

}  // namespace

TEST(ControlStatusPack, DeclaredFieldCountMatchesEncodedKeys) {
  msgpack::sbuffer sbuf;
  std::size_t offset = 0;
  auto handle = PackAndUnpack(sbuf, offset);
  const msgpack::object& root = handle.get();

  ASSERT_EQ(root.type, msgpack::type::MAP);
  EXPECT_EQ(root.via.map.size, ZMQOutputHandler::kControlStatusFieldCount);
  // A count that disagrees with the packed pairs would leave trailing bytes.
  EXPECT_EQ(offset, sbuf.size());

  std::set<std::string> keys;
  for (uint32_t i = 0; i < root.via.map.size; ++i) {
    ASSERT_EQ(root.via.map.ptr[i].key.type, msgpack::type::STR);
    keys.insert(root.via.map.ptr[i].key.as<std::string>());
  }
  EXPECT_EQ(keys.size(), ZMQOutputHandler::kControlStatusFieldCount);
}

TEST(ControlStatusPack, CarriesSequenceEpochAndControllerSourceTime) {
  msgpack::sbuffer sbuf;
  std::size_t offset = 0;
  auto handle = PackAndUnpack(sbuf, offset);
  const msgpack::object& root = handle.get();

  bool saw_time = false;
  bool saw_sequence = false;
  bool saw_epoch = false;
  for (uint32_t i = 0; i < root.via.map.size; ++i) {
    const auto key = root.via.map.ptr[i].key.as<std::string>();
    const auto& value = root.via.map.ptr[i].val;
    if (key == "source_monotonic_s") {
      EXPECT_DOUBLE_EQ(value.as<double>(), 1234.5);
      saw_time = true;
    } else if (key == "heartbeat_sequence") {
      EXPECT_EQ(value.as<uint64_t>(), 42u);
      saw_sequence = true;
    } else if (key == "controller_epoch") {
      EXPECT_EQ(value.as<int64_t>(), 3);
      saw_epoch = true;
    }
  }
  EXPECT_TRUE(saw_time);
  EXPECT_TRUE(saw_sequence);
  EXPECT_TRUE(saw_epoch);
}
