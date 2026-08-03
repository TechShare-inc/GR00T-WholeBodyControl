#include <gtest/gtest.h>

#include "input_interface/input_command.hpp"

namespace {

PlaybackProtocol::JointArray MakeArray(double value) {
  PlaybackProtocol::JointArray result{};
  result.fill(value);
  return result;
}

}  // namespace

TEST(PlaybackProtocolTest, RequiresMatchingAcceptedTerminalAndSettlingHold) {
  const auto defaults = MakeArray(0.0);
  const auto measured = MakeArray(1.0);
  const auto velocities = MakeArray(0.0);
  PlaybackProtocol protocol(defaults, 3.0, 0.05, 0.1, 0.25);
  const auto t0 = PlaybackProtocol::Clock::time_point{};

  protocol.start_action(17);
  EXPECT_FALSE(protocol.request_return_to_stand(18, 10, 10, measured, t0));
  EXPECT_FALSE(protocol.request_return_to_stand(17, 10, 9, measured, t0));
  ASSERT_TRUE(protocol.request_return_to_stand(17, 10, 10, measured, t0));

  const auto halfway = protocol.update(measured, velocities, true, true, t0 + std::chrono::seconds(1));
  EXPECT_EQ(halfway.phase, PlaybackPhase::RETURN_TO_STAND);
  EXPECT_NEAR(halfway.target_position[0], 2.0 / 3.0, 1e-9);
  EXPECT_EQ(halfway.target_velocity[0], 0.0);
  EXPECT_EQ(halfway.feed_forward[0], 0.0);

  const auto ramp_done = protocol.update(defaults, velocities, true, true, t0 + std::chrono::seconds(3));
  EXPECT_EQ(ramp_done.phase, PlaybackPhase::RETURN_TO_STAND);
  const auto stable = protocol.update(defaults, velocities, true, true, t0 + std::chrono::milliseconds(3250));
  EXPECT_EQ(stable.phase, PlaybackPhase::STABLE_STANDING);
}

TEST(PlaybackProtocolTest, FreshStateIsRequiredAndFaultDoesNotStand) {
  const auto defaults = MakeArray(0.0);
  const auto measured = MakeArray(1.0);
  PlaybackProtocol protocol(defaults, 3.0);
  const auto t0 = PlaybackProtocol::Clock::time_point{};

  protocol.start_action(3);
  ASSERT_TRUE(protocol.request_return_to_stand(3, 4, 4, measured, t0));
  const auto result = protocol.update(measured, MakeArray(0.0), false, true, t0 + std::chrono::milliseconds(20));
  EXPECT_EQ(result.phase, PlaybackPhase::FAULT);
  EXPECT_EQ(protocol.phase(), PlaybackPhase::FAULT);
  EXPECT_STREQ(protocol.phase_name(), "FAULT");
}

TEST(PlaybackProtocolTest, StableStandingLosesValidityWhenTelemetryTurnsStale) {
  const auto defaults = MakeArray(0.0);
  const auto measured = MakeArray(1.0);
  const auto velocities = MakeArray(0.0);
  PlaybackProtocol protocol(defaults, 3.0, 0.05, 0.1, 0.25);
  const auto t0 = PlaybackProtocol::Clock::time_point{};

  protocol.start_action(9);
  ASSERT_TRUE(protocol.request_return_to_stand(9, 12, 12, measured, t0));
  ASSERT_EQ(
      protocol.update(defaults, velocities, true, true, t0 + std::chrono::seconds(3)).phase,
      PlaybackPhase::RETURN_TO_STAND);
  ASSERT_EQ(
      protocol.update(defaults, velocities, true, true, t0 + std::chrono::milliseconds(3250)).phase,
      PlaybackPhase::STABLE_STANDING);

  const auto stale_low_state = protocol.update(
      defaults, velocities, false, true, t0 + std::chrono::milliseconds(3300));
  EXPECT_EQ(stale_low_state.phase, PlaybackPhase::FAULT);
  EXPECT_EQ(protocol.phase(), PlaybackPhase::FAULT);
}
