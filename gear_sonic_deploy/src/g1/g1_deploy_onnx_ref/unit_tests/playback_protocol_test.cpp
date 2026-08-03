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
  const auto standing = MakeArray(1.0);
  const auto action_pose = MakeArray(2.0);
  const auto velocities = MakeArray(0.0);
  PlaybackProtocol protocol(3.0, 0.05, 0.1, 0.25);
  const auto t0 = PlaybackProtocol::Clock::time_point{};

  ASSERT_TRUE(protocol.start_action(17, standing));
  EXPECT_FALSE(protocol.request_return_to_stand(18, 10, 10, t0));
  EXPECT_FALSE(protocol.request_return_to_stand(17, 10, 9, t0));
  ASSERT_TRUE(protocol.request_return_to_stand(17, 10, 10, t0));

  const auto halfway = protocol.update(
      action_pose, velocities, true, true, t0 + std::chrono::seconds(1));
  EXPECT_EQ(halfway.phase, PlaybackPhase::RETURN_TO_STAND);

  const auto minimum_return_done = protocol.update(
      standing, velocities, true, true, t0 + std::chrono::seconds(3));
  EXPECT_EQ(minimum_return_done.phase, PlaybackPhase::RETURN_TO_STAND);
  const auto stable = protocol.update(
      standing, velocities, true, true, t0 + std::chrono::milliseconds(3250));
  EXPECT_EQ(stable.phase, PlaybackPhase::STABLE_STANDING);
}

TEST(PlaybackProtocolTest, DuplicateActiveStartIsIdempotent) {
  const auto standing = MakeArray(1.0);
  PlaybackProtocol protocol;

  ASSERT_TRUE(protocol.start_action(17, standing));
  EXPECT_TRUE(protocol.start_action(17, MakeArray(2.0)));
  EXPECT_EQ(protocol.phase(), PlaybackPhase::ACTION);
  EXPECT_EQ(protocol.active_playback_id(), 17);
}

TEST(PlaybackProtocolTest, MatchingAbortFailsClosed) {
  const auto standing = MakeArray(1.0);
  PlaybackProtocol protocol;

  ASSERT_TRUE(protocol.start_action(17, standing));
  EXPECT_FALSE(protocol.abort_action(18));
  EXPECT_EQ(protocol.phase(), PlaybackPhase::ACTION);
  EXPECT_TRUE(protocol.abort_action(17));
  EXPECT_EQ(protocol.phase(), PlaybackPhase::FAULT);
}

TEST(PlaybackProtocolTest, InitialStandingMustBeQualifiedBeforeAction) {
  const auto standing = MakeArray(1.0);
  const auto velocities = MakeArray(0.0);
  PlaybackProtocol protocol(1.0, 0.05, 0.1, 0.1);
  const auto t0 = PlaybackProtocol::Clock::time_point{};

  protocol.begin_standing_qualification(t0);
  EXPECT_EQ(protocol.phase(), PlaybackPhase::RETURN_TO_STAND);
  EXPECT_FALSE(protocol.start_action(17, standing));
  EXPECT_EQ(
      protocol.update(standing, velocities, true, true, t0 + std::chrono::seconds(1)).phase,
      PlaybackPhase::RETURN_TO_STAND);
  EXPECT_EQ(
      protocol.update(
          standing, velocities, true, true, t0 + std::chrono::milliseconds(1100)).phase,
      PlaybackPhase::STABLE_STANDING);
  EXPECT_TRUE(protocol.start_action(17, standing));
}

TEST(PlaybackProtocolTest, AcceptsQuietPolicyEquilibriumAfterMinimumReturn) {
  const auto standing = MakeArray(1.0);
  const auto recovered_equilibrium = MakeArray(1.08);
  const auto velocities = MakeArray(0.0);
  PlaybackProtocol protocol(3.0, 0.05, 0.1, 0.25);
  const auto t0 = PlaybackProtocol::Clock::time_point{};

  ASSERT_TRUE(protocol.start_action(17, standing));
  ASSERT_TRUE(protocol.request_return_to_stand(17, 10, 10, t0));

  EXPECT_EQ(
      protocol.update(
          recovered_equilibrium, velocities, true, true,
          t0 + std::chrono::seconds(3)).phase,
      PlaybackPhase::RETURN_TO_STAND);
  EXPECT_EQ(
      protocol.update(
          recovered_equilibrium, velocities, true, true,
          t0 + std::chrono::milliseconds(3250)).phase,
      PlaybackPhase::STABLE_STANDING);
}

TEST(PlaybackProtocolTest, QuietButTiltedStateIsNotStableStanding) {
  const auto standing = MakeArray(0.0);
  const auto velocities = MakeArray(0.0);
  PlaybackProtocol protocol(3.0, 0.05, 0.1, 0.25);
  const auto t0 = PlaybackProtocol::Clock::time_point{};

  ASSERT_TRUE(protocol.start_action(17, standing));
  ASSERT_TRUE(protocol.request_return_to_stand(17, 10, 10, t0));

  EXPECT_EQ(
      protocol.update(
          standing, velocities, true, true,
          t0 + std::chrono::seconds(3), false).phase,
      PlaybackPhase::RETURN_TO_STAND);
  EXPECT_EQ(
      protocol.update(
          standing, velocities, true, true,
          t0 + std::chrono::milliseconds(3250), false).phase,
      PlaybackPhase::RETURN_TO_STAND);
}

TEST(PlaybackProtocolTest, UnsettledReturnFaultsAfterDeadline) {
  const auto standing = MakeArray(0.0);
  const auto moving_velocity = MakeArray(0.2);
  PlaybackProtocol protocol(3.0, 0.05, 0.1, 0.25, 1.0, 5.0);
  const auto t0 = PlaybackProtocol::Clock::time_point{};

  ASSERT_TRUE(protocol.start_action(17, standing));
  ASSERT_TRUE(protocol.request_return_to_stand(17, 10, 10, t0));

  EXPECT_EQ(
      protocol.update(
          standing, moving_velocity, true, true,
          t0 + std::chrono::milliseconds(5100)).phase,
      PlaybackPhase::FAULT);
}

TEST(PlaybackProtocolTest, FreshStateIsRequiredAndFaultDoesNotStand) {
  const auto defaults = MakeArray(0.0);
  const auto measured = MakeArray(1.0);
  PlaybackProtocol protocol(3.0);
  const auto t0 = PlaybackProtocol::Clock::time_point{};

  ASSERT_TRUE(protocol.start_action(3, defaults));
  ASSERT_TRUE(protocol.request_return_to_stand(3, 4, 4, t0));
  const auto result = protocol.update(measured, MakeArray(0.0), false, true, t0 + std::chrono::milliseconds(20));
  EXPECT_EQ(result.phase, PlaybackPhase::FAULT);
  EXPECT_EQ(protocol.phase(), PlaybackPhase::FAULT);
  EXPECT_STREQ(protocol.phase_name(), "FAULT");
}

TEST(PlaybackProtocolTest, StableStandingLosesValidityWhenTelemetryTurnsStale) {
  const auto defaults = MakeArray(0.0);
  const auto measured = MakeArray(1.0);
  const auto velocities = MakeArray(0.0);
  PlaybackProtocol protocol(3.0, 0.05, 0.1, 0.25);
  const auto t0 = PlaybackProtocol::Clock::time_point{};

  ASSERT_TRUE(protocol.start_action(9, defaults));
  ASSERT_TRUE(protocol.request_return_to_stand(9, 12, 12, t0));
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
