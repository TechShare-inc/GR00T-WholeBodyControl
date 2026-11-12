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

TEST(PlaybackProtocolTest, MatchingAbortReturnsToQualificationWithoutStoppingWbc) {
  const auto standing = MakeArray(1.0);
  PlaybackProtocol protocol;

  ASSERT_TRUE(protocol.start_action(17, standing));
  EXPECT_FALSE(protocol.abort_action(18));
  EXPECT_EQ(protocol.phase(), PlaybackPhase::ACTION);
  EXPECT_TRUE(protocol.abort_action(17));
  EXPECT_EQ(protocol.phase(), PlaybackPhase::RETURN_TO_STAND);
  EXPECT_EQ(protocol.active_playback_id(), 0);
  EXPECT_EQ(protocol.last_action_outcome(), ActionOutcome::CANCELLED);
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

TEST(PlaybackProtocolTest, InitialQualificationDoesNotUsePostActionDeadline) {
  const auto standing = MakeArray(1.0);
  const auto moving_velocity = MakeArray(0.2);
  PlaybackProtocol protocol(3.0, 0.05, 0.1, 0.25, 1.0, 5.0);
  const auto t0 = PlaybackProtocol::Clock::time_point{};

  protocol.begin_standing_qualification(t0);
  const auto still_qualifying = protocol.update(
      standing, moving_velocity, true, true,
      t0 + std::chrono::seconds(30), false);

  EXPECT_EQ(still_qualifying.phase, PlaybackPhase::RETURN_TO_STAND);
  EXPECT_FALSE(protocol.start_action(17, standing));
}

TEST(PlaybackProtocolTest, ReportsStandingGateDiagnostics) {
  const auto standing = MakeArray(1.0);
  const auto displaced = MakeArray(1.5);
  const auto moving_velocity = MakeArray(0.2);
  PlaybackProtocol protocol(3.0, 0.05, 0.1, 0.25, 1.0, 10.0);
  const auto t0 = PlaybackProtocol::Clock::time_point{};

  protocol.begin_standing_qualification(t0);
  protocol.update(standing, MakeArray(0.0), true, true, t0);
  const auto step = protocol.update(
      displaced, moving_velocity, true, true,
      t0 + std::chrono::seconds(4), false);

  EXPECT_TRUE(step.standing.active);
  EXPECT_DOUBLE_EQ(step.standing.elapsed_s, 4.0);
  EXPECT_DOUBLE_EQ(step.standing.max_abs_joint_velocity, 0.2);
  EXPECT_DOUBLE_EQ(step.standing.velocity_tolerance, 0.1);
  EXPECT_FALSE(step.standing.velocity_ok);
  EXPECT_DOUBLE_EQ(step.standing.max_recovery_displacement, 0.5);
  EXPECT_DOUBLE_EQ(step.standing.recovery_displacement_limit, 1.0);
  EXPECT_TRUE(step.standing.recovery_ok);
  EXPECT_FALSE(step.standing.body_ok);
  EXPECT_FALSE(step.standing.position_reference_available);
}

TEST(PlaybackProtocolTest, StandingBodyEnvelopeIncludesMeasuredPolicyEquilibrium) {
  constexpr double equilibrium_tilt_rad = 0.625;
  const std::array<double, 4> equilibrium_quaternion = {
      std::cos(equilibrium_tilt_rad / 2.0),
      0.0,
      std::sin(equilibrium_tilt_rad / 2.0),
      0.0};
  const std::array<double, 3> quiet_gyro = {0.01, 0.02, 0.01};

  const auto equilibrium = EvaluateStandingBody(equilibrium_quaternion, quiet_gyro);
  EXPECT_TRUE(equilibrium.ok);
  EXPECT_NEAR(equilibrium.tilt_rad, equilibrium_tilt_rad, 1e-9);
  EXPECT_DOUBLE_EQ(equilibrium.tilt_limit_rad, 0.70);

  constexpr double excessive_tilt_rad = 0.75;
  const std::array<double, 4> excessive_quaternion = {
      std::cos(excessive_tilt_rad / 2.0),
      0.0,
      std::sin(excessive_tilt_rad / 2.0),
      0.0};
  EXPECT_FALSE(EvaluateStandingBody(excessive_quaternion, quiet_gyro).ok);
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

TEST(PlaybackProtocolTest, UnsettledReturnTimesOutWithoutStoppingWbc) {
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
      PlaybackPhase::RETURN_TO_STAND);
  EXPECT_EQ(protocol.last_action_outcome(), ActionOutcome::STAND_RECOVERY_TIMEOUT);
  EXPECT_EQ(protocol.active_playback_id(), 0);
}

TEST(PlaybackProtocolTest, FreshStateDropsQualificationWithoutStoppingWbc) {
  const auto defaults = MakeArray(0.0);
  const auto measured = MakeArray(1.0);
  PlaybackProtocol protocol(3.0);
  const auto t0 = PlaybackProtocol::Clock::time_point{};

  ASSERT_TRUE(protocol.start_action(3, defaults));
  ASSERT_TRUE(protocol.request_return_to_stand(3, 4, 4, t0));
  const auto result = protocol.update(measured, MakeArray(0.0), false, true, t0 + std::chrono::milliseconds(20));
  EXPECT_EQ(result.phase, PlaybackPhase::RETURN_TO_STAND);
  EXPECT_EQ(protocol.phase(), PlaybackPhase::RETURN_TO_STAND);
  EXPECT_STREQ(protocol.phase_name(), "RETURN_TO_STAND");
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
  EXPECT_EQ(stale_low_state.phase, PlaybackPhase::RETURN_TO_STAND);
  EXPECT_EQ(protocol.phase(), PlaybackPhase::RETURN_TO_STAND);
}

TEST(PlaybackProtocolTest, ShortSoftGateViolationDoesNotDropStableStanding) {
  const auto standing = MakeArray(0.0);
  const auto quiet = MakeArray(0.0);
  const auto transient_velocity = MakeArray(0.2);
  PlaybackProtocol protocol(3.0, 0.05, 0.1, 0.25, 1.0, 5.0, 0.1);
  const auto t0 = PlaybackProtocol::Clock::time_point{};

  ASSERT_TRUE(protocol.start_action(17, standing));
  ASSERT_TRUE(protocol.request_return_to_stand(17, 10, 10, t0));
  ASSERT_EQ(
      protocol.update(standing, quiet, true, true,
                      t0 + std::chrono::seconds(3)).phase,
      PlaybackPhase::RETURN_TO_STAND);
  ASSERT_EQ(
      protocol.update(standing, quiet, true, true,
                      t0 + std::chrono::milliseconds(3250)).phase,
      PlaybackPhase::STABLE_STANDING);

  const auto transient = protocol.update(
      standing, transient_velocity, true, true,
      t0 + std::chrono::milliseconds(3270));
  EXPECT_EQ(transient.phase, PlaybackPhase::STABLE_STANDING);
  EXPECT_FALSE(transient.standing.velocity_ok);
  EXPECT_DOUBLE_EQ(transient.standing.violation_hold_elapsed_s, 0.0);

  const auto recovered = protocol.update(
      standing, quiet, true, true,
      t0 + std::chrono::milliseconds(3290));
  EXPECT_EQ(recovered.phase, PlaybackPhase::STABLE_STANDING);
  EXPECT_TRUE(recovered.standing.velocity_ok);
}

TEST(PlaybackProtocolTest, PersistentSoftGateViolationRestartsQualification) {
  const auto standing = MakeArray(0.0);
  const auto quiet = MakeArray(0.0);
  const auto moving = MakeArray(0.2);
  PlaybackProtocol protocol(3.0, 0.05, 0.1, 0.25, 1.0, 5.0, 0.1);
  const auto t0 = PlaybackProtocol::Clock::time_point{};

  ASSERT_TRUE(protocol.start_action(17, standing));
  ASSERT_TRUE(protocol.request_return_to_stand(17, 10, 10, t0));
  ASSERT_EQ(
      protocol.update(standing, quiet, true, true,
                      t0 + std::chrono::seconds(3)).phase,
      PlaybackPhase::RETURN_TO_STAND);
  ASSERT_EQ(
      protocol.update(standing, quiet, true, true,
                      t0 + std::chrono::milliseconds(3250)).phase,
      PlaybackPhase::STABLE_STANDING);

  EXPECT_EQ(
      protocol.update(standing, moving, true, true,
                      t0 + std::chrono::milliseconds(3270)).phase,
      PlaybackPhase::STABLE_STANDING);
  EXPECT_EQ(
      protocol.update(standing, moving, true, true,
                      t0 + std::chrono::milliseconds(3380)).phase,
      PlaybackPhase::RETURN_TO_STAND);
}

TEST(PlaybackProtocolTest, RejectsFramesFromAnOlderControllerEpoch) {
  const auto standing = MakeArray(1.0);
  PlaybackProtocol protocol;
  const auto t0 = PlaybackProtocol::Clock::time_point{};

  ASSERT_TRUE(protocol.reinitialize(2, t0));
  EXPECT_FALSE(protocol.start_action(1, 17, standing));
  protocol.update(standing, MakeArray(0.0), true, true, t0 + std::chrono::seconds(3));
  protocol.update(standing, MakeArray(0.0), true, true, t0 + std::chrono::milliseconds(3250));
  EXPECT_TRUE(protocol.start_action(2, 17, standing));
  EXPECT_EQ(protocol.controller_epoch(), 2);
}

TEST(PlaybackProtocolTest, TimeoutOutcomeIsNotRewrittenByLaterStability) {
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
      PlaybackPhase::RETURN_TO_STAND);
  EXPECT_EQ(protocol.last_action_outcome(), ActionOutcome::STAND_RECOVERY_TIMEOUT);
  protocol.update(standing, MakeArray(0.0), true, true,
                  t0 + std::chrono::seconds(9));
  EXPECT_EQ(protocol.last_action_outcome(), ActionOutcome::STAND_RECOVERY_TIMEOUT);
}

TEST(PlaybackProtocolTest, BodyGateEdgeStartsANewControllerEpoch) {
  const auto standing = MakeArray(0.0);
  const auto quiet = MakeArray(0.0);
  PlaybackProtocol protocol;
  const auto t0 = PlaybackProtocol::Clock::time_point{};

  EXPECT_EQ(
      protocol.update(standing, quiet, true, true, t0, true).phase,
      PlaybackPhase::STABLE_STANDING);
  EXPECT_EQ(
      protocol.update(standing, quiet, true, true,
                      t0 + std::chrono::milliseconds(20), false).phase,
      PlaybackPhase::RETURN_TO_STAND);
  EXPECT_EQ(protocol.controller_epoch(), 2);
  EXPECT_FALSE(protocol.start_action(1, 17, standing));
}

// Issue #32: the drift reference is captured after a settle window, then stays fixed.
namespace {

constexpr double kSettleS = 1.5;

// Reaches STABLE_STANDING after startup qualification with a settle window; returns the
// time it got there. The robot is quiet and at `pose` from t0.
PlaybackProtocol::Clock::time_point Qualify(
    PlaybackProtocol& protocol,
    const PlaybackProtocol::JointArray& pose,
    PlaybackProtocol::Clock::time_point t0) {
  protocol.begin_standing_qualification(t0);
  const auto quiet = MakeArray(0.0);
  for (int ms = 0; ms <= 10000; ms += 20) {
    const auto now = t0 + std::chrono::milliseconds(ms);
    if (protocol.update(pose, quiet, true, true, now).phase ==
        PlaybackPhase::STABLE_STANDING) {
      return now;
    }
  }
  return t0;
}

}  // namespace

TEST(PlaybackProtocolTest, DriftReferenceWaitsForTheSettleWindow) {
  PlaybackProtocol protocol(3.0, 0.05, 0.1, 0.25, 1.0, 10.0, 0.1, kSettleS);
  const auto t0 = PlaybackProtocol::Clock::time_point{};
  const auto quiet = MakeArray(0.0);
  const auto moving = MakeArray(0.5);
  protocol.begin_standing_qualification(t0);

  // Moving until 4 s: the 3 s minimum is over, but nothing is quiet yet.
  for (int ms = 0; ms < 4000; ms += 20) {
    protocol.update(MakeArray(0.0), moving, true, true, t0 + std::chrono::milliseconds(ms));
  }
  // Quiet from 4 s: the reference must not be taken before 4 s + 1.5 s.
  PlaybackPhase phase = PlaybackPhase::RETURN_TO_STAND;
  bool reference_before_settled = false;
  for (int ms = 4000; ms < 5500; ms += 20) {
    const auto step = protocol.update(
        MakeArray(0.0), quiet, true, true, t0 + std::chrono::milliseconds(ms));
    reference_before_settled |= step.standing.position_reference_available;
    phase = step.phase;
  }
  EXPECT_FALSE(reference_before_settled);
  EXPECT_EQ(phase, PlaybackPhase::RETURN_TO_STAND);

  PlaybackProtocol::StepResult last;
  for (int ms = 5500; ms < 6000; ms += 20) {
    last = protocol.update(
        MakeArray(0.0), quiet, true, true, t0 + std::chrono::milliseconds(ms));
  }
  EXPECT_EQ(last.phase, PlaybackPhase::STABLE_STANDING);
}

TEST(PlaybackProtocolTest, DriftPastToleranceStillRestartsQualification) {
  PlaybackProtocol protocol(3.0, 0.05, 0.1, 0.25, 1.0, 10.0, 0.1, kSettleS);
  const auto t0 = PlaybackProtocol::Clock::time_point{};
  const auto quiet = MakeArray(0.0);
  const auto stood = Qualify(protocol, MakeArray(0.0), t0);
  ASSERT_EQ(protocol.phase(), PlaybackPhase::STABLE_STANDING);

  // 0.04 rad from the reference: still standing.
  EXPECT_EQ(
      protocol.update(MakeArray(0.04), quiet, true, true,
                      stood + std::chrono::seconds(1)).phase,
      PlaybackPhase::STABLE_STANDING);
  // 0.06 rad for longer than the violation hold: back to RETURN_TO_STAND.
  protocol.update(MakeArray(0.06), quiet, true, true, stood + std::chrono::seconds(2));
  EXPECT_EQ(
      protocol.update(MakeArray(0.06), quiet, true, true,
                      stood + std::chrono::milliseconds(2200)).phase,
      PlaybackPhase::RETURN_TO_STAND);
}

TEST(PlaybackProtocolTest, DriftReferenceIsNotRecapturedWhileStanding) {
  PlaybackProtocol protocol(3.0, 0.05, 0.1, 0.25, 1.0, 10.0, 0.1, kSettleS);
  const auto t0 = PlaybackProtocol::Clock::time_point{};
  const auto quiet = MakeArray(0.0);
  const auto stood = Qualify(protocol, MakeArray(0.0), t0);
  ASSERT_EQ(protocol.phase(), PlaybackPhase::STABLE_STANDING);

  // Creep 0.002 rad/s for 30 s while quiet: the reference stays at 0, so the drift
  // reported keeps growing and passes 0.05 near 25 s.
  double last_drift = 0.0;
  bool dropped = false;
  for (int ms = 20; ms <= 30000 && !dropped; ms += 20) {
    const double pose = 0.002 * ms / 1000.0;
    const auto step = protocol.update(
        MakeArray(pose), quiet, true, true, stood + std::chrono::milliseconds(ms));
    if (step.phase == PlaybackPhase::STABLE_STANDING) {
      EXPECT_GE(step.standing.max_position_drift + 1e-9, last_drift);
      last_drift = step.standing.max_position_drift;
    } else {
      dropped = true;
      EXPECT_GT(ms, 25000);
    }
  }
  EXPECT_TRUE(dropped);
  EXPECT_GT(last_drift, 0.049);
}

TEST(PlaybackProtocolTest, SettleWindowAppliesAgainOnRequalification) {
  PlaybackProtocol protocol(3.0, 0.05, 0.1, 0.25, 1.0, 10.0, 0.1, kSettleS);
  const auto t0 = PlaybackProtocol::Clock::time_point{};
  const auto quiet = MakeArray(0.0);
  const auto moving = MakeArray(0.5);
  const auto stood = Qualify(protocol, MakeArray(0.0), t0);
  ASSERT_EQ(protocol.phase(), PlaybackPhase::STABLE_STANDING);

  // Drift past the gate: back to RETURN_TO_STAND.
  auto now = stood;
  while (protocol.phase() == PlaybackPhase::STABLE_STANDING) {
    now += std::chrono::milliseconds(20);
    protocol.update(MakeArray(0.06), quiet, true, true, now);
  }
  const auto restarted = now;

  // Moving for 4 s, then quiet: still no reference 1.4 s into the quiet time ...
  for (int ms = 20; ms < 4000; ms += 20) {
    protocol.update(MakeArray(0.06), moving, true, true,
                    restarted + std::chrono::milliseconds(ms));
  }
  PlaybackProtocol::StepResult step;
  for (int ms = 4000; ms < 5400; ms += 20) {
    step = protocol.update(MakeArray(0.06), quiet, true, true,
                           restarted + std::chrono::milliseconds(ms));
    EXPECT_FALSE(step.standing.position_reference_available);
  }
  // ... and standing again once it has been quiet for 1.5 s plus the hold.
  for (int ms = 5400; ms < 6000; ms += 20) {
    step = protocol.update(MakeArray(0.06), quiet, true, true,
                           restarted + std::chrono::milliseconds(ms));
  }
  EXPECT_EQ(step.phase, PlaybackPhase::STABLE_STANDING);
}
