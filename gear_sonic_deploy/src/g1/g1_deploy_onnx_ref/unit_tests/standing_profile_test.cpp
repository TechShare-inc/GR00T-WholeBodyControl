#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>

#include "input_interface/standing_profile.hpp"

namespace {

// Writes a complete schema-1 profile; `settle` is the optional field's JSON text or "".
std::string WriteProfile(const std::string& name, const std::string& settle) {
  const std::string path = ::testing::TempDir() + name;
  std::ofstream out(path);
  out << R"({"schema_version": 1, "minimum_recovery_interval_s": 3.0,
    "position_drift_tolerance_rad": 0.05, "joint_velocity_tolerance_rad_s": 0.1,
    "recovery_displacement_limit_rad": 1.0, "stable_hold_s": 0.25,
    "standing_violation_hold_s": 0.1, "torso_tilt_limit_rad": 0.7,
    "torso_angular_velocity_limit_rad_s": 0.35, "post_action_recovery_timeout_s": 10.0,
    "low_state_freshness_limit_s": 0.5, "imu_freshness_limit_s": 0.5)"
      << (settle.empty() ? "" : ", \"drift_reference_settle_s\": " + settle) << "}";
  return path;
}

}  // namespace

TEST(StandingProfileTest, SettleWindowDefaultsWhenTheFieldIsAbsent) {
  const auto profile = StandingProfile::Load(WriteProfile("absent.json", ""));
  EXPECT_DOUBLE_EQ(profile.drift_reference_settle_s, 1.5);
}

TEST(StandingProfileTest, SettleWindowInsideOneToTwoSecondsLoads) {
  EXPECT_DOUBLE_EQ(
      StandingProfile::Load(WriteProfile("one.json", "1.0")).drift_reference_settle_s, 1.0);
  EXPECT_DOUBLE_EQ(
      StandingProfile::Load(WriteProfile("two.json", "2.0")).drift_reference_settle_s, 2.0);
}

TEST(StandingProfileTest, SettleWindowOutsideOneToTwoSecondsIsRefused) {
  for (const char* value : {"0", "0.5", "0.99", "2.01", "10"}) {
    EXPECT_THROW(
        StandingProfile::Load(WriteProfile("bad.json", value)), std::runtime_error)
        << "value " << value;
  }
}
