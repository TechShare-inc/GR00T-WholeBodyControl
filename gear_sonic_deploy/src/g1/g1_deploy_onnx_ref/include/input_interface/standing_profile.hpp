/** Versioned standing qualification values shared with agentic-sonic. */
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace standing_profile_detail {

inline uint32_t rotate_right(uint32_t value, uint32_t bits) {
  return (value >> bits) | (value << (32U - bits));
}

inline std::string sha256_hex(const std::string& input) {
  constexpr std::array<uint32_t, 64> round_constants = {
      0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
      0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
      0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
      0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
      0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
      0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
      0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
      0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
      0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
      0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
      0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
      0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
      0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
      0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
      0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
      0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
  std::vector<uint8_t> message(input.begin(), input.end());
  const uint64_t bit_length = static_cast<uint64_t>(message.size()) * 8U;
  message.push_back(0x80U);
  while (message.size() % 64U != 56U) {
    message.push_back(0U);
  }
  for (int shift = 56; shift >= 0; shift -= 8) {
    message.push_back(static_cast<uint8_t>(bit_length >> shift));
  }

  std::array<uint32_t, 8> hash = {
      0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
      0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  for (size_t offset = 0; offset < message.size(); offset += 64U) {
    std::array<uint32_t, 64> schedule{};
    for (size_t index = 0; index < 16U; ++index) {
      const size_t base = offset + index * 4U;
      schedule[index] = (static_cast<uint32_t>(message[base]) << 24U) |
          (static_cast<uint32_t>(message[base + 1U]) << 16U) |
          (static_cast<uint32_t>(message[base + 2U]) << 8U) |
          static_cast<uint32_t>(message[base + 3U]);
    }
    for (size_t index = 16U; index < 64U; ++index) {
      const uint32_t value_a = schedule[index - 15U];
      const uint32_t value_b = schedule[index - 2U];
      const uint32_t small_a = rotate_right(value_a, 7U) ^
          rotate_right(value_a, 18U) ^ (value_a >> 3U);
      const uint32_t small_b = rotate_right(value_b, 17U) ^
          rotate_right(value_b, 19U) ^ (value_b >> 10U);
      schedule[index] = schedule[index - 16U] + small_a +
          schedule[index - 7U] + small_b;
    }

    uint32_t a = hash[0];
    uint32_t b = hash[1];
    uint32_t c = hash[2];
    uint32_t d = hash[3];
    uint32_t e = hash[4];
    uint32_t f = hash[5];
    uint32_t g = hash[6];
    uint32_t h = hash[7];
    for (size_t index = 0; index < 64U; ++index) {
      const uint32_t big_a = rotate_right(a, 2U) ^ rotate_right(a, 13U) ^
          rotate_right(a, 22U);
      const uint32_t choice = (e & f) ^ ((~e) & g);
      const uint32_t big_b = rotate_right(e, 6U) ^ rotate_right(e, 11U) ^
          rotate_right(e, 25U);
      const uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      const uint32_t temp1 = h + big_b + choice + round_constants[index] +
          schedule[index];
      const uint32_t temp2 = big_a + majority;
      h = g;
      g = f;
      f = e;
      e = d + temp1;
      d = c;
      c = b;
      b = a;
      a = temp1 + temp2;
    }
    hash[0] += a;
    hash[1] += b;
    hash[2] += c;
    hash[3] += d;
    hash[4] += e;
    hash[5] += f;
    hash[6] += g;
    hash[7] += h;
  }

  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (uint32_t value : hash) {
    output << std::setw(8) << value;
  }
  return output.str();
}

}  // namespace standing_profile_detail

struct StandingProfile {
  int schema_version = 1;
  double minimum_recovery_interval_s = 3.0;
  double position_drift_tolerance_rad = 0.05;
  double joint_velocity_tolerance_rad_s = 0.10;
  double recovery_displacement_limit_rad = 1.0;
  double stable_hold_s = 0.25;
  double standing_violation_hold_s = 0.1;
  double torso_tilt_limit_rad = 0.70;
  double torso_angular_velocity_limit_rad_s = 0.35;
  double post_action_recovery_timeout_s = 10.0;
  double low_state_freshness_limit_s = 0.5;
  double imu_freshness_limit_s = 0.5;
  std::string digest = "f3e24e130eb8a9ade8f028b32034a24995dfe2c9b261d3ff6241464ba2668031";

  static StandingProfile Load(const std::string& path) {
    if (path.empty()) {
      return StandingProfile{};
    }
    std::ifstream input(path);
    if (!input) {
      throw std::runtime_error("Cannot open standing profile: " + path);
    }
    const std::string raw(
        (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    const auto json = nlohmann::json::parse(raw);
    StandingProfile profile;
    profile.schema_version = json.at("schema_version").get<int>();
    if (profile.schema_version != 1) {
      throw std::runtime_error("Unsupported standing profile schema");
    }
    const auto number = [&json](const char* name) {
      const double value = json.at(name).get<double>();
      if (!std::isfinite(value) || value < 0.0) {
        throw std::runtime_error(std::string("Standing profile field must be finite and non-negative: ") + name);
      }
      return value;
    };
    profile.minimum_recovery_interval_s = number("minimum_recovery_interval_s");
    profile.position_drift_tolerance_rad = number("position_drift_tolerance_rad");
    profile.joint_velocity_tolerance_rad_s = number("joint_velocity_tolerance_rad_s");
    profile.recovery_displacement_limit_rad = number("recovery_displacement_limit_rad");
    profile.stable_hold_s = number("stable_hold_s");
    profile.standing_violation_hold_s = number("standing_violation_hold_s");
    profile.torso_tilt_limit_rad = number("torso_tilt_limit_rad");
    profile.torso_angular_velocity_limit_rad_s = number("torso_angular_velocity_limit_rad_s");
    profile.post_action_recovery_timeout_s = number("post_action_recovery_timeout_s");
    profile.low_state_freshness_limit_s = number("low_state_freshness_limit_s");
    profile.imu_freshness_limit_s = number("imu_freshness_limit_s");
    profile.digest = standing_profile_detail::sha256_hex(raw);
    return profile;
  }
};
