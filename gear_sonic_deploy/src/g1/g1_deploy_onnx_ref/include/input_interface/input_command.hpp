/**
 * @file input_command.hpp
 * @brief Lightweight message structs used for inter-component communication
 *        between ZMQ subscribers and the input managers.
 *
 * Two message types are defined:
 *  - CommandMessage  – carries high-level control signals (start / stop /
 *                      planner-mode toggle) received on the ZMQ "command" topic.
 *  - PlannerMessage  – carries per-frame locomotion commands (mode, movement
 *                      direction, facing direction, speed, height, and optional
 *                      upper-body / hand data) received on the ZMQ "planner" topic.
 *
 * Both structs are plain-old-data (POD-like) value types designed to be written
 * under a mutex by a background ZMQ thread and read by the main control loop.
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <optional>

#include "../localmotion_kplanner.hpp"  // For LocomotionMode enum

// ---------------------------------------------------------------------------
// CommandMessage
// ---------------------------------------------------------------------------
/**
 * @brief Wire format for the ZMQ "command" topic.
 *
 * Packed binary layout sent by the remote controller:
 *   { start: bool, stop: bool, planner: bool, delta_heading?: f32/f64 }
 *
 * Multiple messages between two update() calls are accumulated using OR logic
 * for start/stop (so a transient pulse is never lost), while the planner flag
 * is overwritten with the latest value.
 */
struct CommandMessage {
  bool start = false;     ///< When true, request the control system to start.
  bool stop = false;      ///< When true, request an emergency / graceful stop.
  bool planner = false;   ///< true  → planner mode  (use planner topic for locomotion)
                          ///< false → streamed-motion mode  (use pose topic)
  bool playback_start = false;       ///< Begins a controller-correlated playback.
  bool normal_completion = false;    ///< Requests the controller-owned return-to-stand.
  bool playback_abort = false;       ///< Abnormally terminates playback into FAULT.
  int64_t playback_id = 0;           ///< Correlation ID for playback lifecycle markers.
  int64_t terminal_frame_index = -1; ///< Last frame that must be accepted before return.
  /// Optional absolute heading override (radians).  When set, the value is
  /// written directly into HeadingState.delta_heading.
  std::optional<double> delta_heading;
  bool valid = false;     ///< Set to true once a message has been decoded successfully.
};

// ---------------------------------------------------------------------------
// Controller-owned playback protocol
// ---------------------------------------------------------------------------
/**
 * @brief Safety state machine for streamed playback completion.
 *
 * A normal completion is accepted only after the controller has accepted the
 * matching terminal frame. The policy then runs from the restored planner IDLE
 * context for a minimum recovery interval and settling hold. Stale state
 * during the return fails closed into FAULT; stop and emergency paths are
 * outside this protocol and never request a stand.
 */
enum class PlaybackPhase {
  ACTION,
  RETURN_TO_STAND,
  STABLE_STANDING,
  FAULT,
};

inline const char* PlaybackPhaseName(PlaybackPhase phase) {
  switch (phase) {
    case PlaybackPhase::ACTION: return "ACTION";
    case PlaybackPhase::RETURN_TO_STAND: return "RETURN_TO_STAND";
    case PlaybackPhase::STABLE_STANDING: return "STABLE_STANDING";
    case PlaybackPhase::FAULT: return "FAULT";
  }
  return "FAULT";
}

struct StandingBodyDiagnostics {
  bool ok = false;
  double tilt_rad = -1.0;
  double tilt_limit_rad = 0.70;
  double max_abs_angular_velocity = -1.0;
  double angular_velocity_limit = 0.35;
};

inline StandingBodyDiagnostics EvaluateStandingBody(
    const std::array<double, 4>& quaternion_wxyz,
    const std::array<double, 3>& angular_velocity,
    double tilt_limit_rad = 0.70,
    double angular_velocity_limit = 0.35) {
  StandingBodyDiagnostics result;
  result.tilt_limit_rad = tilt_limit_rad;
  result.angular_velocity_limit = angular_velocity_limit;

  const double w = quaternion_wxyz[0];
  const double x = quaternion_wxyz[1];
  const double y = quaternion_wxyz[2];
  const double z = quaternion_wxyz[3];
  const double norm_squared = w * w + x * x + y * y + z * z;
  if (norm_squared <= 1e-12) {
    return result;
  }

  const double upright_alignment =
      1.0 - 2.0 * (x * x + y * y) / norm_squared;
  result.tilt_rad = std::acos(std::clamp(upright_alignment, -1.0, 1.0));
  result.max_abs_angular_velocity = std::max({
      std::abs(angular_velocity[0]),
      std::abs(angular_velocity[1]),
      std::abs(angular_velocity[2])});
  result.ok = result.tilt_rad <= result.tilt_limit_rad &&
      result.max_abs_angular_velocity <= result.angular_velocity_limit;
  return result;
}

class PlaybackProtocol {
 public:
  static constexpr size_t kJointCount = 29;
  using JointArray = std::array<double, kJointCount>;
  using Clock = std::chrono::steady_clock;

  struct StandingDiagnostics {
    bool active = false;
    double elapsed_s = -1.0;
    double max_abs_joint_velocity = -1.0;
    double velocity_tolerance = -1.0;
    bool velocity_ok = false;
    double max_recovery_displacement = -1.0;
    double recovery_displacement_limit = -1.0;
    bool recovery_ok = false;
    bool body_ok = false;
    bool position_reference_available = false;
    double max_position_drift = -1.0;
    double position_tolerance = -1.0;
    bool position_ok = false;
    double stable_hold_elapsed_s = 0.0;
    double stable_hold_required_s = -1.0;
    bool low_state_fresh = false;
    bool imu_fresh = false;
  };

  struct StepResult {
    PlaybackPhase phase = PlaybackPhase::STABLE_STANDING;
    StandingDiagnostics standing;
  };

  explicit PlaybackProtocol(
      double return_duration_s = 3.0,
      double position_tolerance = 0.05,
      double velocity_tolerance = 0.10,
      double stable_hold_s = 0.25,
      double max_recovery_displacement = 1.0,
      double return_timeout_s = 10.0)
      : return_duration_s_(std::max(0.001, return_duration_s)),
        position_tolerance_(std::max(0.0, position_tolerance)),
        velocity_tolerance_(std::max(0.0, velocity_tolerance)),
        stable_hold_s_(std::max(0.0, stable_hold_s)),
        max_recovery_displacement_(std::max(0.0, max_recovery_displacement)),
        return_timeout_s_(std::max(
            return_duration_s_ + stable_hold_s_, return_timeout_s)) {}

  bool start_action(int64_t playback_id, const JointArray& standing_position) {
    if (playback_id <= 0) {
      return false;
    }
    if (phase_ == PlaybackPhase::ACTION && playback_id == active_playback_id_) {
      return true;
    }
    if (phase_ != PlaybackPhase::STABLE_STANDING) {
      return false;
    }
    active_playback_id_ = playback_id;
    standing_reference_position_ = standing_position;
    phase_ = PlaybackPhase::ACTION;
    stable_since_.reset();
    settled_reference_position_.reset();
    qualification_reference_pending_ = false;
    initial_qualification_active_ = false;
    return true;
  }

  void begin_standing_qualification(Clock::time_point now = Clock::now()) {
    active_playback_id_ = 0;
    return_started_at_ = now;
    stable_since_.reset();
    settled_reference_position_.reset();
    qualification_reference_pending_ = true;
    initial_qualification_active_ = true;
    phase_ = PlaybackPhase::RETURN_TO_STAND;
  }

  bool request_return_to_stand(
      int64_t playback_id,
      int64_t terminal_frame_index,
      int64_t last_accepted_frame_index,
      Clock::time_point now = Clock::now()) {
    if (phase_ != PlaybackPhase::ACTION || playback_id <= 0 ||
        playback_id != active_playback_id_ || terminal_frame_index < 0 ||
        last_accepted_frame_index < terminal_frame_index) {
      return false;
    }
    return_started_at_ = now;
    stable_since_.reset();
    settled_reference_position_.reset();
    qualification_reference_pending_ = false;
    initial_qualification_active_ = false;
    phase_ = PlaybackPhase::RETURN_TO_STAND;
    return true;
  }

  bool abort_action(int64_t playback_id) {
    if (phase_ != PlaybackPhase::ACTION || playback_id <= 0 ||
        playback_id != active_playback_id_) {
      return false;
    }
    fault();
    return true;
  }

  StepResult update(
      const JointArray& measured_position,
      const JointArray& measured_velocity,
      bool low_state_fresh,
      bool imu_fresh,
      Clock::time_point now = Clock::now(),
      bool body_stable = true) {
    StepResult result;
    result.phase = phase_;

    if (phase_ == PlaybackPhase::STABLE_STANDING &&
        (!low_state_fresh || !imu_fresh)) {
      fault();
      result.phase = phase_;
      return result;
    }

    if (phase_ == PlaybackPhase::RETURN_TO_STAND) {
      result.standing.active = true;
      result.standing.elapsed_s =
          std::chrono::duration<double>(now - return_started_at_).count();
      result.standing.velocity_tolerance = velocity_tolerance_;
      result.standing.recovery_displacement_limit = max_recovery_displacement_;
      result.standing.position_tolerance = position_tolerance_;
      result.standing.stable_hold_required_s = stable_hold_s_;
      result.standing.low_state_fresh = low_state_fresh;
      result.standing.imu_fresh = imu_fresh;
      result.standing.body_ok = body_stable;
      result.standing.max_abs_joint_velocity = 0.0;
      for (size_t i = 0; i < kJointCount; ++i) {
        result.standing.max_abs_joint_velocity = std::max(
            result.standing.max_abs_joint_velocity,
            std::abs(measured_velocity[i]));
      }
      result.standing.velocity_ok =
          result.standing.max_abs_joint_velocity <= velocity_tolerance_;

      if (!low_state_fresh || !imu_fresh) {
        fault();
        result.phase = phase_;
        return result;
      }

      if (qualification_reference_pending_) {
        standing_reference_position_ = measured_position;
        qualification_reference_pending_ = false;
      }

      result.standing.max_recovery_displacement = 0.0;
      for (size_t i = 0; i < kJointCount; ++i) {
        result.standing.max_recovery_displacement = std::max(
            result.standing.max_recovery_displacement,
            std::abs(measured_position[i] - standing_reference_position_[i]));
      }
      result.standing.recovery_ok =
          result.standing.max_recovery_displacement <= max_recovery_displacement_;

      result.standing.position_reference_available =
          settled_reference_position_.has_value();
      if (settled_reference_position_.has_value()) {
        result.standing.max_position_drift = 0.0;
        for (size_t i = 0; i < kJointCount; ++i) {
          result.standing.max_position_drift = std::max(
              result.standing.max_position_drift,
              std::abs(measured_position[i] - (*settled_reference_position_)[i]));
        }
        result.standing.position_ok =
            result.standing.max_position_drift <= position_tolerance_;
      }
      if (stable_since_.has_value()) {
        result.standing.stable_hold_elapsed_s =
            std::chrono::duration<double>(now - *stable_since_).count();
      }

      const double elapsed = result.standing.elapsed_s;
      if (!initial_qualification_active_ && elapsed >= return_timeout_s_) {
        fault();
        result.phase = phase_;
        return result;
      }
      if (elapsed >= return_duration_s_) {
        const bool velocity_settled_and_recoverable =
            result.standing.velocity_ok && result.standing.recovery_ok;
        if (!velocity_settled_and_recoverable || !body_stable) {
          stable_since_.reset();
          settled_reference_position_.reset();
        } else if (!stable_since_.has_value()) {
          stable_since_ = now;
          settled_reference_position_ = measured_position;
        } else if (!settled_reference_position_.has_value()) {
          stable_since_ = now;
          settled_reference_position_ = measured_position;
        } else {
          const bool position_settled = result.standing.position_ok;
          if (!position_settled) {
            stable_since_ = now;
            settled_reference_position_ = measured_position;
          } else if (std::chrono::duration<double>(now - *stable_since_).count() >=
                     stable_hold_s_) {
            phase_ = PlaybackPhase::STABLE_STANDING;
            initial_qualification_active_ = false;
            result.phase = phase_;
          }
        }
      }
      result.phase = phase_;
    }
    return result;
  }

  void fault() {
    phase_ = PlaybackPhase::FAULT;
    stable_since_.reset();
    initial_qualification_active_ = false;
  }

  PlaybackPhase phase() const { return phase_; }
  int64_t active_playback_id() const { return active_playback_id_; }
  const char* phase_name() const { return PlaybackPhaseName(phase_); }

 private:
  JointArray standing_reference_position_{};
  double return_duration_s_;
  double position_tolerance_;
  double velocity_tolerance_;
  double stable_hold_s_;
  double max_recovery_displacement_;
  double return_timeout_s_;
  int64_t active_playback_id_ = 0;
  PlaybackPhase phase_ = PlaybackPhase::STABLE_STANDING;
  Clock::time_point return_started_at_{};
  std::optional<Clock::time_point> stable_since_;
  std::optional<JointArray> settled_reference_position_;
  bool qualification_reference_pending_ = false;
  bool initial_qualification_active_ = false;
};

// ---------------------------------------------------------------------------
// PlannerMessage
// ---------------------------------------------------------------------------
/**
 * @brief Wire format for the ZMQ "planner" topic.
 *
 * Required fields (must be present in every message):
 *   - mode      : int32  – LocomotionMode enum cast (IDLE, WALK, RUN, …)
 *   - movement  : float[3] – desired movement direction unit vector (x, y, z)
 *   - facing    : float[3] – desired facing direction unit vector  (x, y, z)
 *
 * Optional fields (may or may not be present):
 *   - speed              : float – desired locomotion speed (-1.0 = use default)
 *   - height             : float – desired body height      (-1.0 = use default)
 *   - upper_body_position: float[17] – target upper-body joint positions  (radians)
 *   - upper_body_velocity: float[17] – target upper-body joint velocities (rad/s)
 *   - left_hand_joints   : float[7]  – Dex3 left-hand joint positions
 *   - right_hand_joints  : float[7]  – Dex3 right-hand joint positions
 *
 * The `timestamp` field is set locally on receipt and used for timeout
 * detection (planner messages older than ~1 s are considered stale).
 */
struct PlannerMessage {
  bool valid = false;  ///< True once this struct contains a successfully decoded message.

  /// Locomotion mode (cast of LocomotionMode enum). Defaults to IDLE.
  int mode = static_cast<int>(LocomotionMode::IDLE);

  /// Desired movement direction as a 3D unit vector [x, y, z].
  /// Zeroed when the robot should stand still.
  std::array<double, 3> movement = {0.0, 0.0, 0.0};

  /// Desired facing direction as a 3D unit vector [x, y, z].
  /// Defaults to facing forward along the +X axis.
  std::array<double, 3> facing = {1.0, 0.0, 0.0};

  /// Optional upper-body joint target positions (17 DOF, radians).
  /// Present when the remote controller provides whole-body commands.
  std::optional<std::array<double, 17>> upper_body_position;

  /// Optional upper-body joint target velocities (17 DOF, rad/s).
  std::optional<std::array<double, 17>> upper_body_velocity;

  /// Optional left-hand Dex3 joint positions (7 DOF).
  std::optional<std::array<double, 7>> left_hand_joints;

  /// Optional right-hand Dex3 joint positions (7 DOF).
  std::optional<std::array<double, 7>> right_hand_joints;

  /// Desired locomotion speed.  -1.0 means "use the default for the current mode".
  double speed = -1.0;

  /// Desired body height.  -1.0 means "use the default for the current mode".
  double height = -1.0;

  /// Local steady-clock timestamp recorded when the message was received.
  /// Used to detect planner timeouts (stale data → fallback to IDLE).
  std::chrono::steady_clock::time_point timestamp{};
};
