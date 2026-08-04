#include <array>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "motion_data_reader.hpp"
#include "input_interface/streamed_motion_merger.hpp"

namespace {

StreamedMotionMerger::IncomingData MakeMotion(int protocol_version) {
  constexpr int kFrames = 2;
  constexpr int kJoints = 29;
  constexpr int kSmplJoints = 24;
  constexpr int kSmplPoses = 21;

  StreamedMotionMerger::IncomingData data;
  data.protocol_version = protocol_version;
  data.num_frames = kFrames;
  data.num_joints = kJoints;
  data.num_quat_bodies = 1;
  data.num_smpl_joints = kSmplJoints;
  data.num_smpl_poses = kSmplPoses;
  data.frame_indices = {10, 11};
  data.joint_pos.assign(kFrames, std::vector<double>(kJoints, 0.1));
  data.joint_vel.assign(kFrames, std::vector<double>(kJoints, 0.0));
  data.body_quat.assign(
      kFrames, std::vector<std::array<double, 4>>(1, {1.0, 0.0, 0.0, 0.0}));
  data.smpl_joints.assign(
      kFrames,
      std::vector<std::array<double, 3>>(kSmplJoints, {0.0, 0.0, 0.0}));
  data.smpl_pose.assign(
      kFrames,
      std::vector<std::array<double, 3>>(kSmplPoses, {0.0, 0.0, 0.0}));
  return data;
}

}  // namespace

TEST(StreamedMotionMergerTest, AcceptsProtocolV4MotionPayload) {
  StreamedMotionMerger merger;

  const auto result = merger.MergeIncomingData(MakeMotion(4), 0);

  ASSERT_NE(result.motion, nullptr);
  EXPECT_EQ(result.protocol_version, 4);
  EXPECT_EQ(result.window_start, 10);
  EXPECT_EQ(result.motion->timesteps, 2);
  EXPECT_EQ(result.motion->GetNumJoints(), 29);
  EXPECT_EQ(result.motion->GetNumSmplJoints(), 24);
  EXPECT_EQ(result.motion->GetNumSmplPoses(), 21);
  EXPECT_DOUBLE_EQ(result.motion->JointPositions(1)[0], 0.1);
}
