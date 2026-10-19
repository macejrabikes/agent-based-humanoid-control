// Parametre politiky "postav sa zo zeme" (Unitree G1, 29 klbov). GENEROVANE z getup_policy_meta.json
// (mjlab, uloha Mjlab-GetUp-Unitree-G1, checkpoint 999). Poradie klbov = poradie politiky.
#pragma once

#include <array>
#include <string>
#include <vector>

namespace humanoid_walk
{
namespace getup_params
{
constexpr int kJoints = 29;
constexpr int kObsDim = 93;
const std::vector<std::string> kNames = {
  "left_hip_pitch_joint",
  "left_hip_roll_joint",
  "left_hip_yaw_joint",
  "left_knee_joint",
  "left_ankle_pitch_joint",
  "left_ankle_roll_joint",
  "right_hip_pitch_joint",
  "right_hip_roll_joint",
  "right_hip_yaw_joint",
  "right_knee_joint",
  "right_ankle_pitch_joint",
  "right_ankle_roll_joint",
  "waist_yaw_joint",
  "waist_roll_joint",
  "waist_pitch_joint",
  "left_shoulder_pitch_joint",
  "left_shoulder_roll_joint",
  "left_shoulder_yaw_joint",
  "left_elbow_joint",
  "left_wrist_roll_joint",
  "left_wrist_pitch_joint",
  "left_wrist_yaw_joint",
  "right_shoulder_pitch_joint",
  "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint",
  "right_elbow_joint",
  "right_wrist_roll_joint",
  "right_wrist_pitch_joint",
  "right_wrist_yaw_joint"};
constexpr std::array<float, kJoints> kDefault = {-0.312f, 0.0f, 0.0f, 0.669f, -0.363f, 0.0f, -0.312f, 0.0f, 0.0f, 0.669f, -0.363f, 0.0f, 0.0f, 0.0f, 0.0f, 0.2f, 0.2f, 0.0f, 0.6f, 0.0f, 0.0f, 0.0f, 0.2f, -0.2f, 0.0f, 0.6f, 0.0f, 0.0f, 0.0f};
constexpr std::array<float, kJoints> kScale = {0.68443f, 0.43833f, 0.68443f, 0.43833f, 0.54822f, 0.54822f, 0.68443f, 0.43833f, 0.68443f, 0.43833f, 0.54822f, 0.54822f, 0.68443f, 0.54822f, 0.54822f, 0.54822f, 0.54822f, 0.54822f, 0.54822f, 0.54822f, 0.09313f, 0.09313f, 0.54822f, 0.54822f, 0.54822f, 0.54822f, 0.54822f, 0.09313f, 0.09313f};
constexpr std::array<float, kJoints> kKp = {40.179f, 99.098f, 40.179f, 99.098f, 28.501f, 28.501f, 40.179f, 99.098f, 40.179f, 99.098f, 28.501f, 28.501f, 40.179f, 28.501f, 28.501f, 14.251f, 14.251f, 14.251f, 14.251f, 14.251f, 16.778f, 16.778f, 14.251f, 14.251f, 14.251f, 14.251f, 14.251f, 16.778f, 16.778f};
constexpr std::array<float, kJoints> kKd = {2.558f, 6.309f, 2.558f, 6.309f, 1.814f, 1.814f, 2.558f, 6.309f, 2.558f, 6.309f, 1.814f, 1.814f, 2.558f, 1.814f, 1.814f, 0.907f, 0.907f, 0.907f, 0.907f, 0.907f, 1.068f, 1.068f, 0.907f, 0.907f, 0.907f, 0.907f, 0.907f, 1.068f, 1.068f};
constexpr std::array<float, kJoints> kEffort = {88.0f, 139.0f, 88.0f, 139.0f, 50.0f, 50.0f, 88.0f, 139.0f, 88.0f, 139.0f, 50.0f, 50.0f, 88.0f, 50.0f, 50.0f, 25.0f, 25.0f, 25.0f, 25.0f, 25.0f, 5.0f, 5.0f, 25.0f, 25.0f, 25.0f, 25.0f, 25.0f, 5.0f, 5.0f};
}  // namespace getup_params
}  // namespace humanoid_walk
