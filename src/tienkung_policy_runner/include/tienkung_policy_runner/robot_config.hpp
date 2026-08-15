#pragma once

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace tienkung_policy_runner {

struct ObservationConfig {
  std::size_t n_mimic_obs{};
  std::size_t n_proprio{};
  std::size_t n_obs_single{};
  std::size_t history_len{};
  std::size_t total_obs_size{};
  float angular_velocity_scale{};
  float joint_velocity_scale{};
};

struct AnkleTransmissionConfig {
  std::string kind;
  std::vector<int> motor_indices;
  std::vector<int> joint_indices;
  bool require_native{true};
};

struct WaistCommandConfig {
  std::vector<int> ids;
  float position{};
  float speed{};
  float current{};
};

struct FixedArmCommandConfig {
  std::vector<int> ids;
  float position{};
  float kp{};
  float kd{};
};

struct RobotConfig {
  std::string robot;
  std::size_t motor_num{};
  std::size_t actions_size{};
  double policy_frequency_hz{};
  ObservationConfig observation;
  std::vector<std::string> joint_order_policy;
  std::vector<std::string> ros_lite_name_by_index;
  std::vector<int> can_id_by_index;
  std::unordered_map<int, std::size_t> index_by_can_id;
  std::vector<int> leg_indices;
  std::vector<int> arm_indices;
  std::vector<float> default_dof_pos;
  float action_scale{};
  float clip_actions{};
  std::vector<float> joint_kp;
  std::vector<float> joint_kd;
  std::vector<float> zero_pos_offset;
  std::vector<float> zero_offset;
  std::vector<float> motor_dir;
  std::vector<float> ct_scale;
  float wrap_threshold_rad{};
  bool enable_jump_filter{true};
  AnkleTransmissionConfig ankle_transmission;
  WaistCommandConfig waist_command;
  FixedArmCommandConfig fixed_arm_command;
};

RobotConfig load_robot_config(const std::string & path);
void validate_manifest(const std::string & path, const RobotConfig & config);

}  // namespace tienkung_policy_runner