#include "tienkung_policy_runner/robot_config.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace tienkung_policy_runner {
namespace {

template<typename T>
T required(const YAML::Node & node, const char * key)
{
  if (!node[key]) {
    throw std::runtime_error(std::string("Missing required config key: ") + key);
  }
  return node[key].as<T>();
}

template<typename T>
std::vector<T> required_vector(
  const YAML::Node & node, const char * key, std::size_t expected_size)
{
  auto values = required<std::vector<T>>(node, key);
  if (values.size() != expected_size) {
    throw std::runtime_error(
            std::string("Config key '") + key + "' has size " +
            std::to_string(values.size()) + ", expected " + std::to_string(expected_size));
  }
  return values;
}

void require_finite(const std::vector<float> & values, const char * name)
{
  if (!std::all_of(values.begin(), values.end(), [](float value) {return std::isfinite(value);})) {
    throw std::runtime_error(std::string("Config key '") + name + "' contains non-finite values");
  }
}

void validate_indices(
  const std::vector<int> & indices, std::size_t size, const char * name)
{
  std::set<int> unique;
  for (const int index : indices) {
    if (index < 0 || static_cast<std::size_t>(index) >= size) {
      throw std::runtime_error(std::string("Config key '") + name + "' contains invalid index");
    }
    if (!unique.insert(index).second) {
      throw std::runtime_error(std::string("Config key '") + name + "' contains duplicate index");
    }
  }
}

void require_near(float actual, float expected, const std::string & name)
{
  if (std::abs(actual - expected) > 1.0e-6F) {
    throw std::runtime_error(
            "Manifest " + name + "=" + std::to_string(actual) +
            " does not match deployment config " + std::to_string(expected));
  }
}

}  // namespace

RobotConfig load_robot_config(const std::string & path)
{
  if (path.empty()) {
    throw std::runtime_error("robot_config_path is required");
  }
  const auto root = YAML::LoadFile(path);
  RobotConfig config;
  config.robot = required<std::string>(root, "robot");
  config.motor_num = required<std::size_t>(root, "motor_num");
  config.actions_size = required<std::size_t>(root, "actions_size");
  config.policy_frequency_hz = required<double>(root, "policy_frequency_hz");
  if (config.motor_num == 0 || config.actions_size != config.motor_num) {
    throw std::runtime_error("motor_num must be positive and equal actions_size");
  }
  if (!(config.policy_frequency_hz > 0.0) || !std::isfinite(config.policy_frequency_hz)) {
    throw std::runtime_error("policy_frequency_hz must be finite and positive");
  }

  const auto observation = root["observation"];
  if (!observation) {
    throw std::runtime_error("Missing required config key: observation");
  }
  config.observation = {
    required<std::size_t>(observation, "n_mimic_obs"),
    required<std::size_t>(observation, "n_proprio"),
    required<std::size_t>(observation, "n_obs_single"),
    required<std::size_t>(observation, "history_len"),
    required<std::size_t>(observation, "total_obs_size"),
    required<float>(observation, "angular_velocity_scale"),
    required<float>(observation, "joint_velocity_scale")};

  config.joint_order_policy =
    required_vector<std::string>(root, "joint_order_policy", config.motor_num);
  config.ros_lite_name_by_index =
    required_vector<std::string>(root, "ros_lite_name_by_index", config.motor_num);
  config.can_id_by_index = required_vector<int>(root, "can_id_by_index", config.motor_num);
  config.leg_indices = required<std::vector<int>>(root, "leg_indices");
  config.arm_indices = required<std::vector<int>>(root, "arm_indices");
  config.default_dof_pos = required_vector<float>(root, "default_dof_pos", config.motor_num);
  config.action_scale = required<float>(root, "action_scale");
  config.clip_actions = required<float>(root, "clip_actions");
  config.joint_kp = required_vector<float>(root, "joint_kp_p", config.motor_num);
  config.joint_kd = required_vector<float>(root, "joint_kd_p", config.motor_num);
  config.zero_pos_offset = required_vector<float>(root, "zero_pos_offset", config.motor_num);
  config.zero_offset = required_vector<float>(root, "zero_offset", config.motor_num);
  config.motor_dir = required_vector<float>(root, "motor_dir", config.motor_num);
  config.ct_scale = required_vector<float>(root, "ct_scale", config.motor_num);

  const auto feedback = root["motor_feedback"];
  config.wrap_threshold_rad = required<float>(feedback, "wrap_threshold_rad");
  config.enable_jump_filter = required<bool>(feedback, "enable_jump_filter");

  const auto transmission = root["ankle_transmission"];
  config.ankle_transmission.kind = required<std::string>(transmission, "kind");
  config.ankle_transmission.motor_indices =
    required<std::vector<int>>(transmission, "motor_indices");
  config.ankle_transmission.joint_indices =
    required<std::vector<int>>(transmission, "joint_indices");
  config.ankle_transmission.require_native = transmission["require_native"] ?
    transmission["require_native"].as<bool>() : true;
  if (config.ankle_transmission.kind == "native_funcsptrans" &&
    !config.ankle_transmission.require_native)
  {
    throw std::runtime_error("native_funcsptrans must be fail-closed with require_native=true");
  }

  const auto waist = root["waist_command"];
  config.waist_command.ids = required<std::vector<int>>(root, "waist_ids");
  config.waist_command.position = required<float>(waist, "position");
  config.waist_command.speed = required<float>(waist, "speed");
  config.waist_command.current = required<float>(waist, "current");

  const auto fixed_arm = root["fixed_arm_command"];
  config.fixed_arm_command.ids = required<std::vector<int>>(fixed_arm, "ids");
  config.fixed_arm_command.position = required<float>(fixed_arm, "position");
  config.fixed_arm_command.kp = required<float>(fixed_arm, "kp");
  config.fixed_arm_command.kd = required<float>(fixed_arm, "kd");

  if (config.waist_command.ids.empty() ||
    !std::isfinite(config.waist_command.position) ||
    !(config.waist_command.speed >= 0.0F) ||
    !std::isfinite(config.waist_command.speed) ||
    !(config.waist_command.current >= 0.0F) ||
    !std::isfinite(config.waist_command.current))
  {
    throw std::runtime_error("waist command configuration is invalid");
  }

  if (config.fixed_arm_command.ids.empty() ||
    !std::isfinite(config.fixed_arm_command.position) ||
    !(config.fixed_arm_command.kp >= 0.0F) || !std::isfinite(config.fixed_arm_command.kp) ||
    !(config.fixed_arm_command.kd >= 0.0F) || !std::isfinite(config.fixed_arm_command.kd))
  {
    throw std::runtime_error("fixed arm command configuration is invalid");
  }

  const std::size_t expected_proprio = 3 + 2 + 3 * config.motor_num;
  const std::size_t expected_single = config.observation.n_mimic_obs + expected_proprio;
  const std::size_t expected_total = expected_single * (config.observation.history_len + 1) +
    config.observation.n_mimic_obs;
  if (config.observation.n_proprio != expected_proprio ||
    config.observation.n_obs_single != expected_single ||
    config.observation.total_obs_size != expected_total)
  {
    throw std::runtime_error("Observation dimensions are internally inconsistent");
  }
  if (!(config.action_scale > 0.0F) || !std::isfinite(config.action_scale)) {
    throw std::runtime_error("action_scale must be finite and positive");
  }
  const float raw_action_limit = config.clip_actions / config.action_scale;
  if (!(config.clip_actions > 0.0F) || !std::isfinite(config.clip_actions) ||
    !std::isfinite(raw_action_limit))
  {
    throw std::runtime_error(
            "clip_actions must be finite and positive, and clip_actions / action_scale must be finite");
  }
  if (!(config.observation.angular_velocity_scale >= 0.0F) ||
    !std::isfinite(config.observation.angular_velocity_scale) ||
    !(config.observation.joint_velocity_scale >= 0.0F) ||
    !std::isfinite(config.observation.joint_velocity_scale))
  {
    throw std::runtime_error("Observation scales must be finite and non-negative");
  }

  require_finite(config.default_dof_pos, "default_dof_pos");
  require_finite(config.joint_kp, "joint_kp_p");
  require_finite(config.joint_kd, "joint_kd_p");
  require_finite(config.zero_pos_offset, "zero_pos_offset");
  require_finite(config.zero_offset, "zero_offset");
  require_finite(config.motor_dir, "motor_dir");
  require_finite(config.ct_scale, "ct_scale");
  if (!(config.wrap_threshold_rad > 0.0F) || !std::isfinite(config.wrap_threshold_rad)) {
    throw std::runtime_error("motor_feedback.wrap_threshold_rad must be finite and positive");
  }
  if (std::any_of(config.ct_scale.begin(), config.ct_scale.end(), [](float value) {
      return value < 0.0F;
    }))
  {
    throw std::runtime_error("ct_scale entries must be non-negative");
  }
  if (std::any_of(config.joint_kp.begin(), config.joint_kp.end(), [](float value) {return value < 0.0F;}) ||
    std::any_of(config.joint_kd.begin(), config.joint_kd.end(), [](float value) {return value < 0.0F;}))
  {
    throw std::runtime_error("PD gains must be non-negative");
  }
  if (std::any_of(config.motor_dir.begin(), config.motor_dir.end(), [](float value) {
      return std::abs(std::abs(value) - 1.0F) > 1.0e-6F;
    }))
  {
    throw std::runtime_error("motor_dir entries must be either -1 or 1");
  }

  validate_indices(config.leg_indices, config.motor_num, "leg_indices");
  validate_indices(config.arm_indices, config.motor_num, "arm_indices");
  std::set<int> command_indices(config.leg_indices.begin(), config.leg_indices.end());
  for (const int index : config.arm_indices) {
    if (!command_indices.insert(index).second) {
      throw std::runtime_error("leg_indices and arm_indices must not overlap");
    }
  }
  if (command_indices.size() != config.motor_num) {
    throw std::runtime_error("leg_indices and arm_indices must cover every motor exactly once");
  }
  validate_indices(
    config.ankle_transmission.motor_indices, config.motor_num,
    "ankle_transmission.motor_indices");
  validate_indices(
    config.ankle_transmission.joint_indices, config.motor_num,
    "ankle_transmission.joint_indices");
  if (config.ankle_transmission.motor_indices.size() !=
    config.ankle_transmission.joint_indices.size())
  {
    throw std::runtime_error("Ankle motor and joint index counts must match");
  }
  if (config.ankle_transmission.kind == "native_funcsptrans" &&
    config.ankle_transmission.motor_indices.size() != 4)
  {
    throw std::runtime_error("native_funcsptrans requires exactly four ankle indices");
  }

  std::set<std::string> joint_names(
    config.joint_order_policy.begin(), config.joint_order_policy.end());
  std::set<std::string> ros_lite_names(
    config.ros_lite_name_by_index.begin(), config.ros_lite_name_by_index.end());
  if (joint_names.size() != config.motor_num || ros_lite_names.size() != config.motor_num) {
    throw std::runtime_error("Joint names and ros_lite names must be unique");
  }

  std::set<int> can_ids;
  for (std::size_t index = 0; index < config.can_id_by_index.size(); ++index) {
    const int can_id = config.can_id_by_index[index];
    if (!can_ids.insert(can_id).second) {
      throw std::runtime_error("can_id_by_index contains duplicate IDs");
    }
    config.index_by_can_id.emplace(can_id, index);
  }
  for (const int can_id : config.fixed_arm_command.ids) {
    if (!can_ids.insert(can_id).second) {
      throw std::runtime_error("fixed_arm_command.ids contains duplicate or managed CAN IDs");
    }
  }
  return config;
}

void validate_manifest(const std::string & path, const RobotConfig & config)
{
  if (path.empty()) {
    return;
  }
  const auto manifest = YAML::LoadFile(path);
  if (required<std::string>(manifest, "robot") != config.robot) {
    throw std::runtime_error("Manifest robot does not match deployment config");
  }
  const std::vector<std::pair<const char *, std::size_t>> dimensions{
    {"num_actions", config.actions_size},
    {"n_mimic_obs", config.observation.n_mimic_obs},
    {"n_proprio", config.observation.n_proprio},
    {"n_obs_single", config.observation.n_obs_single},
    {"history_len", config.observation.history_len},
    {"total_obs_size", config.observation.total_obs_size}};
  for (const auto & [key, expected] : dimensions) {
    if (required<std::size_t>(manifest, key) != expected) {
      throw std::runtime_error(std::string("Manifest field does not match config: ") + key);
    }
  }
  require_near(
    required<float>(manifest, "policy_frequency_hz"),
    static_cast<float>(config.policy_frequency_hz), "policy_frequency_hz");

  const auto order = required_vector<std::string>(
    manifest, "joint_order_policy", config.actions_size);
  if (order != config.joint_order_policy) {
    throw std::runtime_error("Manifest joint_order_policy does not match deployment config");
  }
  const auto defaults = required_vector<float>(manifest, "default_dof_pos", config.actions_size);
  const auto scales = required_vector<float>(manifest, "action_scale", config.actions_size);
  const auto ankle_indices = required<std::vector<int>>(manifest, "ankle_indices");
  if (ankle_indices != config.ankle_transmission.joint_indices) {
    throw std::runtime_error("Manifest ankle_indices does not match deployment config");
  }
  for (std::size_t index = 0; index < config.actions_size; ++index) {
    require_near(defaults[index], config.default_dof_pos[index], "default_dof_pos");
    require_near(scales[index], config.action_scale, "action_scale");
  }
}

}  // namespace tienkung_policy_runner