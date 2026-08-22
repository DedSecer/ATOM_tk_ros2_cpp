#include "tienkung_policy_runner/observation_builder.hpp"

#include <algorithm>
#include <stdexcept>

namespace tienkung_policy_runner {
namespace {

Eigen::Map<const Eigen::VectorXf> map_vector(const std::vector<float> & values)
{
  return {values.data(), static_cast<Eigen::Index>(values.size())};
}

void require_size(const Eigen::VectorXf & value, std::size_t expected, const char * name)
{
  if (value.size() != static_cast<Eigen::Index>(expected)) {
    throw std::runtime_error(
            std::string(name) + " has size " + std::to_string(value.size()) +
            ", expected " + std::to_string(expected));
  }
}

}  // namespace

ObservationBuilder::ObservationBuilder(const RobotConfig & config)
: config_(config)
{
}

void ObservationBuilder::reset()
{
  history_.clear();
}

Eigen::VectorXf ObservationBuilder::build_proprio(
  const Eigen::Vector3f & angular_velocity,
  const Eigen::Vector3f & rpy,
  const Eigen::VectorXf & dof_position,
  const Eigen::VectorXf & dof_velocity,
  const Eigen::VectorXf & last_action) const
{
  require_size(dof_position, config_.motor_num, "dof_position");
  require_size(dof_velocity, config_.motor_num, "dof_velocity");
  require_size(last_action, config_.actions_size, "last_action");
  Eigen::VectorXf proprio(config_.observation.n_proprio);
  Eigen::Index cursor = 0;
  proprio.segment<3>(cursor) = angular_velocity * config_.observation.angular_velocity_scale;
  cursor += 3;
  proprio.segment<2>(cursor) = rpy.head<2>();
  cursor += 2;
  proprio.segment(cursor, dof_position.size()) =
    dof_position - map_vector(config_.default_dof_pos);
  cursor += dof_position.size();
  proprio.segment(cursor, dof_velocity.size()) =
    dof_velocity * config_.observation.joint_velocity_scale;
  cursor += dof_velocity.size();
  proprio.segment(cursor, last_action.size()) = last_action;
  return proprio;
}

Eigen::VectorXf ObservationBuilder::build_observation(
  const Eigen::VectorXf & mimic,
  const Eigen::Vector3f & angular_velocity,
  const Eigen::Vector3f & rpy,
  const Eigen::VectorXf & dof_position,
  const Eigen::VectorXf & dof_velocity,
  const Eigen::VectorXf & last_action)
{
  require_size(mimic, config_.observation.n_mimic_obs, "mimic");
  const auto proprio = build_proprio(
    angular_velocity, rpy, dof_position, dof_velocity, last_action);
  Eigen::VectorXf current(config_.observation.n_obs_single);
  current << mimic, proprio;

  if (history_.empty()) {
    for (std::size_t index = 0; index < config_.observation.history_len; ++index) {
      history_.push_back(current);
    }
  }

  Eigen::VectorXf result(config_.observation.total_obs_size);
  Eigen::Index cursor = 0;
  result.segment(cursor, current.size()) = current;
  cursor += current.size();
  for (const auto & frame : history_) {
    result.segment(cursor, frame.size()) = frame;
    cursor += frame.size();
  }
  result.segment(cursor, mimic.size()) = mimic;

  history_.pop_front();
  history_.push_back(current);
  return result;
}

Eigen::VectorXf postprocess_action(
  const Eigen::VectorXf & raw_action,
  const RobotConfig & config)
{
  require_size(raw_action, config.actions_size, "raw_action");
  const float raw_action_limit = config.clip_actions / config.action_scale;
  const auto clipped = raw_action.array().max(-raw_action_limit).min(raw_action_limit);
  Eigen::VectorXf target =
    (clipped * config.action_scale).matrix() + map_vector(config.default_dof_pos);
  for (const int index : config.blocked_action_indices) {
    target[index] = config.default_dof_pos[static_cast<std::size_t>(index)];
  }
  return target;
}

Eigen::VectorXf default_mimic_observation(const RobotConfig & config)
{
  if (config.observation.n_mimic_obs != config.motor_num + 6) {
    throw std::runtime_error("Default mimic observation expects motor_num + 6 elements");
  }
  Eigen::VectorXf result = Eigen::VectorXf::Zero(config.observation.n_mimic_obs);
  result[2] = 1.0F;
  result.tail(config.motor_num) = map_vector(config.default_dof_pos);
  return result;
}

}  // namespace tienkung_policy_runner