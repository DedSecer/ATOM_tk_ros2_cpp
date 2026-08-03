#include "tienkung_policy_runner/motor_calibration.hpp"

#include <cmath>
#include <stdexcept>

namespace tienkung_policy_runner {
namespace {

constexpr float kTwoPi = 6.28318530717958647692F;

Eigen::VectorXf map_copy(const std::vector<float> & values)
{
  return Eigen::Map<const Eigen::VectorXf>(
    values.data(), static_cast<Eigen::Index>(values.size()));
}

void require_same_size(const Eigen::VectorXf & value, Eigen::Index size, const char * name)
{
  if (value.size() != size) {
    throw std::runtime_error(std::string(name) + " has an unexpected size");
  }
}

}  // namespace

MotorCalibration::MotorCalibration(const RobotConfig & config)
: zero_pos_offset_(map_copy(config.zero_pos_offset)),
  zero_offset_(map_copy(config.zero_offset)),
  motor_dir_(map_copy(config.motor_dir)),
  ct_scale_(map_copy(config.ct_scale)),
  wrap_threshold_rad_(config.wrap_threshold_rad),
  enable_jump_filter_(config.enable_jump_filter)
{
}

CalibratedState MotorCalibration::convert_feedback(
  const Eigen::VectorXf & raw_position,
  const Eigen::VectorXf & raw_velocity,
  const Eigen::VectorXf & raw_current,
  const Eigen::VectorXf & zero_count) const
{
  const auto size = zero_pos_offset_.size();
  require_same_size(raw_position, size, "raw_position");
  require_same_size(raw_velocity, size, "raw_velocity");
  require_same_size(raw_current, size, "raw_current");
  require_same_size(zero_count, size, "zero_count");

  CalibratedState state;
  state.zero_count = zero_count;
  state.position = (raw_position - zero_pos_offset_).cwiseProduct(motor_dir_) + zero_offset_;
  for (Eigen::Index index = 0; index < size; ++index) {
    if (state.position[index] > wrap_threshold_rad_) {
      state.zero_count[index] = -1.0F;
    } else if (state.position[index] < -wrap_threshold_rad_) {
      state.zero_count[index] = 1.0F;
    }
  }
  state.position.array() += state.zero_count.array() * kTwoPi;
  state.velocity = raw_velocity.cwiseProduct(motor_dir_);
  state.torque = raw_current.cwiseProduct(ct_scale_).cwiseProduct(motor_dir_);
  return state;
}

MotorCommandState MotorCalibration::convert_command(
  const Eigen::VectorXf & joint_position,
  const Eigen::VectorXf & joint_velocity,
  const Eigen::VectorXf & joint_torque,
  const Eigen::VectorXf & zero_count) const
{
  const auto size = zero_pos_offset_.size();
  require_same_size(joint_position, size, "joint_position");
  require_same_size(joint_velocity, size, "joint_velocity");
  require_same_size(joint_torque, size, "joint_torque");
  require_same_size(zero_count, size, "zero_count");
  MotorCommandState command;
  const auto unwrapped = joint_position - zero_offset_ -
    (zero_count.array() * kTwoPi).matrix();
  command.position = unwrapped.cwiseProduct(motor_dir_) + zero_pos_offset_;
  command.velocity = joint_velocity.cwiseProduct(motor_dir_);
  command.torque = joint_torque.cwiseProduct(motor_dir_);
  return command;
}

void MotorCalibration::reject_large_position_jumps(
  Eigen::VectorXf & raw_position,
  Eigen::VectorXf & raw_velocity,
  Eigen::VectorXf & raw_current,
  const Eigen::VectorXf & last_position,
  const Eigen::VectorXf & last_velocity,
  const Eigen::VectorXf & last_current,
  const std::vector<bool> & updated_mask,
  std::vector<bool> & valid_mask) const
{
  const auto size = zero_pos_offset_.size();
  if (valid_mask.size() != static_cast<std::size_t>(size) ||
    updated_mask.size() != static_cast<std::size_t>(size))
  {
    throw std::runtime_error("Motor validity masks have an unexpected size");
  }
  if (!enable_jump_filter_) {
    for (std::size_t index = 0; index < valid_mask.size(); ++index) {
      valid_mask[index] = valid_mask[index] || updated_mask[index];
    }
    return;
  }
  for (Eigen::Index index = 0; index < size; ++index) {
    const auto mask_index = static_cast<std::size_t>(index);
    if (!updated_mask[mask_index]) {
      continue;
    }
    if (valid_mask[mask_index] &&
      std::abs(raw_position[index] - last_position[index]) > wrap_threshold_rad_)
    {
      raw_position[index] = last_position[index];
      raw_velocity[index] = last_velocity[index];
      raw_current[index] = last_current[index];
    }
    valid_mask[mask_index] = true;
  }
}

}  // namespace tienkung_policy_runner