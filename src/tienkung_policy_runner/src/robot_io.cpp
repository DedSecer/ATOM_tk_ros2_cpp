#include "tienkung_policy_runner/robot_io.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace tienkung_policy_runner {
namespace {

Eigen::VectorXf zeros_or_value(const Eigen::VectorXf & value, Eigen::Index size)
{
  if (value.size() == 0) {
    return Eigen::VectorXf::Zero(size);
  }
  if (value.size() != size) {
    throw std::runtime_error("Command vector has an unexpected size");
  }
  return value;
}

void require_size(const Eigen::VectorXf & value, std::size_t expected, const char * name)
{
  if (value.size() != static_cast<Eigen::Index>(expected)) {
    throw std::runtime_error(std::string(name) + " has an unexpected size");
  }
}

}  // namespace

RobotIo::RobotIo(const RobotConfig & config)
: config_(config),
  calibration_(config),
  transmission_(make_joint_transmission(config.ankle_transmission)),
  raw_position_(Eigen::VectorXf::Zero(config.motor_num)),
  raw_velocity_(Eigen::VectorXf::Zero(config.motor_num)),
  raw_current_(Eigen::VectorXf::Zero(config.motor_num)),
  last_raw_position_(Eigen::VectorXf::Zero(config.motor_num)),
  last_raw_velocity_(Eigen::VectorXf::Zero(config.motor_num)),
  last_raw_current_(Eigen::VectorXf::Zero(config.motor_num)),
  zero_count_(Eigen::VectorXf::Zero(config.motor_num)),
  valid_mask_(config.motor_num, false)
{
  state_.dof_position = Eigen::VectorXf::Zero(config.motor_num);
  state_.dof_velocity = Eigen::VectorXf::Zero(config.motor_num);
  state_.dof_torque = Eigen::VectorXf::Zero(config.motor_num);
}

void RobotIo::ingest_motor_status(const std::vector<MotorSample> & samples)
{
  std::vector<bool> updated_mask(config_.motor_num, false);
  for (const auto & sample : samples) {
    if (!std::isfinite(sample.position) || !std::isfinite(sample.velocity) ||
      !std::isfinite(sample.current))
    {
      continue;
    }
    const auto found = config_.index_by_can_id.find(sample.can_id);
    if (found == config_.index_by_can_id.end()) {
      continue;
    }
    const auto index = static_cast<Eigen::Index>(found->second);
    raw_position_[index] = sample.position;
    raw_velocity_[index] = sample.velocity;
    raw_current_[index] = sample.current;
    updated_mask[found->second] = true;
  }
  calibration_.reject_large_position_jumps(
    raw_position_, raw_velocity_, raw_current_, last_raw_position_, last_raw_velocity_,
    last_raw_current_, updated_mask, valid_mask_);
  last_raw_position_ = raw_position_;
  last_raw_velocity_ = raw_velocity_;
  last_raw_current_ = raw_current_;
  auto calibrated = calibration_.convert_feedback(
    raw_position_, raw_velocity_, raw_current_, zero_count_);
  zero_count_ = calibrated.zero_count;
  auto joint = apply_motor_to_joint(
    {calibrated.position, calibrated.velocity, calibrated.torque},
    config_.ankle_transmission, *transmission_);
  state_.dof_position = std::move(joint.position);
  state_.dof_velocity = std::move(joint.velocity);
  state_.dof_torque = std::move(joint.torque);
}

void RobotIo::ingest_leg_status(
  const std::vector<MotorSample> & samples, double stamp_sec)
{
  ingest_motor_status(samples);
  state_.leg_timestamp_sec = stamp_sec;
  state_.leg_complete = all_valid(config_.leg_indices);
}

void RobotIo::ingest_arm_status(
  const std::vector<MotorSample> & samples, double stamp_sec)
{
  ingest_motor_status(samples);
  state_.arm_timestamp_sec = stamp_sec;
  state_.arm_complete = all_valid(config_.arm_indices);
}

void RobotIo::ingest_imu(
  const Eigen::Vector3f & rpy,
  const Eigen::Vector3f & angular_velocity,
  double stamp_sec)
{
  if (!rpy.allFinite() || !angular_velocity.allFinite()) {
    return;
  }
  state_.rpy = rpy;
  state_.angular_velocity = angular_velocity;
  state_.imu_timestamp_sec = stamp_sec;
}

void RobotIo::ingest_joystick(const JoystickCommand & command)
{
  last_joystick_ = command;
}

bool RobotIo::all_valid(const std::vector<int> & indices) const
{
  return std::all_of(indices.begin(), indices.end(), [this](int index) {
      return valid_mask_[static_cast<std::size_t>(index)];
    });
}

BodyState RobotIo::snapshot() const
{
  return state_;
}

const std::optional<JoystickCommand> & RobotIo::last_joystick() const noexcept
{
  return last_joystick_;
}

RobotCommand RobotIo::build_command(
  const Eigen::VectorXf & target_position,
  const Eigen::VectorXf & kp,
  const Eigen::VectorXf & kd,
  const Eigen::VectorXf & target_velocity,
  const Eigen::VectorXf & torque) const
{
  require_size(target_position, config_.motor_num, "target_position");
  require_size(kp, config_.motor_num, "kp");
  require_size(kd, config_.motor_num, "kd");
  const auto velocity = zeros_or_value(target_velocity, target_position.size());
  const auto torque_value = zeros_or_value(torque, target_position.size());
  auto motor_space = apply_joint_to_motor(
    {target_position, velocity, torque_value}, config_.ankle_transmission, *transmission_);
  const auto calibrated = calibration_.convert_command(
    motor_space.position, motor_space.velocity, motor_space.torque, zero_count_);

  auto build_group = [&](const std::vector<int> & indices) {
      std::vector<MotorCommand> result;
      result.reserve(indices.size());
      for (const int raw_index : indices) {
        const auto index = static_cast<Eigen::Index>(raw_index);
        result.push_back({
          config_.can_id_by_index[raw_index], kp[index], kd[index],
          calibrated.position[index], calibrated.velocity[index], calibrated.torque[index]});
      }
      return result;
    };

  RobotCommand command;
  command.leg = build_group(config_.leg_indices);
  command.arm = build_group(config_.arm_indices);
  for (const int can_id : config_.waist_command.ids) {
    command.waist.push_back({
      can_id, config_.waist_command.position, config_.waist_command.speed,
      config_.waist_command.current});
  }
  return command;
}

}  // namespace tienkung_policy_runner