#pragma once

#include <vector>

#include <Eigen/Core>

#include "tienkung_policy_runner/robot_config.hpp"

namespace tienkung_policy_runner {

struct CalibratedState {
  Eigen::VectorXf position;
  Eigen::VectorXf velocity;
  Eigen::VectorXf torque;
  Eigen::VectorXf zero_count;
};

struct MotorCommandState {
  Eigen::VectorXf position;
  Eigen::VectorXf velocity;
  Eigen::VectorXf torque;
};

class MotorCalibration {
public:
  explicit MotorCalibration(const RobotConfig & config);

  CalibratedState convert_feedback(
    const Eigen::VectorXf & raw_position,
    const Eigen::VectorXf & raw_velocity,
    const Eigen::VectorXf & raw_current,
    const Eigen::VectorXf & zero_count) const;
  MotorCommandState convert_command(
    const Eigen::VectorXf & joint_position,
    const Eigen::VectorXf & joint_velocity,
    const Eigen::VectorXf & joint_torque,
    const Eigen::VectorXf & zero_count) const;
  void reject_large_position_jumps(
    Eigen::VectorXf & raw_position,
    Eigen::VectorXf & raw_velocity,
    Eigen::VectorXf & raw_current,
    const Eigen::VectorXf & last_position,
    const Eigen::VectorXf & last_velocity,
    const Eigen::VectorXf & last_current,
    const std::vector<bool> & updated_mask,
    std::vector<bool> & valid_mask) const;

private:
  Eigen::VectorXf zero_pos_offset_;
  Eigen::VectorXf zero_offset_;
  Eigen::VectorXf motor_dir_;
  Eigen::VectorXf ct_scale_;
  float wrap_threshold_rad_{};
  bool enable_jump_filter_{};
};

}  // namespace tienkung_policy_runner