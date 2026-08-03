#pragma once

#include <deque>

#include <Eigen/Core>

#include "tienkung_policy_runner/robot_config.hpp"

namespace tienkung_policy_runner {

class ObservationBuilder {
public:
  explicit ObservationBuilder(const RobotConfig & config);

  void reset();
  Eigen::VectorXf build_proprio(
    const Eigen::Vector3f & angular_velocity,
    const Eigen::Vector3f & rpy,
    const Eigen::VectorXf & dof_position,
    const Eigen::VectorXf & dof_velocity,
    const Eigen::VectorXf & last_action) const;
  Eigen::VectorXf build_observation(
    const Eigen::VectorXf & mimic,
    const Eigen::Vector3f & angular_velocity,
    const Eigen::Vector3f & rpy,
    const Eigen::VectorXf & dof_position,
    const Eigen::VectorXf & dof_velocity,
    const Eigen::VectorXf & last_action);

private:
  const RobotConfig & config_;
  std::deque<Eigen::VectorXf> history_;
};

Eigen::VectorXf postprocess_action(
  const Eigen::VectorXf & raw_action,
  const RobotConfig & config);

Eigen::VectorXf default_mimic_observation(const RobotConfig & config);

}  // namespace tienkung_policy_runner