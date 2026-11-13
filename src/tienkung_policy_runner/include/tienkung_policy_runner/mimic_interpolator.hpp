#pragma once

#include <optional>

#include <Eigen/Core>

#include "tienkung_policy_runner/robot_config.hpp"

namespace tienkung_policy_runner {

// Smoothly blends the live motion reference stream from the default mimic
// observation when entering POLICY mode. After the interpolation window the
// output follows the live reference so the policy keeps tracking the motion.
// While the reference stream is stale (motion_reference_valid == false) the
// last valid frame is held instead of jumping back to the default observation,
// so a temporary publish gap does not produce a step change in the policy
// observation. The default observation is only used before any valid frame
// has been received.
class MimicInterpolator {
public:
  explicit MimicInterpolator(const RobotConfig & config, double interpolation_sec);

  Eigen::VectorXf interpolate(
    double now_sec, const Eigen::VectorXf & mimic, bool motion_reference_valid);

  void reset();

private:
  const RobotConfig & config_;
  double interpolation_sec_{};
  std::optional<double> start_time_sec_;
  Eigen::VectorXf start_;
  Eigen::VectorXf last_valid_;
  bool has_last_valid_{false};
};

}  // namespace tienkung_policy_runner
