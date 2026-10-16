#pragma once

#include <optional>

#include <Eigen/Core>

#include "tienkung_policy_runner/robot_config.hpp"

namespace tienkung_policy_runner {

// Smoothly blends the live motion reference stream from the default mimic
// observation when entering POLICY mode. After the interpolation window the
// output follows the live reference so the policy keeps tracking the motion.
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
};

}  // namespace tienkung_policy_runner
