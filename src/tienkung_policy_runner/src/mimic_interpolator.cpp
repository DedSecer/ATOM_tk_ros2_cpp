#include "tienkung_policy_runner/mimic_interpolator.hpp"

#include <algorithm>

#include "tienkung_policy_runner/observation_builder.hpp"

namespace tienkung_policy_runner {

MimicInterpolator::MimicInterpolator(const RobotConfig & config, double interpolation_sec)
: config_(config),
  interpolation_sec_(std::max(1.0e-3, interpolation_sec)),
  start_(default_mimic_observation(config)),
  last_valid_(default_mimic_observation(config))
{
}

Eigen::VectorXf MimicInterpolator::interpolate(
  double now_sec, const Eigen::VectorXf & mimic, bool motion_reference_valid)
{
  const auto default_mimic = default_mimic_observation(config_);
  if (!start_time_sec_) {
    start_time_sec_ = now_sec;
    start_ = default_mimic;
  }
  if (motion_reference_valid) {
    last_valid_ = mimic;
    has_last_valid_ = true;
  }
  // Hold the last valid frame while the stream is stale; fall back to the
  // default observation only before any valid frame has been received.
  const Eigen::VectorXf & target = has_last_valid_ ? last_valid_ : default_mimic;
  const double elapsed = now_sec - *start_time_sec_;
  if (elapsed >= interpolation_sec_) {
    start_ = target;
    return target;
  }
  const float alpha = static_cast<float>(std::clamp(
      elapsed / interpolation_sec_, 0.0, 1.0));
  const float blend = alpha * alpha * alpha * (10.0F - 15.0F * alpha + 6.0F * alpha * alpha);
  return start_ + (target - start_) * blend;
}

void MimicInterpolator::reset()
{
  start_time_sec_.reset();
  start_.resize(0);
  last_valid_ = default_mimic_observation(config_);
  has_last_valid_ = false;
}

}  // namespace tienkung_policy_runner
