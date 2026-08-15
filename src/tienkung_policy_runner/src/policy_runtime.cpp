#include "tienkung_policy_runner/policy_runtime.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>

namespace tienkung_policy_runner {

PolicyRuntime::PolicyRuntime(
  const RobotConfig & config,
  const std::string & policy_path,
  const std::string & device,
  double zero_duration_sec,
  bool infer_in_inactive_modes)
: config_(config),
  observation_builder_(config),
  fsm_(zero_duration_sec),
  policy_(policy_path, device),
  zero_duration_sec_(std::max(1.0e-3, zero_duration_sec)),
  infer_in_inactive_modes_(infer_in_inactive_modes),
  last_action_(Eigen::VectorXf::Zero(config.actions_size)),
  zero_start_position_(Eigen::Map<const Eigen::VectorXf>(
      config.default_dof_pos.data(), config.default_dof_pos.size())),
  zero_start_fixed_arm_position_(Eigen::VectorXf::Constant(
      config.fixed_arm_command.ids.size(), config.fixed_arm_command.position))
{
}

RuntimeStep PolicyRuntime::step(
  double now_sec,
  const BodyState & state,
  const JoystickCommand & joystick,
  const Eigen::VectorXf & mimic,
  bool state_ready,
  bool motion_ready)
{
  RuntimeStep result;
  result.mode = fsm_.update(now_sec, joystick, state_ready, motion_ready, true);

  if (result.mode == ControlMode::Zero && previous_mode_ != ControlMode::Zero) {
    zero_start_time_sec_ = now_sec;
    zero_start_position_ = state.dof_position;
    zero_start_fixed_arm_position_ = state.fixed_arm_position;
  } else if (result.mode != ControlMode::Zero) {
    zero_start_time_sec_.reset();
  }

  if (result.mode == ControlMode::Stop) {
    result.target_position = state.dof_position;
  } else if (result.mode == ControlMode::Zero) {
    if (!zero_start_time_sec_) {
      zero_start_time_sec_ = now_sec;
      zero_start_position_ = state.dof_position;
      zero_start_fixed_arm_position_ = state.fixed_arm_position;
    }
    const float alpha = static_cast<float>(std::clamp(
        (now_sec - *zero_start_time_sec_) / zero_duration_sec_, 0.0, 1.0));
    const auto default_position = Eigen::Map<const Eigen::VectorXf>(
      config_.default_dof_pos.data(), config_.default_dof_pos.size());
    result.target_position = zero_start_position_ +
      (default_position - zero_start_position_) * quintic_blend(alpha);
    result.fixed_arm_target_position = zero_start_fixed_arm_position_ +
      (Eigen::VectorXf::Constant(
        zero_start_fixed_arm_position_.size(), config_.fixed_arm_command.position) -
      zero_start_fixed_arm_position_) * quintic_blend(alpha);
  }

  const bool run_inference = result.mode == ControlMode::Policy || infer_in_inactive_modes_;
  if (run_inference) {
    result.observation = observation_builder_.build_observation(
      mimic, state.angular_velocity, state.rpy, state.dof_position, state.dof_velocity,
      last_action_);
    const auto started = std::chrono::steady_clock::now();
    result.raw_action = policy_.infer(result.observation);
    if (result.raw_action.size() != static_cast<Eigen::Index>(config_.actions_size)) {
      throw std::runtime_error("ONNX policy output size does not match actions_size");
    }
    if (!result.raw_action.allFinite()) {
      throw std::runtime_error("ONNX policy output contains NaN or Inf");
    }
    const auto finished = std::chrono::steady_clock::now();
    result.inference_latency_ms =
      std::chrono::duration<double, std::milli>(finished - started).count();
    result.inference_executed = true;
  }

  if (result.mode == ControlMode::Policy) {
    last_action_ = result.raw_action;
    result.target_position = postprocess_action(result.raw_action, config_);
  } else {
    last_action_.setZero();
  }

  previous_mode_ = result.mode;
  return result;
}

const Eigen::VectorXf & PolicyRuntime::last_action() const noexcept
{
  return last_action_;
}

float PolicyRuntime::quintic_blend(float alpha)
{
  alpha = std::clamp(alpha, 0.0F, 1.0F);
  return alpha * alpha * alpha * (10.0F - 15.0F * alpha + 6.0F * alpha * alpha);
}

}  // namespace tienkung_policy_runner