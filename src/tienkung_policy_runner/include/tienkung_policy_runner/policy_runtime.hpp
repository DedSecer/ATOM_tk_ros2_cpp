#pragma once

#include <optional>

#include <Eigen/Core>

#include "tienkung_policy_runner/fsm.hpp"
#include "tienkung_policy_runner/observation_builder.hpp"
#include "tienkung_policy_runner/onnx_policy.hpp"
#include "tienkung_policy_runner/robot_config.hpp"
#include "tienkung_policy_runner/robot_io.hpp"

namespace tienkung_policy_runner {

struct RuntimeStep {
  ControlMode mode{ControlMode::Stop};
  Eigen::VectorXf target_position;
  Eigen::VectorXf observation;
  Eigen::VectorXf raw_action;
  bool inference_executed{false};
  double inference_latency_ms{};
};

class PolicyRuntime {
public:
  PolicyRuntime(
    const RobotConfig & config,
    const std::string & policy_path,
    const std::string & device,
    double zero_duration_sec,
    bool infer_in_inactive_modes);

  RuntimeStep step(
    double now_sec,
    const BodyState & state,
    const JoystickCommand & joystick,
    const Eigen::VectorXf & mimic,
    bool state_ready,
    bool motion_ready);
  const Eigen::VectorXf & last_action() const noexcept;

private:
  static float quintic_blend(float alpha);

  const RobotConfig & config_;
  ObservationBuilder observation_builder_;
  PolicyFsm fsm_;
  OnnxPolicy policy_;
  double zero_duration_sec_{};
  bool infer_in_inactive_modes_{};
  Eigen::VectorXf last_action_;
  Eigen::VectorXf zero_start_position_;
  std::optional<double> zero_start_time_sec_;
  ControlMode previous_mode_{ControlMode::Stop};
};

}  // namespace tienkung_policy_runner