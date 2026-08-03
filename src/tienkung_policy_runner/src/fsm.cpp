#include "tienkung_policy_runner/fsm.hpp"

namespace tienkung_policy_runner {

PolicyFsm::PolicyFsm(double zero_duration_sec)
: zero_duration_sec_(zero_duration_sec)
{
}

ControlMode PolicyFsm::update(
  double now_sec,
  const JoystickCommand & joystick,
  bool state_ready,
  bool motion_ready,
  bool policy_ready)
{
  if (joystick.disable || !state_ready) {
    mode_ = ControlMode::Stop;
    zero_start_time_sec_.reset();
    return mode_;
  }

  if (joystick.requested_mode == ControlMode::Stop) {
    mode_ = ControlMode::Stop;
    zero_start_time_sec_.reset();
  } else if (joystick.requested_mode == ControlMode::Zero) {
    if (mode_ != ControlMode::Zero) {
      zero_start_time_sec_ = now_sec;
    }
    mode_ = ControlMode::Zero;
  } else if (joystick.requested_mode == ControlMode::Policy) {
    const bool zero_done = zero_start_time_sec_.has_value() &&
      now_sec - *zero_start_time_sec_ >= zero_duration_sec_;
    if (motion_ready && policy_ready && (mode_ == ControlMode::Policy || zero_done)) {
      mode_ = ControlMode::Policy;
    } else if (mode_ != ControlMode::Zero) {
      mode_ = ControlMode::Zero;
      zero_start_time_sec_ = now_sec;
    }
  }

  if (mode_ == ControlMode::Policy && (!motion_ready || !policy_ready)) {
    mode_ = state_ready ? ControlMode::Zero : ControlMode::Stop;
  }
  return mode_;
}

ControlMode PolicyFsm::mode() const noexcept
{
  return mode_;
}

JoystickCommand decode_joy_message(
  const std::vector<float> & axes,
  const std::vector<int> & buttons)
{
  JoystickCommand command;
  if (axes.size() == 12) {
    const float a = axes[8];
    const float c = axes[10];
    const float d = axes[11];
    const float e = axes[4];
    const float g = axes[5];
    command.disable = e == 1.0F && axes[9] == 1.0F;
    if (d == 1.0F) {
      command.requested_mode = ControlMode::Zero;
    } else if (c == 1.0F) {
      command.requested_mode = ControlMode::Stop;
    } else if (a == 1.0F && g == 0.0F) {
      command.requested_mode = ControlMode::Policy;
    }
    command.y_speed_command = axes[3] * -0.4F;
    command.x_speed_command = axes[2] * (axes[2] >= 0.0F ? 0.8F : 0.5F);
    command.yaw_speed_command = axes[0] * -0.4F;
  } else if (axes.size() >= 5 && buttons.size() >= 8) {
    command.disable = buttons[4] != 0 && buttons[1] != 0;
    if (buttons[2] != 0) {
      command.requested_mode = ControlMode::Zero;
    } else if (buttons[3] != 0) {
      command.requested_mode = ControlMode::Stop;
    } else if (buttons[0] != 0 && buttons[6] == 0) {
      command.requested_mode = ControlMode::Policy;
    }
    command.y_speed_command = axes[0] * -0.4F;
    command.x_speed_command = axes[1] * (axes[1] >= 0.0F ? 0.8F : 0.5F);
    command.yaw_speed_command = axes[4] * -0.4F;
  }
  return command;
}

const char * mode_name(ControlMode mode) noexcept
{
  switch (mode) {
    case ControlMode::Stop:
      return "STOP";
    case ControlMode::Zero:
      return "ZERO";
    case ControlMode::Policy:
      return "POLICY";
  }
  return "UNKNOWN";
}

}  // namespace tienkung_policy_runner