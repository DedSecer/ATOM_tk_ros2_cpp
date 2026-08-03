#pragma once

#include <optional>
#include <vector>

namespace tienkung_policy_runner {

enum class ControlMode : unsigned char { Stop = 0, Zero = 1, Policy = 2 };

struct JoystickCommand {
  std::optional<ControlMode> requested_mode;
  bool disable{false};
  float x_speed_command{};
  float y_speed_command{};
  float yaw_speed_command{};
};

JoystickCommand decode_joy_message(
  const std::vector<float> & axes,
  const std::vector<int> & buttons);

class PolicyFsm {
public:
  explicit PolicyFsm(double zero_duration_sec = 2.0);

  ControlMode update(
    double now_sec,
    const JoystickCommand & joystick,
    bool state_ready,
    bool motion_ready,
    bool policy_ready);

  ControlMode mode() const noexcept;

private:
  ControlMode mode_{ControlMode::Stop};
  double zero_duration_sec_{};
  std::optional<double> zero_start_time_sec_;
};

const char * mode_name(ControlMode mode) noexcept;

}  // namespace tienkung_policy_runner