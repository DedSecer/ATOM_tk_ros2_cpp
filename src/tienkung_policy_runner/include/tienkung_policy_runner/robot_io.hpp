#pragma once

#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "tienkung_policy_runner/fsm.hpp"
#include "tienkung_policy_runner/joint_transmission.hpp"
#include "tienkung_policy_runner/motor_calibration.hpp"
#include "tienkung_policy_runner/robot_config.hpp"

namespace tienkung_policy_runner {

struct MotorSample {
  int can_id{};
  float position{};
  float velocity{};
  float current{};
};

struct BodyState {
  Eigen::VectorXf dof_position;
  Eigen::VectorXf dof_velocity;
  Eigen::VectorXf dof_torque;
  Eigen::VectorXf fixed_arm_position;
  Eigen::Vector3f rpy{Eigen::Vector3f::Zero()};
  Eigen::Vector3f angular_velocity{Eigen::Vector3f::Zero()};
  double leg_timestamp_sec{};
  double arm_timestamp_sec{};
  double imu_timestamp_sec{};
  bool leg_complete{false};
  bool arm_complete{false};
  bool fixed_arm_complete{false};
};

struct MotorCommand {
  int can_id{};
  float kp{};
  float kd{};
  float position{};
  float velocity{};
  float torque{};
};

struct WaistCommand {
  int can_id{};
  float position{};
  float speed{};
  float current{};
};

struct RobotCommand {
  std::vector<MotorCommand> leg;
  std::vector<MotorCommand> arm;
  std::vector<WaistCommand> waist;
};

class RobotIo {
public:
  explicit RobotIo(const RobotConfig & config);

  void ingest_leg_status(const std::vector<MotorSample> & samples, double stamp_sec);
  void ingest_arm_status(const std::vector<MotorSample> & samples, double stamp_sec);
  void ingest_imu(
    const Eigen::Vector3f & rpy,
    const Eigen::Vector3f & angular_velocity,
    double stamp_sec);
  void ingest_joystick(const JoystickCommand & command);

  BodyState snapshot() const;
  const std::optional<JoystickCommand> & last_joystick() const noexcept;
  RobotCommand build_command(
    const Eigen::VectorXf & target_position,
    const Eigen::VectorXf & kp,
    const Eigen::VectorXf & kd,
    const Eigen::VectorXf & target_velocity = Eigen::VectorXf(),
    const Eigen::VectorXf & torque = Eigen::VectorXf(),
    const Eigen::VectorXf & fixed_arm_target_position = Eigen::VectorXf()) const;

private:
  void ingest_motor_status(const std::vector<MotorSample> & samples);
  bool all_valid(const std::vector<int> & indices) const;

  const RobotConfig & config_;
  MotorCalibration calibration_;
  std::unique_ptr<JointTransmission> transmission_;
  Eigen::VectorXf raw_position_;
  Eigen::VectorXf raw_velocity_;
  Eigen::VectorXf raw_current_;
  Eigen::VectorXf last_raw_position_;
  Eigen::VectorXf last_raw_velocity_;
  Eigen::VectorXf last_raw_current_;
  Eigen::VectorXf zero_count_;
  std::vector<bool> valid_mask_;
  std::vector<bool> fixed_arm_valid_mask_;
  BodyState state_;
  std::optional<JoystickCommand> last_joystick_;
};

}  // namespace tienkung_policy_runner