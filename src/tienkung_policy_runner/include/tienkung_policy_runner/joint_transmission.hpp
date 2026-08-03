#pragma once

#include <memory>

#include <Eigen/Core>

#include "tienkung_policy_runner/robot_config.hpp"

namespace tienkung_policy_runner {

struct TransmissionState {
  Eigen::VectorXf position;
  Eigen::VectorXf velocity;
  Eigen::VectorXf torque;
};

class JointTransmission {
public:
  virtual ~JointTransmission() = default;
  virtual TransmissionState motor_to_joint(const TransmissionState & motor) const = 0;
  virtual TransmissionState joint_to_motor(const TransmissionState & joint) const = 0;
};

class LinearJointTransmission final : public JointTransmission {
public:
  explicit LinearJointTransmission(const Eigen::MatrixXf & motor_to_joint);
  TransmissionState motor_to_joint(const TransmissionState & motor) const override;
  TransmissionState joint_to_motor(const TransmissionState & joint) const override;

private:
  Eigen::MatrixXf motor_to_joint_;
  Eigen::MatrixXf joint_to_motor_;
  Eigen::MatrixXf motor_torque_to_joint_;
  Eigen::MatrixXf joint_torque_to_motor_;
};

std::unique_ptr<JointTransmission> make_joint_transmission(
  const AnkleTransmissionConfig & config);

TransmissionState apply_motor_to_joint(
  const TransmissionState & motor,
  const AnkleTransmissionConfig & config,
  const JointTransmission & transmission);
TransmissionState apply_joint_to_motor(
  const TransmissionState & joint,
  const AnkleTransmissionConfig & config,
  const JointTransmission & transmission);

}  // namespace tienkung_policy_runner