#include "tienkung_policy_runner/joint_transmission.hpp"

#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/LU>

namespace tienkung_policy_runner {

extern "C" {
void * funcsptrans_create();
void funcsptrans_destroy(void * handle);
int funcsptrans_state_to_joint(
  void * handle, const double * q_p, const double * qdot_p, const double * tor_p,
  double * q_s, double * qdot_s, double * tor_s);
int funcsptrans_joint_to_motor(
  void * handle, const double * q_s, const double * qdot_s, const double * tor_s,
  double * q_p, double * qdot_p, double * tor_p);
}

namespace {

void require_block_size(const TransmissionState & state, Eigen::Index expected)
{
  if (state.position.size() != expected || state.velocity.size() != expected ||
    state.torque.size() != expected)
  {
    throw std::runtime_error("Transmission state dimensions do not match");
  }
}

class NativeFuncSpTrans final : public JointTransmission {
public:
  NativeFuncSpTrans()
  : handle_(funcsptrans_create())
  {
    if (handle_ == nullptr) {
      throw std::runtime_error("funcSPTrans initialization failed");
    }
  }

  ~NativeFuncSpTrans() override
  {
    funcsptrans_destroy(handle_);
  }

  TransmissionState motor_to_joint(const TransmissionState & motor) const override
  {
    return convert(motor, true);
  }

  TransmissionState joint_to_motor(const TransmissionState & joint) const override
  {
    return convert(joint, false);
  }

private:
  TransmissionState convert(const TransmissionState & input, bool state_to_joint) const
  {
    require_block_size(input, 4);
    const Eigen::VectorXd position = input.position.cast<double>();
    const Eigen::VectorXd velocity = input.velocity.cast<double>();
    const Eigen::VectorXd torque = input.torque.cast<double>();
    Eigen::Vector4d output_position = Eigen::Vector4d::Zero();
    Eigen::Vector4d output_velocity = Eigen::Vector4d::Zero();
    Eigen::Vector4d output_torque = Eigen::Vector4d::Zero();
    const int result = state_to_joint ?
      funcsptrans_state_to_joint(
      handle_, position.data(), velocity.data(), torque.data(),
      output_position.data(), output_velocity.data(), output_torque.data()) :
      funcsptrans_joint_to_motor(
      handle_, position.data(), velocity.data(), torque.data(),
      output_position.data(), output_velocity.data(), output_torque.data());
    if (result != 0) {
      throw std::runtime_error("funcSPTrans conversion failed with code " + std::to_string(result));
    }
    return {output_position.cast<float>(), output_velocity.cast<float>(), output_torque.cast<float>()};
  }

  void * handle_{};
};

TransmissionState gather(
  const TransmissionState & full,
  const std::vector<int> & indices)
{
  TransmissionState block{
    Eigen::VectorXf(indices.size()),
    Eigen::VectorXf(indices.size()),
    Eigen::VectorXf(indices.size())};
  for (std::size_t cursor = 0; cursor < indices.size(); ++cursor) {
    const auto index = static_cast<Eigen::Index>(indices[cursor]);
    block.position[cursor] = full.position[index];
    block.velocity[cursor] = full.velocity[index];
    block.torque[cursor] = full.torque[index];
  }
  return block;
}

void scatter(
  TransmissionState & full,
  const std::vector<int> & indices,
  const TransmissionState & block)
{
  require_block_size(block, static_cast<Eigen::Index>(indices.size()));
  for (std::size_t cursor = 0; cursor < indices.size(); ++cursor) {
    const auto index = static_cast<Eigen::Index>(indices[cursor]);
    full.position[index] = block.position[cursor];
    full.velocity[index] = block.velocity[cursor];
    full.torque[index] = block.torque[cursor];
  }
}

}  // namespace

LinearJointTransmission::LinearJointTransmission(const Eigen::MatrixXf & motor_to_joint)
: motor_to_joint_(motor_to_joint)
{
  if (motor_to_joint_.rows() == 0 || motor_to_joint_.rows() != motor_to_joint_.cols()) {
    throw std::runtime_error("motor_to_joint must be a non-empty square matrix");
  }
  if (!motor_to_joint_.fullPivLu().isInvertible()) {
    throw std::runtime_error("motor_to_joint must be invertible");
  }
  joint_to_motor_ = motor_to_joint_.inverse();
  motor_torque_to_joint_ = joint_to_motor_.transpose();
  joint_torque_to_motor_ = motor_to_joint_.transpose();
}

TransmissionState LinearJointTransmission::motor_to_joint(
  const TransmissionState & motor) const
{
  require_block_size(motor, motor_to_joint_.cols());
  return {
    motor_to_joint_ * motor.position,
    motor_to_joint_ * motor.velocity,
    motor_torque_to_joint_ * motor.torque};
}

TransmissionState LinearJointTransmission::joint_to_motor(
  const TransmissionState & joint) const
{
  require_block_size(joint, joint_to_motor_.cols());
  return {
    joint_to_motor_ * joint.position,
    joint_to_motor_ * joint.velocity,
    joint_torque_to_motor_ * joint.torque};
}

std::unique_ptr<JointTransmission> make_joint_transmission(
  const AnkleTransmissionConfig & config)
{
  if (config.kind == "native_funcsptrans") {
    return std::make_unique<NativeFuncSpTrans>();
  }
  if (config.kind == "identity") {
    return std::make_unique<LinearJointTransmission>(
      Eigen::MatrixXf::Identity(config.motor_indices.size(), config.motor_indices.size()));
  }
  throw std::runtime_error("Unsupported ankle transmission kind: " + config.kind);
}

TransmissionState apply_motor_to_joint(
  const TransmissionState & motor,
  const AnkleTransmissionConfig & config,
  const JointTransmission & transmission)
{
  auto result = motor;
  scatter(
    result, config.joint_indices,
    transmission.motor_to_joint(gather(motor, config.motor_indices)));
  return result;
}

TransmissionState apply_joint_to_motor(
  const TransmissionState & joint,
  const AnkleTransmissionConfig & config,
  const JointTransmission & transmission)
{
  auto result = joint;
  scatter(
    result, config.motor_indices,
    transmission.joint_to_motor(gather(joint, config.joint_indices)));
  return result;
}

}  // namespace tienkung_policy_runner