#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "tienkung_policy_runner/fsm.hpp"
#include "tienkung_policy_runner/joint_transmission.hpp"
#include "tienkung_policy_runner/motor_calibration.hpp"
#include "tienkung_policy_runner/observation_builder.hpp"
#include "tienkung_policy_runner/robot_config.hpp"
#include "tienkung_policy_runner/robot_io.hpp"

#ifndef TEST_ROBOT_CONFIG_PATH
#error "TEST_ROBOT_CONFIG_PATH must be defined"
#endif

namespace tk = tienkung_policy_runner;

TEST(RobotConfig, LoadsUniformActionScaleAndPdGains)
{
  const auto config = tk::load_robot_config(TEST_ROBOT_CONFIG_PATH);
  EXPECT_EQ(config.motor_num, 20U);
  EXPECT_FLOAT_EQ(config.action_scale, 0.5F);
  EXPECT_FLOAT_EQ(config.clip_actions, 5.0F);
  EXPECT_EQ(config.joint_kp.size(), config.motor_num);
  EXPECT_EQ(config.joint_kd.size(), config.motor_num);
  EXPECT_FLOAT_EQ(config.joint_kp[0], 700.0F);
  EXPECT_FLOAT_EQ(config.joint_kd[19], 1.0F);
}

TEST(RobotConfig, RejectsNonPositiveActionContractValues)
{
  std::ifstream source(TEST_ROBOT_CONFIG_PATH);
  ASSERT_TRUE(source);
  const std::string content(
    (std::istreambuf_iterator<char>(source)), std::istreambuf_iterator<char>());

  const auto expect_rejected = [&](const std::string & field, const std::string & invalid_value) {
      std::string invalid = content;
      const auto offset = invalid.find(field);
      EXPECT_NE(offset, std::string::npos);
      if (offset == std::string::npos) {
        return;
      }
      invalid.replace(offset, field.size(), invalid_value);
      const auto path = std::filesystem::temp_directory_path() / "invalid_tg22_config.yaml";
      std::ofstream(path) << invalid;
      EXPECT_THROW(tk::load_robot_config(path.string()), std::runtime_error);
      std::filesystem::remove(path);
    };

  expect_rejected("action_scale: 0.5", "action_scale: 0.0");
  expect_rejected("clip_actions: 5.0", "clip_actions: 0.0");
}

TEST(ObservationBuilder, MatchesHistoryAndUniformActionContract)
{
  const auto config = tk::load_robot_config(TEST_ROBOT_CONFIG_PATH);
  tk::ObservationBuilder builder(config);
  const auto mimic = tk::default_mimic_observation(config);
  const Eigen::Vector3f angular_velocity(1.0F, 2.0F, 3.0F);
  const Eigen::Vector3f rpy(0.1F, -0.2F, 0.3F);
  const auto dof_position = Eigen::Map<const Eigen::VectorXf>(
    config.default_dof_pos.data(), config.default_dof_pos.size());
  const auto dof_velocity = Eigen::VectorXf::LinSpaced(config.motor_num, 0.0F, 19.0F);
  const auto last_action = Eigen::VectorXf::Ones(config.actions_size);

  const auto proprio = builder.build_proprio(
    angular_velocity, rpy, dof_position, dof_velocity, last_action);
  ASSERT_EQ(proprio.size(), static_cast<Eigen::Index>(config.observation.n_proprio));
  EXPECT_FLOAT_EQ(proprio[0], 0.25F);
  EXPECT_FLOAT_EQ(proprio[1], 0.5F);
  EXPECT_FLOAT_EQ(proprio[2], 0.75F);
  EXPECT_TRUE(proprio.segment(5, config.motor_num).isZero(1.0e-6F));

  const auto observation = builder.build_observation(
    mimic, angular_velocity, rpy, dof_position, dof_velocity, last_action);
  ASSERT_EQ(observation.size(), static_cast<Eigen::Index>(config.observation.total_obs_size));
  Eigen::VectorXf current(config.observation.n_obs_single);
  current << mimic, proprio;
  for (std::size_t frame = 0; frame < config.observation.history_len; ++frame) {
    const auto start = static_cast<Eigen::Index>(
      config.observation.n_obs_single * (frame + 1));
    EXPECT_TRUE(observation.segment(start, current.size()).isApprox(current));
  }

  const auto raw = Eigen::VectorXf::Constant(config.actions_size, 20.0F);
  const auto target = tk::postprocess_action(raw, config);
  EXPECT_TRUE(target.isApprox((dof_position.array() + 5.0F).matrix()));
}

TEST(ActionPostprocess, DerivesRawClipLimitFromTrainingContract)
{
  auto config = tk::load_robot_config(TEST_ROBOT_CONFIG_PATH);
  config.action_scale = 0.25F;
  config.clip_actions = 5.0F;
  const auto defaults = Eigen::Map<const Eigen::VectorXf>(
    config.default_dof_pos.data(), config.default_dof_pos.size());

  const auto within_raw_limit = Eigen::VectorXf::Constant(config.actions_size, 12.0F);
  const auto within_target = tk::postprocess_action(within_raw_limit, config);
  EXPECT_TRUE(within_target.isApprox((defaults.array() + 3.0F).matrix()));

  const auto beyond_raw_limit = Eigen::VectorXf::Constant(config.actions_size, 40.0F);
  const auto clipped_target = tk::postprocess_action(beyond_raw_limit, config);
  EXPECT_TRUE(clipped_target.isApprox((defaults.array() + config.clip_actions).matrix()));
}

TEST(PolicyFsm, RequiresCompletedZeroTransitionBeforePolicy)
{
  tk::PolicyFsm fsm(2.0);
  tk::JoystickCommand command;
  command.requested_mode = tk::ControlMode::Policy;
  EXPECT_EQ(fsm.update(0.0, command, true, true, true), tk::ControlMode::Zero);
  EXPECT_EQ(fsm.update(1.9, command, true, true, true), tk::ControlMode::Zero);
  EXPECT_EQ(fsm.update(2.0, command, true, true, true), tk::ControlMode::Policy);
  EXPECT_EQ(fsm.update(2.1, command, false, true, true), tk::ControlMode::Stop);
}

TEST(MotorCalibration, FeedbackCommandRoundTrip)
{
  auto config = tk::load_robot_config(TEST_ROBOT_CONFIG_PATH);
  config.motor_num = 2;
  config.zero_pos_offset = {0.1F, 0.2F};
  config.zero_offset = {0.0F, 0.0F};
  config.motor_dir = {1.0F, -1.0F};
  config.ct_scale = {2.0F, 3.0F};
  tk::MotorCalibration calibration(config);
  Eigen::Vector2f position(0.4F, 0.8F);
  Eigen::Vector2f velocity(1.5F, -2.0F);
  Eigen::Vector2f current(0.5F, 1.0F);
  const auto state = calibration.convert_feedback(
    position, velocity, current, Eigen::Vector2f::Zero());
  EXPECT_TRUE(state.position.isApprox(Eigen::Vector2f(0.3F, -0.6F)));
  EXPECT_TRUE(state.velocity.isApprox(Eigen::Vector2f(1.5F, 2.0F)));
  EXPECT_TRUE(state.torque.isApprox(Eigen::Vector2f(1.0F, -3.0F)));
  const auto command = calibration.convert_command(
    state.position, state.velocity, state.torque, state.zero_count);
  EXPECT_TRUE(command.position.isApprox(position));
  EXPECT_TRUE(command.velocity.isApprox(velocity));
}

TEST(JointTransmission, LinearRoundTrip)
{
  Eigen::Matrix2f matrix;
  matrix << 0.5F, 0.5F, 0.5F, -0.5F;
  tk::LinearJointTransmission transmission(matrix);
  const tk::TransmissionState motor{
    Eigen::Vector2f(-0.6F, -0.4F),
    Eigen::Vector2f(1.0F, -0.5F),
    Eigen::Vector2f(2.0F, 0.5F)};
  const auto joint = transmission.motor_to_joint(motor);
  EXPECT_TRUE(joint.position.isApprox(Eigen::Vector2f(-0.5F, -0.1F)));
  EXPECT_TRUE(transmission.joint_to_motor(joint).position.isApprox(motor.position));
}

TEST(RobotIo, BuildsLegAndArmGroupsFromConfig)
{
  auto config = tk::load_robot_config(TEST_ROBOT_CONFIG_PATH);
  config.ankle_transmission.kind = "identity";
  tk::RobotIo robot_io(config);
  const auto target = Eigen::Map<const Eigen::VectorXf>(
    config.default_dof_pos.data(), config.default_dof_pos.size());
  const auto kp = Eigen::Map<const Eigen::VectorXf>(
    config.joint_kp.data(), config.joint_kp.size());
  const auto kd = Eigen::Map<const Eigen::VectorXf>(
    config.joint_kd.data(), config.joint_kd.size());
  const auto command = robot_io.build_command(target, kp, kd);
  ASSERT_EQ(command.leg.size(), 12U);
  ASSERT_EQ(command.arm.size(), 8U);
  EXPECT_EQ(command.leg.front().can_id, 51);
  EXPECT_EQ(command.arm.front().can_id, 11);
  EXPECT_FLOAT_EQ(command.leg.front().kp, 700.0F);
}