#include "tienkung_policy_runner/policy_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <bodyctrl_msgs/msg/cmd_motor_ctrl.hpp>
#include <bodyctrl_msgs/msg/cmd_set_motor_position.hpp>
#include <bodyctrl_msgs/msg/imu.hpp>
#include <bodyctrl_msgs/msg/motor_ctrl.hpp>
#include <bodyctrl_msgs/msg/motor_status_msg.hpp>
#include <bodyctrl_msgs/msg/set_motor_position.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tienkung_interfaces/msg/control_mode.hpp>
#include <tienkung_interfaces/msg/motion_reference.hpp>

#include "tienkung_policy_runner/jsonl_logger.hpp"
#include "tienkung_policy_runner/observation_builder.hpp"
#include "tienkung_policy_runner/policy_runtime.hpp"
#include "tienkung_policy_runner/robot_config.hpp"
#include "tienkung_policy_runner/robot_io.hpp"

namespace tienkung_policy_runner {
namespace {

std::vector<MotorSample> motor_samples(const bodyctrl_msgs::msg::MotorStatusMsg & message)
{
  std::vector<MotorSample> samples;
  samples.reserve(message.status.size());
  for (const auto & status : message.status) {
    samples.push_back({
      static_cast<int>(status.name), static_cast<float>(status.pos),
      static_cast<float>(status.speed), static_cast<float>(status.current)});
  }
  return samples;
}

Eigen::VectorXf map_copy(const std::vector<float> & values)
{
  return Eigen::Map<const Eigen::VectorXf>(
    values.data(), static_cast<Eigen::Index>(values.size()));
}

}  // namespace

class PolicyNode::Impl {
public:
  Impl(PolicyNode & node, PolicyNodeMode mode)
  : node_(node), mode_(mode)
  {
    declare_parameters();
    config_ = load_robot_config(parameter<std::string>("robot_config_path"));
    validate_manifest(parameter<std::string>("manifest_path"), config_);
    robot_io_ = std::make_unique<RobotIo>(config_);
    runtime_ = std::make_unique<PolicyRuntime>(
      config_, parameter<std::string>("policy_path"), parameter<std::string>("device"),
      parameter<double>("zero_duration_sec"), mode_ == PolicyNodeMode::DryRun);
    latest_motion_ = default_mimic_observation(config_);
    kp_ = map_copy(config_.joint_kp);
    kd_ = map_copy(config_.joint_kd);
    state_timeout_sec_ = parameter<double>("state_timeout_sec");
    motion_timeout_sec_ = parameter<double>("motion_timeout_sec");
    require_motion_source_ = parameter<bool>("require_motion_source");
    report_interval_sec_ = parameter<double>("report_interval_sec");
    if (parameter<bool>("enable_runtime_log")) {
      logger_ = std::make_unique<JsonlLogger>(
        parameter<std::string>("log_dir"),
        mode_ == PolicyNodeMode::DryRun ? "dry_run_log" : "policy_log");
    }
    create_ros_entities();

    RCLCPP_INFO(
      node_.get_logger(),
      "%s started with config='%s', policy='%s', action_scale=%.3f, "
      "clip_actions=%.3f, raw_action_limit=%.3f",
      mode_ == PolicyNodeMode::DryRun ? "Dry-run node" : "Policy runner",
      parameter<std::string>("robot_config_path").c_str(),
      parameter<std::string>("policy_path").c_str(), config_.action_scale, config_.clip_actions,
      config_.clip_actions / config_.action_scale);
    if (logger_) {
      if (logger_->is_open()) {
        RCLCPP_INFO(node_.get_logger(), "Logging runtime data to %s", logger_->path().c_str());
      } else {
        RCLCPP_WARN(node_.get_logger(), "Runtime log could not be opened");
      }
    } else {
      RCLCPP_INFO(node_.get_logger(), "Runtime data logging is disabled");
    }
    if (mode_ == PolicyNodeMode::DryRun) {
      RCLCPP_WARN(
        node_.get_logger(),
        "Dry-run safety: ZERO sends real motor commands; POLICY performs inference only");
    }
  }

private:
  template<typename T>
  T parameter(const char * name) const
  {
    return node_.get_parameter(name).get_value<T>();
  }

  void declare_parameters()
  {
    node_.declare_parameter("device", "cpu");
    node_.declare_parameter("policy_path", "");
    node_.declare_parameter("manifest_path", "");
    node_.declare_parameter("robot_config_path", "");
    node_.declare_parameter("zero_duration_sec", 2.0);
    node_.declare_parameter("state_timeout_sec", 0.1);
    node_.declare_parameter("motion_timeout_sec", 0.1);
    node_.declare_parameter("require_motion_source", false);
    node_.declare_parameter("report_interval_sec", 5.0);
    node_.declare_parameter("motion_reference_topic", "/tienkung/motion_reference");
    node_.declare_parameter("control_mode_topic", "/tienkung/control_mode");
    node_.declare_parameter("leg_status_topic", "/leg/status");
    node_.declare_parameter("arm_status_topic", "/arm/status");
    node_.declare_parameter("imu_status_topic", "/imu/status");
    node_.declare_parameter("joy_topic", "/sbus_data");
    node_.declare_parameter("leg_command_topic", "/leg/cmd_ctrl");
    node_.declare_parameter("arm_command_topic", "/arm/cmd_ctrl");
    node_.declare_parameter("waist_command_topic", "/waist/cmd_pos");
    node_.declare_parameter("enable_runtime_log", false);
    node_.declare_parameter("log_dir", "/tmp");
  }

  void create_ros_entities()
  {
    using std::placeholders::_1;
    motion_subscription_ = node_.create_subscription<tienkung_interfaces::msg::MotionReference>(
      parameter<std::string>("motion_reference_topic"), 10,
      std::bind(&Impl::on_motion, this, _1));
    leg_subscription_ = node_.create_subscription<bodyctrl_msgs::msg::MotorStatusMsg>(
      parameter<std::string>("leg_status_topic"), 100,
      std::bind(&Impl::on_leg, this, _1));
    arm_subscription_ = node_.create_subscription<bodyctrl_msgs::msg::MotorStatusMsg>(
      parameter<std::string>("arm_status_topic"), 100,
      std::bind(&Impl::on_arm, this, _1));
    imu_subscription_ = node_.create_subscription<bodyctrl_msgs::msg::Imu>(
      parameter<std::string>("imu_status_topic"), 100,
      std::bind(&Impl::on_imu, this, _1));
    joy_subscription_ = node_.create_subscription<sensor_msgs::msg::Joy>(
      parameter<std::string>("joy_topic"), 100,
      std::bind(&Impl::on_joy, this, _1));

    control_mode_publisher_ = node_.create_publisher<tienkung_interfaces::msg::ControlMode>(
      parameter<std::string>("control_mode_topic"), 10);
    leg_command_publisher_ = node_.create_publisher<bodyctrl_msgs::msg::CmdMotorCtrl>(
      parameter<std::string>("leg_command_topic"), 10);
    arm_command_publisher_ = node_.create_publisher<bodyctrl_msgs::msg::CmdMotorCtrl>(
      parameter<std::string>("arm_command_topic"), 10);
    waist_command_publisher_ = node_.create_publisher<bodyctrl_msgs::msg::CmdSetMotorPosition>(
      parameter<std::string>("waist_command_topic"), 10);

    const double frequency = config_.policy_frequency_hz;
    if (!(frequency > 0.0)) {
      throw std::runtime_error("policy_frequency_hz must be positive");
    }
    timer_ = node_.create_wall_timer(
      std::chrono::duration<double>(1.0 / frequency), std::bind(&Impl::tick, this));
  }

  double now_sec() const
  {
    return node_.get_clock()->now().seconds();
  }

  void on_motion(const tienkung_interfaces::msg::MotionReference::SharedPtr message)
  {
    if (message->body_mimic.size() != config_.observation.n_mimic_obs) {
      RCLCPP_WARN(
        node_.get_logger(), "Ignoring motion reference with dim=%zu, expected=%zu",
        message->body_mimic.size(), config_.observation.n_mimic_obs);
      return;
    }
    latest_motion_ = map_copy(message->body_mimic);
    motion_timestamp_sec_ = now_sec();
  }

  void on_leg(const bodyctrl_msgs::msg::MotorStatusMsg::SharedPtr message)
  {
    robot_io_->ingest_leg_status(motor_samples(*message), now_sec());
  }

  void on_arm(const bodyctrl_msgs::msg::MotorStatusMsg::SharedPtr message)
  {
    robot_io_->ingest_arm_status(motor_samples(*message), now_sec());
  }

  void on_imu(const bodyctrl_msgs::msg::Imu::SharedPtr message)
  {
    robot_io_->ingest_imu(
      Eigen::Vector3f(
        message->euler.roll, message->euler.pitch, message->euler.yaw),
      Eigen::Vector3f(
        message->angular_velocity.x, message->angular_velocity.y,
        message->angular_velocity.z),
      now_sec());
  }

  void on_joy(const sensor_msgs::msg::Joy::SharedPtr message)
  {
    std::vector<int> buttons(message->buttons.begin(), message->buttons.end());
    robot_io_->ingest_joystick(decode_joy_message(message->axes, buttons));
  }

  void publish_control_mode(ControlMode mode)
  {
    tienkung_interfaces::msg::ControlMode message;
    message.header.stamp = node_.get_clock()->now().to_msg();
    message.mode = static_cast<unsigned char>(mode);
    control_mode_publisher_->publish(message);
  }

  void publish_robot_command(const RobotCommand & command)
  {
    bodyctrl_msgs::msg::CmdMotorCtrl leg_message;
    bodyctrl_msgs::msg::CmdMotorCtrl arm_message;
    bodyctrl_msgs::msg::CmdSetMotorPosition waist_message;
    const auto stamp = node_.get_clock()->now().to_msg();
    leg_message.header.stamp = stamp;
    arm_message.header.stamp = stamp;
    waist_message.header.stamp = stamp;

    auto append_motor = [](auto & destination, const MotorCommand & source) {
        bodyctrl_msgs::msg::MotorCtrl message;
        message.name = source.can_id;
        message.kp = source.kp;
        message.kd = source.kd;
        message.pos = source.position;
        message.spd = source.velocity;
        message.tor = source.torque;
        destination.cmds.push_back(message);
      };
    for (const auto & item : command.leg) {
      append_motor(leg_message, item);
    }
    for (const auto & item : command.arm) {
      append_motor(arm_message, item);
    }
    for (const auto & item : command.waist) {
      bodyctrl_msgs::msg::SetMotorPosition message;
      message.name = item.can_id;
      message.pos = item.position;
      message.spd = item.speed;
      message.cur = item.current;
      waist_message.cmds.push_back(message);
    }
    leg_command_publisher_->publish(leg_message);
    arm_command_publisher_->publish(arm_message);
    waist_command_publisher_->publish(waist_message);
  }

  void tick()
  {
    JoystickCommand joystick;
    if (robot_io_->last_joystick()) {
      joystick = *robot_io_->last_joystick();
    } else if (mode_ == PolicyNodeMode::Production) {
      return;
    }
    const double now = now_sec();
    const auto state = robot_io_->snapshot();
    const auto fresh = [now, this](double stamp) {
        return stamp > 0.0 && now - stamp <= state_timeout_sec_;
      };
    const bool state_ready = state.leg_complete && state.arm_complete &&
      fresh(state.leg_timestamp_sec) && fresh(state.arm_timestamp_sec) &&
      fresh(state.imu_timestamp_sec);
    const bool motion_ready = !require_motion_source_ ||
      (motion_timestamp_sec_ > 0.0 && now - motion_timestamp_sec_ <= motion_timeout_sec_);
    const auto step = runtime_->step(
      now, state, joystick, latest_motion_, state_ready, motion_ready);
    publish_control_mode(step.mode);

    const bool publish = mode_ == PolicyNodeMode::Production || step.mode == ControlMode::Zero;
    if (publish) {
      const Eigen::VectorXf effective_kp = step.mode == ControlMode::Stop ?
        Eigen::VectorXf::Zero(kp_.size()) : kp_;
      publish_robot_command(robot_io_->build_command(step.target_position, effective_kp, kd_));
    }

    if (logger_) {
      logger_->write(
        now, step.mode, step.observation, step.target_position, state.dof_position,
        state.dof_velocity, latest_motion_);
    }

    if (step.mode != previous_mode_) {
      RCLCPP_INFO(
        node_.get_logger(), "Control mode changed: %s -> %s",
        mode_name(previous_mode_), mode_name(step.mode));
      previous_mode_ = step.mode;
    }
    if (mode_ == PolicyNodeMode::DryRun && now - last_report_sec_ >= report_interval_sec_) {
      RCLCPP_INFO(
        node_.get_logger(),
        "Dry-run mode=%s state_ready=%s motion_ready=%s inference=%s latency=%.3f ms action_norm=%.4f",
        mode_name(step.mode), state_ready ? "true" : "false",
        motion_ready ? "true" : "false", step.inference_executed ? "true" : "false",
        step.inference_latency_ms,
        step.raw_action.size() > 0 ? step.raw_action.norm() : 0.0F);
      last_report_sec_ = now;
    }
  }

  PolicyNode & node_;
  PolicyNodeMode mode_;
  RobotConfig config_;
  std::unique_ptr<RobotIo> robot_io_;
  std::unique_ptr<PolicyRuntime> runtime_;
  std::unique_ptr<JsonlLogger> logger_;
  Eigen::VectorXf latest_motion_;
  Eigen::VectorXf kp_;
  Eigen::VectorXf kd_;
  double motion_timestamp_sec_{};
  double state_timeout_sec_{};
  double motion_timeout_sec_{};
  double report_interval_sec_{};
  double last_report_sec_{};
  bool require_motion_source_{};
  ControlMode previous_mode_{ControlMode::Stop};

  rclcpp::Subscription<tienkung_interfaces::msg::MotionReference>::SharedPtr motion_subscription_;
  rclcpp::Subscription<bodyctrl_msgs::msg::MotorStatusMsg>::SharedPtr leg_subscription_;
  rclcpp::Subscription<bodyctrl_msgs::msg::MotorStatusMsg>::SharedPtr arm_subscription_;
  rclcpp::Subscription<bodyctrl_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Publisher<tienkung_interfaces::msg::ControlMode>::SharedPtr control_mode_publisher_;
  rclcpp::Publisher<bodyctrl_msgs::msg::CmdMotorCtrl>::SharedPtr leg_command_publisher_;
  rclcpp::Publisher<bodyctrl_msgs::msg::CmdMotorCtrl>::SharedPtr arm_command_publisher_;
  rclcpp::Publisher<bodyctrl_msgs::msg::CmdSetMotorPosition>::SharedPtr waist_command_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

PolicyNode::PolicyNode(PolicyNodeMode mode)
: rclcpp::Node(
    mode == PolicyNodeMode::DryRun ? "tienkung_dry_run" : "tienkung_policy_runner"),
  impl_(std::make_unique<Impl>(*this, mode))
{
}

PolicyNode::~PolicyNode() = default;

}  // namespace tienkung_policy_runner