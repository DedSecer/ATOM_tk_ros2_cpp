#include "tienkung_policy_runner/policy_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>

#include <builtin_interfaces/msg/time.hpp>
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

builtin_interfaces::msg::Time time_message(const rclcpp::Time & time)
{
  constexpr int64_t nanoseconds_per_second = 1'000'000'000;
  const int64_t nanoseconds = time.nanoseconds();
  builtin_interfaces::msg::Time message;
  message.sec = static_cast<int32_t>(nanoseconds / nanoseconds_per_second);
  message.nanosec = static_cast<uint32_t>(nanoseconds % nanoseconds_per_second);
  return message;
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
      parameter<double>("zero_duration_sec"), parameter<double>("motion_reference_interpolation_sec"),
      mode_ == PolicyNodeMode::DryRun);
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
        mode_ == PolicyNodeMode::DryRun ? "dry_run_log" : "policy_log",
        static_cast<std::size_t>(std::max<int64_t>(
          0, parameter<int64_t>("runtime_log_queue_capacity"))));
      TelemetryMetadata metadata;
      metadata.config = config_;
      metadata.policy_path = parameter<std::string>("policy_path");
      metadata.robot_config_path = parameter<std::string>("robot_config_path");
      metadata.manifest_path = parameter<std::string>("manifest_path");
      metadata.device = parameter<std::string>("device");
      metadata.dry_run = mode_ == PolicyNodeMode::DryRun;
      metadata.require_motion_source = require_motion_source_;
      metadata.zero_duration_sec = parameter<double>("zero_duration_sec");
      metadata.motion_reference_interpolation_sec =
        parameter<double>("motion_reference_interpolation_sec");
      metadata.state_timeout_sec = state_timeout_sec_;
      metadata.motion_timeout_sec = motion_timeout_sec_;
      for (const auto * name : {"motion_reference_topic", "control_mode_topic", "leg_status_topic",
        "arm_status_topic", "head_status_topic", "waist_status_topic", "imu_status_topic",
        "joy_topic", "leg_command_topic", "arm_command_topic", "waist_command_topic"})
      {
        metadata.topics[name] = parameter<std::string>(name);
      }
      logger_->write_metadata(std::move(metadata));
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
    node_.declare_parameter("motion_reference_interpolation_sec", 2.0);
    node_.declare_parameter("state_timeout_sec", 0.1);
    node_.declare_parameter("motion_timeout_sec", 0.1);
    node_.declare_parameter("require_motion_source", false);
    node_.declare_parameter("report_interval_sec", 5.0);
    node_.declare_parameter("motion_reference_topic", "/tienkung/motion_reference");
    node_.declare_parameter("control_mode_topic", "/tienkung/control_mode");
    node_.declare_parameter("leg_status_topic", "/leg/status");
    node_.declare_parameter("arm_status_topic", "/arm/status");
    node_.declare_parameter("head_status_topic", "/head/status");
    node_.declare_parameter("waist_status_topic", "/waist/status");
    node_.declare_parameter("imu_status_topic", "/imu/status");
    node_.declare_parameter("joy_topic", "/sbus_data");
    node_.declare_parameter("leg_command_topic", "/leg/cmd_ctrl");
    node_.declare_parameter("arm_command_topic", "/arm/cmd_ctrl");
    node_.declare_parameter("waist_command_topic", "/waist/cmd_pos");
    node_.declare_parameter("enable_runtime_log", false);
    node_.declare_parameter("log_dir", "/tmp");
    node_.declare_parameter("runtime_log_queue_capacity", 256);
  }

  void create_ros_entities()
  {
    using std::placeholders::_1;
    motion_subscription_ = node_.create_subscription<tienkung_interfaces::msg::MotionReference>(
      parameter<std::string>("motion_reference_topic"), 10,
      std::bind(&Impl::on_motion, this, _1));
    leg_subscription_ = node_.create_subscription<bodyctrl_msgs::msg::MotorStatusMsg>(
      parameter<std::string>("leg_status_topic"), 10,
      std::bind(&Impl::on_leg, this, _1));
    arm_subscription_ = node_.create_subscription<bodyctrl_msgs::msg::MotorStatusMsg>(
      parameter<std::string>("arm_status_topic"), 10,
      std::bind(&Impl::on_arm, this, _1));
    head_subscription_ = node_.create_subscription<bodyctrl_msgs::msg::MotorStatusMsg>(
      parameter<std::string>("head_status_topic"), 10,
      std::bind(&Impl::on_head, this, _1));
    waist_subscription_ = node_.create_subscription<bodyctrl_msgs::msg::MotorStatusMsg>(
      parameter<std::string>("waist_status_topic"), 10,
      std::bind(&Impl::on_waist, this, _1));
    imu_subscription_ = node_.create_subscription<bodyctrl_msgs::msg::Imu>(
      parameter<std::string>("imu_status_topic"), 10,
      std::bind(&Impl::on_imu, this, _1));
    joy_subscription_ = node_.create_subscription<sensor_msgs::msg::Joy>(
      parameter<std::string>("joy_topic"), 10,
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

  void on_head(const bodyctrl_msgs::msg::MotorStatusMsg::SharedPtr message)
  {
    robot_io_->ingest_head_status(motor_samples(*message), now_sec());
  }

  void on_waist(const bodyctrl_msgs::msg::MotorStatusMsg::SharedPtr message)
  {
    robot_io_->ingest_waist_status(motor_samples(*message), now_sec());
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
    joy_timestamp_sec_ = now_sec();
    std::vector<int> buttons(message->buttons.begin(), message->buttons.end());
    robot_io_->ingest_joystick(decode_joy_message(message->axes, buttons));
  }

  void publish_control_mode(ControlMode mode)
  {
    tienkung_interfaces::msg::ControlMode message;
    message.header.stamp = time_message(node_.get_clock()->now());
    message.mode = static_cast<unsigned char>(mode);
    control_mode_publisher_->publish(message);
  }

  void publish_robot_command(const RobotCommand & command, PublishedCommand * telemetry)
  {
    bodyctrl_msgs::msg::CmdMotorCtrl leg_message;
    bodyctrl_msgs::msg::CmdMotorCtrl arm_message;
    bodyctrl_msgs::msg::CmdSetMotorPosition waist_message;
    const auto stamp = time_message(node_.get_clock()->now());
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
    if (telemetry) {
      telemetry->stamp_sec = stamp.sec;
      telemetry->stamp_nanosec = stamp.nanosec;
      const auto copy_motor = [](const auto & message, auto & destination) {
          for (const auto & item : message.cmds) {
            destination.push_back({static_cast<int64_t>(item.name),
              static_cast<double>(item.kp), static_cast<double>(item.kd),
              static_cast<double>(item.pos), static_cast<double>(item.spd),
              static_cast<double>(item.tor)});
          }
        };
      copy_motor(leg_message, telemetry->leg);
      copy_motor(arm_message, telemetry->arm);
      for (const auto & item : waist_message.cmds) {
        telemetry->waist.push_back({static_cast<int64_t>(item.name),
          static_cast<double>(item.pos), static_cast<double>(item.spd),
          static_cast<double>(item.cur)});
      }
    }
  }

  void tick()
  {
    const auto tick_started = std::chrono::steady_clock::now();
    std::optional<double> tick_dt;
    if (previous_tick_) {
      tick_dt = std::chrono::duration<double>(tick_started - *previous_tick_).count();
    }
    previous_tick_ = tick_started;
    JoystickCommand joystick;
    if (robot_io_->last_joystick()) {
      joystick = *robot_io_->last_joystick();
    }
    const double now = now_sec();
    const auto state = robot_io_->snapshot();
    const auto fresh = [now, this](double stamp) {
        return stamp > 0.0 && now - stamp <= state_timeout_sec_;
      };
    const bool head_ready = !config_.require_head_status || state.head_complete;
    const bool waist_ready = config_.waist_indices.empty() || state.waist_complete;
    const bool state_ready = state.leg_complete && state.arm_complete && head_ready && waist_ready &&
      state.fixed_arm_complete && fresh(state.leg_timestamp_sec) && fresh(state.arm_timestamp_sec) &&
      (config_.require_head_status ? fresh(state.head_timestamp_sec) : true) &&
      (!config_.waist_indices.empty() ? fresh(state.waist_timestamp_sec) : true) &&
      fresh(state.imu_timestamp_sec);
    const bool motion_ready = !require_motion_source_ ||
      (motion_timestamp_sec_ > 0.0 && now - motion_timestamp_sec_ <= motion_timeout_sec_);
    const bool motion_reference_valid = motion_timestamp_sec_ > 0.0 &&
      now - motion_timestamp_sec_ <= motion_timeout_sec_ &&
      latest_motion_.size() == static_cast<Eigen::Index>(config_.observation.n_mimic_obs);
    // Telemetry observes the original early return without executing the FSM or publishing.
    if (!robot_io_->last_joystick() && mode_ == PolicyNodeMode::Production) {
      RuntimeStep idle;
      idle.target_position = state.dof_position;
      log_sample(now, state, idle, state_ready, motion_ready, motion_reference_valid,
        false, std::nullopt, motion_timestamp_sec_, tick_started, tick_dt, 0.0, 0.0);
      return;
    }
    const double input_motion_timestamp = motion_timestamp_sec_;
    const auto runtime_started = std::chrono::steady_clock::now();
    const auto step = runtime_->step(
      now, state, joystick, latest_motion_, state_ready, motion_ready,
      motion_reference_valid);
    const double runtime_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - runtime_started).count();
    publish_control_mode(step.mode);

    if (step.mode == ControlMode::Stop) {
      latest_motion_ = default_mimic_observation(config_);
      motion_timestamp_sec_ = 0.0;
    }

    const bool publish = mode_ == PolicyNodeMode::Production ||
      step.mode == ControlMode::Zero || step.mode == ControlMode::Stop;
    std::optional<PublishedCommand> published_command;
    const auto publish_started = std::chrono::steady_clock::now();
    if (publish) {
      if (logger_) {published_command.emplace();}
      const Eigen::VectorXf effective_kp = step.mode == ControlMode::Stop ?
        Eigen::VectorXf::Zero(kp_.size()) : kp_;
      publish_robot_command(robot_io_->build_command(
        step.target_position, effective_kp, kd_, Eigen::VectorXf(), Eigen::VectorXf(),
        step.fixed_arm_target_position), published_command ? &*published_command : nullptr);
    }

    const double publish_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - publish_started).count();
    log_sample(now, state, step, state_ready, motion_ready, motion_reference_valid,
      true, std::move(published_command), input_motion_timestamp,
      tick_started, tick_dt, runtime_ms, publish_ms);

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

  void log_sample(
    double now, const BodyState & state, const RuntimeStep & step,
    bool state_ready, bool motion_ready, bool motion_reference_valid, bool runtime_executed,
    std::optional<PublishedCommand> command, double input_motion_timestamp,
    std::chrono::steady_clock::time_point started, std::optional<double> tick_dt,
    double runtime_ms, double publish_ms)
  {
    if (!logger_) {return;}
    TelemetrySample sample;
    sample.timestamp = now;
    sample.mode = step.mode;
    sample.observation = step.observation;
    sample.target = step.target_position;
    // Preserve the legacy field's post-STOP-reset semantics.
    sample.motion_reference = latest_motion_;
    sample.raw_action = step.raw_action;
    sample.effective_motion_reference = step.effective_mimic;
    sample.state = state;
    sample.inference_executed = step.inference_executed;
    sample.inference_latency_ms = step.inference_latency_ms;
    sample.command_published = command.has_value();
    sample.command_published_mask.assign(config_.motor_num, false);
    if (command) {
      const auto mark = [&](const auto & items) {
          for (const auto & item : items) {
            const auto found = config_.index_by_can_id.find(static_cast<int>(item.name));
            if (found != config_.index_by_can_id.end()) {
              sample.command_published_mask[found->second] = true;
            }
          }
        };
      mark(command->leg); mark(command->arm); mark(command->waist);
    }
    sample.command = std::move(command);
    sample.state_ready = state_ready;
    sample.motion_ready = motion_ready;
    sample.motion_reference_valid = motion_reference_valid;
    sample.runtime_executed = runtime_executed;
    const auto age = [now](double stamp) -> std::optional<double> {
        return stamp > 0.0 ? std::optional<double>(now - stamp) : std::nullopt;
      };
    sample.input_age_sec = {
      {"leg", age(state.leg_timestamp_sec)}, {"arm", age(state.arm_timestamp_sec)},
      {"head", age(state.head_timestamp_sec)}, {"waist", age(state.waist_timestamp_sec)},
      {"imu", age(state.imu_timestamp_sec)}, {"motion", age(input_motion_timestamp)},
      {"joy", joy_timestamp_sec_ ? std::optional<double>(now - *joy_timestamp_sec_) : std::nullopt}};
    sample.motor_feedback = robot_io_->feedback_snapshot();
    sample.steady_timestamp_sec = std::chrono::duration<double>(
      started.time_since_epoch()).count();
    sample.tick_dt_sec = tick_dt;
    sample.runtime_ms = runtime_ms;
    sample.command_publish_ms = publish_ms;
    // Excludes enqueue, asynchronous serialization/disk IO, and diagnostic console reporting.
    sample.processing_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    logger_->write(std::move(sample));
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
  std::optional<double> joy_timestamp_sec_;
  std::optional<std::chrono::steady_clock::time_point> previous_tick_;
  double state_timeout_sec_{};
  double motion_timeout_sec_{};
  double report_interval_sec_{};
  double last_report_sec_{};
  bool require_motion_source_{};
  ControlMode previous_mode_{ControlMode::Stop};

  rclcpp::Subscription<tienkung_interfaces::msg::MotionReference>::SharedPtr motion_subscription_;
  rclcpp::Subscription<bodyctrl_msgs::msg::MotorStatusMsg>::SharedPtr leg_subscription_;
  rclcpp::Subscription<bodyctrl_msgs::msg::MotorStatusMsg>::SharedPtr arm_subscription_;
  rclcpp::Subscription<bodyctrl_msgs::msg::MotorStatusMsg>::SharedPtr head_subscription_;
  rclcpp::Subscription<bodyctrl_msgs::msg::MotorStatusMsg>::SharedPtr waist_subscription_;
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