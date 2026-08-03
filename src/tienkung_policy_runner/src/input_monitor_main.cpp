#include <algorithm>
#include <chrono>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <bodyctrl_msgs/msg/imu.hpp>
#include <bodyctrl_msgs/msg/motor_status_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tienkung_interfaces/msg/control_mode.hpp>
#include <tienkung_interfaces/msg/motion_reference.hpp>

#include "tienkung_policy_runner/observation_builder.hpp"
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

template<typename Derived>
std::string format_vector(const Eigen::MatrixBase<Derived> & values)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << '[';
  for (Eigen::Index index = 0; index < values.size(); ++index) {
    if (index != 0) {
      stream << ' ';
    }
    stream << std::showpos << values[index];
  }
  stream << ']';
  return stream.str();
}

}  // namespace

class InputMonitorNode final : public rclcpp::Node {
public:
  InputMonitorNode()
  : Node("tienkung_input_monitor")
  {
    declare_parameter("robot_config_path", "");
    declare_parameter("print_hz", 1.0);
    declare_parameter("verbose", false);
    declare_parameter("leg_status_topic", "/leg/status");
    declare_parameter("arm_status_topic", "/arm/status");
    declare_parameter("imu_status_topic", "/imu/status");
    declare_parameter("motion_reference_topic", "/tienkung/motion_reference");
    declare_parameter("joy_topic", "/sbus_data");
    declare_parameter("control_mode_topic", "/tienkung/control_mode");

    config_ = load_robot_config(get_parameter("robot_config_path").as_string());
    robot_io_ = std::make_unique<RobotIo>(config_);
    motion_ = default_mimic_observation(config_);
    verbose_ = get_parameter("verbose").as_bool();

    leg_subscription_ = create_subscription<bodyctrl_msgs::msg::MotorStatusMsg>(
      get_parameter("leg_status_topic").as_string(), 100,
      [this](const bodyctrl_msgs::msg::MotorStatusMsg::SharedPtr message) {
        robot_io_->ingest_leg_status(motor_samples(*message), now_sec());
        ++leg_count_;
      });
    arm_subscription_ = create_subscription<bodyctrl_msgs::msg::MotorStatusMsg>(
      get_parameter("arm_status_topic").as_string(), 100,
      [this](const bodyctrl_msgs::msg::MotorStatusMsg::SharedPtr message) {
        robot_io_->ingest_arm_status(motor_samples(*message), now_sec());
        ++arm_count_;
      });
    imu_subscription_ = create_subscription<bodyctrl_msgs::msg::Imu>(
      get_parameter("imu_status_topic").as_string(), 100,
      [this](const bodyctrl_msgs::msg::Imu::SharedPtr message) {
        robot_io_->ingest_imu(
          Eigen::Vector3f(message->euler.roll, message->euler.pitch, message->euler.yaw),
          Eigen::Vector3f(
            message->angular_velocity.x, message->angular_velocity.y,
            message->angular_velocity.z), now_sec());
        ++imu_count_;
      });
    motion_subscription_ = create_subscription<tienkung_interfaces::msg::MotionReference>(
      get_parameter("motion_reference_topic").as_string(), 10,
      [this](const tienkung_interfaces::msg::MotionReference::SharedPtr message) {
        if (message->body_mimic.size() == config_.observation.n_mimic_obs) {
          motion_ = Eigen::Map<const Eigen::VectorXf>(
            message->body_mimic.data(), message->body_mimic.size());
        }
        ++motion_count_;
      });
    joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
      get_parameter("joy_topic").as_string(), 100,
      [this](const sensor_msgs::msg::Joy::SharedPtr message) {
        joy_axes_ = message->axes;
        joy_buttons_.assign(message->buttons.begin(), message->buttons.end());
        ++joy_count_;
      });
    mode_subscription_ = create_subscription<tienkung_interfaces::msg::ControlMode>(
      get_parameter("control_mode_topic").as_string(), 10,
      [this](const tienkung_interfaces::msg::ControlMode::SharedPtr message) {
        control_mode_ = message->mode;
        ++mode_count_;
      });

    const double print_hz = std::max(0.1, get_parameter("print_hz").as_double());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / print_hz),
      std::bind(&InputMonitorNode::print_summary, this));
  }

private:
  double now_sec() const
  {
    return get_clock()->now().seconds();
  }

  void print_summary()
  {
    const auto state = robot_io_->snapshot();
    const auto defaults = Eigen::Map<const Eigen::VectorXf>(
      config_.default_dof_pos.data(), config_.default_dof_pos.size());
    Eigen::VectorXf proprio(config_.observation.n_proprio);
    ObservationBuilder builder(config_);
    proprio = builder.build_proprio(
      state.angular_velocity, state.rpy, state.dof_position, state.dof_velocity,
      Eigen::VectorXf::Zero(config_.actions_size));

    std::ostringstream stream;
    stream << "\n========================================================================\n"
           << "Input counts leg=" << leg_count_ << " arm=" << arm_count_
           << " imu=" << imu_count_ << " motion=" << motion_count_
           << " joy=" << joy_count_ << " mode=" << mode_count_ << '\n'
           << "Control mode: " << mode_name(static_cast<ControlMode>(control_mode_)) << '\n'
           << "RPY: " << format_vector(state.rpy) << '\n'
           << "Angular velocity: " << format_vector(state.angular_velocity) << '\n'
           << "Joint position: " << format_vector(state.dof_position) << '\n'
           << "Joint offset: " << format_vector(state.dof_position - defaults) << '\n'
           << "Joint velocity: " << format_vector(state.dof_velocity) << '\n'
           << "Motion reference: " << format_vector(motion_) << '\n'
           << "Proprio norm: " << proprio.norm() << " dim=" << proprio.size();
    if (verbose_) {
      stream << "\nJoy axes=" << joy_axes_.size() << " buttons=" << joy_buttons_.size();
    }
    stream << "\n========================================================================";
    RCLCPP_INFO(get_logger(), "%s", stream.str().c_str());
  }

  RobotConfig config_;
  std::unique_ptr<RobotIo> robot_io_;
  Eigen::VectorXf motion_;
  std::vector<float> joy_axes_;
  std::vector<int> joy_buttons_;
  bool verbose_{};
  unsigned char control_mode_{};
  std::size_t leg_count_{};
  std::size_t arm_count_{};
  std::size_t imu_count_{};
  std::size_t motion_count_{};
  std::size_t joy_count_{};
  std::size_t mode_count_{};

  rclcpp::Subscription<bodyctrl_msgs::msg::MotorStatusMsg>::SharedPtr leg_subscription_;
  rclcpp::Subscription<bodyctrl_msgs::msg::MotorStatusMsg>::SharedPtr arm_subscription_;
  rclcpp::Subscription<bodyctrl_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<tienkung_interfaces::msg::MotionReference>::SharedPtr motion_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Subscription<tienkung_interfaces::msg::ControlMode>::SharedPtr mode_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tienkung_policy_runner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<tienkung_policy_runner::InputMonitorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("input_monitor_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}