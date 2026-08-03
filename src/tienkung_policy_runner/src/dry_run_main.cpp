#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "tienkung_policy_runner/policy_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<tienkung_policy_runner::PolicyNode>(
        tienkung_policy_runner::PolicyNodeMode::DryRun));
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("dry_run_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}