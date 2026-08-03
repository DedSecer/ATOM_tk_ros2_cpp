#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace tienkung_policy_runner {

enum class PolicyNodeMode { Production, DryRun };

class PolicyNode : public rclcpp::Node {
public:
  explicit PolicyNode(PolicyNodeMode mode);
  ~PolicyNode() override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tienkung_policy_runner