#pragma once

#include <memory>
#include <string>

#include <Eigen/Core>

namespace tienkung_policy_runner {

class OnnxPolicy {
public:
  OnnxPolicy(const std::string & policy_path, const std::string & device);
  ~OnnxPolicy();
  OnnxPolicy(OnnxPolicy &&) noexcept;
  OnnxPolicy & operator=(OnnxPolicy &&) noexcept;
  OnnxPolicy(const OnnxPolicy &) = delete;
  OnnxPolicy & operator=(const OnnxPolicy &) = delete;

  Eigen::VectorXf infer(const Eigen::VectorXf & observation);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tienkung_policy_runner