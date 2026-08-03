#pragma once

#include <fstream>
#include <string>

#include <Eigen/Core>

#include "tienkung_policy_runner/fsm.hpp"

namespace tienkung_policy_runner {

class JsonlLogger {
public:
  JsonlLogger(const std::string & directory, const std::string & prefix);
  bool is_open() const noexcept;
  const std::string & path() const noexcept;
  void write(
    double timestamp,
    ControlMode mode,
    const Eigen::VectorXf & observation,
    const Eigen::VectorXf & target,
    const Eigen::VectorXf & dof_position,
    const Eigen::VectorXf & dof_velocity,
    const Eigen::VectorXf & motion_reference);

private:
  static void write_vector(std::ostream & stream, const Eigen::VectorXf & value);

  std::string path_;
  std::ofstream stream_;
};

}  // namespace tienkung_policy_runner