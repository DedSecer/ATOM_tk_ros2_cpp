#include "tienkung_policy_runner/jsonl_logger.hpp"

#include <chrono>
#include <filesystem>
#include <iomanip>

namespace tienkung_policy_runner {

JsonlLogger::JsonlLogger(const std::string & directory, const std::string & prefix)
{
  const auto epoch = std::chrono::duration_cast<std::chrono::seconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  path_ = (std::filesystem::path(directory) /
    (prefix + "_" + std::to_string(epoch) + ".jsonl")).string();
  stream_.open(path_);
  if (stream_) {
    stream_ << std::setprecision(9);
  }
}

bool JsonlLogger::is_open() const noexcept
{
  return stream_.is_open();
}

const std::string & JsonlLogger::path() const noexcept
{
  return path_;
}

void JsonlLogger::write_vector(std::ostream & stream, const Eigen::VectorXf & value)
{
  stream << '[';
  for (Eigen::Index index = 0; index < value.size(); ++index) {
    if (index != 0) {
      stream << ',';
    }
    stream << value[index];
  }
  stream << ']';
}

void JsonlLogger::write(
  double timestamp,
  ControlMode mode,
  const Eigen::VectorXf & observation,
  const Eigen::VectorXf & target,
  const Eigen::VectorXf & dof_position,
  const Eigen::VectorXf & dof_velocity,
  const Eigen::VectorXf & motion_reference)
{
  if (!stream_) {
    return;
  }
  stream_ << "{\"timestamp\":" << timestamp << ",\"mode\":\"" << mode_name(mode) <<
    "\",\"obs\":";
  write_vector(stream_, observation);
  stream_ << ",\"target\":";
  write_vector(stream_, target);
  stream_ << ",\"dof_pos\":";
  write_vector(stream_, dof_position);
  stream_ << ",\"dof_vel\":";
  write_vector(stream_, dof_velocity);
  stream_ << ",\"motion_ref\":";
  write_vector(stream_, motion_reference);
  stream_ << "}\n";
  stream_.flush();
}

}  // namespace tienkung_policy_runner