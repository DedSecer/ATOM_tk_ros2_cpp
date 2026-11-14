#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <variant>

#include "tienkung_policy_runner/robot_io.hpp"

namespace tienkung_policy_runner {

struct TelemetryMetadata {
  RobotConfig config;
  std::string policy_path;
  std::string robot_config_path;
  std::string manifest_path;
  std::string device;
  bool dry_run{};
  bool require_motion_source{};
  double zero_duration_sec{};
  double motion_reference_interpolation_sec{};
  double state_timeout_sec{};
  double motion_timeout_sec{};
  std::map<std::string, std::string> topics;
};

// Values copied from the ROS messages, after assignment to the wire field types.
struct PublishedMotorCommand {
  int64_t name{};
  double kp{}, kd{}, pos{}, spd{}, tor{};
};
struct PublishedWaistCommand {
  int64_t name{};
  double pos{}, spd{}, cur{};
};
struct PublishedCommand {
  int32_t stamp_sec{};
  uint32_t stamp_nanosec{};
  std::vector<PublishedMotorCommand> leg, arm;
  std::vector<PublishedWaistCommand> waist;
};

struct TelemetrySample {
  double timestamp{};
  ControlMode mode{ControlMode::Stop};
  Eigen::VectorXf observation, target, motion_reference, raw_action, effective_motion_reference;
  BodyState state;
  bool inference_executed{};
  double inference_latency_ms{};
  bool command_published{}, state_ready{}, motion_ready{}, motion_reference_valid{};
  bool runtime_executed{};
  std::vector<bool> command_published_mask;
  // Ages use ROS receipt time, not message header time. Unseen inputs are null.
  std::map<std::string, std::optional<double>> input_age_sec;
  std::optional<double> tick_dt_sec;
  double steady_timestamp_sec{};
  double processing_ms{};
  double runtime_ms{};
  double command_publish_ms{};
  MotorFeedbackSnapshot motor_feedback;
  std::optional<PublishedCommand> command;
};

// Serialization is public for offline schema tests; production calls it only on the writer thread.
std::string serialize_metadata(const TelemetryMetadata & metadata, std::size_t queue_capacity);
std::string serialize_sample(const TelemetrySample & sample, uint64_t dropped_records);

class JsonlLogger {
public:
  JsonlLogger(
    const std::string & directory, const std::string & prefix, std::size_t queue_capacity = 256);
  ~JsonlLogger();
  JsonlLogger(const JsonlLogger &) = delete;
  JsonlLogger & operator=(const JsonlLogger &) = delete;
  bool is_open() const noexcept;
  const std::string & path() const noexcept;
  // Startup only, before any samples. Ensures metadata is the first queued record.
  void write_metadata(TelemetryMetadata metadata);
  bool write(TelemetrySample sample);
  // Legacy source API. Legacy JSON fields are also preserved in all sample records.
  void write(
    double timestamp, ControlMode mode, const Eigen::VectorXf & observation,
    const Eigen::VectorXf & target, const Eigen::VectorXf & dof_position,
    const Eigen::VectorXf & dof_velocity, const Eigen::VectorXf & motion_reference);
  // Drain accepted records, flush, and join. Idempotent; no producer may outlive this object.
  void shutdown();
  uint64_t dropped_records() const noexcept;
  uint64_t written_records() const noexcept;
  uint64_t write_errors() const noexcept;

private:
  void writer_loop() noexcept;
  using Record = std::variant<TelemetryMetadata, TelemetrySample>;
  std::string path_;
  std::ofstream stream_;
  const std::size_t queue_capacity_;
  std::mutex mutex_;
  std::mutex shutdown_mutex_;
  std::condition_variable wake_;
  std::deque<Record> queue_;
  bool stopping_{false};
  bool metadata_written_{false};
  std::atomic<bool> open_{false};
  std::atomic<uint64_t> dropped_{0}, written_{0}, errors_{0};
  std::thread writer_;
};

}  // namespace tienkung_policy_runner
