#include "tienkung_policy_runner/jsonl_logger.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <type_traits>
#include <utility>

namespace tienkung_policy_runner {
namespace {

void value(std::ostream & out, const std::string & text)
{
  out << '"';
  constexpr char hex[] = "0123456789abcdef";
  for (std::size_t i = 0; i < text.size(); ++i) {
    const auto ch = static_cast<unsigned char>(text[i]);
    // Preserve valid UTF-8; escape invalid filesystem bytes rather than emit invalid JSON.
    if (ch >= 0x80) {
      const std::size_t length = ch >= 0xc2 && ch <= 0xdf ? 2 :
        (ch >= 0xe0 && ch <= 0xef ? 3 : (ch >= 0xf0 && ch <= 0xf4 ? 4 : 0));
      bool valid = length != 0 && i + length <= text.size();
      for (std::size_t j = 1; valid && j < length; ++j) {
        const auto next = static_cast<unsigned char>(text[i + j]);
        valid = next >= 0x80 && next <= 0xbf;
        if (j == 1) {
          valid = valid && !(ch == 0xe0 && next < 0xa0) && !(ch == 0xed && next > 0x9f) &&
            !(ch == 0xf0 && next < 0x90) && !(ch == 0xf4 && next > 0x8f);
        }
      }
      if (valid) {
        out.write(text.data() + i, static_cast<std::streamsize>(length));
        i += length - 1;
        continue;
      }
    }
    if (ch == '"' || ch == '\\') {
      out << '\\' << static_cast<char>(ch);
    } else if (ch < 0x20 || ch >= 0x80) {
      // Invalid bytes and ASCII controls use JSON escapes.
      out << "\\u00" << hex[ch >> 4] << hex[ch & 15];
    } else {
      out << static_cast<char>(ch);
    }
  }
  out << '"';
}
void value(std::ostream & out, const char * text) {value(out, std::string(text));}
void value(std::ostream & out, bool number) {out << (number ? "true" : "false");}
template<typename T, std::enable_if_t<std::is_arithmetic_v<T>, int> = 0>
void value(std::ostream & out, T number)
{
  if constexpr (std::is_floating_point_v<T>) {
    if (!std::isfinite(number)) {
      out << "null";
      return;
    }
  }
  out << number;
}
template<typename T>
void value(std::ostream & out, const std::optional<T> & item)
{
  if (item) {value(out, *item);} else {out << "null";}
}
template<typename T>
void value(std::ostream & out, const Eigen::MatrixBase<T> & items)
{
  out << '[';
  for (Eigen::Index i = 0; i < items.size(); ++i) {
    if (i) {out << ',';}
    value(out, items[i]);
  }
  out << ']';
}
template<typename T>
void value(std::ostream & out, const std::vector<T> & items)
{
  out << '[';
  for (std::size_t i = 0; i < items.size(); ++i) {
    if (i) {out << ',';}
    value(out, static_cast<T>(items[i]));
  }
  out << ']';
}

class Object {
public:
  explicit Object(std::ostream & out) : out_(out) {out_ << '{';}
  ~Object() {out_ << '}';}
  template<typename T>
  void field(const char * name, const T & item)
  {
    key(name);
    value(out_, item);
  }
  template<typename F>
  void nested(const char * name, F write)
  {
    key(name);
    write(out_);
  }
private:
  void key(const char * name)
  {
    if (!first_) {out_ << ',';}
    first_ = false;
    value(out_, name);
    out_ << ':';
  }
  std::ostream & out_;
  bool first_{true};
};

template<typename T>
void write_map(std::ostream & out, const std::map<std::string, T> & items)
{
  Object object(out);
  for (const auto & item : items) {object.field(item.first.c_str(), item.second);}
}

template<typename T, typename F>
void write_array(std::ostream & out, const std::vector<T> & items, F write)
{
  out << '[';
  bool first = true;
  for (const auto & item : items) {
    if (!first) {out << ',';}
    first = false;
    write(out, item);
  }
  out << ']';
}

void write_config(std::ostream & out, const RobotConfig & v)
{
  Object o(out);
#define FIELD(name) o.field(#name, v.name)
  FIELD(robot); FIELD(motor_num); FIELD(actions_size); FIELD(policy_frequency_hz);
  FIELD(joint_order_policy); FIELD(ros_lite_name_by_index); FIELD(can_id_by_index);
  FIELD(leg_indices); FIELD(arm_indices); FIELD(head_indices); FIELD(waist_indices);
  FIELD(require_head_status); FIELD(default_dof_pos); FIELD(blocked_action_indices);
  FIELD(action_scale); FIELD(clip_actions); FIELD(joint_kp); FIELD(joint_kd);
  FIELD(zero_pos_offset); FIELD(zero_offset); FIELD(motor_dir); FIELD(ct_scale);
  FIELD(wrap_threshold_rad); FIELD(enable_jump_filter);
  o.nested("observation", [&](auto & stream) {
    Object o(stream);
    const auto & c = v.observation;
    o.field("n_mimic_obs", c.n_mimic_obs); o.field("n_proprio", c.n_proprio);
    o.field("n_obs_single", c.n_obs_single); o.field("history_len", c.history_len);
    o.field("total_obs_size", c.total_obs_size);
    o.field("angular_velocity_scale", c.angular_velocity_scale);
    o.field("joint_velocity_scale", c.joint_velocity_scale);
  });
  o.nested("ankle_transmission", [&](auto & stream) {
    Object o(stream);
    const auto & c = v.ankle_transmission;
    o.field("kind", c.kind); o.field("motor_indices", c.motor_indices);
    o.field("joint_indices", c.joint_indices); o.field("require_native", c.require_native);
  });
  o.nested("waist_command", [&](auto & stream) {
    Object o(stream);
    const auto & c = v.waist_command;
    o.field("ids", c.ids); o.field("position", c.position);
    o.field("speed", c.speed); o.field("current", c.current);
  });
  o.nested("fixed_arm_command", [&](auto & stream) {
    Object o(stream);
    const auto & c = v.fixed_arm_command;
    o.field("ids", c.ids); o.field("position", c.position);
    o.field("kp", c.kp); o.field("kd", c.kd);
  });
#undef FIELD
}

void write_feedback(std::ostream & out, const MotorFeedbackSnapshot & feedback, const BodyState & state)
{
  Object object(out);
  const auto group = [&](const MotorFeedbackGroup & data, bool complete) {
      return [&data, complete](auto & stream) {
        Object o(stream);
        o.field("timestamp_sec", data.timestamp_sec);
        o.field("complete", complete);
        o.nested("samples", [&](auto & s) {
          write_array(s, data.samples, [](auto & s, const MotorSample & v) {
            Object o(s);
            o.field("can_id", v.can_id);
            o.field("position", v.position);
            o.field("velocity", v.velocity);
            o.field("current", v.current);
            o.field("finite", std::isfinite(v.position) && std::isfinite(v.velocity) &&
              std::isfinite(v.current));
          });
        });
      };
    };
  object.nested("leg", group(feedback.leg, state.leg_complete));
  object.nested("arm", group(feedback.arm, state.arm_complete));
  object.nested("head", group(feedback.head, state.head_complete));
  object.nested("waist", group(feedback.waist, state.waist_complete));
  object.field("raw_current", feedback.raw_current);
  object.field("raw_valid", feedback.raw_valid);
  object.nested("filtered", [&](auto & s) {
    Object o(s);
    o.field("position", feedback.position);
    o.field("velocity", feedback.velocity);
    o.field("current", feedback.current);
    o.field("valid", feedback.valid);
    o.field("fixed_arm_valid", feedback.fixed_arm_valid);
    o.field("fixed_arm_complete", state.fixed_arm_complete);
  });
}

void write_command(std::ostream & out, const PublishedCommand & command)
{
  Object o(out);
  o.field("stamp_sec", command.stamp_sec);
  o.field("stamp_nanosec", command.stamp_nanosec);
  const auto motor = [](auto & s, const PublishedMotorCommand & v) {
      Object o(s);
      o.field("name", v.name); o.field("kp", v.kp); o.field("kd", v.kd);
      o.field("pos", v.pos); o.field("spd", v.spd); o.field("tor", v.tor);
    };
  o.nested("leg", [&](auto & s) {write_array(s, command.leg, motor);});
  o.nested("arm", [&](auto & s) {write_array(s, command.arm, motor);});
  o.nested("waist", [&](auto & s) {
    write_array(s, command.waist, [](auto & s, const PublishedWaistCommand & v) {
      Object o(s);
      o.field("name", v.name); o.field("pos", v.pos);
      o.field("spd", v.spd); o.field("cur", v.cur);
    });
  });
}

std::ostringstream json_stream()
{
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(std::numeric_limits<double>::max_digits10);
  return out;
}

}  // namespace

std::string serialize_metadata(const TelemetryMetadata & v, std::size_t queue_capacity)
{
  auto out = json_stream();
  {
    Object o(out);
    o.field("type", "metadata"); o.field("schema_version", 2);
    o.field("joint_order_policy", v.config.joint_order_policy);
    o.field("can_id_by_index", v.config.can_id_by_index);
    o.field("policy_path", v.policy_path);
    o.field("robot_config_path", v.robot_config_path);
    o.field("manifest_path", v.manifest_path);
    o.field("path_encoding", "UTF-8; invalid bytes escaped as U+00xx");
    o.field("timestamp_clock", "ROS receipt clock, seconds");
    o.field("timing_clock", "steady_clock");
    o.field("queue_capacity", queue_capacity);
    o.nested("config", [&](auto & s) {write_config(s, v.config);});
    o.nested("runtime", [&](auto & s) {
      Object o(s);
      o.field("device", v.device); o.field("dry_run", v.dry_run);
      o.field("require_motion_source", v.require_motion_source);
      o.field("zero_duration_sec", v.zero_duration_sec);
      o.field("motion_reference_interpolation_sec", v.motion_reference_interpolation_sec);
      o.field("state_timeout_sec", v.state_timeout_sec);
      o.field("motion_timeout_sec", v.motion_timeout_sec);
      o.nested("topics", [&](auto & stream) {write_map(stream, v.topics);});
    });
  }
  return out.str();
}

std::string serialize_sample(const TelemetrySample & v, uint64_t dropped_records)
{
  auto out = json_stream();
  {
    Object o(out);
    o.field("type", "sample"); o.field("schema_version", 2);
    o.field("timestamp", v.timestamp); o.field("mode", mode_name(v.mode));
    o.field("obs", v.observation); o.field("target", v.target);
    o.field("dof_pos", v.state.dof_position); o.field("dof_vel", v.state.dof_velocity);
    o.field("motion_ref", v.motion_reference);
    o.field("raw_action", v.raw_action);
    o.field("effective_motion_ref", v.effective_motion_reference);
    o.field("rpy", v.state.rpy); o.field("angular_velocity", v.state.angular_velocity);
    o.field("dof_torque_estimated", v.state.dof_torque);
    o.field("inference_executed", v.inference_executed);
    o.field("inference_latency_ms", v.inference_latency_ms);
    o.field("command_published", v.command_published);
    o.field("command_published_mask", v.command_published_mask);
    o.field("state_ready", v.state_ready); o.field("motion_ready", v.motion_ready);
    o.field("motion_reference_valid", v.motion_reference_valid);
    o.field("runtime_executed", v.runtime_executed);
    o.field("motion_reference_state", v.motion_reference_valid ? "fresh" : "stale_or_missing");
    o.field("dropped_records", dropped_records);
    o.nested("input_age_sec", [&](auto & s) {write_map(s, v.input_age_sec);});
    o.nested("timing", [&](auto & s) {
      Object o(s);
      o.field("steady_timestamp_sec", v.steady_timestamp_sec);
      o.field("tick_dt_sec", v.tick_dt_sec);
      o.field("processing_ms", v.processing_ms);
      o.field("runtime_ms", v.runtime_ms);
      o.field("command_publish_ms", v.command_publish_ms);
    });
    o.nested("motor_feedback", [&](auto & s) {write_feedback(s, v.motor_feedback, v.state);});
    o.nested("command", [&](auto & s) {
      if (v.command) {write_command(s, *v.command);} else {s << "null";}
    });
  }
  return out.str();
}

JsonlLogger::JsonlLogger(
  const std::string & directory, const std::string & prefix, std::size_t queue_capacity)
: queue_capacity_(queue_capacity)
{
  static std::atomic<uint64_t> sequence{0};
  const auto epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  path_ = (std::filesystem::path(directory) /
    (prefix + "_" + std::to_string(epoch) + "_" + std::to_string(sequence++) + ".jsonl")).string();
  stream_.imbue(std::locale::classic());
  stream_.open(path_);
  open_ = stream_.is_open() && stream_.good();
  if (open_) {writer_ = std::thread(&JsonlLogger::writer_loop, this);}
}

JsonlLogger::~JsonlLogger() {shutdown();}
bool JsonlLogger::is_open() const noexcept {return open_.load();}
const std::string & JsonlLogger::path() const noexcept {return path_;}
uint64_t JsonlLogger::dropped_records() const noexcept {return dropped_.load();}
uint64_t JsonlLogger::written_records() const noexcept {return written_.load();}
uint64_t JsonlLogger::write_errors() const noexcept {return errors_.load();}

void JsonlLogger::write_metadata(TelemetryMetadata metadata)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!open_ || stopping_ || metadata_written_) {return;}
  metadata_written_ = true;
  // One startup record may exceed the sample bound (including a zero-capacity queue).
  queue_.emplace_front(std::move(metadata));
  wake_.notify_one();
}

bool JsonlLogger::write(TelemetrySample sample)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock() || !open_ || stopping_ || queue_.size() >= queue_capacity_) {
    ++dropped_;
    return false;
  }
  queue_.emplace_back(std::move(sample));
  lock.unlock();
  wake_.notify_one();
  return true;
}

void JsonlLogger::write(
  double timestamp, ControlMode mode, const Eigen::VectorXf & observation,
  const Eigen::VectorXf & target, const Eigen::VectorXf & dof_position,
  const Eigen::VectorXf & dof_velocity, const Eigen::VectorXf & motion_reference)
{
  TelemetrySample sample;
  sample.timestamp = timestamp; sample.mode = mode; sample.observation = observation;
  sample.target = target; sample.state.dof_position = dof_position;
  sample.state.dof_velocity = dof_velocity; sample.motion_reference = motion_reference;
  write(std::move(sample));
}

void JsonlLogger::shutdown()
{
  std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  wake_.notify_one();
  if (writer_.joinable()) {writer_.join();}
  open_ = false;
}

void JsonlLogger::writer_loop() noexcept
{
  bool record_in_flight = false;
  try {
    while (true) {
      Record record;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait(lock, [&] {return stopping_ || !queue_.empty();});
        if (queue_.empty()) {break;}
        record = std::move(queue_.front());
        queue_.pop_front();
        record_in_flight = true;
      }
      const auto line = std::holds_alternative<TelemetryMetadata>(record) ?
        serialize_metadata(std::get<TelemetryMetadata>(record), queue_capacity_) :
        serialize_sample(std::get<TelemetrySample>(record), dropped_.load());
      stream_ << line << '\n';
      if (!stream_) {throw std::ios_base::failure("runtime log write failed");}
      ++written_;
      record_in_flight = false;
      // Flush on the writer thread so even small/idle logs become visible promptly.
      // No disk IO or serialization runs in the control callback.
      stream_.flush();
      if (!stream_) {throw std::ios_base::failure("runtime log flush failed");}
    }
    stream_ << "{\"type\":\"logger_stats\",\"schema_version\":2,\"dropped_records\":" <<
      dropped_.load() << ",\"written_records\":" << written_.load() <<
      ",\"write_errors\":" << errors_.load() << "}\n";
    stream_.flush();
    if (!stream_) {throw std::ios_base::failure("runtime log flush failed");}
  } catch (...) {
    ++errors_;
    open_ = false;
    std::lock_guard<std::mutex> lock(mutex_);
    dropped_ += queue_.size() + (record_in_flight ? 1U : 0U);
    queue_.clear();
  }
  stream_.close();
  open_ = false;
}

}  // namespace tienkung_policy_runner
