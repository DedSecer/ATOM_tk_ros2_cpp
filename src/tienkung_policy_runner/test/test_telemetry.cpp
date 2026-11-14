#include <atomic>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <locale>
#include <stdexcept>
#include <thread>

#include "tienkung_policy_runner/jsonl_logger.hpp"

// Standalone macOS test only: native symbols fail closed and are never invoked.
#ifdef TELEMETRY_STANDALONE
extern "C" {
void * funcsptrans_create() {throw std::runtime_error("native unavailable in telemetry test");}
void funcsptrans_destroy(void *) {}
int funcsptrans_state_to_joint(void *, const double *, const double *, const double *,
  double *, double *, double *) {return -1;}
int funcsptrans_joint_to_motor(void *, const double *, const double *, const double *,
  double *, double *, double *) {return -1;}
}
#endif

namespace tk = tienkung_policy_runner;

void check(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

std::vector<std::string> lines(const std::string & path)
{
  std::ifstream file(path);
  check(file.good(), "log did not open");
  std::vector<std::string> result;
  std::string line;
  while (std::getline(file, line)) {result.push_back(line);}
  return result;
}

struct CommaPunctuation : std::numpunct<char> {
  char do_decimal_point() const override {return ',';}
};

int main(int argc, char ** argv)
{
  try {
    const auto directory = argc > 1 ? std::filesystem::path(argv[1]) :
      std::filesystem::temp_directory_path();
    tk::RobotConfig config;
    config.robot = "fixture";
    config.motor_num = config.actions_size = 3;
    config.joint_order_policy = {"leg", "arm", "unseen_head"};
    config.can_id_by_index = {51, 11, 1};
    config.index_by_can_id = {{51, 0}, {11, 1}, {1, 2}};
    config.leg_indices = {0}; config.arm_indices = {1}; config.head_indices = {2};
    config.zero_pos_offset = {0, 0, 0}; config.zero_offset = {0, 0, 0};
    config.motor_dir = {1, -1, 1}; config.ct_scale = {2, 3, 1};
    config.wrap_threshold_rad = 3.14F;
    config.ankle_transmission.kind = "identity";
    config.ankle_transmission.motor_indices = {0};
    config.ankle_transmission.joint_indices = {0};
    tk::RobotIo io(config);
    auto initial = io.feedback_snapshot();
    check(!initial.valid[2] && !initial.raw_valid[2] && std::isnan(initial.raw_current[2]),
      "unseen head falsely marked measured");
    check(!initial.head.timestamp_sec, "unseen group has timestamp");
    io.ingest_leg_status({{51, 0.25F, 0.5F, 1.5F}, {999, 3, 4, 5}}, 1.0);
    io.ingest_arm_status({{11, 0.5F, 1.0F, 2.0F}}, 1.1);
    const auto before = io.snapshot();
    const auto snapshot = io.feedback_snapshot();
    check(snapshot.valid == std::vector<bool>({true, true, false}), "wrong validity mask");
    check(snapshot.current[0] == 1.5F && snapshot.raw_current[1] == 2.0F,
      "current not in policy order");
    check(snapshot.leg.samples[1].can_id == 999, "raw unknown CAN lost");
    check(before.dof_torque[0] == 3.0F && before.dof_torque[1] == -6.0F,
      "estimated torque changed");
    check(io.snapshot().dof_position == before.dof_position, "snapshot changed state");
    io.ingest_leg_status({{51, 10.0F, 10.0F, 10.0F}}, 2.0);
    check(io.feedback_snapshot().current[0] == 1.5F, "jump filter changed");
    check(io.feedback_snapshot().raw_current[0] == 10.0F, "unfiltered current lost");
    io.ingest_head_status({{1, std::numeric_limits<float>::quiet_NaN(), 0, 0}}, 2.1);
    check(!io.feedback_snapshot().valid[2] && !io.feedback_snapshot().raw_valid[2],
      "invalid head accepted");
    check(snapshot.leg.timestamp_sec == 1.0, "snapshot aliases live state");

    tk::TelemetryMetadata metadata;
    metadata.config = config;
    metadata.policy_path = "/tmp/a\"b\\c\n\t.onnx";
    metadata.robot_config_path = "/tmp/机器人.yaml";
    metadata.manifest_path = "";
    tk::TelemetrySample sample;
    sample.timestamp = 1.25;
    sample.observation = Eigen::Vector3f(1, std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity());
    sample.target = Eigen::Vector3f::Ones();
    sample.state = io.snapshot();
    sample.motor_feedback = io.feedback_snapshot();
    sample.raw_action = Eigen::Vector3f(0.1F, 0.2F, 0.3F);
    sample.motion_reference = Eigen::Vector3f(4, 5, 6);
    sample.effective_motion_reference = Eigen::Vector3f(1, 2, 3);
    sample.command_published_mask = {true, true, false};
    sample.command_published = true;
    sample.command.emplace();
    sample.command->stamp_sec = 42;
    sample.command->stamp_nanosec = 123;
    sample.command->leg.push_back({51, 20, 2, 0.25, 0.5, 0});
    sample.command->arm.push_back({11, 30, 3, 0.5, 1, 0});
    sample.command->waist.push_back({31, 0.75, 0.2, 0.3});
    sample.input_age_sec = {{"head", std::nullopt}, {"leg", 0.1}};
    sample.processing_ms = std::numeric_limits<double>::infinity();
    std::locale::global(std::locale(std::locale::classic(), new CommaPunctuation));
    const auto json = tk::serialize_sample(sample, 7);
    check(json.find("\"timestamp\":1.25") != std::string::npos, "locale corrupted JSON");
    check(json.find("\"obs\":[1,null,null]") != std::string::npos, "nonfinite not null");
    check(json.find("\"command_published_mask\":[true,true,false]") != std::string::npos,
      "publication mask missing");
    check(json.find("\"processing_ms\":null") != std::string::npos, "timing nonfinite not null");
    check(tk::serialize_metadata(metadata, 16).find("a\\\"b\\\\c\\u000a\\u0009.onnx") !=
      std::string::npos, "path not escaped");
    check(tk::serialize_metadata(metadata, 16).find("机器人") != std::string::npos,
      "UTF-8 path corrupted");
    auto invalid_path = metadata;
    invalid_path.manifest_path = std::string(1, static_cast<char>(0xff));
    check(tk::serialize_metadata(invalid_path, 16).find("\\u00ff") != std::string::npos,
      "invalid UTF-8 byte not escaped");
    std::locale::global(std::locale::classic());

    // A stable schema fixture for a strict JSON parser (no additional C++ JSON dependency).
    const auto fixture = directory / "telemetry_schema_fixture.jsonl";
    {
      std::ofstream out(fixture);
      out << tk::serialize_metadata(metadata, 16) << '\n' << json << '\n';
      sample.command.reset(); sample.command_published = false;
      sample.command_published_mask.assign(3, false);
      out << tk::serialize_sample(sample, 8) << '\n';
    }

    tk::JsonlLogger logger(directory.string(), "telemetry_test", 16);
    check(logger.is_open(), "logger failed to open");
    logger.write_metadata(metadata);
    uint64_t accepted = 0;
    constexpr uint64_t attempts = 3000;
    for (uint64_t i = 0; i < attempts; ++i) {
      sample.timestamp = static_cast<double>(i);
      if (logger.write(sample)) {++accepted;}
    }
    logger.shutdown();
    logger.shutdown();
    check(!logger.is_open(), "logger open after shutdown");
    check(logger.written_records() == accepted + 1, "accepted records not drained");
    check(logger.dropped_records() == attempts - accepted, "wrong drop count");
    check(logger.write_errors() == 0, "unexpected writer error");
    auto records = lines(logger.path());
    check(records.size() == accepted + 2, "missing final statistics");
    check(records.front().find("\"type\":\"metadata\"") != std::string::npos,
      "metadata not first");
    check(records.back().find("\"type\":\"logger_stats\"") != std::string::npos,
      "statistics not last");
    check(!logger.write(sample), "write accepted after shutdown");

    tk::JsonlLogger zero(directory.string(), "telemetry_zero", 0);
    zero.write_metadata(metadata);
    check(!zero.write(sample), "zero capacity accepted sample");
    zero.shutdown();
    check(zero.dropped_records() == 1 && lines(zero.path()).size() == 2,
      "zero-capacity statistics wrong");

    tk::JsonlLogger failed((directory / "missing_directory").string(), "test");
    check(!failed.is_open() && !failed.write(sample), "failed-open writer accepted sample");
    failed.shutdown();

    std::string destructor_path;
    uint64_t destructor_accepted = 0;
    {
      tk::JsonlLogger draining(directory.string(), "telemetry_destructor", 64);
      destructor_path = draining.path();
      for (int i = 0; i < 64; ++i) {
        if (draining.write(sample)) {++destructor_accepted;}
      }
    }
    check(lines(destructor_path).size() == destructor_accepted + 1, "destructor lost records");

    tk::JsonlLogger concurrent(directory.string(), "telemetry_concurrent", 16);
    std::atomic<uint64_t> concurrent_accepted{0};
    const auto produce = [&] {
        for (int i = 0; i < 1000; ++i) {
          if (concurrent.write(sample)) {++concurrent_accepted;}
        }
      };
    std::thread a(produce), b(produce);
    concurrent.shutdown();
    a.join(); b.join();
    check(concurrent.written_records() == concurrent_accepted, "shutdown race lost accepted records");
    check(concurrent.dropped_records() + concurrent_accepted == 2000, "shutdown race count mismatch");
    std::cout << "Telemetry serialization, snapshots, queue bounds, and shutdown tests passed.\n";
    std::cout << "Strict-parser fixture: " << fixture << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
