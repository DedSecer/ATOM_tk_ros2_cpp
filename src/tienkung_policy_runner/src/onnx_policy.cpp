#include "tienkung_policy_runner/onnx_policy.hpp"

#include <stdexcept>
#include <utility>
#include <vector>

#include <onnxruntime_cxx_api.h>

namespace tienkung_policy_runner {

class OnnxPolicy::Impl {
public:
  Impl(const std::string & policy_path, const std::string & device)
  : environment_(ORT_LOGGING_LEVEL_WARNING, "tienkung_policy_runner")
  {
    if (policy_path.empty()) {
      throw std::runtime_error("policy_path is required");
    }
    if (device != "cpu") {
      throw std::runtime_error(
              "This build enables the ONNX Runtime CPU provider only; set device=cpu");
    }
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_options_.SetIntraOpNumThreads(1);
    session_ = std::make_unique<Ort::Session>(
      environment_, policy_path.c_str(), session_options_);

    Ort::AllocatorWithDefaultOptions allocator;
    auto input_name = session_->GetInputNameAllocated(0, allocator);
    auto output_name = session_->GetOutputNameAllocated(0, allocator);
    input_name_ = input_name.get();
    output_name_ = output_name.get();
  }

  Eigen::VectorXf infer(const Eigen::VectorXf & observation)
  {
    std::vector<int64_t> shape{1, observation.size()};
    auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    auto input = Ort::Value::CreateTensor<float>(
      memory,
      const_cast<float *>(observation.data()),
      static_cast<std::size_t>(observation.size()),
      shape.data(),
      shape.size());
    const char * input_names[] = {input_name_.c_str()};
    const char * output_names[] = {output_name_.c_str()};
    auto outputs = session_->Run(
      Ort::RunOptions{nullptr}, input_names, &input, 1, output_names, 1);
    const auto info = outputs.front().GetTensorTypeAndShapeInfo();
    const auto count = static_cast<Eigen::Index>(info.GetElementCount());
    return Eigen::Map<const Eigen::VectorXf>(outputs.front().GetTensorData<float>(), count);
  }

private:
  Ort::Env environment_;
  Ort::SessionOptions session_options_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_;
  std::string output_name_;
};

OnnxPolicy::OnnxPolicy(const std::string & policy_path, const std::string & device)
: impl_(std::make_unique<Impl>(policy_path, device))
{
}

OnnxPolicy::~OnnxPolicy() = default;
OnnxPolicy::OnnxPolicy(OnnxPolicy &&) noexcept = default;
OnnxPolicy & OnnxPolicy::operator=(OnnxPolicy &&) noexcept = default;

Eigen::VectorXf OnnxPolicy::infer(const Eigen::VectorXf & observation)
{
  return impl_->infer(observation);
}

}  // namespace tienkung_policy_runner