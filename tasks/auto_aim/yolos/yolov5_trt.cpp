#include "yolov5_trt.hpp"

#include <yaml-cpp/yaml.h>

#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "tasks/auto_aim/yolos/yolov5_postprocess.hpp"
#include "tools/logger.hpp"

#ifdef HAVE_TENSORRT
#include <NvInfer.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>
#endif

namespace auto_aim
{
#ifdef HAVE_TENSORRT
namespace
{
// TensorRT 的日志回调（TRT 要求必须提供，且签名上不能抛异常）
class TrtLogger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity == Severity::kERROR || severity == Severity::kINTERNAL_ERROR) {
      tools::logger()->error("[YOLOV5_TRT] {}", msg);
    } else if (severity == Severity::kWARNING) {
      tools::logger()->warn("[YOLOV5_TRT] {}", msg);
    } else {
      tools::logger()->debug("[YOLOV5_TRT] {}", msg);
    }
  }
};

void cuda_check(cudaError_t code, const char * what)
{
  if (code != cudaSuccess) {
    throw std::runtime_error(
      std::string("[YOLOV5_TRT] CUDA ") + what + " failed: " + cudaGetErrorString(code));
  }
}

std::size_t dims_volume(const nvinfer1::Dims & dims)
{
  std::size_t v = 1;
  for (int i = 0; i < dims.nbDims; i++) {
    if (dims.d[i] <= 0) {
      throw std::runtime_error(
        "[YOLOV5_TRT] 引擎 binding 含动态维度，暂不支持（请用固定 shape 导出）");
    }
    v *= static_cast<std::size_t>(dims.d[i]);
  }
  return v;
}

std::size_t dtype_size(nvinfer1::DataType dtype)
{
  switch (dtype) {
    case nvinfer1::DataType::kFLOAT:
      return 4;
    case nvinfer1::DataType::kHALF:
      return 2;
    case nvinfer1::DataType::kINT32:
      return 4;
    default:
      throw std::runtime_error("[YOLOV5_TRT] 不支持的 binding 数据类型");
  }
}

std::string dtype_name(nvinfer1::DataType dtype)
{
  switch (dtype) {
    case nvinfer1::DataType::kFLOAT:
      return "fp32";
    case nvinfer1::DataType::kHALF:
      return "fp16";
    case nvinfer1::DataType::kINT32:
      return "int32";
    default:
      return "other";
  }
}

std::string dims_to_string(const nvinfer1::Dims & dims)
{
  std::string s = "[";
  for (int i = 0; i < dims.nbDims; i++) {
    s += std::to_string(dims.d[i]);
    if (i + 1 < dims.nbDims) s += ", ";
  }
  return s + "]";
}
}  // namespace

struct YOLOV5_TRT::Impl
{
  TrtLogger logger;  // 必须在 runtime 之前构造、之后析构
  nvinfer1::IRuntime * runtime = nullptr;
  nvinfer1::ICudaEngine * engine = nullptr;
  nvinfer1::IExecutionContext * context = nullptr;
  cudaStream_t stream = nullptr;

  int input_index = -1, output_index = -1;
  std::size_t input_elems = 0, output_elems = 0;
  nvinfer1::DataType output_dtype = nvinfer1::DataType::kFLOAT;

  void * dev_input = nullptr;
  void * dev_output = nullptr;
  std::vector<float> host_input;
  std::vector<float> host_output_float;
  std::vector<__half> host_output_half;
  std::vector<void *> bindings;

  ~Impl() { release(); }

  void release() noexcept
  {
    if (context) {
      context->destroy();
      context = nullptr;
    }
    if (engine) {
      engine->destroy();
      engine = nullptr;
    }
    if (runtime) {
      runtime->destroy();
      runtime = nullptr;
    }
    if (stream) {
      cudaStreamDestroy(stream);
      stream = nullptr;
    }
    if (dev_input) {
      cudaFree(dev_input);
      dev_input = nullptr;
    }
    if (dev_output) {
      cudaFree(dev_output);
      dev_output = nullptr;
    }
  }
};
#else
// 本机未装 TensorRT（CMake 未找到 libnvinfer）：保留空实现，构造函数直接抛异常
struct YOLOV5_TRT::Impl
{
};
#endif

YOLOV5_TRT::YOLOV5_TRT(const std::string & config_path, bool debug)
: debug_(debug), impl_(std::make_unique<Impl>())
{
  auto yaml = YAML::LoadFile(config_path);

  engine_path_ = yaml["yolov5_trt_engine_path"] ? yaml["yolov5_trt_engine_path"].as<std::string>()
                                                : std::string("assets/yolov5_0526_fp16.engine");
  min_confidence_ = yaml["min_confidence"].as<double>();
  use_roi_ = yaml["use_roi"] ? yaml["use_roi"].as<bool>() : false;

  int x = 0, y = 0, width = 0, height = 0;
  if (use_roi_) {
    x = yaml["roi"]["x"].as<int>();
    y = yaml["roi"]["y"].as<int>();
    width = yaml["roi"]["width"].as<int>();
    height = yaml["roi"]["height"].as<int>();
  }
  roi_ = cv::Rect(x, y, width, height);
  offset_ = cv::Point2f(x, y);

  if (yaml["use_traditional"] && yaml["use_traditional"].as<bool>()) {
    configure_traditional(config_path, "YOLOV5_TRT");
  }

#ifdef HAVE_TENSORRT
  // 1. 读引擎文件（由 trtexec 离线生成，见头文件注释 / AGENTS.md §4.0）
  std::ifstream file(engine_path_, std::ios::binary);
  if (!file) {
    throw std::runtime_error(
      "[YOLOV5_TRT] 打不开引擎文件: " + engine_path_ +
      "（请先用 trtexec 生成: /usr/src/tensorrt/bin/trtexec --onnx=assets/yolov5_0526.onnx "
      "--saveEngine=assets/yolov5_0526_fp16.engine --fp16）");
  }
  file.seekg(0, std::ios::end);
  auto file_size = static_cast<std::size_t>(file.tellg());
  file.seekg(0, std::ios::beg);
  std::vector<char> blob(file_size);
  file.read(blob.data(), static_cast<std::streamsize>(file_size));
  file.close();

  // 2. 反序列化 + 建 context
  impl_->runtime = nvinfer1::createInferRuntime(impl_->logger);
  if (!impl_->runtime) throw std::runtime_error("[YOLOV5_TRT] createInferRuntime failed");

  impl_->engine = impl_->runtime->deserializeCudaEngine(blob.data(), file_size);
  if (!impl_->engine) {
    throw std::runtime_error(
      "[YOLOV5_TRT] deserializeCudaEngine failed: " + engine_path_ +
      "（引擎与当前 TensorRT/CUDA 版本不匹配时需用 trtexec 重新生成）");
  }
  impl_->context = impl_->engine->createExecutionContext();
  if (!impl_->context) throw std::runtime_error("[YOLOV5_TRT] createExecutionContext failed");

  // 3. 绑定：约定一个输入 + 一个输出，输入固定 1x3x640x640
  auto nb = impl_->engine->getNbBindings();
  for (int i = 0; i < static_cast<int>(nb); i++) {
    auto name = std::string(impl_->engine->getBindingName(i));
    auto dims = impl_->engine->getBindingDimensions(i);
    if (impl_->engine->bindingIsInput(i)) {
      impl_->input_index = i;
      impl_->input_elems = dims_volume(dims);
      if (
        dims.nbDims != 4 || dims.d[0] != 1 || dims.d[1] != 3 || dims.d[2] != 640 ||
        dims.d[3] != 640) {
        throw std::runtime_error(
          "[YOLOV5_TRT] 输入 " + name + " 形状 " + dims_to_string(dims) + " 不是 [1, 3, 640, 640]");
      }
      tools::logger()->info(
        "[YOLOV5_TRT] binding[{}] \"{}\" input {}", i, name, dims_to_string(dims));
    } else {
      impl_->output_index = i;
      impl_->output_elems = dims_volume(dims);
      impl_->output_dtype = impl_->engine->getBindingDataType(i);
      tools::logger()->info(
        "[YOLOV5_TRT] binding[{}] \"{}\" output {} {}", i, name, dims_to_string(dims),
        dtype_name(impl_->output_dtype));
    }
  }
  if (impl_->input_index < 0 || impl_->output_index < 0) {
    throw std::runtime_error("[YOLOV5_TRT] 引擎必须有且仅有一个输入 / 一个输出");
  }

  // 4. 分配 host / device buffer
  impl_->host_input.resize(impl_->input_elems);
  if (impl_->output_dtype == nvinfer1::DataType::kHALF) {
    impl_->host_output_half.resize(impl_->output_elems);
  } else {
    impl_->host_output_float.resize(impl_->output_elems);
  }
  impl_->bindings.resize(nb, nullptr);

  cuda_check(
    cudaMalloc(&impl_->dev_input, impl_->input_elems * sizeof(float)), "cudaMalloc(input)");
  cuda_check(
    cudaMalloc(&impl_->dev_output, impl_->output_elems * dtype_size(impl_->output_dtype)),
    "cudaMalloc(output)");
  cuda_check(cudaStreamCreate(&impl_->stream), "cudaStreamCreate");

  impl_->bindings[impl_->input_index] = impl_->dev_input;
  impl_->bindings[impl_->output_index] = impl_->dev_output;

  tools::logger()->info(
    "[YOLOV5_TRT] {} loaded (TensorRT), input {} elems, output {} elems ({})", engine_path_,
    impl_->input_elems, impl_->output_elems, dtype_name(impl_->output_dtype));
#else
  throw std::runtime_error(
    "[YOLOV5_TRT] 本机未编译 TensorRT 支持（CMake 未找到 libnvinfer）。"
    "请按 AGENTS.md §4.0 安装: sudo apt install tensorrt libnvinfer-dev libnvinfer-plugin-dev，"
    "然后重新 cmake -B build。");
#endif
}

YOLOV5_TRT::~YOLOV5_TRT() = default;

std::list<Armor> YOLOV5_TRT::detect(const cv::Mat & raw_img, int frame_count)
{
  if (raw_img.empty()) {
    tools::logger()->warn("Empty img!, camera drop!");
    return std::list<Armor>();
  }

  cv::Mat bgr_img;
  if (use_roi_) {
    if (roi_.width == -1) {  // -1 表示该维度不裁切
      roi_.width = raw_img.cols;
    }
    if (roi_.height == -1) {  // -1 表示该维度不裁切
      roi_.height = raw_img.rows;
    }
    bgr_img = raw_img(roi_);
  } else {
    bgr_img = raw_img;
  }

  // preprocess：与 yolos/yolov5.cpp / yolov5_onnx.cpp 完全一致（共用 yolov5_post::letterbox）
  double scale = 1.0;
  auto input = yolov5_post::letterbox(bgr_img, scale);

  // u8 BGR HWC -> f32 RGB CHW /255，与 ONNX 后端用的是同一个转换（cv::dnn::blobFromImage）
  auto blob = cv::dnn::blobFromImage(input, 1 / 255.0, {640, 640}, cv::Scalar(), true, false);

#ifdef HAVE_TENSORRT
  if (blob.total() != impl_->input_elems) {
    throw std::runtime_error("[YOLOV5_TRT] 预处理输出元素数与引擎输入不一致");
  }
  std::memcpy(impl_->host_input.data(), blob.ptr<float>(), impl_->input_elems * sizeof(float));

  // infer：H2D -> enqueueV2 -> D2H
  cuda_check(
    cudaMemcpyAsync(
      impl_->dev_input, impl_->host_input.data(), impl_->input_elems * sizeof(float),
      cudaMemcpyHostToDevice, impl_->stream),
    "cudaMemcpyAsync(H2D)");

  if (!impl_->context->enqueueV2(impl_->bindings.data(), impl_->stream, nullptr)) {
    throw std::runtime_error("[YOLOV5_TRT] enqueueV2 failed");
  }

  void * host_output = impl_->output_dtype == nvinfer1::DataType::kHALF
                         ? static_cast<void *>(impl_->host_output_half.data())
                         : static_cast<void *>(impl_->host_output_float.data());
  cuda_check(
    cudaMemcpyAsync(
      impl_->dev_output, host_output, impl_->output_elems * dtype_size(impl_->output_dtype),
      cudaMemcpyDeviceToHost, impl_->stream),
    "cudaMemcpyAsync(D2H)");
  cuda_check(cudaStreamSynchronize(impl_->stream), "cudaStreamSynchronize");

  // 引擎输出若为 fp16，转成 fp32 再交给共享后处理
  if (impl_->output_dtype == nvinfer1::DataType::kHALF) {
    impl_->host_output_float.resize(impl_->output_elems);
    for (std::size_t i = 0; i < impl_->output_elems; i++) {
      impl_->host_output_float[i] = __half2float(impl_->host_output_half[i]);
    }
  }

  // postprocess：输出 [1, 25200, 22]（AGENTS.md §4.2）
  constexpr int rows = 25200;
  constexpr int cols = yolov5_post::kOutputCols;
  if (impl_->output_elems != static_cast<std::size_t>(rows) * cols) {
    throw std::runtime_error(
      "[YOLOV5_TRT] 输出元素数不是 25200*22，模型与后处理约定不符（AGENTS.md §4.2）");
  }
  cv::Mat output_2d(rows, cols, CV_32F, impl_->host_output_float.data());

  return yolov5_post::parse(
    scale, output_2d, raw_img, frame_count, min_confidence_, use_roi_, roi_, offset_, debug_,
    traditional());
#else
  (void)blob;
  (void)scale;
  throw std::runtime_error(
    "[YOLOV5_TRT] 本机未编译 TensorRT 支持（CMake 未找到 libnvinfer），见 AGENTS.md §4.0");
#endif
}

std::list<Armor> YOLOV5_TRT::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return yolov5_post::parse(
    scale, output, bgr_img, frame_count, min_confidence_, use_roi_, roi_, offset_, debug_,
    traditional());
}

}  // namespace auto_aim
