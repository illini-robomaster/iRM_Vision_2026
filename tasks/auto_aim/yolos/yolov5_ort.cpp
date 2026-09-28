#include "yolov5_ort.hpp"

#ifdef SPVISION_HAS_ORT

#include <fmt/format.h>
#include <yaml-cpp/yaml.h>

#include <stdexcept>

#include "tasks/auto_aim/yolos/yolov5_postprocess.hpp"
#include "tools/logger.hpp"

// 说明（AGENTS.md §8.5）：本文件是 cv::dnn 后端（yolov5_onnx.cpp）在推理调用点上的等价替换。
//   预处理 / 后处理**复用** yolos/yolov5_postprocess.*（与 TensorRT、cv::dnn 后端同一份实现），
//   本文件只保留 ORT 专属部分：Session 构造、输入张量的元素类型（0526.onnx 是 FP16）、
//   以及输出张量到 cv::Mat 的视图 / 转换。
namespace auto_aim
{
namespace
{
// ORT 官方建议：整个进程共用一个 Ort::Env（线程安全，可被多个 Session 共享）。
// 用函数内静态量交给运行时管理生命周期，避免与 Session 的析构顺序纠缠。
Ort::Env & ort_env()
{
  static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "sp_vision");
  return env;
}

// CreateCpu 返回的是指向 ORT 内部静态对象的轻量句柄，这里也做成进程级单例。
const Ort::MemoryInfo & ort_memory_info()
{
  static const Ort::MemoryInfo info =
    Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  return info;
}
}  // namespace

YOLOV5_ORT::YOLOV5_ORT(const std::string & config_path, bool debug) : debug_(debug)
{
  auto yaml = YAML::LoadFile(config_path);

  // 权重路径：优先新键，其次复用 cv::dnn 后端的键，最后用仓库默认权重
  model_path_ = yaml["yolov5_ort_path"] ? yaml["yolov5_ort_path"].as<std::string>()
              : yaml["yolov5_onnx_path"] ? yaml["yolov5_onnx_path"].as<std::string>()
                                         : std::string("assets/yolov5_0526.onnx");
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
    configure_traditional(config_path, "YOLOV5_ORT");
  }

  Ort::SessionOptions options;
  options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  auto intra_threads =
    yaml["yolov5_ort_intra_threads"] ? yaml["yolov5_ort_intra_threads"].as<int>() : 0;
  if (intra_threads > 0) options.SetIntraOpNumThreads(intra_threads);

  session_ = std::make_unique<Ort::Session>(ort_env(), model_path_.c_str(), options);

  // 输入/输出名与元素类型（本模型：images[1,3,640,640] FP16 -> output[1,25200,22] FP32）
  Ort::AllocatorWithDefaultOptions allocator;
  input_name_ = session_->GetInputNameAllocated(0, allocator).get();
  output_name_ = session_->GetOutputNameAllocated(0, allocator).get();

  Ort::TypeInfo input_info = session_->GetInputTypeInfo(0);
  auto input_ts = input_info.GetTensorTypeAndShapeInfo();  // 借用，无需释放
  fp16_input_ = input_ts.GetElementType() == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16;
  auto input_shape = input_ts.GetShape();

  int net_w = 640, net_h = 640;
  if (input_shape.size() == 4) {
    if (input_shape[2] > 0) net_h = static_cast<int>(input_shape[2]);
    if (input_shape[3] > 0) net_w = static_cast<int>(input_shape[3]);
  }
  input_size_ = cv::Size(net_w, net_h);

  // 共享的 letterbox 固定 640x640（P24-DetectionModel 的输入，AGENTS.md §4.2）。模型若换了
  // 输入尺寸必须同步改 yolov5_postprocess，否则关键点映射会静默错位 —— 直接拒绝启动。
  if (input_size_ != cv::Size(640, 640)) {
    throw std::runtime_error(
      "[YOLOV5_ORT] 模型输入尺寸 " + std::to_string(net_w) + "x" + std::to_string(net_h) +
      " 与共享预处理（yolov5_postprocess，固定 640x640）不一致，见 AGENTS.md §4.2");
  }

  tools::logger()->info(
    "[YOLOV5_ORT] {} loaded (ONNX Runtime {}), input={} [1,3,{},{}] {}, output={}", model_path_,
    Ort::GetVersionString(), input_name_, net_h, net_w, fp16_input_ ? "FP16" : "F32", output_name_);
}

std::list<Armor> YOLOV5_ORT::detect(const cv::Mat & raw_img, int frame_count)
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

  const int net_w = input_size_.width;
  const int net_h = input_size_.height;

  // preprocess：复用共享 letterbox（等比缩放 + 左上角贴黑边），与 TensorRT / cv::dnn 后端
  // 完全同一份实现（AGENTS.md §4.2）
  double scale = 1.0;
  auto input = yolov5_post::letterbox(bgr_img, scale);

  auto blob = cv::dnn::blobFromImage(input, 1 / 255.0, input_size_, cv::Scalar(), true, false);

  // infer：ONNX Runtime（CPU），等价于 OpenVINO 的 u8 NHWC -> f32 NCHW RGB /255 预处理
  std::vector<int64_t> shape{1, 3, net_h, net_w};
  std::vector<Ort::Float16_t> blob_fp16;
  Ort::Value input_tensor{nullptr};
  if (fp16_input_) {
    // 模型输入是 FP16：用 ORT 自带的半精度转换，避免依赖 OpenCV 的 CV_16F 支持
    blob_fp16.resize(blob.total());
    const float * src = blob.ptr<float>();
    for (std::size_t i = 0; i < blob_fp16.size(); i++) blob_fp16[i] = Ort::Float16_t(src[i]);
    input_tensor = Ort::Value::CreateTensor<Ort::Float16_t>(
      ort_memory_info(), blob_fp16.data(), blob_fp16.size(), shape.data(), shape.size());
  } else {
    input_tensor = Ort::Value::CreateTensor<float>(
      ort_memory_info(), blob.ptr<float>(), blob.total(), shape.data(), shape.size());
  }

  const char * input_names[] = {input_name_.c_str()};
  const char * output_names[] = {output_name_.c_str()};
  auto outputs =
    session_->Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);

  if (outputs.size() != 1) {
    throw std::runtime_error("[YOLOV5_ORT] unexpected output count, expect 1");
  }

  Ort::TensorTypeAndShapeInfo output_ts = outputs[0].GetTensorTypeAndShapeInfo();
  auto output_shape = output_ts.GetShape();
  if (output_shape.size() != 3 || output_shape[2] != 22) {
    throw std::runtime_error("[YOLOV5_ORT] unexpected output shape, expect [1, 25200, 22]");
  }

  const int rows = static_cast<int>(output_shape[1]);
  if (rows <= 0) return std::list<Armor>();

  // output 的生命周期只到本次调用结束（FP32 时是 ORT 输出缓冲区的只读视图）
  cv::Mat output;
  if (output_ts.GetElementType() == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16) {
    const Ort::Float16_t * src = outputs[0].GetTensorData<Ort::Float16_t>();
    output = cv::Mat(rows, 22, CV_32F);
    float * dst = output.ptr<float>();
    for (int i = 0; i < rows * 22; i++) dst[i] = src[i].ToFloat();
  } else if (output_ts.GetElementType() == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
    output = cv::Mat(rows, 22, CV_32F, const_cast<float *>(outputs[0].GetTensorData<float>()));
  } else {
    throw std::runtime_error("[YOLOV5_ORT] unsupported output element type, expect FP16/FP32");
  }

  return parse(scale, output, raw_img, frame_count);
}

std::list<Armor> YOLOV5_ORT::parse(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  // 后处理统一走共享实现（sigmoid 置信度 -> 阈值 -> argmax 颜色/编号 -> NMS -> 名称/类型
  // 过滤 -> use_traditional 的二次矫正 -> center_norm），与 TensorRT / cv::dnn 后端逐行一致
  // （AGENTS.md §4.1.4 / §4.2）
  return yolov5_post::parse(
    scale, output, bgr_img, frame_count, min_confidence_, use_roi_, roi_, offset_, debug_,
    traditional());
}

std::list<Armor> YOLOV5_ORT::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return parse(scale, output, bgr_img, frame_count);
}

}  // namespace auto_aim

#endif  // SPVISION_HAS_ORT
