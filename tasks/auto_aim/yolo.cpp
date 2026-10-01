#include "yolo.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <stdexcept>
#include <string>

#include "detector.hpp"
#include "tools/logger.hpp"
#include "yolos/yolov5_onnx.hpp"
#include "yolos/yolov5_trt.hpp"

#ifdef SPVISION_HAS_ORT
#include "yolos/yolov5_ort.hpp"
#endif

// 说明（AGENTS.md §4、§8.5）：
//   本文件是 auto_aim::YOLO 的后端分发点。原先这里直接构造 OpenVINO 版
//   YOLOV5 / YOLOV8 / YOLO11；OpenVINO 在本机不存在且被规则禁用（§0.4），因此按
//   下面的优先级挑选「真的可用」的后端（编译期存在性由 CMake 的编译定义决定）：
//     1) TensorRT    —— Jetson 目标后端（HAVE_TENSORRT，见 §4.0）
//     2) ONNX Runtime—— x86_64 / WSL2 主力后端（SPVISION_HAS_ORT，见 §8.5）
//     3) OpenCV DNN  —— 兜底后端，需要 OpenCV >= 4.9（§8.5）
//   TensorRT 与 OpenCV DNN 共用 yolos/yolov5_postprocess.* 的 letterbox/parse（唯一实现，
//   避免漂移）；ORT 后端沿用同一套约定（§4.2）。只替换推理调用点，对外接口（类名 /
//   构造签名 / detect 返回类型）与 YAML 已有键均未改动。

namespace auto_aim
{
void YOLOBase::configure_traditional(
  const std::string & config_path, const std::string & backend_name)
{
  auto yaml = YAML::LoadFile(config_path);
  if (!(yaml["use_traditional"] && yaml["use_traditional"].as<bool>())) return;

  try {
    // debug=false：与原 OpenVINO 版 yolos/yolov5.cpp 的 detector_(config_path, false) 一致，
    // 避免 Detector 自己再开一个显示窗口
    traditional_ = std::make_unique<Detector>(config_path, false);
  } catch (const std::exception & e) {
    throw std::runtime_error(
      "[" + backend_name +
      "] use_traditional=true 但构造 Detector/Classifier 失败: " + e.what() +
      "（传统方法需要 threshold / max_angle_error / min_lightbar_* / max_armor_ratio 等键，"
      "见 configs/*.yaml；只想用网络关键点时把 use_traditional 设为 false）");
  }

  tools::logger()->info(
    "[{}] use_traditional=true：已启用传统方法二次矫正角点（Detector+Classifier，原 OpenVINO "
    "版的行为，AGENTS.md §4.1）",
    backend_name);
}

YOLO::YOLO(const std::string & config_path, bool debug)
{
  auto yaml = YAML::LoadFile(config_path);
  auto yolo_name = yaml["yolo_name"].as<std::string>();

  // 新增键，缺失时默认 "auto"（已有 configs/*.yaml 无需修改）
  auto backend =
    yaml["yolov5_backend"] ? yaml["yolov5_backend"].as<std::string>() : std::string("auto");

  if (backend == "openvino") {
    throw std::runtime_error(
      "yolov5_backend=openvino 已不可用：本机没有 OpenVINO 且禁止引入（AGENTS.md §0.4），"
      "请用 auto / trt / ort / onnx_dnn");
  }

  const bool want_trt = backend == "auto" || backend == "tensorrt" || backend == "trt";
  const bool want_ort = backend == "auto" || backend == "ort" || backend == "onnxruntime";
  const bool want_dnn = backend == "auto" || backend == "onnx_dnn" || backend == "dnn" ||
                        backend == "cv_dnn";

  if (!want_trt && !want_ort && !want_dnn) {
    throw std::runtime_error("Unknown yolov5_backend: " + backend + "!");
  }

  if (yolo_name != "yolov5") {
    throw std::runtime_error(
      "推理后端目前只支持 yolo_name=yolov5，当前为: " + yolo_name +
      "（yolov8/yolo11 待迁移，见 AGENTS.md §4.1）");
  }

  // 1) TensorRT —— Jetson 目标后端
  //    显式 trt：交给后端自己处理（没装 TensorRT / 没生成 .engine 时抛清晰异常，不静默退化）
  //    auto：只有「编译期有 TensorRT + 运行期 .engine 真的在」才选它，否则回退下一个后端，
  //          避免 Jetson 上还没离线生成 .engine 时整条链路直接抛异常
#ifdef HAVE_TENSORRT
  if (want_trt) {
    auto engine_path = yaml["yolov5_trt_engine_path"]
                         ? yaml["yolov5_trt_engine_path"].as<std::string>()
                         : std::string("assets/yolov5_0526_fp16.engine");
    if (backend != "auto" || std::filesystem::exists(engine_path)) {
      yolo_ = std::make_unique<YOLOV5_TRT>(config_path, debug);
      tools::logger()->info("[YOLO] backend=tensorrt (yolov5_backend={})", backend);
      return;
    }
    tools::logger()->warn(
      "[YOLO] 未找到 TensorRT 引擎 {}（先用 trtexec --fp16 离线生成，见 AGENTS.md §4.0），"
      "自动回退到下一个可用后端",
      engine_path);
  }
#else
  if (backend == "trt" || backend == "tensorrt") {
    // 显式指定：交给 YOLOV5_TRT 抛出「本机未编译 TensorRT 后端」的清晰异常，不静默退化
    yolo_ = std::make_unique<YOLOV5_TRT>(config_path, debug);
    return;
  }
  if (want_trt) {
    tools::logger()->warn(
      "[YOLO] 本机未编译 TensorRT 后端（CMake 未找到 libnvinfer，见 AGENTS.md §4.0），"
      "自动回退到下一个可用后端");
  }
#endif

  // 2) ONNX Runtime
#ifdef SPVISION_HAS_ORT
  if (want_ort) {
    yolo_ = std::make_unique<YOLOV5_ORT>(config_path, debug);
    tools::logger()->info("[YOLO] backend=onnxruntime (yolov5_backend={})", backend);
    return;
  }
#else
  if (want_ort && !want_dnn) {
    throw std::runtime_error(
      "yolov5_backend=" + backend +
      " 需要 ONNX Runtime，但本机没有编译该后端：执行 scripts/fetch_onnxruntime.sh 后重新 "
      "cmake（AGENTS.md §8.5）");
  }
  tools::logger()->warn(
    "[YOLO] 未找到 ONNX Runtime（third_party/onnxruntime），回退 OpenCV DNN；"
    "如需 ORT 请执行 scripts/fetch_onnxruntime.sh");
#endif

  // 3) OpenCV DNN 兜底（OpenCV >= 4.9 才能 forward 本模型，见 AGENTS.md §8.5）
  yolo_ = std::make_unique<YOLOV5_ONNX>(config_path, debug);
  tools::logger()->info("[YOLO] backend=onnx_dnn/cv::dnn (yolov5_backend={})", backend);
}

std::list<Armor> YOLO::detect(const cv::Mat & img, int frame_count)
{
  return yolo_->detect(img, frame_count);
}

std::list<Armor> YOLO::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return yolo_->postprocess(scale, output, bgr_img, frame_count);
}

}  // namespace auto_aim