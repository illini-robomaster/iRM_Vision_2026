#include "yolo.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

#include "tools/logger.hpp"
#include "yolos/yolov5_onnx.hpp"

#ifdef SPVISION_HAS_ORT
#include "yolos/yolov5_ort.hpp"
#endif

// 说明（AGENTS.md §4、§8.5）：
//   本文件是 auto_aim::YOLO 的后端分发点。原先这里直接构造 OpenVINO 版
//   YOLOV5 / YOLOV8 / YOLO11；OpenVINO 在本机不存在且被规则禁用（§0.4），因此按
//   下面的优先级挑选「编译时真的存在」的后端（谁存在由 CMake 的编译定义决定）：
//     1) TensorRT    —— Jetson 目标后端，本仓库尚未接入（§4.0 / §4.1）
//     2) ONNX Runtime—— x86_64 / WSL2 主力后端（SPVISION_HAS_ORT，见 §8.5）
//     3) OpenCV DNN  —— 兜底后端，需要 OpenCV >= 4.9
//   三者共用同一个 parse 约定（§4.2），只替换推理调用点，对外接口不变。

namespace auto_aim
{
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
      "请用 auto / ort / onnx_dnn");
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

  // 1) TensorRT：迁移完成前只有提示，随后自动回退到下一个可用后端
  if (want_trt) {
    tools::logger()->warn(
      "[YOLO] TensorRT 后端尚未接入（AGENTS.md §4.0/§4.1），自动回退到下一个可用后端");
  }

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