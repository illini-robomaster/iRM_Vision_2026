#include "yolo.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

#include "yolos/yolov5_onnx.hpp"

// 说明（AGENTS.md §4）：
//   本文件是 auto_aim::YOLO 的后端分发点。原先这里直接构造 OpenVINO 版
//   YOLOV5 / YOLOV8 / YOLO11，OpenVINO 在本机不存在且被规则禁用，因此在
//   TensorRT 迁移完成前改走 OpenCV DNN 的 ONNX 后端（同样只替换推理调用点，
//   后处理沿用 yolos/yolov5.cpp 的 parse 约定）。
//   迁移完成后在此处补 TensorRT 后端分支，保持对外接口不变。

namespace auto_aim
{
YOLO::YOLO(const std::string & config_path, bool debug)
{
  auto yaml = YAML::LoadFile(config_path);
  auto yolo_name = yaml["yolo_name"].as<std::string>();

  // 新增键，缺失时默认 "auto"（已有 configs/*.yaml 无需修改）
  auto backend =
    yaml["yolov5_backend"] ? yaml["yolov5_backend"].as<std::string>() : std::string("auto");

  if (backend == "auto" || backend == "onnx_dnn") {
    if (yolo_name != "yolov5") {
      throw std::runtime_error(
        "backend " + backend + " 目前只支持 yolo_name=yolov5，当前为: " + yolo_name +
        "（yolov8/yolo11 待迁移，见 AGENTS.md §4.1）");
    }
    yolo_ = std::make_unique<YOLOV5_ONNX>(config_path, debug);
  }

  else if (backend == "openvino") {
    throw std::runtime_error(
      "yolov5_backend=openvino 已不可用：本机没有 OpenVINO 且禁止引入（AGENTS.md §0.4），"
      "请用 onnx_dnn，或等 TensorRT 后端接入");
  }

  else {
    throw std::runtime_error("Unknown yolov5_backend: " + backend + "!");
  }
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