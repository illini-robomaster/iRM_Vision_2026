#include "yolov5_onnx.hpp"

#include <fmt/format.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <stdexcept>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{
YOLOV5_ONNX::YOLOV5_ONNX(const std::string & config_path, bool debug) : debug_(debug)
{
  auto yaml = YAML::LoadFile(config_path);

  model_path_ = yaml["yolov5_onnx_path"] ? yaml["yolov5_onnx_path"].as<std::string>()
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
    tools::logger()->warn(
      "[YOLOV5_ONNX] use_traditional=true 暂不支持：传统方法依赖 auto_aim::Detector/Classifier，"
      "尚未迁移（AGENTS.md §4.1）；本后端只使用网络输出的关键点");
  }

  net_ = cv::dnn::readNetFromONNX(model_path_);
  if (net_.empty()) throw std::runtime_error("[YOLOV5_ONNX] failed to load: " + model_path_);

  tools::logger()->info("[YOLOV5_ONNX] {} loaded (OpenCV DNN, CPU)", model_path_);
}

std::list<Armor> YOLOV5_ONNX::detect(const cv::Mat & raw_img, int frame_count)
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

  // preprocess：与 yolos/yolov5.cpp 一致（等比缩放 + 左上角贴黑边），与 TensorRT 后端共用
  double scale = 1.0;
  auto input = yolov5_post::letterbox(bgr_img, scale);

  // infer：OpenCV DNN（CPU），等价于 OpenVINO 的 u8 NHWC -> f32 NCHW RGB /255 预处理
  auto blob = cv::dnn::blobFromImage(input, 1 / 255.0, {640, 640}, cv::Scalar(), true, false);
  net_.setInput(blob);
  cv::Mat output = net_.forward();  // [1, 25200, 22]

  if (output.dims != 3 || output.size[2] != 22) {
    throw std::runtime_error("[YOLOV5_ONNX] unexpected output shape, expect [1, 25200, 22]");
  }

  cv::Mat output_2d(output.size[1], output.size[2], CV_32F, output.ptr<float>());

  return parse(scale, output_2d, raw_img, frame_count);
}

std::list<Armor> YOLOV5_ONNX::parse(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return yolov5_post::parse(
    scale, output, bgr_img, frame_count, min_confidence_, use_roi_, roi_, offset_, debug_);
}

std::list<Armor> YOLOV5_ONNX::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return parse(scale, output, bgr_img, frame_count);
}

}  // namespace auto_aim
