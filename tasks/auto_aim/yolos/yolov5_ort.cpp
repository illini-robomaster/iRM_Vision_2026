#include "yolov5_ort.hpp"

#ifdef SPVISION_HAS_ORT

#include <fmt/format.h>
#include <yaml-cpp/yaml.h>

#include <stdexcept>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

// 说明（AGENTS.md §8.5）：本文件是 cv::dnn 后端（yolov5_onnx.cpp）在推理调用点上的等价替换。
//   预处理、后处理（parse）与原实现逐行一致，只是把 cv::dnn::Net 换成 ONNX Runtime 的
//   Ort::Session，并把输入张量按模型实际元素类型（可能是 FP16）喂进去。
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
    tools::logger()->warn(
      "[YOLOV5_ORT] use_traditional=true 暂不支持：传统方法依赖 auto_aim::Detector/Classifier，"
      "尚未迁移（AGENTS.md §4.1）；本后端只使用网络输出的关键点");
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

  auto x_scale = static_cast<double>(net_h) / bgr_img.rows;
  auto y_scale = static_cast<double>(net_w) / bgr_img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(bgr_img.rows * scale);
  auto w = static_cast<int>(bgr_img.cols * scale);

  // preprocess：与 yolos/yolov5.cpp 一致（等比缩放 + 左上角贴黑边 + /255 + BGR->RGB）
  auto input = cv::Mat(net_h, net_w, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(bgr_img, input(roi), {w, h});

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
  // 每一行：4 个关键点(0..7) + 置信度(8，raw logits) + 颜色(9..12) + 编号(13..21)
  std::vector<int> color_ids, num_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;

  for (int r = 0; r < output.rows; r++) {
    double score = output.at<float>(r, 8);
    score = sigmoid(score);

    if (score < score_threshold_) continue;

    std::vector<cv::Point2f> armor_key_points;

    //颜色和类别独热向量
    cv::Mat color_scores = output.row(r).colRange(9, 13);     //color
    cv::Mat classes_scores = output.row(r).colRange(13, 22);  //num
    cv::Point class_id, color_id;
    int _class_id, _color_id;
    double score_color, score_num;
    cv::minMaxLoc(classes_scores, NULL, &score_num, NULL, &class_id);
    cv::minMaxLoc(color_scores, NULL, &score_color, NULL, &color_id);
    _class_id = class_id.x;
    _color_id = color_id.x;

    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 0) / scale, output.at<float>(r, 1) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 6) / scale, output.at<float>(r, 7) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 4) / scale, output.at<float>(r, 5) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 2) / scale, output.at<float>(r, 3) / scale));

    float min_x = armor_key_points[0].x;
    float max_x = armor_key_points[0].x;
    float min_y = armor_key_points[0].y;
    float max_y = armor_key_points[0].y;

    for (std::size_t i = 1; i < armor_key_points.size(); i++) {
      if (armor_key_points[i].x < min_x) min_x = armor_key_points[i].x;
      if (armor_key_points[i].x > max_x) max_x = armor_key_points[i].x;
      if (armor_key_points[i].y < min_y) min_y = armor_key_points[i].y;
      if (armor_key_points[i].y > max_y) max_y = armor_key_points[i].y;
    }

    cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);

    color_ids.emplace_back(_color_id);
    num_ids.emplace_back(_class_id);
    boxes.emplace_back(rect);
    confidences.emplace_back(score);
    armors_key_points.emplace_back(armor_key_points);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  std::list<Armor> armors;
  for (const auto & i : indices) {
    if (use_roi_) {
      armors.emplace_back(
        color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i], offset_);
    } else {
      armors.emplace_back(color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i]);
    }
  }

  for (auto it = armors.begin(); it != armors.end();) {
    if (!check_name(*it)) {
      it = armors.erase(it);
      continue;
    }

    if (!check_type(*it)) {
      it = armors.erase(it);
      continue;
    }

    // 注：原 OpenVINO 版在此处用传统方法二次矫正角点（use_traditional），依赖尚未迁移的
    // auto_aim::Detector / Classifier，本后端跳过（见头文件说明）
    it->center_norm = get_center_norm(bgr_img, it->center);
    ++it;
  }

  if (debug_) draw_detections(bgr_img, armors, frame_count);

  return armors;
}

std::list<Armor> YOLOV5_ORT::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return parse(scale, output, bgr_img, frame_count);
}

bool YOLOV5_ORT::check_name(const Armor & armor) const
{
  auto name_ok = armor.name != ArmorName::not_armor;
  auto confidence_ok = armor.confidence > min_confidence_;

  return name_ok && confidence_ok;
}

bool YOLOV5_ORT::check_type(const Armor & armor) const
{
  auto name_ok = (armor.type == ArmorType::small)
                   ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
                   : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
                      armor.name != ArmorName::outpost);

  return name_ok;
}

cv::Point2f YOLOV5_ORT::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  auto h = bgr_img.rows;
  auto w = bgr_img.cols;
  return {center.x / w, center.y / h};
}

void YOLOV5_ORT::draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
{
  auto detection = img.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});
  for (const auto & armor : armors) {
    auto info = fmt::format(
      "{:.2f} {} {} {}", armor.confidence, COLORS[armor.color], ARMOR_NAMES[armor.name],
      ARMOR_TYPES[armor.type]);
    tools::draw_points(detection, armor.points, {0, 255, 0});
    tools::draw_text(detection, info, armor.center, {0, 255, 0});
  }

  if (use_roi_) {
    cv::Scalar green(0, 255, 0);
    cv::rectangle(detection, roi_, green, 2);
  }
  cv::resize(detection, detection, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
  cv::imshow("detection", detection);
  cv::waitKey(1);
}

double YOLOV5_ORT::sigmoid(double x)
{
  if (x > 0)
    return 1.0 / (1.0 + exp(-x));
  else
    return exp(x) / (1.0 + exp(x));
}

}  // namespace auto_aim

#endif  // SPVISION_HAS_ORT
