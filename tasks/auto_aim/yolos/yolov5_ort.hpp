#ifndef AUTO_AIM__YOLOV5_ORT_HPP
#define AUTO_AIM__YOLOV5_ORT_HPP

// ONNX Runtime 推理后端（x86_64 开发机 / WSL2 的主力检测后端，见 AGENTS.md §8.5）。
//
// 与 cv::dnn 后端（yolov5_onnx.hpp）的关系：
//   letterbox、/255、BGR->RGB、关键点顺序、NMS、置信度 sigmoid 全部沿用同一份约定
//   （AGENTS.md §4.2），只替换推理调用点：cv::dnn Net -> Ort::Session。
//   动机：OpenCV < 4.9 的 DNN 跑不了本模型（尾部 5D Reshape/Transpose 断言失败），
//   而 assets/yolov5_0526.onnx 的输入是 FP16，ORT 能原样吃下。
//
// 是否参与编译由 CMake 决定：找到 ONNX Runtime 时在 auto_aim 目标上定义
// SPVISION_HAS_ORT；未找到时本头文件不声明任何东西，调用方（yolo.cpp）用同一个宏
// 做条件包含。依赖由 scripts/fetch_onnxruntime.sh 下载到 third_party/onnxruntime。
//
// 配置（新增键，缺失时用默认值，已有 configs/*.yaml 无需修改）：
//   yolov5_backend:            "auto"(默认) / "ort" / "onnxruntime"，见 yolo.cpp 的分发
//   yolov5_ort_path:           默认沿用 yolov5_onnx_path，再退到 assets/yolov5_0526.onnx
//   yolov5_ort_intra_threads:  默认 0（不设置，交给 ORT 按物理核数决定）
#ifdef SPVISION_HAS_ORT

#include <list>
#include <memory>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <string>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace auto_aim
{
class YOLOV5_ORT : public YOLOBase
{
public:
  YOLOV5_ORT(const std::string & config_path, bool debug);

  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count) override;

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

  // 便于上层做后端自检 / 日志
  const std::string & model_path() const { return model_path_; }
  const std::string & input_name() const { return input_name_; }
  const std::string & output_name() const { return output_name_; }
  bool fp16_input() const { return fp16_input_; }
  const cv::Size & input_size() const { return input_size_; }

private:
  std::string model_path_;
  bool debug_, use_roi_;

  const int class_num_ = 13;
  const float nms_threshold_ = 0.3;
  const float score_threshold_ = 0.7;
  double min_confidence_;

  cv::Rect roi_;
  cv::Point2f offset_;

  // 推理会话（unique_ptr：Ort::Session 不可拷贝，构造失败时直接抛异常）
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_, output_name_;
  cv::Size input_size_;  // 模型输入尺寸（本模型固定 640x640）
  bool fp16_input_;      // 输入张量元素类型是否为 FP16（0526.onnx 是）

  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;

  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  std::list<Armor> parse(double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);

  void draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const;
  double sigmoid(double x);
};

}  // namespace auto_aim

#endif  // SPVISION_HAS_ORT

#endif  // AUTO_AIM__YOLOV5_ORT_HPP
