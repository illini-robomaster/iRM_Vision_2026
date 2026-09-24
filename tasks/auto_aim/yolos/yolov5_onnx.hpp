#ifndef AUTO_AIM__YOLOV5_ONNX_HPP
#define AUTO_AIM__YOLOV5_ONNX_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace auto_aim
{
// P24-DetectionModel（魔改 YOLOv5 + MobileNetV3）的 OpenCV DNN 推理后端。
//
// 用途：在 TensorRT 尚未就绪（x86_64 笔记本 / WSL2 没有 TensorRT）或迁移完成前，
// 提供一个能真正跑起来的检测器；Jetson 上后续以 TensorRT 后端替换推理调用点，
// 后处理逻辑（parse）保持同一份约定（见 AGENTS.md §4.2）。
//
// 配置（新增键，缺失时用默认值，已有 configs/*.yaml 无需修改）：
//   yolov5_backend:   "auto"(默认) / "onnx_dnn"
//   yolov5_onnx_path: 默认 assets/yolov5_0526.onnx
//
// 与 yolos/yolov5.cpp（OpenVINO 版）的差异：
//   - 推理调用换成 cv::dnn（letterbox、/255、BGR->RGB、NMS、关键点顺序全部一致）
//   - use_traditional（传统方法二次矫正角点）依赖 auto_aim::Detector / Classifier，
//     而它们尚未迁移（AGENTS.md §4.1），因此本后端只用网络输出的关键点
class YOLOV5_ONNX : public YOLOBase
{
public:
  YOLOV5_ONNX(const std::string & config_path, bool debug);

  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count) override;

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

private:
  std::string model_path_;
  bool debug_, use_roi_;

  const int class_num_ = 13;
  const float nms_threshold_ = 0.3;
  const float score_threshold_ = 0.7;
  double min_confidence_;

  cv::dnn::Net net_;
  cv::Rect roi_;
  cv::Point2f offset_;

  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;

  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  std::list<Armor> parse(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);

  void draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const;
  double sigmoid(double x);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLOV5_ONNX_HPP
