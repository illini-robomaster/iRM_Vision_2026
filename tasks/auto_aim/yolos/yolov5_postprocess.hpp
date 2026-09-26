#ifndef AUTO_AIM__YOLOV5_POSTPROCESS_HPP
#define AUTO_AIM__YOLOV5_POSTPROCESS_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <vector>

#include "tasks/auto_aim/armor.hpp"

namespace auto_aim
{
// P24-DetectionModel（魔改 YOLOv5 + MobileNetV3）的**共享**预处理 / 后处理。
//
// 为什么单独抽出来：本仓库有多个推理后端（yolos/yolov5_onnx.cpp 的 OpenCV DNN、
// yolos/yolov5_trt.cpp 的 TensorRT）。若每个后端各写一份 letterbox / parse，
// 一旦改动就会漂移，无法满足 AGENTS.md §4.1.4「合入前必须给出与参考实现的一致性对比」。
// 因此 letterbox、sigmoid、NMS、名称/类型过滤、center_norm 只在这里实现一份。
//
// 模型输出约定（AGENTS.md §4.2，与 yolos/yolov5.cpp::parse() 一致）：
//   [1, 25200, 22]，每行 = 4 个关键点(0..7) + 置信度 raw logits(8，代码里再 sigmoid)
//                       + 颜色(9..12) + 编号(13..21)
namespace yolov5_post
{
constexpr int kOutputCols = 22;
constexpr float kNmsThreshold = 0.3f;
constexpr float kScoreThreshold = 0.7f;

// letterbox：等比缩放到 640x640、贴左上角黑边（与 yolos/yolov5.cpp 的预处理一致），
// 并把缩放比写入 scale（后处理用 1/scale 把关键点映射回原图）
cv::Mat letterbox(const cv::Mat & bgr_img, double & scale);

// 解析 [25200, 22] 的输出：sigmoid 置信度 -> 阈值 -> argmax 颜色/编号 -> NMS
// -> 名称/类型过滤 -> center_norm（debug 为 true 时额外画框到 "detection" 窗口）
std::list<Armor> parse(
  double scale, const cv::Mat & output_2d, const cv::Mat & bgr_img, int frame_count,
  double min_confidence, bool use_roi, const cv::Rect & roi, const cv::Point2f & offset,
  bool debug);

double sigmoid(double x);

void draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count, bool use_roi,
  const cv::Rect & roi);

}  // namespace yolov5_post
}  // namespace auto_aim

#endif  // AUTO_AIM__YOLOV5_POSTPROCESS_HPP
