#ifndef AUTO_AIM__YOLOV5_TRT_HPP
#define AUTO_AIM__YOLOV5_TRT_HPP

#include <list>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace auto_aim
{
// P24-DetectionModel（魔改 YOLOv5 + MobileNetV3）的 TensorRT 推理后端（AGENTS.md §4）。
//
// 设计要点：
//   - 推理只用 TensorRT（反序列化离线生成的 .engine），预处理/后处理与 OpenCV DNN 后端
//     共用 yolos/yolov5_postprocess.*，两个后端行为一致（AGENTS.md §4.1.4）
//   - 本机没装 TensorRT 时（CMake 里未找到 libnvinfer），本文件的实现被 HAVE_TENSORRT
//     宏整体屏蔽，构造函数抛出清晰异常，而不是静默退化到别的后端
//   - TensorRT / CUDA 的类型全部藏在 Impl 里，头文件不含 <NvInfer.h>，
//     因此未装 TensorRT 的机器（x86 CI / 笔记本）也能编译
//
// 新增配置键（缺失时用默认值，已有 configs/*.yaml 无需修改）：
//   yolov5_backend:         "trt"（在 yolo.cpp 里分发）
//   yolov5_trt_engine_path: 默认 assets/yolov5_0526_fp16.engine
//
// 引擎生成（离线；.engine 不入库，见 .gitignore，AGENTS.md §4.0）：
//   /usr/src/tensorrt/bin/trtexec --onnx=assets/yolov5_0526.onnx \
//     --saveEngine=assets/yolov5_0526_fp16.engine --fp16
class YOLOV5_TRT : public YOLOBase
{
public:
  YOLOV5_TRT(const std::string & config_path, bool debug);
  ~YOLOV5_TRT() override;

  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count) override;

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

private:
  std::string engine_path_;
  bool debug_, use_roi_;
  double min_confidence_;
  cv::Rect roi_;
  cv::Point2f offset_;

  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLOV5_TRT_HPP
