#ifndef AUTO_AIM__YOLO_HPP
#define AUTO_AIM__YOLO_HPP

#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

#include "armor.hpp"
#include "detector.hpp"

namespace auto_aim
{
class YOLOBase
{
public:
  // 必须虚析构：YOLO 用 unique_ptr<YOLOBase> 持有后端，
  // 否则析构时不会调用后端的析构函数（TensorRT 引擎 / cudaMalloc 的 buffer 等不会释放）
  virtual ~YOLOBase() = default;

  virtual std::list<Armor> detect(const cv::Mat & img, int frame_count) = 0;

  virtual std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) = 0;

protected:
  // use_traditional（传统方法二次矫正角点）的公共部分：三个后端（TRT / ORT / cv::dnn）都只在
  // 自己的构造函数里调一次 configure_traditional()，再把 traditional()（可能为 nullptr）交给
  // 共享的 yolos/yolov5_postprocess::parse —— 二次矫正的实现只有一份。
  //
  // yaml 的 use_traditional 为 true 时才构造 Detector（它同时加载 32x32 分类器模型），
  // 与原 OpenVINO 版 yolos/yolov5.cpp 的 `use_traditional_` 分支行为一致。
  void configure_traditional(
    const std::string & config_path, const std::string & backend_name);

  Detector * traditional() const { return traditional_.get(); }

private:
  std::unique_ptr<Detector> traditional_;
};

class YOLO
{
public:
  YOLO(const std::string & config_path, bool debug = true);

  std::list<Armor> detect(const cv::Mat & img, int frame_count = -1);

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);

private:
  std::unique_ptr<YOLOBase> yolo_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO_HPP