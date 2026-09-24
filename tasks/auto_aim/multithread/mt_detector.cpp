#include "mt_detector.hpp"

#include <yaml-cpp/yaml.h>

namespace auto_aim
{
namespace multithread
{

MultiThreadDetector::MultiThreadDetector(const std::string & config_path, bool debug)
: debug_(debug), yolo_(config_path, debug)
{
  auto yaml = YAML::LoadFile(config_path);
  auto yolo_name = yaml["yolo_name"].as<std::string>();
  auto backend =
    yaml["yolov5_backend"] ? yaml["yolov5_backend"].as<std::string>() : std::string("auto");

  tools::logger()->info(
    "[MultiThreadDetector] yolo_name={}, backend={}（推理在后端内部完成）", yolo_name, backend);

  detect_thread_ = std::thread(&MultiThreadDetector::detect_thread, this);
  tools::logger()->info("[MultiThreadDetector] initialized !");
}

MultiThreadDetector::~MultiThreadDetector()
{
  stop_ = true;
  in_queue_.clear();  // 先清空，保证停止信号一定能入队
  in_queue_.push({cv::Mat(), std::chrono::steady_clock::now()});
  if (detect_thread_.joinable()) detect_thread_.join();

  tools::logger()->info("[MultiThreadDetector] destructed.");
}

void MultiThreadDetector::detect_thread()
{
  while (true) {
    auto frame = in_queue_.pop();

    if (frame.img.empty()) {
      if (stop_) break;
      tools::logger()->warn("[MultiThreadDetector] received empty frame, skip");
      continue;
    }

    // 推理 + 置信度阈值 + NMS + 关键点解码都在 YOLO 后端内部完成
    auto armors = yolo_.detect(frame.img, 0);
    out_queue_.push({frame.img, std::move(armors), frame.t});
  }
}

void MultiThreadDetector::push(cv::Mat img, std::chrono::steady_clock::time_point t)
{
  if (img.empty()) {
    tools::logger()->warn("[MultiThreadDetector] push an empty img, camera drop!");
    return;
  }

  // 相机缓冲会被复用，这里必须拷贝（debug_pop 还要把原图交回调用方）
  in_queue_.push({img.clone(), t});
}

std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> MultiThreadDetector::pop()
{
  auto result = out_queue_.pop();
  return {std::move(result.armors), result.t};
}

std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point>
MultiThreadDetector::debug_pop()
{
  auto result = out_queue_.pop();
  return {result.img, std::move(result.armors), result.t};
}

}  // namespace multithread

}  // namespace auto_aim
