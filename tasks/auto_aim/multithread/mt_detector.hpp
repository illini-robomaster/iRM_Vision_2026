#ifndef AUTO_AIM__MT_DETECTOR_HPP
#define AUTO_AIM__MT_DETECTOR_HPP

#include <atomic>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>
#include <tuple>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace auto_aim
{
namespace multithread
{

// 多线程检测器：push() 把图像投递进内部队列，工作线程做推理，
// pop() / debug_pop() 取出「图像 + 装甲板 + 时间戳」。
//
// 迁移说明（AGENTS.md §4.1）：原实现用 OpenVINO 的 ov::InferRequest 做异步推理，
// 队列元素里带 ov::InferRequest，因此在没有 OpenVINO 的机器上无法编译。
// 现在改为「投递帧 -> 工作线程推理」的结构，对外接口（push/pop/debug_pop）不变；
// 具体推理后端由 auto_aim::YOLO 分发（当前为 OpenCV DNN/ONNX，TensorRT 接入后不变）。
class MultiThreadDetector
{
public:
  MultiThreadDetector(const std::string & config_path, bool debug = false);
  ~MultiThreadDetector();

  void push(cv::Mat img, std::chrono::steady_clock::time_point t);

  std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> pop();

  std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point> debug_pop();

private:
  struct Frame
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;
  };

  struct Result
  {
    cv::Mat img;
    std::list<Armor> armors;
    std::chrono::steady_clock::time_point t;
  };

  void detect_thread();

  bool debug_;
  YOLO yolo_;

  tools::ThreadSafeQueue<Frame> in_queue_{
    16, [] { tools::logger()->debug("[MultiThreadDetector] in queue is full!"); }};
  tools::ThreadSafeQueue<Result> out_queue_{
    16, [] { tools::logger()->debug("[MultiThreadDetector] out queue is full!"); }};

  std::thread detect_thread_;
  std::atomic<bool> stop_{false};
};

}  // namespace multithread

}  // namespace auto_aim

#endif  // AUTO_AIM__MT_DETECTOR_HPP