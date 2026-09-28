#ifndef AUTO_AIM__CLASSIFIER_HPP
#define AUTO_AIM__CLASSIFIER_HPP

#include <opencv2/opencv.hpp>
#include <string>

#include "armor.hpp"

namespace auto_aim
{
// 装甲板数字分类器（configs 的 classify_model 键，32x32 灰度输入）。
//
// 迁移说明（AGENTS.md §4.1）：原实现同时持有 OpenVINO 与 cv::dnn 两条路径 —— 构造函数里用
// ov::Core 读模型 + compile_model，另有一个 ovclassify()。本机没有任何 OpenVINO 运行时且禁止
// 引入（§0.4），因此这里只保留 cv::dnn 路径：classify() 的预处理 / softmax / 标签映射一行未改；
// **没有任何调用点**的 ovclassify() 与其 ov:: 成员一并删除（它与 classify() 的区别只有推理调用点，
// 预处理与后处理逻辑相同）。
class Classifier
{
public:
  explicit Classifier(const std::string & config_path);

  void classify(Armor & armor);

private:
  cv::dnn::Net net_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__CLASSIFIER_HPP