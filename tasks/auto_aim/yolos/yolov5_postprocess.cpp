#include "yolov5_postprocess.hpp"

#include <fmt/format.h>

#include <cmath>

#include "tools/img_tools.hpp"
#include "tasks/auto_aim/detector.hpp"  // use_traditional 的二次矫正（traditional 非空时）

namespace auto_aim
{
namespace yolov5_post
{
cv::Mat letterbox(const cv::Mat & bgr_img, double & scale)
{
  auto x_scale = static_cast<double>(640) / bgr_img.rows;
  auto y_scale = static_cast<double>(640) / bgr_img.cols;
  scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(bgr_img.rows * scale);
  auto w = static_cast<int>(bgr_img.cols * scale);

  auto input = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(bgr_img, input(roi), {w, h});

  return input;
}

double sigmoid(double x)
{
  if (x > 0)
    return 1.0 / (1.0 + exp(-x));
  else
    return exp(x) / (1.0 + exp(x));
}

namespace
{
bool check_name(const Armor & armor, double min_confidence)
{
  auto name_ok = armor.name != ArmorName::not_armor;
  auto confidence_ok = armor.confidence > min_confidence;

  return name_ok && confidence_ok;
}

bool check_type(const Armor & armor)
{
  auto name_ok = (armor.type == ArmorType::small)
                   ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
                   : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
                      armor.name != ArmorName::outpost);

  return name_ok;
}

cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center)
{
  auto h = bgr_img.rows;
  auto w = bgr_img.cols;
  return {center.x / w, center.y / h};
}
}  // namespace

std::list<Armor> parse(
  double scale, const cv::Mat & output, const cv::Mat & bgr_img, int frame_count,
  double min_confidence, bool use_roi, const cv::Rect & roi, const cv::Point2f & offset, bool debug,
  Detector * traditional)
{
  // 每一行：4 个关键点(0..7) + 置信度(8，raw logits) + 颜色(9..12) + 编号(13..21)
  std::vector<int> color_ids, num_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;

  for (int r = 0; r < output.rows; r++) {
    double score = output.at<float>(r, 8);
    score = sigmoid(score);

    if (score < kScoreThreshold) continue;

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
  cv::dnn::NMSBoxes(boxes, confidences, kScoreThreshold, kNmsThreshold, indices);

  std::list<Armor> armors;
  for (const auto & i : indices) {
    if (use_roi) {
      armors.emplace_back(
        color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i], offset);
    } else {
      armors.emplace_back(color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i]);
    }
  }

  for (auto it = armors.begin(); it != armors.end();) {
    if (!check_name(*it, min_confidence)) {
      it = armors.erase(it);
      continue;
    }

    if (!check_type(*it)) {
      it = armors.erase(it);
      continue;
    }

    // 传统方法二次矫正角点（= 原 OpenVINO 版 yolos/yolov5.cpp 的
    // `if (use_traditional_) detector_.detect(*it, bgr_img);`）。位置固定在 center_norm 之前，
    // 与原实现一致：矫正后角点变了，center_norm 必须用新角点重算。
    if (traditional != nullptr) traditional->detect(*it, bgr_img);

    it->center_norm = get_center_norm(bgr_img, it->center);
    ++it;
  }

  if (debug) draw_detections(bgr_img, armors, frame_count, use_roi, roi);

  return armors;
}

void draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count, bool use_roi,
  const cv::Rect & roi)
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

  if (use_roi) {
    cv::Scalar green(0, 255, 0);
    cv::rectangle(detection, roi, green, 2);
  }
  cv::resize(detection, detection, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
  cv::imshow("detection", detection);
  cv::waitKey(1);
}

}  // namespace yolov5_post
}  // namespace auto_aim
