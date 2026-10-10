#include "calibration/intrinsic_session.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace calibration
{
void Pattern::validate() const
{
  if (cols < 3 || cols > 20 || rows < 3 || rows > 20 || !std::isfinite(spacing_mm) ||
      spacing_mm <= 0 || spacing_mm > 1000)
    throw std::invalid_argument("Pattern: rows/cols must be 3..20; spacing must be (0,1000] mm");
}

std::vector<cv::Point3f> Pattern::object_points() const
{
  validate();
  std::vector<cv::Point3f> points;
  for (int i = 0; i < rows; ++i)
    for (int j = 0; j < cols; ++j) points.emplace_back(j * spacing_mm, i * spacing_mm, 0);
  return points;
}

bool detect(const cv::Mat & image, const Pattern & pattern, std::vector<cv::Point2f> & points)
{
  pattern.validate();
  points.clear();
  if (image.empty()) return false;
  return cv::findCirclesGrid(image, {pattern.cols, pattern.rows}, points,
                             cv::CALIB_CB_SYMMETRIC_GRID);
}

cv::Mat draw_detection(
  const cv::Mat & image, const Pattern & pattern, const std::vector<cv::Point2f> & points)
{
  auto drawing = image.clone();
  if (drawing.empty() || points.size() != static_cast<size_t>(pattern.cols * pattern.rows))
    return drawing;
  cv::drawChessboardCorners(drawing, {pattern.cols, pattern.rows}, points, true);
  for (size_t i = 0; i < points.size(); ++i) {
    cv::circle(drawing, points[i], 5, {0, 255, 0}, 2, cv::LINE_AA);
    cv::putText(drawing, std::to_string(i), points[i] + cv::Point2f(7, -7),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, {0, 0, 0}, 3, cv::LINE_AA);
    cv::putText(drawing, std::to_string(i), points[i] + cv::Point2f(7, -7),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, {0, 255, 255}, 1, cv::LINE_AA);
  }
  return drawing;
}

Result calibrate(const std::vector<Sample> & samples, const Pattern & pattern)
{
  pattern.validate();
  if (samples.size() < 5) throw std::invalid_argument("At least 5 valid views are required");
  const auto size = samples.front().image.size();
  if (size.empty()) throw std::invalid_argument("Empty sample image");
  std::vector<std::vector<cv::Point3f>> objects;
  std::vector<std::vector<cv::Point2f>> images;
  const auto object = pattern.object_points();
  for (const auto & sample : samples) {
    if (sample.image.size() != size || sample.points.size() != object.size())
      throw std::invalid_argument("Samples must share resolution and contain the complete pattern");
    for (const auto & point : sample.points)
      if (!std::isfinite(point.x) || !std::isfinite(point.y))
        throw std::invalid_argument("Non-finite image point");
    objects.push_back(object);
    images.push_back(sample.points);
  }
  Result result;
  result.image_size = size;
  std::vector<cv::Mat> rotations, translations;
  result.rms = cv::calibrateCamera(
    objects, images, size, result.camera_matrix, result.distort_coeffs, rotations, translations,
    cv::CALIB_FIX_K3,
    {cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, std::numeric_limits<double>::epsilon()});
  if (!std::isfinite(result.rms) || !cv::checkRange(result.camera_matrix) ||
      !cv::checkRange(result.distort_coeffs) || result.camera_matrix.at<double>(0, 0) <= 0 ||
      result.camera_matrix.at<double>(1, 1) <= 0)
    throw std::runtime_error("Invalid calibration solution; collect more diverse views");
  double total = 0;
  for (size_t i = 0; i < samples.size(); ++i) {
    std::vector<cv::Point2f> projected;
    cv::projectPoints(object, rotations[i], translations[i], result.camera_matrix,
                      result.distort_coeffs, projected);
    double sum = 0, squared = 0;
    for (size_t j = 0; j < projected.size(); ++j) {
      const auto error = cv::norm(images[i][j] - projected[j]);
      sum += error;
      squared += error * error;
    }
    result.per_view_mean.push_back(sum / projected.size());
    result.per_view_rms.push_back(std::sqrt(squared / projected.size()));
    total += sum;
  }
  result.mean_error = total / (samples.size() * object.size());
  return result;
}

std::string Result::yaml() const
{
  YAML::Emitter out;
  out.SetDoublePrecision(17);
  out << YAML::BeginMap << YAML::Key << "camera_matrix" << YAML::Value << YAML::Flow
      << std::vector<double>(camera_matrix.begin<double>(), camera_matrix.end<double>())
      << YAML::Key << "distort_coeffs" << YAML::Value << YAML::Flow
      << std::vector<double>(distort_coeffs.begin<double>(), distort_coeffs.end<double>())
      << YAML::Key << "image_width" << YAML::Value << image_size.width << YAML::Key
      << "image_height" << YAML::Value << image_size.height << YAML::Key << "rms_px" << YAML::Value
      << rms << YAML::Key << "mean_error_px" << YAML::Value << mean_error << YAML::Key
      << "per_view_mean_px" << YAML::Value << YAML::Flow << per_view_mean << YAML::Key
      << "per_view_rms_px" << YAML::Value << YAML::Flow << per_view_rms << YAML::EndMap;
  return std::string(out.c_str()) + "\n";
}

void save_manifest(const std::string & directory, const std::vector<Sample> & samples,
                   const Pattern & pattern)
{
  YAML::Emitter out;
  out.SetDoublePrecision(17);
  out << YAML::BeginMap << YAML::Key << "pattern_cols" << YAML::Value << pattern.cols << YAML::Key
      << "pattern_rows" << YAML::Value << pattern.rows << YAML::Key << "center_distance_mm"
      << YAML::Value << pattern.spacing_mm << YAML::Key << "timestamp_clock" << YAML::Value
      << "host steady_clock; valid only within this process session" << YAML::Key << "samples"
      << YAML::Value << YAML::BeginSeq;
  for (const auto & sample : samples) {
    out << YAML::BeginMap << YAML::Key << "id" << YAML::Value << sample.id << YAML::Key << "image"
        << YAML::Value << std::to_string(sample.id) + ".png" << YAML::Key << "timestamp_ns"
        << YAML::Value
        << std::chrono::duration_cast<std::chrono::nanoseconds>(sample.timestamp.time_since_epoch())
             .count()
        << YAML::Key << "orientation_wxyz" << YAML::Value;
    if (sample.orientation_wxyz)
      out << YAML::Flow
          << std::vector<double>(sample.orientation_wxyz->begin(), sample.orientation_wxyz->end());
    else
      out << YAML::Null;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq << YAML::EndMap;
  std::ofstream file(directory + "/samples.yaml");
  file << out.c_str() << '\n';
  if (!file) throw std::runtime_error("Cannot write sample manifest");
}
}  // namespace calibration