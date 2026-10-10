#ifndef CALIBRATION__INTRINSIC_SESSION_HPP
#define CALIBRATION__INTRINSIC_SESSION_HPP

#include <array>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>

namespace calibration
{
using Timestamp = std::chrono::steady_clock::time_point;

// Extension contract: return body-to-reference orientation in wxyz order at image time.
// No provider is constructed by the intrinsic GUI. A missing pose is never replaced by identity.
class PoseProvider
{
 public:
  virtual ~PoseProvider() = default;
  virtual std::optional<std::array<double, 4>> orientation_at(Timestamp timestamp) = 0;
};

struct Pattern
{
  int cols = 10;
  int rows = 7;
  double spacing_mm = 40;
  void validate() const;
  std::vector<cv::Point3f> object_points() const;
};

struct Sample
{
  int id = 0;
  cv::Mat image;
  std::vector<cv::Point2f> points;
  Timestamp timestamp;
  std::optional<std::array<double, 4>> orientation_wxyz;
};

struct Result
{
  cv::Mat camera_matrix;
  cv::Mat distort_coeffs;
  cv::Size image_size;
  double rms = 0;
  double mean_error = 0;
  std::vector<double> per_view_mean;
  std::vector<double> per_view_rms;
  std::string yaml() const;
};

bool detect(const cv::Mat & image, const Pattern & pattern, std::vector<cv::Point2f> & points);
Result calibrate(const std::vector<Sample> & samples, const Pattern & pattern);
// Export metadata and raw images only. Hand-eye solving remains explicitly unavailable.
void save_manifest(const std::string & directory, const std::vector<Sample> & samples,
                   const Pattern & pattern);
}  // namespace calibration
#endif