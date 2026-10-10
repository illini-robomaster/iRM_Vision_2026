#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <iostream>
#include <stdexcept>

#include "calibration/intrinsic_session.hpp"

namespace
{
void require(bool condition, const std::string & message)
{
  if (!condition) throw std::runtime_error(message);
}

template <typename Function>
void rejects(Function function)
{
  bool rejected = false;
  try {
    function();
  } catch (const std::exception &) {
    rejected = true;
  }
  require(rejected, "Expected rejection");
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    calibration::Pattern pattern;
    rejects([] {
      calibration::Pattern p;
      p.cols = 0;
      p.validate();
    });
    rejects([] {
      calibration::Pattern p;
      p.spacing_mm = 0;
      p.validate();
    });
    std::vector<calibration::Sample> samples;
    cv::Mat k = (cv::Mat_<double>(3, 3) << 900, 0, 640, 0, 920, 480, 0, 0, 1);
    cv::Mat d = (cv::Mat_<double>(1, 5) << -0.04, 0.015, 0.001, -0.001, 0);
    for (int i = 0; i < 16; ++i) {
      calibration::Sample sample;
      sample.id = i + 1;
      sample.image = cv::Mat(960, 1280, CV_8UC3, cv::Scalar(255, 255, 255));
      sample.timestamp = std::chrono::steady_clock::now();
      cv::Vec3d r(0.12 * ((i % 4) - 1.5), 0.15 * ((i / 4) - 1.5), 0.03 * i);
      cv::Vec3d t(-180 + 30 * (i % 4), -130 + 25 * (i / 4), 750 + 35 * i);
      cv::projectPoints(pattern.object_points(), r, t, k, d, sample.points);
      samples.push_back(sample);
    }
    rejects([&] { calibrate({}, pattern); });
    auto result = calibrate(samples, pattern);
    require(result.rms < 0.001 && result.mean_error < 0.001, "Synthetic reprojection error");
    require(std::abs(result.camera_matrix.at<double>(0, 0) - 900) < 0.1, "fx recovery");
    require(std::abs(result.camera_matrix.at<double>(1, 1) - 920) < 0.1, "fy recovery");
    const auto yaml = YAML::Load(result.yaml());
    require(yaml["camera_matrix"].size() == 9 && yaml["distort_coeffs"].size() == 5, "YAML shape");
    require(yaml["per_view_mean_px"].size() == samples.size(), "Per-view report");
    auto bad = samples;
    bad.back().image = cv::Mat(100, 100, CV_8UC3);
    rejects([&] { calibrate(bad, pattern); });
    bad = samples;
    bad.back().points.pop_back();
    rejects([&] { calibrate(bad, pattern); });

    // Render a clean symmetric circle grid and test the actual detector, not a fake result.
    cv::Mat board(420, 600, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int row = 0; row < pattern.rows; ++row)
      for (int col = 0; col < pattern.cols; ++col)
        cv::circle(board, {75 + col * 50, 60 + row * 50}, 12, {0, 0, 0}, -1, cv::LINE_AA);
    std::vector<cv::Point2f> centers;
    require(detect(board, pattern, centers) && centers.size() == 70, "Rendered grid detection");
    cv::Mat blank(board.size(), board.type(), cv::Scalar(255, 255, 255));
    require(!detect(blank, pattern, centers), "Blank frame rejected");
    if (argc == 2) {
      std::filesystem::create_directories(argv[1]);
      require(cv::imwrite(std::string(argv[1]) + "/00.png", board), "Write fixture");
      cv::Mat offline_blank(900, 1200, CV_8UC3, cv::Scalar(255, 255, 255));
      require(cv::imwrite(std::string(argv[1]) + "/99.png", offline_blank), "Write blank fixture");
      for (int i = 0; i < 8; ++i) {
        cv::Mat image(900, 1200, CV_8UC3, cv::Scalar(255, 255, 255));
        cv::Mat camera = (cv::Mat_<double>(3, 3) << 950, 0, 600, 0, 970, 450, 0, 0, 1);
        std::vector<cv::Point2f> projected;
        cv::projectPoints(
          pattern.object_points(), cv::Vec3d(0.12 * (i % 3 - 1), 0.1 * (i / 3 - 1), 0.04 * i),
          cv::Vec3d(-180 + 20 * i, -120, 900 + 35 * i), camera, cv::Mat(), projected);
        for (const auto & point : projected)
          cv::circle(image, point, 9, {0, 0, 0}, -1, cv::LINE_AA);
        require(detect(image, pattern, centers), "Perspective fixture detection");
        require(cv::imwrite(std::string(argv[1]) + "/1" + std::to_string(i) + ".png", image),
                "Write perspective fixture");
      }
      save_manifest(argv[1], samples, pattern);
      const auto manifest = YAML::LoadFile(std::string(argv[1]) + "/samples.yaml");
      require(manifest["samples"][0]["orientation_wxyz"].IsNull(), "Missing pose must be null");
    }
    std::cout << "PASS: synthetic solve, YAML, invalid inputs, circle detector, optional pose\n"
              << "RMS=" << result.rms << " mean=" << result.mean_error << '\n';
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}