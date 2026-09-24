#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace io
{
class CameraBase
{
public:
  virtual ~CameraBase() = default;
  virtual void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
};

// 视频文件回放后端（x86_64 / WSL2 离线调试用；接不到相机时替代 mindvision / hikrobot）
// 对应配置（均为新增键，已有 configs 不受影响）：
//   camera_name: "video"     视频文件回放
//   video_path: assets/demo/demo.avi
//   video_loop: true         播放到结尾后回到开头（默认 true）
//   video_frame_rate: 0      取流帧率，0 表示使用视频文件自带帧率（默认 0）
class VideoFile : public CameraBase
{
public:
  VideoFile(const std::string & path, double frame_rate, bool loop);

  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
  void rewind();

  cv::VideoCapture cap_;
  std::string path_;
  double frame_rate_;
  bool loop_;
  std::chrono::steady_clock::time_point next_t_;
};

class Camera
{
public:
  Camera(const std::string & config_path);
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);

private:
  std::unique_ptr<CameraBase> camera_;
};

}  // namespace io

#endif  // IO__CAMERA_HPP