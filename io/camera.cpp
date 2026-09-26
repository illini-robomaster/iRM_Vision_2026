#include "camera.hpp"

#include <stdexcept>
#include <thread>

#include "hikrobot/hikrobot.hpp"
#include "mindvision/mindvision.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{
Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto camera_name = tools::read<std::string>(yaml, "camera_name");

  if (camera_name == "mindvision") {
    // 前三个键是所有 configs/*.yaml 都有的；后面 mv_* 为新增键（都有默认值），
    // 主要给老的 USB2.0 相机用，详见 io/mindvision/mindvision.hpp 里的 MindVisionConfig
    MindVisionConfig config;
    config.exposure_ms = tools::read<double>(yaml, "exposure_ms");
    config.gamma = tools::read<double>(yaml, "gamma");
    config.vid_pid = tools::read<std::string>(yaml, "vid_pid");

    config.device_index = yaml["mv_device_index"] ? yaml["mv_device_index"].as<int>() : 0;
    config.friendly_name =
      yaml["mv_friendly_name"] ? yaml["mv_friendly_name"].as<std::string>() : std::string("");
    config.frame_speed = yaml["mv_frame_speed"] ? yaml["mv_frame_speed"].as<int>() : 1;
    config.resolution_width =
      yaml["mv_resolution_width"] ? yaml["mv_resolution_width"].as<int>() : -1;
    config.resolution_height =
      yaml["mv_resolution_height"] ? yaml["mv_resolution_height"].as<int>() : -1;
    config.media_type = yaml["mv_media_type"] ? yaml["mv_media_type"].as<int>() : -1;
    config.gain = yaml["mv_gain"] ? yaml["mv_gain"].as<double>() : -1.0;
    config.frame_timeout_ms =
      yaml["mv_frame_timeout_ms"] ? yaml["mv_frame_timeout_ms"].as<int>() : 1000;
    config.usb_reset = yaml["mv_usb_reset"] ? yaml["mv_usb_reset"].as<bool>() : true;

    camera_ = std::make_unique<MindVision>(config);
  }

  else if (camera_name == "hikrobot") {
    auto exposure_ms = tools::read<double>(yaml, "exposure_ms");
    auto gain = tools::read<double>(yaml, "gain");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid);
  }

  else if (camera_name == "video") {
    // 离线回放（x86_64 / WSL2）：新增键，均带默认值
    auto video_path = tools::read<std::string>(yaml, "video_path");
    auto frame_rate = yaml["video_frame_rate"] ? yaml["video_frame_rate"].as<double>() : 0.0;
    auto loop = yaml["video_loop"] ? yaml["video_loop"].as<bool>() : true;
    camera_ = std::make_unique<VideoFile>(video_path, frame_rate, loop);
  }

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

VideoFile::VideoFile(const std::string & path, double frame_rate, bool loop)
: path_(path), frame_rate_(frame_rate), loop_(loop)
{
  if (!cap_.open(path_, cv::CAP_FFMPEG)) {
    throw std::runtime_error("Failed to open video file: " + path_);
  }

  if (frame_rate_ <= 0) frame_rate_ = cap_.get(cv::CAP_PROP_FPS);
  if (frame_rate_ <= 0) {
    frame_rate_ = 30.0;  // 兜底
    tools::logger()->warn("[VideoFile] cannot get fps from {}, fallback to 30", path_);
  }

  tools::logger()->info(
    "[VideoFile] {} opened, {} frames (0 = 无索引的 avi), {:.1f} fps, loop={}", path_,
    cap_.get(cv::CAP_PROP_FRAME_COUNT), frame_rate_, loop_);

  next_t_ = std::chrono::steady_clock::now();
}

void VideoFile::rewind()
{
  // 这个 avi 可能没有索引（CAP_PROP_FRAME_COUNT=0），逐帧 seek 会失败，因此用重新打开的方式回卷
  cap_.release();
  if (!cap_.open(path_, cv::CAP_FFMPEG)) {
    throw std::runtime_error("[VideoFile] failed to reopen video file: " + path_);
  }
}

void VideoFile::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  // 按帧率节流，模拟真实相机的取流节奏
  const auto period = std::chrono::duration<double>(1.0 / frame_rate_);
  next_t_ += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
  std::this_thread::sleep_until(next_t_);

  if (!cap_.read(img) || img.empty()) {
    if (!loop_) throw std::runtime_error("[VideoFile] reached end of " + path_);

    tools::logger()->warn("[VideoFile] {} reached the end, rewinding", path_);
    rewind();
    if (!cap_.read(img) || img.empty()) {
      throw std::runtime_error("[VideoFile] failed to read video file: " + path_);
    }
  }

  timestamp = std::chrono::steady_clock::now();
}

}  // namespace io