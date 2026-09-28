#ifndef IO__MINDVISION_HPP
#define IO__MINDVISION_HPP

#include <atomic>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>

#include "CameraApi.h"
#include "io/camera.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
// MindVision 相机的打开参数。
//
// 每个字段对应一个「新增配置键」，缺失时用下面的默认值，因此已有 configs/*.yaml
// （只写 exposure_ms / gamma / vid_pid）行为与移植前完全一致。
//
// 这些新增键主要服务于老的 USB2.0 相机（MV-SUA133GC-T1V-C）：
// USB2.0 只有 480Mbps（实测可用带宽约 40MB/s），1280x1024 的原始帧就要 1.3MB 以上，
// 带宽不够时相机会持续丢帧、SDK 报错、守护线程不停重连。所以需要能：
//   - 在多相机里选对设备（mv_device_index / mv_friendly_name）
//   - 降帧速模式（mv_frame_speed：SDK 的 0 低速 / 1 普通 / 2 高速）
//   - 必要时改分辨率或输出原始格式（mv_resolution_* / mv_media_type）
//   - 把链路速度、帧统计打到日志里（USB2.0 还是 USB3.0，丢了多少帧）
struct MindVisionConfig
{
  double exposure_ms = 2.0;  // exposure_ms，单位 ms
  double gamma = 1.0;        // gamma
  std::string vid_pid = "";  // vid_pid，如 "f622:d13a"（USB 复位与链路诊断用）

  int device_index = 0;            // mv_device_index：枚举到的第 N 个设备
  std::string friendly_name = "";  // mv_friendly_name：按相机昵称选设备（优先于 index）

  // mv_frame_speed：CameraSetFrameSpeed 索引，SDK 定义 0 低速 / 1 普通 / 2 高速
  int frame_speed = 1;

  // mv_resolution_width / mv_resolution_height：-1 表示不改（用相机当前预设分辨率）
  // 注意：改分辨率会让现有标定参数（camera_matrix 等）失效，需要重新标定
  int resolution_width = -1;
  int resolution_height = -1;

  // mv_media_type：输出原始数据格式索引（pMediaTypeDesc 的索引），-1 表示不改
  int media_type = -1;

  // mv_frame_timeout_ms：CameraGetImageBuffer 等一帧的超时（ms）。
  // 原实现写死 100ms；USB2.0 老相机一帧要传 1MB+，低速模式下帧间隔可能 >100ms，
  // 写死 100ms 会把正常出图误判成掉线，所以改成可配置（默认 1000ms）
  int frame_timeout_ms = 1000;

  // mv_gain：数字增益的 SDK 设定值（100 表示 1.0 倍，范围见 sRgbGainRange），-1 表示不改
  double gain = -1;

  // mv_usb_reset：掉线时是否先用 libusb 复位设备再重连（默认 true，与移植前一致）
  bool usb_reset = true;
};
class MindVision : public CameraBase
{
public:
  explicit MindVision(const MindVisionConfig & config);
  // 兼容旧签名：只配置曝光 / 伽马 / vid_pid，其余用默认值
  MindVision(double exposure_ms, double gamma, const std::string & vid_pid);
  ~MindVision() override;
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
  struct CameraData
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
  };

  MindVisionConfig config_;
  CameraHandle handle_;
  int height_, width_;  // 采集缓冲尺寸（按相机最大分辨率分配，真实输出尺寸以帧头为准）
  bool quit_, ok_;
  std::thread capture_thread_;
  std::thread daemon_thread_;
  tools::ThreadSafeQueue<CameraData> queue_;
  int vid_, pid_;
  int usb_link_speed_ = 0;  // libusb_get_device_speed：3 = USB2.0(480M)，4 = USB3.0
  double raw_bytes_per_px_ = 1.0;  // 原始输出格式的每像素字节数（Bayer8 = 1），带宽估算用
  int fallback_vid_ = -1;  // 配置的 vid_pid 在总线上找不到时，退回用它复位 USB
  int fallback_pid_ = -1;
  std::atomic<bool> got_frame_this_open_{false};  // 本次 open 是否真的出过图（守护线程退避用）
  bool logged_capability_ = false;  // 相机能力只打一次（重连时不必重复）
  bool logged_first_frame_ = false;

  void open();
  void try_open();
  void close();
  void set_vid_pid(const std::string & vid_pid);
  void reset_usb() const;

  // 把枚举到的设备逐条打出来（现场多相机 / 认错设备时排查用）
  void log_devices(const std::vector<tSdkCameraDevInfo> & devices) const;
  // 按 friendly_name（优先）或 device_index 选设备；选不到就抛异常并列出候选
  int select_device(const std::vector<tSdkCameraDevInfo> & devices) const;
  // 打印相机能力：分辨率预设 / 帧速模式 / 输出格式（选参数全靠它）
  void log_capability(const tSdkCameraCapbility & cap) const;
  // 打印本次实际生效的参数（分辨率、帧速模式、输出格式、增益）
  void log_effective_settings() const;
  // 用 libusb 查这枚相机协商到的 USB 链路速度并记到 usb_link_speed_
  void log_usb_link();
  // USB2.0 带宽提示：一帧原始数据多大、按实测带宽估算上限帧率、不够 30fps 就告警
  void check_usb_bandwidth(int frame_w, int frame_h, std::size_t frame_bytes) const;
  // usbfs 连续内存上限检查（默认 16MB，USB2.0 大帧传输失败时的常见原因）
  void check_usbfs_memory() const;
};

}  // namespace io

#endif  // IO__MINDVISION_HPP