#include "mindvision.hpp"

#include <libusb-1.0/libusb.h>

#include <algorithm>
#include <atomic>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{
namespace
{
// 一次最多枚举多少个 MindVision 设备（SDK 需要调用方给出数组容量）
constexpr int kMaxDevices = 8;

// USB2.0 高速 480Mbps 的实测可用带宽（本机实测：640x480 ~130fps、800x600 ~95fps、
// 1024x768 ~59fps、1280x1024 ~36fps，都落在 41~47MB/s 的原始数据带宽上）
constexpr double kUsb2UsableBytesPerSec = 45e6;

// SDK 里的 char[32] 字段不保证以 '\0' 结尾，按长度安全转 std::string 再打日志
std::string str32(const char * s, std::size_t max_len = 32)
{
  return std::string(s, ::strnlen(s, max_len));
}

const char * usb_speed_name(int speed)
{
  switch (speed) {
    case LIBUSB_SPEED_LOW:
      return "USB1.0 低速 (1.5Mbps)";
    case LIBUSB_SPEED_FULL:
      return "USB1.1 全速 (12Mbps)";
    case LIBUSB_SPEED_HIGH:
      return "USB2.0 高速 (480Mbps)";
    case LIBUSB_SPEED_SUPER:
      return "USB3.0 超速 (5Gbps)";
#ifdef LIBUSB_SPEED_SUPER_PLUS
    case LIBUSB_SPEED_SUPER_PLUS:
      return "USB3.1 超速+ (10Gbps)";
#endif
    default:
      return "未知 / 无法查询";
  }
}

MindVisionConfig make_config(double exposure_ms, double gamma, const std::string & vid_pid)
{
  MindVisionConfig config;
  config.exposure_ms = exposure_ms;
  config.gamma = gamma;
  config.vid_pid = vid_pid;
  return config;
}
}  // namespace

MindVision::MindVision(const MindVisionConfig & config)
: config_(config),
  handle_(-1),
  height_(0),
  width_(0),
  quit_(false),
  ok_(false),
  queue_(1),
  vid_(-1),
  pid_(-1)
{
  tools::logger()->info(
    "[MindVision] 配置：exposure={:.3f}ms gamma={:.2f} 设备[{}]{} frame_speed={} 分辨率={}x{} "
    "media_type={} gain={:.0f} usb_reset={}",
    config_.exposure_ms, config_.gamma, config_.device_index,
    config_.friendly_name.empty() ? std::string("") : ("(昵称 " + config_.friendly_name + ")"),
    config_.frame_speed, config_.resolution_width, config_.resolution_height, config_.media_type,
    config_.gain, config_.usb_reset);

  set_vid_pid(config_.vid_pid);
  if (libusb_init(NULL)) tools::logger()->warn("Unable to init libusb!");

  try_open();

  // 守护线程：采集断了就关掉重开（是否需要 libusb 复位由 mv_usb_reset 决定）。
  // 重试间隔递增（0.3s -> 0.6s -> 1.2s ... 上限 5s）：USB2.0 老相机进异常状态后，
  // 每 100ms 疯狂 open/reset 只会一直失败，留点时间给驱动/相机恢复；恢复出图后重置间隔。
  daemon_thread_ = std::thread{[this] {
    auto wait = 300ms;
    while (!quit_) {
      std::this_thread::sleep_for(wait);

      if (ok_) {
        if (got_frame_this_open_.load()) wait = 300ms;  // 真的出图了才重置重试间隔
        continue;
      }

      if (capture_thread_.joinable()) capture_thread_.join();

      close();
      if (config_.usb_reset) reset_usb();
      try_open();
      wait = std::min(wait * 2, std::chrono::milliseconds(5000));
    }
  }};
}

MindVision::MindVision(double exposure_ms, double gamma, const std::string & vid_pid)
: MindVision(make_config(exposure_ms, gamma, vid_pid))
{
}

MindVision::~MindVision()
{
  quit_ = true;
  if (daemon_thread_.joinable()) daemon_thread_.join();
  if (capture_thread_.joinable()) capture_thread_.join();
  close();
  tools::logger()->info("Mindvision destructed.");
}

void MindVision::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  queue_.pop(data);

  img = data.img;
  timestamp = data.timestamp;
}

void MindVision::open()
{
  CameraSdkInit(1);

  // 枚举所有设备（老 USB2.0 相机往往和别的相机同时插着，必须能确认枚举顺序）
  std::vector<tSdkCameraDevInfo> devices(kMaxDevices);
  int camera_num = static_cast<int>(devices.size());
  if (CameraEnumerateDevice(devices.data(), &camera_num) != CAMERA_STATUS_SUCCESS) {
    throw std::runtime_error("Failed to enumerate camera!");
  }
  if (camera_num <= 0) throw std::runtime_error("Not found camera!");
  devices.resize(camera_num);
  log_devices(devices);

  auto index = select_device(devices);

  auto status = CameraInit(&devices[index], -1, -1, &handle_);
  if (status != CAMERA_STATUS_SUCCESS) {
    handle_ = -1;

    if (status == CAMERA_STATUS_DEVICE_IS_OPENED) {
      throw std::runtime_error(
        "Failed to init camera! (-18 设备已经打开：同一枚相机只能被一个进程打开，多半是上一次的程序"
        "没退干净（被 kill / Ctrl+C），用 pgrep -a 看一下占用者，杀掉后再试)");
    }

    throw std::runtime_error("Failed to init camera! (status=" + std::to_string(status) + ")");
  }

  tSdkCameraCapbility cap;
  CameraGetCapability(handle_, &cap);
  if (!logged_capability_) {
    log_capability(cap);
    logged_capability_ = true;
  }

  // 采集缓冲按相机最大分辨率分配；真实输出尺寸以每帧帧头为准（取图线程里会裁掉多余部分）
  width_ = cap.sResolutionRange.iWidthMax;
  height_ = cap.sResolutionRange.iHeightMax;

  CameraSetAeState(handle_, FALSE);                           // 关闭自动曝光
  CameraSetExposureTime(handle_, config_.exposure_ms * 1e3);  // 曝光 ms -> us
  CameraSetGamma(handle_, config_.gamma * 1e2);               // 伽马
  // host 侧 ISP 输出 BGR8，CameraImageProcess 出来的就是 cv::Mat 能直接用的数据
  CameraSetIspOutFormat(handle_, CAMERA_MEDIA_TYPE_BGR8);
  CameraSetTriggerMode(handle_, 0);  // 连续采集模式

  // 帧速模式：SDK 定义 0 低速 / 1 普通 / 2 高速（可选项见上面的「帧速模式」列表）
  // USB2.0 相机带宽不够时必须走低速，否则会持续丢帧、守护线程反复重连
  if (CameraSetFrameSpeed(handle_, config_.frame_speed) != CAMERA_STATUS_SUCCESS) {
    tools::logger()->warn(
      "[MindVision] mv_frame_speed={} 设置失败（可用值见上面的「帧速模式」列表）",
      config_.frame_speed);
  }

  // 数字增益（可选）：USB2.0 高分辨率下曝光时间受限时用它补亮度
  if (config_.gain >= 0) {
    auto g = static_cast<int>(config_.gain);
    if (CameraSetGain(handle_, g, g, g) != CAMERA_STATUS_SUCCESS) {
      tools::logger()->warn(
        "[MindVision] mv_gain={} 设置失败（SDK 设定值，100 = 1.0 倍；可用范围 R[{},{}] G[{},{}] "
        "B[{},{}]）",
        g, cap.sRgbGainRange.iRGainMin, cap.sRgbGainRange.iRGainMax, cap.sRgbGainRange.iGGainMin,
        cap.sRgbGainRange.iGGainMax, cap.sRgbGainRange.iBGainMin, cap.sRgbGainRange.iBGainMax);
    }
  }

  // 输出分辨率（可选，-1 表示不改）：0xFF = 自定义分辨率（ROI），Zoom 传 0 表示不缩放。
  // 注意：改了分辨率，camera_matrix 等标定参数就失效了，必须重新标定
  if (config_.resolution_width > 0 && config_.resolution_height > 0) {
    auto status = CameraSetImageResolutionEx(
      handle_, 0xFF, 0, 0, 0, 0, config_.resolution_width, config_.resolution_height, 0, 0);
    if (status != CAMERA_STATUS_SUCCESS) {
      tools::logger()->warn(
        "[MindVision] mv_resolution={}x{} 设置失败（相机范围 {}x{} ~ {}x{}）",
        config_.resolution_width, config_.resolution_height, cap.sResolutionRange.iWidthMin,
        cap.sResolutionRange.iHeightMin, cap.sResolutionRange.iWidthMax,
        cap.sResolutionRange.iHeightMax);
    } else {
      tools::logger()->info(
        "[MindVision] 输出分辨率已设为 {}x{}（改过分辨率就要重新标定）", config_.resolution_width,
        config_.resolution_height);
    }
  }

  // 输出原始数据格式（可选，-1 表示不改）：索引来自上面的「输出格式」列表
  if (config_.media_type >= 0) {
    if (CameraSetMediaType(handle_, config_.media_type) != CAMERA_STATUS_SUCCESS) {
      tools::logger()->warn(
        "[MindVision] mv_media_type={} 设置失败（可用值见上面的「输出格式」列表）",
        config_.media_type);
    } else {
      tools::logger()->info("[MindVision] 输出原始格式索引已设为 {}", config_.media_type);
    }
  }

  log_usb_link();
  check_usbfs_memory();

  // 记录当前原始输出格式的 bits/px：帧头里的 uBytes 是 host 侧 ISP 输出（BGR8）的大小，
  // 而 USB 上真正传的是原始格式（本相机 Bayer8 = 1B/px），带宽估算必须用原始大小
  INT media_type_index = -1;
  if (
    CameraGetMediaType(handle_, &media_type_index) == CAMERA_STATUS_SUCCESS &&
    media_type_index >= 0 && media_type_index < cap.iMediaTypdeDesc) {
    auto bits = CAMERA_MEDIA_TYPE_PIXEL_SIZE(cap.pMediaTypeDesc[media_type_index].iMediaType);
    if (bits > 0) raw_bytes_per_px_ = bits / 8.0;
  }

  log_effective_settings();

  CameraPlay(handle_);

  got_frame_this_open_.store(false);

  // 取图线程
  capture_thread_ = std::thread{[this] {
    tSdkFrameHead head;
    BYTE * raw;

    ok_ = true;
    while (!quit_) {
      std::this_thread::sleep_for(1ms);

      auto img = cv::Mat(height_, width_, CV_8UC3);

      auto status = CameraGetImageBuffer(handle_, &head, &raw, config_.frame_timeout_ms);
      auto timestamp = std::chrono::steady_clock::now();

      if (status != CAMERA_STATUS_SUCCESS) {
        // 立刻看一眼统计：有效 0 / 丢帧 N 说明不是"暂时没帧"，而是相机/USB 链路进了异常状态
        tSdkFrameStatistic stat;
        if (CameraGetFrameStatistic(handle_, &stat) == CAMERA_STATUS_SUCCESS && stat.iTotal > 0 &&
            stat.iCapture == 0) {
          tools::logger()->warn(
            "[MindVision] Camera dropped!（共 {} 帧、有效 {}、丢帧 {}）有效 0 说明相机/USB 链路进了"
            "异常状态：拔插一次 USB（或换 USB3.0 口/换线）再试",
            stat.iTotal, stat.iCapture, stat.iLost);
        } else {
          tools::logger()->warn("Camera dropped!");
        }

        ok_ = false;
        break;
      }

      CameraImageProcess(handle_, raw, img.data, &head);
      CameraReleaseImageBuffer(handle_, raw);

      // 帧头里的尺寸是处理后的真实输出尺寸：预设分辨率可能小于按最大分辨率分配的缓冲，
      // 这时要裁掉多余部分（否则下游会拿到一张尺寸不对的图）
      if (
        head.iWidth > 0 && head.iHeight > 0 &&
        (head.iWidth != img.cols || head.iHeight != img.rows)) {
        if (head.iWidth <= img.cols && head.iHeight <= img.rows) {
          img = img(cv::Rect(0, 0, head.iWidth, head.iHeight)).clone();
        } else {
          tools::logger()->warn(
            "[MindVision] 输出 {}x{} 超过缓冲 {}x{}，丢弃该帧", head.iWidth, head.iHeight, img.cols,
            img.rows);
          continue;
        }
      }

      if (!logged_first_frame_) {
        logged_first_frame_ = true;
        got_frame_this_open_.store(true);
        tools::logger()->info(
          "[MindVision] 第一帧：{}x{} media=0x{:x}（host ISP 输出 {:.2f} MB/帧）", img.cols,
          img.rows, static_cast<unsigned>(head.uiMediaType), head.uBytes / 1048576.0);
        check_usb_bandwidth(img.cols, img.rows, head.uBytes);
      }

      queue_.push({img, timestamp});
    }
  }};

  tools::logger()->info("Mindvision opened.");
}

void MindVision::try_open()
{
  try {
    open();
  } catch (const std::exception & e) {
    tools::logger()->warn("{}", e.what());
  }
}

void MindVision::close()
{
  if (handle_ == -1) return;

  // 采集统计：丢帧多说明带宽/帧速模式不够（USB2.0 老相机最容易在这里出问题）
  tSdkFrameStatistic stat;
  if (CameraGetFrameStatistic(handle_, &stat) == CAMERA_STATUS_SUCCESS) {
    if (stat.iLost > 0) {
      tools::logger()->warn(
        "[MindVision] 采集统计：共 {} 帧，有效 {}，丢帧 {}（丢帧说明带宽或帧速模式不够，"
        "试 mv_frame_speed=0 或降低 mv_resolution_*）",
        stat.iTotal, stat.iCapture, stat.iLost);
    } else {
      tools::logger()->info(
        "[MindVision] 采集统计：共 {} 帧，有效 {}，丢帧 {}", stat.iTotal, stat.iCapture,
        stat.iLost);
    }
  }

  CameraUnInit(handle_);
  handle_ = -1;  // 复位句柄，避免守护线程重连时对旧句柄重复 CameraUnInit
}

void MindVision::log_devices(const std::vector<tSdkCameraDevInfo> & devices) const
{
  tools::logger()->info("[MindVision] 枚举到 {} 个设备：", devices.size());
  for (std::size_t i = 0; i < devices.size(); i++) {
    const auto & d = devices[i];
    tools::logger()->info(
      "[MindVision]   [{}] {} | 昵称 {} | 接口 {} | sensor {} | SN {} | 实例 {}", i,
      str32(d.acProductName), str32(d.acFriendlyName), str32(d.acPortType), str32(d.acSensorType),
      str32(d.acSn), d.uInstance);
  }
}

int MindVision::select_device(const std::vector<tSdkCameraDevInfo> & devices) const
{
  if (!config_.friendly_name.empty()) {
    for (std::size_t i = 0; i < devices.size(); i++) {
      if (config_.friendly_name == str32(devices[i].acFriendlyName)) return static_cast<int>(i);
    }

    std::string candidates;
    for (const auto & d : devices) candidates += " [" + str32(d.acFriendlyName) + "]";
    throw std::runtime_error(
      "mv_friendly_name=\"" + config_.friendly_name + "\" 没找到，本机候选：" + candidates);
  }

  if (config_.device_index < 0 || config_.device_index >= static_cast<int>(devices.size())) {
    throw std::runtime_error(
      "mv_device_index=" + std::to_string(config_.device_index) + " 超出范围，本机枚举到 " +
      std::to_string(devices.size()) + " 个设备");
  }

  return config_.device_index;
}

void MindVision::log_capability(const tSdkCameraCapbility & cap) const
{
  tools::logger()->info(
    "[MindVision] 分辨率范围 {}x{} ~ {}x{}；预设 {} 个", cap.sResolutionRange.iWidthMin,
    cap.sResolutionRange.iHeightMin, cap.sResolutionRange.iWidthMax,
    cap.sResolutionRange.iHeightMax, cap.iImageSizeDesc);
  for (int i = 0; i < cap.iImageSizeDesc; i++) {
    const auto & r = cap.pImageSizeDesc[i];
    tools::logger()->info(
      "[MindVision]   分辨率[{}] {}x{} ({})", i, r.iWidth, r.iHeight, str32(r.acDescription));
  }

  for (int i = 0; i < cap.iFrameSpeedDesc; i++) {
    tools::logger()->info(
      "[MindVision]   帧速模式[{}] ({})", i, str32(cap.pFrameSpeedDesc[i].acDescription));
  }

  for (int i = 0; i < cap.iMediaTypdeDesc; i++) {
    tools::logger()->info(
      "[MindVision]   输出格式[{}] media=0x{:x} ({})", i,
      static_cast<unsigned>(cap.pMediaTypeDesc[i].iMediaType),
      str32(cap.pMediaTypeDesc[i].acDescription));
  }

  if (cap.sIspCapacity.bMonoSensor) tools::logger()->warn("[MindVision] 这是黑白相机");
}

void MindVision::log_effective_settings() const
{
  tSdkImageResolution res{};
  if (CameraGetImageResolution(handle_, &res) == CAMERA_STATUS_SUCCESS) {
    tools::logger()->info(
      "[MindVision] 实际分辨率 {}x{}（预设索引 {} {}）", res.iWidth, res.iHeight, res.iIndex,
      str32(res.acDescription));
  }

  int frame_speed = -1;
  if (CameraGetFrameSpeed(handle_, &frame_speed) == CAMERA_STATUS_SUCCESS) {
    tools::logger()->info("[MindVision] 实际帧速模式索引 {}", frame_speed);
  }

  INT media_type = -1;
  if (CameraGetMediaType(handle_, &media_type) == CAMERA_STATUS_SUCCESS) {
    tools::logger()->info("[MindVision] 实际输出格式索引 {}", media_type);
  }
}

void MindVision::log_usb_link()
{
  usb_link_speed_ = 0;
  fallback_vid_ = -1;
  fallback_pid_ = -1;

  libusb_device ** list = nullptr;
  auto count = libusb_get_device_list(NULL, &list);
  if (count < 0) {
    tools::logger()->warn("Unable to get usb device list!");
    return;
  }

  // MindVision 的 USB 厂商号，用于在配置的 vid_pid 对不上时提示该填什么
  constexpr unsigned kMindVisionVid = 0xf622;
  int mindvision_devices = 0;
  int vendor_speed = 0;
  unsigned vendor_pid = 0;

  for (ssize_t i = 0; i < count; i++) {
    libusb_device_descriptor desc{};
    if (libusb_get_device_descriptor(list[i], &desc)) continue;

    if (desc.idVendor == kMindVisionVid) {
      mindvision_devices++;
      if (mindvision_devices == 1) {
        vendor_pid = desc.idProduct;
        vendor_speed = libusb_get_device_speed(list[i]);
      }
    }

    if (vid_ == -1 || pid_ == -1) continue;
    if (desc.idVendor != vid_ || desc.idProduct != pid_) continue;

    usb_link_speed_ = libusb_get_device_speed(list[i]);
    tools::logger()->info(
      "[MindVision] USB 链路 {}（配置 VID:PID {:04x}:{:04x}，设备声明 USB{}.{}，bus {} addr {}）",
      usb_speed_name(usb_link_speed_), desc.idVendor, desc.idProduct, (desc.bcdUSB >> 8) & 0xff,
      (desc.bcdUSB >> 4) & 0xf, libusb_get_bus_number(list[i]), libusb_get_device_address(list[i]));
    break;
  }

  if (usb_link_speed_ == 0 && mindvision_devices > 0) {
    if (vid_ != -1 && pid_ != -1) {
      tools::logger()->warn(
        "[MindVision] 配置的 vid_pid=\"{:04x}:{:04x}\" 不在 USB 总线上！本机找到 {} 个 MindVision "
        "设备，第一个是 {:04x}:{:04x} @ {}",
        static_cast<unsigned>(vid_), static_cast<unsigned>(pid_), mindvision_devices,
        kMindVisionVid, vendor_pid, usb_speed_name(vendor_speed));
    } else {
      tools::logger()->info(
        "[MindVision] 没配 vid_pid；本机找到 {} 个 MindVision 设备，第一个是 {:04x}:{:04x} @ {}",
        mindvision_devices, kMindVisionVid, vendor_pid, usb_speed_name(vendor_speed));
    }

    // 只有一枚 MindVision 相机时，按它推断链路速度并作为复位时的兜底 vid_pid
    if (mindvision_devices == 1) {
      usb_link_speed_ = vendor_speed;
      fallback_vid_ = static_cast<int>(kMindVisionVid);
      fallback_pid_ = static_cast<int>(vendor_pid);
    }
  } else if (usb_link_speed_ == 0) {
    tools::logger()->warn(
      "[MindVision] 没找到 VID:PID {:04x}:{:04x} 的 USB 设备，总线上也没有 MindVision 设备",
      static_cast<unsigned>(vid_), static_cast<unsigned>(pid_));
  }

  libusb_free_device_list(list, 1);
}

void MindVision::check_usbfs_memory() const
{
  std::ifstream file("/sys/module/usbcore/parameters/usbfs_memory_mb");
  int mb = -1;
  if (!(file >> mb)) return;

  tools::logger()->info("[MindVision] usbfs 连续内存上限 {} MB", mb);
  if (mb >= 0 && mb < 100) {
    tools::logger()->warn(
      "[MindVision] usbfs 连续内存上限偏小（{} MB，默认 16）：USB2.0 大帧传输失败时先调大它："
      "sudo sh -c 'echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb'",
      mb);
  }
}

void MindVision::check_usb_bandwidth(int frame_w, int frame_h, std::size_t processed_bytes) const
{
  // 只有确认是 USB2.0 链路时才提示，避免在 USB3.0 上误报
  if (usb_link_speed_ != LIBUSB_SPEED_HIGH) return;

  // 注意：帧头里的 uBytes 是 host 侧 ISP 输出（BGR8，3B/px）的大小；
  // USB 上真正传的是原始格式（本相机 Bayer8，1B/px），带宽要按原始大小估算
  auto raw_bytes = static_cast<double>(frame_w) * frame_h * raw_bytes_per_px_;
  auto max_fps = raw_bytes > 0 ? kUsb2UsableBytesPerSec / raw_bytes : 0.0;

  tools::logger()->info(
    "[MindVision] USB2.0 链路：{}x{} 原始 {:.2f} MB/帧（host ISP 输出 {:.2f} MB/帧），"
    "按实测 ~{:.0f} MB/s 估算上限 ~{:.0f} fps",
    frame_w, frame_h, raw_bytes / 1048576.0, processed_bytes / 1048576.0,
    kUsb2UsableBytesPerSec / 1e6, max_fps);

  if (max_fps < 30.0) {
    tools::logger()->warn(
      "[MindVision] 这个分辨率在 USB2.0 上跑不满 30fps（估算 ~{:.0f} fps）：建议 "
      "mv_frame_speed=0（低速）或降低 mv_resolution_width/height（会失效标定）",
      max_fps);
  }
}

void MindVision::set_vid_pid(const std::string & vid_pid)
{
  auto index = vid_pid.find(':');
  if (index == std::string::npos) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
    return;
  }

  auto vid_str = vid_pid.substr(0, index);
  auto pid_str = vid_pid.substr(index + 1);

  try {
    vid_ = std::stoi(vid_str, 0, 16);
    pid_ = std::stoi(pid_str, 0, 16);
  } catch (const std::exception &) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
  }
}

void MindVision::reset_usb() const
{
  auto vid = vid_;
  auto pid = pid_;

  if (vid == -1 || pid == -1) {
    if (fallback_vid_ == -1) return;
    vid = fallback_vid_;
    pid = fallback_pid_;
  }

  // https://github.com/ralight/usb-reset/blob/master/usb-reset.c
  auto handle = libusb_open_device_with_vid_pid(NULL, vid, pid);

  // 配置里的 vid_pid 打不开时，退回用总线上那枚 MindVision 设备（否则重连永远失败）
  if (!handle && fallback_vid_ != -1 && (vid != fallback_vid_ || pid != fallback_pid_)) {
    handle = libusb_open_device_with_vid_pid(NULL, fallback_vid_, fallback_pid_);
    if (handle) {
      tools::logger()->warn(
        "[MindVision] 配置的 vid_pid 打不开，改用总线上实测的 {:04x}:{:04x} 复位",
        static_cast<unsigned>(fallback_vid_), static_cast<unsigned>(fallback_pid_));
    }
  }

  if (!handle) {
    tools::logger()->warn(
      "Unable to open usb! 请用 lsusb 核对 vid_pid（启动日志里有本次实际找到的 USB 链路）");
    return;
  }

  if (libusb_reset_device(handle))
    tools::logger()->warn("Unable to reset usb!");
  else
    tools::logger()->info("Reset usb successfully :)");

  libusb_close(handle);
}

}  // namespace io
