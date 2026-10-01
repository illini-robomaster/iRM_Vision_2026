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

  // ---------------------------------------------------------------------------
  // 固定 pipeline（AGENTS.md §8.8）
  //
  // 为什么需要：移植后的 open() 只设了 曝光 / 伽马 /（可选）数字增益，其余 ISP 参数
  // （模拟增益、白平衡、色温、锐度、对比度、饱和度、抗闪、光源频率）全部沿用相机或
  // SDK 上一次会话留下的值 —— 也就是说换台机器、或者被别的程序（MindVision 官方
  // 演示程序）动过之后，成像会悄悄变化，标定与阈值就跟着漂。
  //
  // 现在：凡是在 yaml 里显式写了的键，open() 都会按固定顺序置位，然后逐项 CameraGet*
  // 回读并打到日志里（可直接 diff 两次启动的日志确认一致）；没写（-1 / 空）的键保持
  // "不改"，所以已有 configs/*.yaml 的行为不变。
  // ---------------------------------------------------------------------------
  // mv_data_dir：SDK 数据目录（参数文件 .config / 设备参数 .mvdat 的存放位置），
  //              空 = 不调用 CameraSetDataDirectory（保持 SDK 默认，即当前工作目录下的 Camera/）
  std::string data_dir = "";

  // 参数存取对象：0 按型号 / 1 按昵称 / 2 按序列号 / 3 相机内置存储，-1 = 不改
  int parameter_mode = -1;
  // CameraSetParameterMask 的掩码（位 = PROP_SHEET_INDEX_*，0..16），-1 = 不改。
  // 想"存/取全部参数"就用 0x1FFFF（bit0~bit16 全置位）
  int parameter_mask = -1;
  // 从参数组加载：0..3 = A/B/C/D，255 = 出厂默认，-1 = 不加载
  int parameter_load_group = -1;
  // 从文件加载整组参数（.config / .mvdat），空 = 不加载
  std::string parameter_file = "";
  // 退出（析构）时把当前参数存成文件 —— 用来产出"黄金方案"，跨机器复制
  std::string parameter_save_file = "";
  // 退出（析构）时把当前参数存进相机/SDK 的 A/B/C/D 参数组，-1 = 不保存
  int parameter_save_group = -1;

  // mv_analog_gain：模拟增益（SDK 设定值，100 = 1.0 倍），-1 = 不改
  int analog_gain = -1;
  // mv_wb_mode：0 = 手动白平衡 / 1 = 自动白平衡，-1 = 不改
  int wb_mode = -1;
  // mv_clr_temp_mode：0 = 自动识别色温 / 1 = 预设色温 / 2 = 自定义色温，-1 = 不改
  int clr_temp_mode = -1;
  // mv_clr_temp_gain：自定义色温增益，格式 "R,G,B"（0~400，100 = 1.0 倍），空 = 不改
  std::string clr_temp_gain = "";
  // mv_once_wb：置位后做一次白平衡（手动白平衡下用它把当前光色对齐）
  bool once_wb = false;
  // mv_sharpness / mv_contrast / mv_saturation：-1 = 不改
  int sharpness = -1;
  int contrast = -1;
  int saturation = -1;
  // mv_anti_flick：抗闪 0 = 关 / 1 = 开，-1 = 不改
  int anti_flick = -1;
  // mv_light_frequency：光源频率 0 = 50Hz / 1 = 60Hz，-1 = 不改（室内灯光下必须与电网一致）
  int light_frequency = -1;
  // mv_frame_rate：期望输出帧率（Hz），-1 = 不改（部分型号不支持，设置失败会告警）
  int frame_rate = -1;
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
  // 参数来源（固定 pipeline 第一步）：data_dir / 参数存取对象 / 掩码 / 从文件或参数组加载
  void setup_parameter_source();
  // 固定 pipeline 第二步：按固定顺序逐项置位（模拟增益、白平衡、色温、锐度、对比度、
  // 饱和度、抗闪、光源频率、帧率）。只有 yaml 里显式写了（!= -1 / 非空）的项才动，
  // 每一项失败都会告警（不静默忽略）
  void apply_pipeline(const tSdkCameraCapbility & cap);
  // 固定 pipeline 第三步（可选）：退出时把当前参数存成文件 / 存进参数组，产出"黄金方案"
  void save_pipeline() const;
  // 打印本次实际生效的参数（逐项 CameraGet* 回读：分辨率、帧速、输出格式、曝光、
  // 增益、白平衡、色温、锐度、对比度、饱和度、抗闪、光源频率、帧率、触发模式）
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