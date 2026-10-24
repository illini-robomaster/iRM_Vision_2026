#ifndef AUTO_AIM__CURRENT_RELATIVE_HPP
#define AUTO_AIM__CURRENT_RELATIVE_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "armor.hpp"
#include "io/command.hpp"
#include "visual_servo.hpp"

namespace auto_aim
{
// 相对角差的「参考轴」——两种语义必须分开写清楚，不能混称「相对相机光轴」：
//   camera_optical：参考点 = 相机光心，参考轴 = 相机光轴（+z_cam）。
//                   只需要内参（camera_matrix / distort_coeffs），**不需要任何外参**；
//   gimbal        ：参考点/轴取云台系（F-L-U），因此必须给 R_camera2gimbal +
//                   t_camera2gimbal。相机的安装偏角、相机光心相对云台的原点偏移
//                   都会真实反映在角度里（与 Solver/Aimer 的云台系约定一致）。
enum class RelativeReference
{
  camera_optical,
  gimbal
};

std::string to_string(RelativeReference reference);

// "camera_optical" / "gimbal"（大小写不敏感）；非法值返回 false
bool parse_relative_reference(const std::string & text, RelativeReference & reference);

// 当前帧几何解算所需的相机参数与有效性门限
struct RelativeGeometry
{
  cv::Mat camera_matrix;   // 3x3 CV_64F
  cv::Mat distort_coeffs;  // 1x5 CV_64F
  RelativeReference reference = RelativeReference::camera_optical;
  bool has_extrinsic = false;  // gimbal 模式必须为 true
  Eigen::Matrix3d R_camera2gimbal = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_camera2gimbal = Eigen::Vector3d::Zero();
  double min_depth = 0.1;          // m，目标在相机前方的最小距离
  double max_reprojection = 15.0;  // px，允许的平均重投影误差
};

// 多目标选择 / 时效 / 过滤配置
struct RelativeFilter
{
  bool filter_color = false;  // true 时只保留 enemy_color
  Color enemy_color = Color::blue;
  double min_confidence = 0.0;
  double max_frame_age_ms = 120.0;  // 观测过期阈值（超过就不发有效值）
};

// 单帧观测结果：raw（几何解算）与 TX（整形 + 量化后真正上总线的值）分离。
// 有效帧：valid=true、command.control=true、command.shoot 恒为 false；
// 无效帧：valid=false、reason 给出原因、command 全为 0（控制权交回电控）。
struct RelativeObservation
{
  bool valid = false;
  std::string reason = "no_target";  // 无效原因（CSV / 日志直接用这个字符串）
  int candidates = 0;                // 通过过滤的候选数
  std::string target_id;             // 本帧选中目标：颜色:编号:大小@像素中心
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();  // 参考系下目标中心（m）
  double raw_yaw = 0.0;                           // rad，几何解算值
  double raw_pitch = 0.0;                         // rad
  double tx_yaw = 0.0;                            // rad，量化后（= 总线上的值）
  double tx_pitch = 0.0;                          // rad
  double reprojection = 0.0;                      // px，平均重投影误差
  bool deadband = false;                          // 整形时落进死区
  bool clamped = false;                           // 整形时被 max_step 截断
  bool saturated = false;                         // 量化时饱和到 int16 边界
  io::Command command{false, false, 0.0, 0.0};
};

// 装甲板物点（big/small 两套，尺寸与 Solver 内部那组一致）。解算与离线测试共用同一份，
// 避免测试用一套物点、实现用另一套而「互相验证了个假的」。
const std::vector<cv::Point3f> & armor_object_points(ArmorType type);

// 角点 → 参考系下的目标中心 → Δyaw / Δpitch（纯函数，便于离线测试）。
// 返回 false 时 reason 给出原因（bad_keypoints / solvepnp_failed /
// behind_camera_or_too_close / reprojection_error / missing_extrinsic）。
bool solve_current_angles(
  const RelativeGeometry & geo, ArmorType type, const std::vector<cv::Point2f> & points,
  Eigen::Vector3d & xyz, double & yaw, double & pitch, double & reprojection, std::string & reason);

// 目标标识字符串：颜色:编号:大小@像素中心，例如 blue:one:small@640/480。
// CSV、HUD、以及「哪一块被选中」的比对都用同一个函数，避免三处格式漂移。
// 用 '/' 而不是 ','：这个字符串要进逐帧 CSV，字段里不能带逗号。
std::string target_id(const Armor & armor);

// 当前帧观测器：**不做跟踪、不做预测**。每帧独立地从这一帧的检测结果里
// 确定性地选一块装甲板，用这一块自己的角点做 PnP，输出该帧的角差。
//
// 为什么不用 Tracker：Tracker 需要连续多帧命中（min_detect_count）才会 set_target，
// 期间 targets 为空 → 恒 control=0；换目标的第一帧还要跳过一次（同型号装甲板尚未
// solve）。这些行为对「预测+绝对角」是必要的，对「单帧相对角差」只是延迟和不确定性。
class CurrentRelativeEstimator
{
public:
  explicit CurrentRelativeEstimator(const std::string & config_path);

  // 直接注入参数（离线测试用，不读 yaml）
  CurrentRelativeEstimator(
    const RelativeGeometry & geometry, const ServoLimits & limits, const RelativeFilter & filter);

  // armors    ：detector.detect() 的**原始**结果（本帧）
  // image_size：本帧图像尺寸，用于「角点必须落在当前帧内」的检查
  // frame_time：该帧时间戳；now：调用时刻（两者之差 = 观测年龄）
  RelativeObservation observe(
    const std::list<Armor> & armors, const cv::Size & image_size,
    std::chrono::steady_clock::time_point frame_time,
    std::chrono::steady_clock::time_point now) const;

  const RelativeGeometry & geometry() const { return geometry_; }
  const ServoLimits & limits() const { return limits_; }
  const RelativeFilter & filter() const { return filter_; }
  RelativeReference reference() const { return geometry_.reference; }

  // 一行摘要（启动日志用：参考系、门限、整形参数）
  std::string describe() const;

private:
  RelativeGeometry geometry_;
  ServoLimits limits_;
  RelativeFilter filter_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__CURRENT_RELATIVE_HPP
