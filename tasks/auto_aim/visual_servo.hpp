#ifndef AUTO_AIM__VISUAL_SERVO_HPP
#define AUTO_AIM__VISUAL_SERVO_HPP

#include <Eigen/Dense>
#include <list>
#include <string>

#include "armor.hpp"
#include "io/command.hpp"
#include "target.hpp"

namespace auto_aim
{
// 视觉伺服参数（configs/*.yaml 里的 servo_* 键，缺键取下面的默认值）
struct ServoLimits
{
  double gain = 1.0;         // 单帧角差比例系数
  double deadband = 0.0;     // rad，偏差小于该值不发
  double max_step = 0.35;    // rad，单帧最大角差（≈20°）
  double yaw_sign = 1.0;     // 转向反了改成 -1
  double pitch_sign = 1.0;   // 转向反了改成 -1
  double yaw_offset = 0.0;   // rad，静态偏置
  double pitch_offset = 0.0; // rad，静态偏置（弹道/安装误差）
};

// 相对角差（视觉伺服）命令生成器
// ---------------------------------------------------------------------------
// 与 Aimer 的区别：Aimer 输出「世界坐标系下的绝对角」，需要电机/云台姿态反馈；
// VisualServo 只用 solver 算出的 armor.xyz_in_gimbal（目标在云台系下的坐标，
// 与 IMU 四元数无关），得到「目标相对**云台系参考轴**」的 yaw/pitch 偏差，
// 由电控按增量执行 —— 图像上的偏差会逐帧收敛到 0，不需要编码器。
//
// 坐标系：云台系 F-L-U（x 前、y 左、z 上），与 Solver::solve() 一致
//   Δyaw   = atan2(y, x)          （左转为正）
//   Δpitch = -atan2(z, sqrt(x²+y²))（沿用 CBoard/Aimer 约定：向上为负）
//
// ⚠️ 这里的参考轴是**云台系**（相机光心到云台原点的平移 t_camera2gimbal 已包含在
// xyz_in_gimbal 里）。「相对相机光轴」是另一套语义，见
// auto_aim::RelativeReference::camera_optical（tasks/auto_aim/current_relative.hpp）：
// 只有 t_camera2gimbal == 0 且 R_camera2gimbal 恰好把光轴对到云台 x 轴时两者才等价。
//
// 从 yaml 读 servo_* 键（缺键取默认值）。VisualServo 与 CurrentRelativeEstimator 共用，
// 保证「跟踪路径」与「当前帧路径」的 gain/deadband/limit 语义完全一致。
ServoLimits load_servo_limits(const std::string & config_path);

class VisualServo
{
public:
  explicit VisualServo(const std::string & config_path);

  // armors：tracker.track() 过滤过颜色的列表（同型号装甲板需已被 solver 解算过）
  // targets：tracker.track() 的返回值，空 = 没锁定目标，此时不产生命令
  io::Command aim(const std::list<Armor> & armors, const std::list<Target> & targets);

  // 云台系坐标 → 相对角差（纯函数，便于离线测试）
  static void angles(const Eigen::Vector3d & xyz_in_gimbal, double & yaw, double & pitch);

  // 死区 / 限幅 / 增益 / 符号 / 偏置（纯函数，便于离线测试）
  static io::Command shape(const ServoLimits & limits, double yaw, double pitch);

private:
  ServoLimits limits_;

  // 换目标后的第一帧不发命令：set_target() 只对选中的那一块调用了 solver.solve()，
  // 其余同型号装甲板的 xyz_in_gimbal 还没算过（不可用）。下一帧 update_target()
  // 会把所有命中的同型号装甲板都解算一遍，从第二帧开始发才安全。
  bool has_last_target_ = false;
  ArmorName last_name_ = ArmorName::not_armor;
  ArmorType last_type_ = ArmorType::big;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__VISUAL_SERVO_HPP
