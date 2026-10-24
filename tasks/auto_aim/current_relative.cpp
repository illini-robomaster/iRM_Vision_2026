#include "current_relative.hpp"

#include <fmt/core.h>

#include <Eigen/Dense>  // 必须在 opencv2/core/eigen.hpp 之前
#include <algorithm>
#include <cctype>
#include <cmath>
#include <opencv2/core/eigen.hpp>
#include <stdexcept>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace auto_aim
{
namespace
{
// 装甲板物理尺寸：与 solver.cpp 里的 BIG_ARMOR_POINTS / SMALL_ARMOR_POINTS **必须一致**
// （同样的角点顺序 + 同样的尺寸，解算出的 tvec 才能与 Solver::solve() 逐值对齐）。
// 这里重复一份是因为它们在 solver.cpp 里是文件作用域的常量，而 Solver 的对外接口
// 按 AGENTS §5 不可改；改尺寸时两边一起改。
constexpr double kLightbarLength = 56e-3;    // m
constexpr double kBigArmorWidth = 230e-3;    // m
constexpr double kSmallArmorWidth = 135e-3;  // m

const std::vector<cv::Point3f> kBigArmorPoints{
  {0, kBigArmorWidth / 2, kLightbarLength / 2},
  {0, -kBigArmorWidth / 2, kLightbarLength / 2},
  {0, -kBigArmorWidth / 2, -kLightbarLength / 2},
  {0, kBigArmorWidth / 2, -kLightbarLength / 2}};
const std::vector<cv::Point3f> kSmallArmorPoints{
  {0, kSmallArmorWidth / 2, kLightbarLength / 2},
  {0, -kSmallArmorWidth / 2, kLightbarLength / 2},
  {0, -kSmallArmorWidth / 2, -kLightbarLength / 2},
  {0, kSmallArmorWidth / 2, -kLightbarLength / 2}};

// 相机光学系（x 右、y 下、z 前）→ 以光轴为 +x 的 F-L-U 语义坐标（x 前、y 左、z 上）。
// 这是**固定约定**（一次 90° 旋转），不是标定结果：它只把「光轴」搬到 F-L-U 的 x 轴上，
// 从而复用 VisualServo::angles() 的符号约定（左正、上负）。
Eigen::Vector3d camera_optical_to_flu(const Eigen::Vector3d & xyz_camera)
{
  Eigen::Vector3d xyz;
  xyz << xyz_camera.z(), -xyz_camera.x(), -xyz_camera.y();
  return xyz;
}

bool inside_image(const cv::Point2f & point, const cv::Size & size)
{
  return point.x >= 0.0f && point.y >= 0.0f && point.x <= size.width - 1.0f &&
         point.y <= size.height - 1.0f;
}

// 候选被剔除的原因统计：没有候选时用固定优先级里第一个出现过的原因作为无效原因，
// 比笼统的 no_target 更能说明是颜色过滤、角点越界、还是置信度不够。
struct Rejections
{
  int color = 0;
  int not_armor = 0;
  int keypoints = 0;
  int out_of_image = 0;
  int confidence = 0;

  std::string reason() const
  {
    if (color > 0) return "color_filtered";
    if (not_armor > 0) return "not_armor";
    if (keypoints > 0) return "bad_keypoints";
    if (out_of_image > 0) return "out_of_image";
    if (confidence > 0) return "low_confidence";
    return "no_target";
  }
};

// 确定性选目标：以画面中心为距离度量；完全同距时用 (color, name, type, center) 兜底，
// 保证同一帧输入永远选出同一块装甲板（不依赖 std::list 的遍历偶然性）。
bool better_candidate(const Armor & a, double da, const Armor & b, double db)
{
  if (da != db) return da < db;
  if (a.color != b.color) return a.color < b.color;
  if (a.name != b.name) return a.name < b.name;
  if (a.type != b.type) return a.type < b.type;
  if (a.center.x != b.center.x) return a.center.x < b.center.x;
  return a.center.y < b.center.y;
}
}  // namespace

std::string to_string(RelativeReference reference)
{
  return reference == RelativeReference::gimbal ? "gimbal" : "camera_optical";
}

bool parse_relative_reference(const std::string & text, RelativeReference & reference)
{
  std::string lower = text;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  if (lower == "camera_optical" || lower == "camera") {
    reference = RelativeReference::camera_optical;
    return true;
  }
  if (lower == "gimbal") {
    reference = RelativeReference::gimbal;
    return true;
  }
  return false;
}

const std::vector<cv::Point3f> & armor_object_points(ArmorType type)
{
  return (type == ArmorType::big) ? kBigArmorPoints : kSmallArmorPoints;
}

bool solve_current_angles(
  const RelativeGeometry & geo, ArmorType type, const std::vector<cv::Point2f> & points,
  Eigen::Vector3d & xyz, double & yaw, double & pitch, double & reprojection, std::string & reason)
{
  reprojection = 0.0;

  if (geo.camera_matrix.rows != 3 || geo.camera_matrix.cols != 3) {
    reason = "bad_intrinsics";
    return false;
  }
  if (geo.reference == RelativeReference::gimbal && !geo.has_extrinsic) {
    reason = "missing_extrinsic";
    return false;
  }
  if (points.size() != 4) {
    reason = "bad_keypoints";
    return false;
  }
  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      reason = "bad_keypoints";
      return false;
    }
  }

  const auto & object_points = armor_object_points(type);

  cv::Vec3d rvec, tvec;
  try {
    cv::solvePnP(
      object_points, points, geo.camera_matrix, geo.distort_coeffs, rvec, tvec, false,
      cv::SOLVEPNP_IPPE);
  } catch (const cv::Exception &) {
    reason = "solvepnp_failed";
    return false;
  }

  Eigen::Vector3d xyz_camera;
  cv::cv2eigen(tvec, xyz_camera);
  if (!xyz_camera.allFinite() || xyz_camera[2] < geo.min_depth) {
    reason = "behind_camera_or_too_close";
    return false;
  }

  // 用解出来的位姿反投影一遍：IPPE 在角点抖动 / 姿态退化时会给出发散的姿态，
  // 重投影误差是最便宜的可信度指标（也是「这一帧的几何解算是否可信」的判据）。
  std::vector<cv::Point2f> reprojected;
  try {
    cv::projectPoints(
      object_points, rvec, tvec, geo.camera_matrix, geo.distort_coeffs, reprojected);
  } catch (const cv::Exception &) {
    reason = "solvepnp_failed";
    return false;
  }
  if (reprojected.size() != points.size()) {
    reason = "solvepnp_failed";
    return false;
  }

  double error = 0.0;
  for (std::size_t i = 0; i < points.size(); ++i) error += cv::norm(reprojected[i] - points[i]);
  reprojection = error / static_cast<double>(points.size());
  if (!std::isfinite(reprojection) || reprojection > geo.max_reprojection) {
    reason = "reprojection_error";
    return false;
  }

  // 参考系：只有 gimbal 模式才用外参；camera_optical 模式只做「光轴当 +x」的约定变换，
  // 参考点就是相机光心，所以不需要 t_camera2gimbal。
  if (geo.reference == RelativeReference::gimbal) {
    xyz = geo.R_camera2gimbal * xyz_camera + geo.t_camera2gimbal;
  } else {
    xyz = camera_optical_to_flu(xyz_camera);
  }
  if (!xyz.allFinite()) {
    reason = "non_finite";
    return false;
  }

  // 角度语义与跟踪路径完全一致（同一个函数）：左正、向上为负
  VisualServo::angles(xyz, yaw, pitch);
  if (!std::isfinite(yaw) || !std::isfinite(pitch)) {
    reason = "non_finite";
    return false;
  }

  reason = "ok";
  return true;
}

CurrentRelativeEstimator::CurrentRelativeEstimator(const std::string & config_path)
{
  auto yaml = tools::load(config_path);

  // ---- 参考系 ----
  const auto reference_text = tools::optional_string(
    yaml, "relative_output_frame",
    tools::optional_string(yaml, "relative_reference", "camera_optical"));
  if (!parse_relative_reference(reference_text, geometry_.reference)) {
    throw std::runtime_error(
      "relative_reference 取值非法：'" + reference_text +
      "'（只能是 \"camera_optical\" 或 \"gimbal\"）");
  }

  // ---- 内参（必需，与 Solver 同键名同含义）----
  if (!yaml["camera_matrix"] || !yaml["distort_coeffs"]) {
    throw std::runtime_error(
      "相对角差需要 camera_matrix / distort_coeffs（与 Solver 同一组键），" + config_path +
      " 里缺少其中之一");
  }
  const auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  if (camera_matrix_data.size() != 9) {
    throw std::runtime_error(
      "camera_matrix 必须是 9 个数（3x3 行优先），当前 " +
      std::to_string(camera_matrix_data.size()) + " 个");
  }
  geometry_.camera_matrix = cv::Mat(3, 3, CV_64F, cv::Scalar(0));
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      geometry_.camera_matrix.at<double>(r, c) = camera_matrix_data[r * 3 + c];

  const auto distort_data = yaml["distort_coeffs"].as<std::vector<double>>();
  if (
    distort_data.size() != 4 && distort_data.size() != 5 && distort_data.size() != 8 &&
    distort_data.size() != 12) {
    throw std::runtime_error(
      "distort_coeffs 长度必须是 4/5/8/12，当前 " + std::to_string(distort_data.size()) + " 个");
  }
  geometry_.distort_coeffs = cv::Mat(1, static_cast<int>(distort_data.size()), CV_64F);
  for (std::size_t i = 0; i < distort_data.size(); ++i)
    geometry_.distort_coeffs.at<double>(0, static_cast<int>(i)) = distort_data[i];

  // ---- gimbal 模式的外参（缺键时明确报错，不静默按 camera_optical 处理）----
  if (geometry_.reference == RelativeReference::gimbal) {
    if (!yaml["R_camera2gimbal"] || !yaml["t_camera2gimbal"]) {
      throw std::runtime_error(
        "relative_reference: \"gimbal\" 需要 R_camera2gimbal（9 个数）+ t_camera2gimbal（3 "
        "个数）；" +
        config_path +
        " 里缺这两个键。要么补手眼外参，要么改回 relative_reference: \"camera_optical\"");
    }
    const auto R = yaml["R_camera2gimbal"].as<std::vector<double>>();
    const auto t = yaml["t_camera2gimbal"].as<std::vector<double>>();
    if (R.size() != 9 || t.size() != 3) {
      throw std::runtime_error(
        "R_camera2gimbal 必须是 9 个数、t_camera2gimbal 必须是 3 个数（当前 " +
        std::to_string(R.size()) + " / " + std::to_string(t.size()) + "）");
    }
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_camera2gimbal(R.data());
    geometry_.R_camera2gimbal = R_camera2gimbal;
    geometry_.t_camera2gimbal = Eigen::Vector3d(t[0], t[1], t[2]);
    geometry_.has_extrinsic = true;
  }

  geometry_.min_depth = tools::optional_double(yaml, "relative_min_depth_m", 0.1);
  geometry_.max_reprojection = tools::optional_double(yaml, "relative_max_reprojection_px", 15.0);

  // ---- 过滤 / 时效 ----
  filter_.max_frame_age_ms = tools::optional_double(yaml, "relative_max_frame_age_ms", 120.0);
  filter_.min_confidence = tools::optional_double(
    yaml, "relative_min_confidence",
    yaml["min_confidence"] ? yaml["min_confidence"].as<double>() : 0.0);

  if (yaml["enemy_color"]) {
    const auto text = yaml["enemy_color"].as<std::string>();
    const auto it = std::find(COLORS.begin(), COLORS.end(), text);
    if (it == COLORS.end()) {
      throw std::runtime_error(
        "enemy_color 取值非法：'" + text + "'（只能是 red/blue/extinguish/purple）");
    }
    filter_.enemy_color = static_cast<Color>(std::distance(COLORS.begin(), it));
    filter_.filter_color = true;
  }
  // 显式开关优先于「有没有写 enemy_color」
  filter_.filter_color = tools::optional_bool(yaml, "relative_filter_color", filter_.filter_color);

  // TX 整形开关：默认 false（TX = 符号 × (raw + 偏置) → 量化，不引入 gain/死区/限幅）
  filter_.shape_tx = tools::optional_bool(yaml, "relative_tx_shape", false);

  // 与跟踪路径共用同一份 servo_* 读取，两条路的 gain/deadband/limit 不会各说各话
  limits_ = load_servo_limits(config_path);
}

CurrentRelativeEstimator::CurrentRelativeEstimator(
  const RelativeGeometry & geometry, const ServoLimits & limits, const RelativeFilter & filter)
: geometry_(geometry), limits_(limits), filter_(filter)
{
}

std::string CurrentRelativeEstimator::describe() const
{
  std::string text = fmt::format(
    "[CurrentRelative] 参考系={} 最小距离={:.2f}m 最大重投影={:.1f}px 过期={:.0f}ms 颜色过滤={}",
    to_string(geometry_.reference), geometry_.min_depth, geometry_.max_reprojection,
    filter_.max_frame_age_ms, filter_.filter_color ? COLORS[filter_.enemy_color] : "off");

  if (geometry_.reference == RelativeReference::gimbal) {
    text += fmt::format(
      " 外参t_camera2gimbal=[{:.1f},{:.1f},{:.1f}]mm", geometry_.t_camera2gimbal[0] * 1000.0,
      geometry_.t_camera2gimbal[1] * 1000.0, geometry_.t_camera2gimbal[2] * 1000.0);
  }

  text += fmt::format(
    " | 整形 gain={:.2f} deadband={:.2f}deg max_step={:.2f}deg yaw_sign={:+.0f} "
    "pitch_sign={:+.0f} yaw_offset={:.2f}deg pitch_offset={:.2f}deg",
    limits_.gain, limits_.deadband * 57.3, limits_.max_step * 57.3, limits_.yaw_sign,
    limits_.pitch_sign, limits_.yaw_offset * 57.3, limits_.pitch_offset * 57.3);

  return text;
}

RelativeObservation CurrentRelativeEstimator::observe(
  const std::list<Armor> & armors, const cv::Size & image_size,
  std::chrono::steady_clock::time_point frame_time, std::chrono::steady_clock::time_point now,
  const RelativeFilter * filter_override) const
{
  RelativeObservation obs;
  const RelativeFilter & filter = filter_override ? *filter_override : filter_;

  // ---- 1) 过滤 + 收集候选（只用**这一帧**的信息）----
  const Armor * selected = nullptr;
  double selected_distance = 0.0;
  int candidates = 0;
  Rejections rejected;

  struct Candidate
  {
    const Armor * armor;
    double distance;
  };
  std::vector<Candidate> ordered;
  ordered.reserve(armors.size());

  for (const auto & armor : armors) {
    if (filter.filter_color && armor.color != filter.enemy_color) {
      ++rejected.color;
      continue;
    }
    if (armor.name == ArmorName::not_armor) {
      ++rejected.not_armor;
      continue;
    }
    if (armor.points.size() != 4) {
      ++rejected.keypoints;
      continue;
    }
    bool all_inside = true;
    for (const auto & point : armor.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !inside_image(point, image_size)) {
        all_inside = false;
        break;
      }
    }
    if (!all_inside) {
      ++rejected.out_of_image;
      continue;
    }
    if (armor.confidence < filter.min_confidence) {
      ++rejected.confidence;
      continue;
    }

    ++candidates;
    const double distance = std::hypot(armor.center_norm.x - 0.5f, armor.center_norm.y - 0.5f);
    ordered.push_back({&armor, distance});
    if (selected == nullptr || better_candidate(armor, distance, *selected, selected_distance)) {
      selected = &armor;
      selected_distance = distance;
    }
  }

  obs.candidates = candidates;
  if (selected == nullptr) {
    obs.reason = rejected.reason();
    return obs;
  }

  // 用 '/' 而不是 ','：这个字符串要进逐帧 CSV，字段里不能带逗号（避免再做转义）
  obs.target_id = target_id(*selected);

  // 按「离画面中心最近」排序（完全同距时用 better_candidate 兜底，保证确定性）
  std::sort(ordered.begin(), ordered.end(), [](const Candidate & a, const Candidate & b) {
    if (a.distance != b.distance) return a.distance < b.distance;
    return better_candidate(*a.armor, a.distance, *b.armor, b.distance);
  });

  // ---- 2) 时效：过期帧不能当有效观测（控制权交回电控）----
  const double age_ms = tools::delta_time(now, frame_time) * 1000.0;
  if (age_ms > filter.max_frame_age_ms) {
    obs.reason = "stale_frame";
    return obs;
  }

  // ---- 3) 几何解算：最近的解不出来就退到下一块 ----
  // 单帧观测没有跟踪器帮忙，一块装甲板角点退化时不该让整帧变成 control=0：
  // 按离画面中心的顺序尝试，最多试 kMaxGeometryAttempts 块（成本上限 = 常数次 PnP）。
  std::string reason = "no_target";
  bool solved = false;
  for (std::size_t i = 0; i < ordered.size() && i < kMaxGeometryAttempts; ++i) {
    const Armor & armor = *ordered[i].armor;
    if (solve_current_angles(
          geometry_, armor.type, armor.points, obs.xyz, obs.raw_yaw, obs.raw_pitch,
          obs.reprojection, reason)) {
      obs.target_id = target_id(armor);
      solved = true;
      break;
    }
  }
  if (!solved) {
    obs.reason = reason;
    return obs;
  }

  // ---- 4) TX：默认 raw → 符号/偏置 → 量化，**不经过** VisualServo::shape() ----
  // 为什么分开：gain / 死区 / max_step 是「执行侧该怎么动」的整形，而 current 路径
  // 的输出语义是「这一帧测到的角差」。观测本身被这些参数改过以后，CSV / HUD 上的 TX
  // 就不再是测量值，现场无法区分「测错了」还是「整形改过头了」。
  // 需要旧行为（gain/死区/20° 限幅）时把 relative_tx_shape 置 true。
  const double biased_yaw = obs.raw_yaw + limits_.yaw_offset;
  const double biased_pitch = obs.raw_pitch + limits_.pitch_offset;

  io::Command command{false, false, 0.0, 0.0};
  double tx_yaw = 0.0;
  double tx_pitch = 0.0;
  if (filter.shape_tx) {
    command = VisualServo::shape(limits_, obs.raw_yaw, obs.raw_pitch);
    obs.deadband =
      std::abs(biased_yaw) < limits_.deadband || std::abs(biased_pitch) < limits_.deadband;
    obs.clamped =
      std::abs(biased_yaw) > limits_.max_step || std::abs(biased_pitch) > limits_.max_step;
    tx_yaw = command.yaw;
    tx_pitch = command.pitch;
  } else {
    // 只做符号（镜像安装）与静态偏置，不做增益/死区/限幅
    tx_yaw = limits_.yaw_sign * biased_yaw;
    tx_pitch = limits_.pitch_sign * biased_pitch;
  }

  // 量化与 CAN 编码共用同一个函数，保证 CSV 里的 TX 与总线字节逐位一致
  const int16_t yaw_1e4 = io::quantize_angle_1e4(tx_yaw);
  const int16_t pitch_1e4 = io::quantize_angle_1e4(tx_pitch);
  obs.tx_yaw = static_cast<double>(yaw_1e4) / 1e4;
  obs.tx_pitch = static_cast<double>(pitch_1e4) / 1e4;
  obs.saturated =
    yaw_1e4 == 32767 || yaw_1e4 == -32768 || pitch_1e4 == 32767 || pitch_1e4 == -32768;

  // 发出去的 Command 直接用「量化后」的值：CSV 里的 TX 与总线上的字节逐位一致，
  // 不会出现「日志写 0.12345、总线上是 0.1234」这种对不上的情况。
  obs.command.control = true;
  obs.command.shoot = false;  // 相对角差模式不开火
  obs.command.yaw = obs.tx_yaw;
  obs.command.pitch = obs.tx_pitch;
  obs.command.horizon_distance = 0.0;

  obs.valid = true;
  obs.reason = "ok";
  return obs;
}

std::string target_id(const Armor & armor)
{
  return fmt::format(
    "{}:{}:{}@{:.0f}/{:.0f}", COLORS[armor.color], ARMOR_NAMES[armor.name], ARMOR_TYPES[armor.type],
    armor.center.x, armor.center.y);
}

}  // namespace auto_aim
