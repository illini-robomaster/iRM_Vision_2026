// ---------------------------------------------------------------------------
// 相对角差（视觉伺服）离线自检：不需要相机 / CAN / 电机
//   1) VisualServo::angles：云台系坐标 → Δyaw / Δpitch（符号、坐标系）
//   2) VisualServo::shape ：死区 / 限幅 / 增益 / 符号 / 偏置
//   3) encode_relative_command：CAN 帧字节布局（int16 大端 ×1e4 rad、饱和）
//
// 用法：./build/relative_servo_test        # 全部通过返回 0，有 FAIL 返回 1
// ---------------------------------------------------------------------------
#include <fmt/core.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "io/cboard.hpp"
#include "tasks/auto_aim/current_relative.hpp"
#include "tasks/auto_aim/visual_servo.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

int checks = 0;
int failures = 0;

void expect(bool ok, const std::string & name)
{
  ++checks;
  if (ok) {
    fmt::print("ok    {}\n", name);
  } else {
    ++failures;
    fmt::print("FAIL  {}\n", name);
  }
}

bool near(double a, double b, double eps = 1e-9) { return std::abs(a - b) <= eps; }

// 与固件侧一致的解码方式（大端、有符号）
int decode_i16(const uint8_t * p)
{
  const int raw = (p[0] << 8) | p[1];
  return raw >= 0x8000 ? raw - 0x10000 : raw;
}

void test_angles()
{
  using auto_aim::VisualServo;
  double yaw = 1, pitch = 1;

  VisualServo::angles({1, 0, 0}, yaw, pitch);
  expect(near(yaw, 0) && near(pitch, 0), "angles: 目标在正前方 -> (0, 0)");

  VisualServo::angles({0, 1, 0}, yaw, pitch);
  expect(near(yaw, kPi / 2) && near(pitch, 0), "angles: 目标在正左方 -> dYaw=+90deg");

  VisualServo::angles({0, -1, 0}, yaw, pitch);
  expect(near(yaw, -kPi / 2), "angles: 目标在正右方 -> dYaw=-90deg");

  VisualServo::angles({1, 0, 1}, yaw, pitch);
  expect(near(pitch, -kPi / 4), "angles: 目标在上方 -> dPitch 为负（向上为负）");

  VisualServo::angles({1, 0, -1}, yaw, pitch);
  expect(near(pitch, kPi / 4), "angles: 目标在下方 -> dPitch 为正");
}

void test_shape()
{
  using auto_aim::ServoLimits;
  using auto_aim::VisualServo;

  ServoLimits limits;
  limits.gain = 1.0;
  limits.deadband = 0.1 * kDeg;
  limits.max_step = 20.0 * kDeg;

  auto inside = VisualServo::shape(limits, 0.05 * kDeg, -0.05 * kDeg);
  expect(
    inside.control && near(inside.yaw, 0) && near(inside.pitch, 0), "shape: 死区内不发角差");

  auto outside = VisualServo::shape(limits, 0.2 * kDeg, -0.2 * kDeg);
  expect(
    outside.control && near(outside.yaw, 0.2 * kDeg) && near(outside.pitch, -0.2 * kDeg),
    "shape: 死区外原样下发（gain=1, sign=+1）");

  auto clamped = VisualServo::shape(limits, 100 * kDeg, -100 * kDeg);
  expect(
    near(clamped.yaw, limits.max_step) && near(clamped.pitch, -limits.max_step),
    "shape: 单帧角差超过 max_step 时饱和到 +/-20deg");

  ServoLimits signd;
  signd.gain = 0.5;
  signd.yaw_sign = -1;
  signd.pitch_sign = -1;
  auto flipped = VisualServo::shape(signd, 2 * kDeg, 4 * kDeg);
  expect(
    near(flipped.yaw, -1 * kDeg) && near(flipped.pitch, -2 * kDeg),
    "shape: gain=0.5 且 sign=-1 -> 角度减半并反向");

  ServoLimits offset;
  offset.yaw_offset = 1 * kDeg;
  offset.pitch_offset = -1 * kDeg;
  auto biased = VisualServo::shape(offset, 0, 0);
  expect(
    near(biased.yaw, 1 * kDeg) && near(biased.pitch, -1 * kDeg),
    "shape: 静态偏置总是叠加（servo_*_offset_deg）");

  auto combined = VisualServo::shape(limits, -30 * kDeg, 30 * kDeg);
  expect(
    near(combined.yaw, -limits.max_step) && near(combined.pitch, limits.max_step),
    "shape: 负方向同样饱和");
}

void test_encode()
{
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  io::Command command{true, true, 0.1234, -0.0567, 0.0};
  io::encode_relative_command(command, data);
  expect(data[0] == 0x03, "encode: data[0] bit0=control bit1=shoot");
  expect(data[1] == 0 && data[6] == 0 && data[7] == 0, "encode: 保留字节为 0");
  expect(decode_i16(data + 2) == 1234, "encode: dYaw 大端 int16，单位 1e-4 rad");
  expect(decode_i16(data + 4) == -567, "encode: dPitch 大端 int16，负值");

  io::Command idle{false, false, 0, 0, 0};
  io::encode_relative_command(idle, data);
  expect(
    data[0] == 0x00 && decode_i16(data + 2) == 0 && decode_i16(data + 4) == 0,
    "encode: 无目标（control=0）时角差为 0，不会跳变");

  io::Command saturated{true, false, 10.0, -10.0, 0.0};
  io::encode_relative_command(saturated, data);
  expect(
    decode_i16(data + 2) == 32767 && decode_i16(data + 4) == -32768,
    "encode: 超出 int16 范围时饱和（不回绕）");

  io::Command roundtrip{true, false, -0.3, 0.3, 0.0};
  io::encode_relative_command(roundtrip, data);
  expect(
    near(decode_i16(data + 2) / 1e4, -0.3) && near(decode_i16(data + 4) / 1e4, 0.3),
    "encode: 编解码往返误差 <= 1e-4 rad");
}

void test_pipeline()
{
  using auto_aim::ServoLimits;
  using auto_aim::VisualServo;

  ServoLimits limits;
  limits.gain = 1.0;
  limits.deadband = 0.1 * kDeg;
  limits.max_step = 20.0 * kDeg;

  // 目标在前方 2m、右侧 0.3m（云台系 +y 向左）、上方 0.4m，角度都在 max_step 之内
  double yaw = 0, pitch = 0;
  VisualServo::angles({2, -0.3, 0.4}, yaw, pitch);
  auto command = VisualServo::shape(limits, yaw, pitch);

  expect(
    command.control && command.yaw < 0 && command.pitch < 0,
    "pipeline: 右前上方目标 -> dYaw<0（右转）且 dPitch<0（向上为负）");

  expect(
    near(command.yaw, std::atan2(-0.3, 2.0)) &&
      near(command.pitch, -std::atan2(0.4, std::sqrt(2.0 * 2.0 + 0.3 * 0.3))),
    "pipeline: 幅度与 atan2 解析解一致");

  // 更偏右（yaw≈-26.6deg）时必须被 max_step 截断，防止首帧猛甩
  VisualServo::angles({2, -1, 0.4}, yaw, pitch);
  expect(
    near(VisualServo::shape(limits, yaw, pitch).yaw, -limits.max_step),
    "pipeline: 超出 max_step 的 dYaw 被截断");

  // 极近且几乎在正侧方（≈90deg）：必须被 max_step 截断，防止首帧猛甩
  VisualServo::angles({0.001, 1, 0}, yaw, pitch);
  auto extreme = VisualServo::shape(limits, yaw, pitch);
  expect(near(extreme.yaw, limits.max_step), "pipeline: 极端角差被 max_step 截断");
}

void test_encode_hardening()
{
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  // NaN / Inf 角度 = 无效观测：control 与 shoot 必须清零，角度写 0（不能让 lround 吃到 NaN）
  io::Command nan_cmd{true, true, nan, 0.2, 0.0};
  io::encode_relative_command(nan_cmd, data);
  expect(
    data[0] == 0x00 && decode_i16(data + 2) == 0 && decode_i16(data + 4) == 0,
    "encode: NaN 角差 -> control/shoot 清零、角差写 0");

  io::Command inf_cmd{true, true, 0.3, -inf, 0.0};
  io::encode_relative_command(inf_cmd, data);
  expect(
    data[0] == 0x00 && decode_i16(data + 2) == 0 && decode_i16(data + 4) == 0,
    "encode: Inf 角差 -> control/shoot 清零、角差写 0");

  // control=0 时即使调用方忘了清零角度，帧里也不允许残留「有效值」
  io::Command stale{false, true, 0.5, -0.4, 0.0};
  io::encode_relative_command(stale, data);
  expect(
    data[0] == 0x00 && decode_i16(data + 2) == 0 && decode_i16(data + 4) == 0,
    "encode: control=0 -> 角差与 shoot 强制清零（不把上一帧角度留在帧里）");

  // 量化：饱和不回绕、NaN/Inf -> 0、正常四舍五入
  expect(io::quantize_angle_1e4(3.2767) == 32767, "quantize: +3.2767 rad 饱和到 32767");
  expect(io::quantize_angle_1e4(-3.2768) == -32768, "quantize: -3.2768 rad 饱和到 -32768");
  expect(io::quantize_angle_1e4(100.0) == 32767, "quantize: 超大幅度不回绕（+100 rad -> 32767）");
  expect(
    io::quantize_angle_1e4(nan) == 0 && io::quantize_angle_1e4(inf) == 0,
    "quantize: NaN/Inf -> 0");
  expect(
    io::quantize_angle_1e4(0.00016) == 2, "quantize: 0.00016 rad 四舍五入为 2（×1e4）");
}

// ---------------------------------------------------------------------------
// 当前帧几何解算 / 选目标：用「自己投影出来的角点」反解，验证解出来的角差
// 与解析解一致（相机系 → 光轴当 +x 的 F-L-U），并验证两种参考系不可混用。
// ---------------------------------------------------------------------------
cv::Mat make_camera_matrix()
{
  // 与 configs/mv_sua133gc.yaml / records/calibration 同一组内参（1280x1024）
  cv::Mat K(3, 3, CV_64F);
  K.at<double>(0, 0) = 1646.694479464929;
  K.at<double>(0, 1) = 0;
  K.at<double>(0, 2) = 629.3347317116185;
  K.at<double>(1, 0) = 0;
  K.at<double>(1, 1) = 1649.159708561839;
  K.at<double>(1, 2) = 518.9448176031;
  K.at<double>(2, 0) = 0;
  K.at<double>(2, 1) = 0;
  K.at<double>(2, 2) = 1;
  return K;
}

cv::Mat make_distort(const std::vector<double> & values)
{
  cv::Mat d(1, static_cast<int>(values.size()), CV_64F);
  for (std::size_t i = 0; i < values.size(); ++i)
    d.at<double>(0, static_cast<int>(i)) = values[i];
  return d;
}

// 把装甲板放在相机系的 tvec 处（正对相机）并投影出 4 个角点
std::vector<cv::Point2f> project_armor(
  const cv::Mat & K, const cv::Mat & D, auto_aim::ArmorType type, const cv::Vec3d & tvec)
{
  std::vector<cv::Point2f> points;
  cv::projectPoints(auto_aim::armor_object_points(type), cv::Vec3d(0, 0, 0), tvec, K, D, points);
  return points;
}

void test_current_geometry()
{
  using auto_aim::ArmorType;
  using auto_aim::RelativeReference;
  using auto_aim::solve_current_angles;

  const auto type = ArmorType::small;
  const auto K = make_camera_matrix();
  const auto zero = make_distort({0, 0, 0, 0, 0});
  // 相机光学系：x 右、y 下、z 前。目标在左 0.3m、上 0.2m（y 为负=上）、前 2m
  const cv::Vec3d tvec(-0.3, -0.2, 2.0);
  const auto points = project_armor(K, zero, type, tvec);

  // 解析解：F-L-U 下 (x,y,z) = (2.0, 0.3, 0.2)
  const double expected_yaw = std::atan2(0.3, 2.0);
  const double expected_pitch = -std::atan2(0.2, std::sqrt(2.0 * 2.0 + 0.3 * 0.3));

  auto_aim::RelativeGeometry cam;
  cam.camera_matrix = K;
  cam.distort_coeffs = zero;
  cam.reference = RelativeReference::camera_optical;

  Eigen::Vector3d xyz;
  double yaw = 0, pitch = 0, reproj = 0;
  std::string reason;

  bool ok = solve_current_angles(cam, type, points, xyz, yaw, pitch, reproj, reason);
  expect(
    ok && near(yaw, expected_yaw, 1e-4) && near(pitch, expected_pitch, 1e-4),
    "current: camera_optical 左上目标 -> 左正 / 上负，与解析解一致");
  expect(
    near(xyz[0], 2.0, 1e-3) && near(xyz[1], 0.3, 1e-3) && near(xyz[2], 0.2, 1e-3),
    "current: camera_optical 参考系坐标 = (前, 左, 上)");
  expect(ok && reproj < 0.5, "current: 无噪声角点的平均重投影误差 < 0.5 px");

  // 带畸变时必须同样恢复（证明 distort_coeffs 真被传进 solvePnP / projectPoints）
  const auto D = make_distort({-0.07563482581065475, 0.274743634586268, 0.001505720850613093,
                               0.0007716521495002592, 0});
  auto cam_distorted = cam;
  cam_distorted.distort_coeffs = D;
  ok = solve_current_angles(
    cam_distorted, type, project_armor(K, D, type, tvec), xyz, yaw, pitch, reproj, reason);
  expect(
    ok && near(yaw, expected_yaw, 1e-4) && near(pitch, expected_pitch, 1e-4),
    "current: 带畸变内参解出同一角差（D 传对了）");

  // gimbal 模式 + 「光轴→F-L-U」的正确外参 → 与 camera_optical 完全一致
  auto_aim::RelativeGeometry gimbal = cam;
  gimbal.reference = RelativeReference::gimbal;
  gimbal.has_extrinsic = true;
  // clang-format off
  gimbal.R_camera2gimbal << 0,  0, 1,
                            -1,  0, 0,
                             0, -1, 0;
  // clang-format on
  gimbal.t_camera2gimbal = Eigen::Vector3d::Zero();

  Eigen::Vector3d xyz_g;
  double yaw_g = 0, pitch_g = 0, reproj_g = 0;
  std::string reason_g;
  ok = solve_current_angles(gimbal, type, points, xyz_g, yaw_g, pitch_g, reproj_g, reason_g);
  expect(
    ok && near(yaw_g, expected_yaw, 1e-4) && near(pitch_g, expected_pitch, 1e-4),
    "current: gimbal + 光轴→F-L-U 外参 与 camera_optical 结果一致");

  // 参考点平移参与解算：t_camera2gimbal=(0.1,0,0) → 角差真的变了
  gimbal.t_camera2gimbal = Eigen::Vector3d(0.1, 0.0, 0.0);
  ok = solve_current_angles(gimbal, type, points, xyz_g, yaw_g, pitch_g, reproj_g, reason_g);
  expect(
    ok && near(yaw_g, std::atan2(0.3, 2.1), 1e-4) && std::abs(yaw_g - expected_yaw) > 1e-3,
    "current: gimbal 模式的 t_camera2gimbal 参与解算（平移会改变角差）");

  // 缺外参：明确报错，不静默按 camera_optical 算
  gimbal.has_extrinsic = false;
  ok = solve_current_angles(gimbal, type, points, xyz_g, yaw_g, pitch_g, reproj_g, reason_g);
  expect(!ok && reason_g == "missing_extrinsic", "current: gimbal 模式缺外参 -> missing_extrinsic");

  // 用单位阵外参（把相机系当云台系）→ 角差完全不同：两种参考系不可混用
  auto wrong = gimbal;
  wrong.has_extrinsic = true;
  wrong.R_camera2gimbal = Eigen::Matrix3d::Identity();
  wrong.t_camera2gimbal = Eigen::Vector3d::Zero();
  ok = solve_current_angles(wrong, type, points, xyz_g, yaw_g, pitch_g, reproj_g, reason_g);
  expect(
    ok && std::abs(yaw_g) > 1.5,
    "current: gimbal 用单位阵外参 -> 角差与 camera_optical 完全不同（不可混用）");

  // min_depth：目标进入最小距离之内判为无效
  auto too_close_geo = cam;
  too_close_geo.min_depth = 0.5;
  ok = solve_current_angles(
    too_close_geo, type, project_armor(K, zero, type, cv::Vec3d(0, 0, 0.3)), xyz, yaw, pitch,
    reproj, reason);
  expect(!ok && reason == "behind_camera_or_too_close", "current: 目标近于 min_depth -> 判为无效");

  // 角点被污染：重投影误差超限 → 这一帧不可信
  auto corrupted = points;
  corrupted[0].x += 60.0f;
  auto strict = cam;
  strict.max_reprojection = 5.0;
  ok = solve_current_angles(strict, type, corrupted, xyz, yaw, pitch, reproj, reason);
  expect(
    !ok && reason == "reprojection_error" && reproj > 5.0,
    "current: 角点异常 -> 重投影误差超限，判为无效");

  // 角点数不足
  const std::vector<cv::Point2f> three(points.begin(), points.begin() + 3);
  ok = solve_current_angles(cam, type, three, xyz, yaw, pitch, reproj, reason);
  expect(!ok && reason == "bad_keypoints", "current: 角点数 != 4 -> bad_keypoints");
}

// ---------------------------------------------------------------------------
// 当前帧观测器：过滤 / 确定性选目标 / 时效 / 首帧即可用 / TX 量化
// ---------------------------------------------------------------------------
auto_aim::Armor make_armor(
  int class_id, float confidence, const std::vector<cv::Point2f> & points,
  const cv::Point2f & center_norm)
{
  auto_aim::Armor armor(class_id, confidence, cv::Rect(), points);
  // 真实链路里 center_norm 由 postprocess 计算，这里手动给（选目标用它算距离）
  armor.center_norm = center_norm;
  return armor;
}

void test_estimator()
{
  using auto_aim::ArmorType;
  using auto_aim::CurrentRelativeEstimator;
  using auto_aim::RelativeFilter;
  using auto_aim::RelativeGeometry;
  using auto_aim::RelativeReference;
  using auto_aim::ServoLimits;

  const auto K = make_camera_matrix();
  const auto zero = make_distort({0, 0, 0, 0, 0});
  const auto type = ArmorType::small;
  const auto points = project_armor(K, zero, type, cv::Vec3d(-0.3, -0.2, 2.0));

  RelativeGeometry geo;
  geo.camera_matrix = K;
  geo.distort_coeffs = zero;
  geo.reference = RelativeReference::camera_optical;

  ServoLimits limits;  // 默认 gain=1、deadband=0、max_step=20deg、符号 +1
  RelativeFilter filter;
  filter.filter_color = true;
  filter.enemy_color = auto_aim::Color::blue;
  filter.min_confidence = 0.8;
  filter.max_frame_age_ms = 50.0;

  const CurrentRelativeEstimator est(geo, limits, filter);

  const cv::Size size(1280, 1024);
  const auto t0 = std::chrono::steady_clock::now();

  auto obs = est.observe({}, size, t0, t0);
  expect(!obs.valid && obs.reason == "no_target", "est: 本帧无检出 -> no_target");

  // 首帧即可用：不要求连续命中、不存在「换目标首帧不发」
  const auto blue = make_armor(3 /*blue:one:small*/, 0.9f, points, {0.5f, 0.5f});
  obs = est.observe({blue}, size, t0, t0);
  expect(
    obs.valid && obs.command.control && !obs.command.shoot,
    "est: 第一帧就给出 valid 观测，且 shoot 恒为 0（不依赖 tracker 锁定）");
  expect(
    near(obs.tx_yaw, obs.raw_yaw, 1e-4) && near(obs.tx_pitch, obs.raw_pitch, 1e-4),
    "est: gain=1 / 无死区时 TX 与 raw 只差 1e-4 rad 量化");
  expect(
    obs.reason == "ok" && obs.reprojection < 1.0 && obs.target_id.find(',') == std::string::npos,
    "est: 有效帧 reason=ok、重投影小、target_id 不含逗号（CSV 安全）");
  expect(
    std::abs(obs.raw_yaw - std::atan2(0.3, 2.0)) < 1e-3,
    "est: raw 角差与解析解一致（左正）");

  // 颜色过滤
  const auto red = make_armor(4 /*red:one:small*/, 0.9f, points, {0.5f, 0.5f});
  obs = est.observe({red}, size, t0, t0);
  expect(!obs.valid && obs.reason == "color_filtered", "est: 颜色与 enemy_color 不符 -> color_filtered");

  // 置信度
  obs = est.observe({make_armor(3, 0.5f, points, {0.5f, 0.5f})}, size, t0, t0);
  expect(!obs.valid && obs.reason == "low_confidence", "est: 置信度低于阈值 -> low_confidence");

  // 角点必须落在当前帧内
  auto outside_points = points;
  outside_points[1].x = -3.0f;
  obs = est.observe({make_armor(3, 0.9f, outside_points, {0.5f, 0.5f})}, size, t0, t0);
  expect(!obs.valid && obs.reason == "out_of_image", "est: 角点不在当前帧内 -> out_of_image");

  // class_id 非法（not_armor）
  obs = est.observe({make_armor(-1, 0.9f, points, {0.5f, 0.5f})}, size, t0, t0);
  expect(!obs.valid && obs.reason == "not_armor", "est: not_armor 被过滤");

  // 时效：过期就不能当有效观测
  obs = est.observe({blue}, size, t0, t0 + std::chrono::milliseconds(60));
  expect(
    !obs.valid && obs.reason == "stale_frame",
    "est: 观测年龄超过 relative_max_frame_age_ms -> stale_frame");

  // 确定性选目标：更靠近画面中心者胜（与 list 顺序无关）
  const auto far = make_armor(3, 0.9f, points, {0.1f, 0.1f});
  const auto near_center = make_armor(3, 0.9f, points, {0.5f, 0.5f});
  obs = est.observe({far, near_center}, size, t0, t0);
  expect(
    obs.candidates == 2 && obs.target_id == auto_aim::target_id(near_center),
    "est: 多目标时确定性地选最靠近画面中心的一块");
  obs = est.observe({near_center, far}, size, t0, t0);
  expect(
    obs.candidates == 2 && obs.target_id == auto_aim::target_id(near_center),
    "est: 选目标与检测列表顺序无关（确定性）");

  // 整形：max_step 生效且被记录
  auto small_step = limits;
  small_step.max_step = 2.0 / 57.3;  // 2deg
  const CurrentRelativeEstimator est_clamped(geo, small_step, filter);
  obs = est_clamped.observe({blue}, size, t0, t0);
  expect(
    obs.valid && obs.clamped && std::abs(obs.tx_yaw) <= 2.0 / 57.3 + 1e-9,
    "est: 超出 max_step 时 TX 被截断并标记 clamped");
}

}  // namespace

int main()
{
  test_angles();
  test_shape();
  test_encode();
  test_encode_hardening();
  test_pipeline();
  test_current_geometry();
  test_estimator();

  fmt::print("\n{} checks, {} failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
