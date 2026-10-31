// ---------------------------------------------------------------------------
// 相对角差（视觉伺服）离线自检：不需要相机 / CAN / 电机
//   1) VisualServo::angles：云台系坐标 → Δyaw / Δpitch（符号、坐标系）
//   2) VisualServo::shape ：死区 / 限幅 / 增益 / 符号 / 偏置
//   3) encode_relative_command：CAN 帧字节布局（int16 大端 ×1e4 rad、饱和）
//
// 用法：./build/relative_servo_test        # 全部通过返回 0，有 FAIL 返回 1
// ---------------------------------------------------------------------------
#include <fmt/core.h>
#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <future>
#include <limits>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>

#include "io/cboard.hpp"
#include "io/vision_cboard.hpp"
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
  expect(inside.control && near(inside.yaw, 0) && near(inside.pitch, 0), "shape: 死区内不发角差");

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
  expect(data[0] == 0x01, "encode: data[0] bit0=control，bit1=shoot 恒为 0（强制不开火）");
  expect(data[1] == 0 && data[6] == 0 && data[7] == 0, "encode: 保留字节为 0");
  expect(decode_i16(data + 2) == 1234, "encode: dYaw 大端 int16，单位 1e-4 rad");
  expect(decode_i16(data + 4) == -567, "encode: dPitch 大端 int16，负值");

  // 整帧黄金向量（documentation/relative_can_protocol.md §3.3）：电控侧按这张表解码，
  // 任何字节变化都必须先改文档 + 版本号，不能只改代码。
  const uint8_t golden[8] = {0x01, 0x00, 0x04, 0xD2, 0xFD, 0xC9, 0x00, 0x00};
  expect(
    std::memcmp(data, golden, 8) == 0,
    "encode: 示例角差 +0.1234/-0.0567 -> 01 00 04 D2 FD C9 00 00（整帧黄金向量）");

  io::Command idle{false, false, 0, 0, 0};
  io::encode_relative_command(idle, data);
  expect(
    data[0] == 0x00 && decode_i16(data + 2) == 0 && decode_i16(data + 4) == 0,
    "encode: 无目标（control=0）时角差为 0，不会跳变");
  const uint8_t stop_golden[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  expect(
    std::memcmp(data, stop_golden, 8) == 0,
    "encode: stop 帧是 8 个 0 字节（不留旧角度、不留 shoot）");

  io::Command saturated{true, false, 10.0, -10.0, 0.0};
  io::encode_relative_command(saturated, data);
  expect(
    decode_i16(data + 2) == 32767 && decode_i16(data + 4) == -32768,
    "encode: 超出 int16 范围时饱和（不回绕）");
  const uint8_t saturated_golden[8] = {0x01, 0x00, 0x7F, 0xFF, 0x80, 0x00, 0x00, 0x00};
  expect(
    std::memcmp(data, saturated_golden, 8) == 0,
    "encode: 饱和帧 -> 01 00 7F FF 80 00 00 00（整帧黄金向量）");

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
    io::quantize_angle_1e4(nan) == 0 && io::quantize_angle_1e4(inf) == 0, "quantize: NaN/Inf -> 0");
  expect(io::quantize_angle_1e4(0.00016) == 2, "quantize: 0.00016 rad 四舍五入为 2（×1e4）");
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
  for (std::size_t i = 0; i < values.size(); ++i) d.at<double>(0, static_cast<int>(i)) = values[i];
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
  const auto D = make_distort(
    {-0.07563482581065475, 0.274743634586268, 0.001505720850613093, 0.0007716521495002592, 0});
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
  expect(std::abs(obs.raw_yaw - std::atan2(0.3, 2.0)) < 1e-3, "est: raw 角差与解析解一致（左正）");

  // 颜色过滤
  const auto red = make_armor(4 /*red:one:small*/, 0.9f, points, {0.5f, 0.5f});
  obs = est.observe({red}, size, t0, t0);
  expect(
    !obs.valid && obs.reason == "color_filtered", "est: 颜色与 enemy_color 不符 -> color_filtered");

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

  // Color-agnostic deployment: choose nearest valid armor across both teams.
  auto any_color = filter;
  any_color.filter_color = false;
  const auto red_near = make_armor(4, 0.9f, points, {0.5f, 0.5f});
  obs = est.observe({far, red_near}, size, t0, t0, &any_color);
  expect(
    obs.valid && obs.candidates == 2 && obs.target_id == auto_aim::target_id(red_near),
    "est: 不筛颜色时，近处红板胜过远处蓝板");
  const auto red_far = make_armor(4, 0.9f, points, {0.1f, 0.1f});
  obs = est.observe({red_far, near_center}, size, t0, t0, &any_color);
  expect(
    obs.valid && obs.candidates == 2 && obs.target_id == auto_aim::target_id(near_center),
    "est: 不筛颜色时，近处蓝板胜过远处红板");
  obs = est.observe({red_near, far}, size, t0, t0, &any_color);
  expect(
    obs.valid && obs.target_id == auto_aim::target_id(red_near),
    "est: 不筛颜色的最近目标选择与输入顺序无关");

  // 整形：max_step 生效且被记录（相对角差默认**不做**整形，这里显式打开）
  auto small_step = limits;
  small_step.max_step = 2.0 / 57.3;  // 2deg
  auto shaped = filter;
  shaped.shape_tx = true;
  const CurrentRelativeEstimator est_clamped(geo, small_step, shaped);
  obs = est_clamped.observe({blue}, size, t0, t0);
  expect(
    obs.valid && obs.clamped && std::abs(obs.tx_yaw) <= 2.0 / 57.3 + 1e-9,
    "est: relative_tx_shape=true 时 TX 被 max_step 截断并标记 clamped");

  // 默认（relative_tx_shape 未开）：TX 只做符号/偏置，gain/死区/限幅一律不参与，
  // 否则 CSV 上的 TX 就不再是「这一帧测到的角差」。
  auto no_shape = limits;
  no_shape.max_step = 1e-4;  // 若误走 shape()，TX 会被压到 0
  no_shape.deadband = 10.0;  // 若误走 shape()，TX 会变成 0
  const CurrentRelativeEstimator est_no_shape(geo, no_shape, filter);
  obs = est_no_shape.observe({blue}, size, t0, t0);
  expect(
    obs.valid && !obs.clamped && near(obs.tx_yaw, obs.raw_yaw, 1e-4) && std::abs(obs.tx_yaw) > 1e-3,
    "est: 默认相对角差 TX 绕过 shape()（gain/死区/限幅不污染测量值）");

  // 符号与静态偏置仍然生效（镜像安装 / 相机-枪管不同轴）
  auto signed_limits = limits;
  signed_limits.yaw_sign = -1.0;
  signed_limits.yaw_offset = 1.0 / 57.3;
  const CurrentRelativeEstimator est_signed(geo, signed_limits, filter);
  obs = est_signed.observe({blue}, size, t0, t0);
  expect(
    obs.valid && near(obs.tx_yaw, -(obs.raw_yaw + 1.0 / 57.3), 1e-4),
    "est: 默认路径仍应用 servo_yaw_sign 与 servo_yaw_offset");

  // 过滤覆盖：板端 CONTROL 下发的颜色优先于 config 里的 enemy_color
  auto_aim::RelativeFilter red_filter = filter;
  red_filter.enemy_color = auto_aim::Color::red;
  auto red_armor = make_armor(4 /*red:one:small*/, 0.9f, points, {0.5f, 0.5f});
  obs = est.observe({red_armor}, size, t0, t0, &red_filter);
  expect(obs.valid, "est: 颜色覆盖（observe 的 filter 参数）能选中红方装甲板");
  obs = est.observe({red_armor}, size, t0, t0);
  expect(
    !obs.valid && obs.reason == "color_filtered",
    "est: 不传覆盖时仍按构造时的 enemy_color 过滤（默认行为不变）");

  // 几何回退：最近的一块解不出来时，退到下一块仍然能给出有效观测。
  // （4 点 PnP 对「尺寸写错」不敏感——物点与距离等比缩放会投影到同一组角点，
  //   所以这里用「角点被污染」制造几何不可信，和 test_current_geometry 的用例一致。）
  auto bad_points = points;
  bad_points[0].x += 80.0f;
  auto bad = make_armor(3, 0.9f, bad_points, {0.5f, 0.5f});
  auto good = make_armor(3, 0.9f, points, {0.2f, 0.2f});
  good.name = auto_aim::ArmorName::two;
  auto strict_geo = geo;
  strict_geo.max_reprojection = 5.0;
  const CurrentRelativeEstimator est_fallback(strict_geo, limits, filter);
  obs = est_fallback.observe({bad, good}, size, t0, t0);
  expect(
    obs.valid && obs.candidates == 2 && obs.target_id == auto_aim::target_id(good) &&
      obs.target_id != auto_aim::target_id(bad),
    "est: 最近的候选几何不可信时回退到下一块有效目标");
}

// ---------------------------------------------------------------------------
// vision ↔ CBoard v1 协议：编解码 + 会话状态机（不需要 CAN / 相机）
// ---------------------------------------------------------------------------
io::VisionProtocolConfig make_v1_config()
{
  io::VisionProtocolConfig config;
  config.v1 = true;
  config.relative_canid = 0x1A1;
  config.handshake_request_canid = 0x1A2;
  config.handshake_response_canid = 0x1A3;
  config.control_canid = 0x1A4;
  config.control_timeout_ms = 500.0;
  config.observation_timeout_ms = 100.0;
  config.tx_min_interval_ms = 0.0;
  config.ids_confirmed = true;
  return config;
}

void test_vision_codec()
{
  uint8_t data[8] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};

  io::encode_handshake_request(0x11223344, data);
  expect(
    data[0] == 0x01 && data[1] == 1 && data[2] == 0x11 && data[3] == 0x22 && data[4] == 0x33 &&
      data[5] == 0x44 && data[6] == 0 && data[7] == 0,
    "v1: HELLO 字节布局（01 01 session(BE) 00 00）");

  uint8_t version = 0;
  uint32_t session = 0;
  expect(
    io::decode_handshake_request(data, 8, version, session) && version == 1 &&
      session == 0x11223344,
    "v1: HELLO 编解码往返");

  io::encode_handshake_response(0x11223344, io::HandshakeStatus::not_ready, data);
  expect(
    data[0] == 0x02 && data[1] == 1 && data[6] == 0x01 && data[7] == 0,
    "v1: HELLO_ACK 字节布局（status 在 data[6]）");

  io::encode_control(0x11223344, true, io::ControlColor::blue, 7, data);
  expect(
    data[0] == 0x03 && data[1] == 1 && data[6] == 0x05 && data[7] == 7,
    "v1: CONTROL 字节布局（enable=1 color=blue seq=7 -> data[6]=0x05）");

  uint32_t sid = 0;
  bool enable = false;
  uint8_t color = 0;
  uint8_t sequence = 0;
  expect(
    io::decode_control(data, 8, sid, enable, color, sequence) && sid == 0x11223344 && enable &&
      color == 2 && sequence == 7,
    "v1: CONTROL 编解码往返");

  io::encode_control(1, false, io::ControlColor::red, 0, data);
  expect(data[6] == 0x02, "v1: CONTROL enable=0 时 data[6] 只保留颜色位");

  // 畸形帧必须被结构性拒绝（否则状态机会被垃圾帧带偏）
  io::encode_control(0x10, true, io::ControlColor::red, 1, data);
  expect(
    !io::decode_control(data, 7, sid, enable, color, sequence), "v1: DLC!=8 的 CONTROL 被拒绝");

  io::encode_control(0x10, true, io::ControlColor::red, 1, data);
  data[0] = 0x02;
  expect(!io::decode_control(data, 8, sid, enable, color, sequence), "v1: 类型字节不对的帧被拒绝");

  io::encode_control(0x10, true, io::ControlColor::red, 1, data);
  data[1] = 2;
  expect(
    !io::decode_control(data, 8, sid, enable, color, sequence),
    "v1: CONTROL 版本字节不为 1 被拒绝");

  io::encode_control(0x10, true, io::ControlColor::red, 1, data);
  data[6] = 0x91;
  expect(
    !io::decode_control(data, 8, sid, enable, color, sequence), "v1: CONTROL 保留位非 0 被拒绝");

  expect(
    io::valid_control_color(1) && io::valid_control_color(2) && !io::valid_control_color(0) &&
      !io::valid_control_color(3),
    "v1: 控制颜色只允许 1=red / 2=blue");
}

void test_vision_session()
{
  using Clock = std::chrono::steady_clock;
  using io::VisionLinkCore;
  using io::VisionLinkState;

  const auto config = make_v1_config();
  VisionLinkCore core(config);

  auto now = Clock::now();
  uint8_t hello[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t frame[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint32_t can_id = 0;
  VisionLinkCore::FrameKind kind = VisionLinkCore::FrameKind::none;

  const io::Command valid{true, false, 0.1, -0.2, 0.0};
  const io::Command idle{false, false, 0.0, 0.0};

  // ---- READY 之前：HELLO 只得到 NOT_READY，不建立会话 ----
  io::encode_handshake_request(0xABCD0001, hello);
  core.on_frame(config.handshake_request_canid, hello, 8, now);
  auto snapshot = core.snapshot(now);
  expect(
    snapshot.state == VisionLinkState::wait_handshake && !snapshot.has_session,
    "v1: 未 READY 时 HELLO 不建立会话");
  expect(
    core.next_frame(now, can_id, frame, kind) &&
      kind == VisionLinkCore::FrameKind::handshake_response &&
      can_id == config.handshake_response_canid && frame[6] == 1,
    "v1: 未 READY 时回 NOT_READY");
  core.note_send(true, kind);

  // ---- READY 之后：建立会话，进入 wait_control ----
  core.set_ready(true, "单测");
  core.on_frame(config.handshake_request_canid, hello, 8, now);
  snapshot = core.snapshot(now);
  expect(
    snapshot.state == VisionLinkState::wait_control && snapshot.has_session &&
      snapshot.session_id == 0xABCD0001,
    "v1: READY 时 HELLO 建立会话 -> wait_control");
  expect(
    core.next_frame(now, can_id, frame, kind) && frame[6] == 0 && frame[2] == 0xAB &&
      frame[5] == 0x01,
    "v1: HELLO_ACK=READY 并回显 session");
  core.note_send(true, kind);

  // ---- 未 enable：不发角差 ----
  expect(
    !core.publish(valid, core.snapshot(now).generation, now), "v1: wait_control 下 publish 不生效");
  expect(!core.next_frame(now, can_id, frame, kind), "v1: wait_control 下没有角差帧");

  // ---- 旧 / 未知 session 的 CONTROL 被拒绝 ----
  io::encode_control(0xDEAD0002, true, io::ControlColor::blue, 1, frame);
  core.on_frame(config.control_canid, frame, 8, now);
  snapshot = core.snapshot(now);
  expect(
    snapshot.state == VisionLinkState::wait_control && !snapshot.enable &&
      core.stats().rejected_session == 1,
    "v1: 未知 session 的 CONTROL 被拒绝，不改状态");

  // ---- 正确 session + enable -> active ----
  io::encode_control(0xABCD0001, true, io::ControlColor::blue, 1, frame);
  core.on_frame(config.control_canid, frame, 8, now);
  snapshot = core.snapshot(now);
  expect(
    snapshot.state == VisionLinkState::active && snapshot.enable && snapshot.color == 2 &&
      snapshot.sequence == 1,
    "v1: CONTROL enable=1 color=blue -> active");

  // ---- active 下 publish -> 发送线程取到角差帧 ----
  expect(core.publish(valid, snapshot.generation, now), "v1: active 下 publish 生效");
  expect(
    core.next_frame(now, can_id, frame, kind) && kind == VisionLinkCore::FrameKind::relative &&
      can_id == config.relative_canid,
    "v1: 角差帧用 relative_canid");
  expect(
    (frame[0] & 0x01) != 0 && decode_i16(frame + 2) == 1000 && decode_i16(frame + 4) == -2000 &&
      (frame[0] & 0x02) == 0,
    "v1: 角差帧字节 = 量化后的 Δyaw/Δpitch，shoot 位恒为 0");
  core.note_send(true, kind);
  expect(!core.next_frame(now, can_id, frame, kind), "v1: 同一份观测只发一次（不重复灌角差）");

  // ---- 同 session 重复 HELLO 幂等 ----
  const uint32_t generation_before = core.snapshot(now).generation;
  core.on_frame(config.handshake_request_canid, hello, 8, now);
  snapshot = core.snapshot(now);
  expect(
    snapshot.state == VisionLinkState::active && snapshot.enable &&
      snapshot.generation == generation_before,
    "v1: 同 session 重复 HELLO 幂等（不清 enable / generation）");
  expect(
    core.next_frame(now, can_id, frame, kind) && frame[6] == 0,
    "v1: 幂等 HELLO 仍然重发 READY（应答可能丢包）");
  core.note_send(true, kind);

  // ---- 颜色切换：generation +1，变色前的观测被拒绝 ----
  const uint32_t generation_blue = snapshot.generation;
  io::encode_control(0xABCD0001, true, io::ControlColor::red, 2, frame);
  core.on_frame(config.control_canid, frame, 8, now);
  snapshot = core.snapshot(now);
  expect(
    snapshot.color == 1 && snapshot.generation == generation_blue + 1,
    "v1: CONTROL 换色 -> 颜色生效且 generation+1");
  expect(
    !core.publish(valid, generation_blue, now) && core.stats().tx_skipped_generation > 0,
    "v1: 换色前算出来的观测被拒绝（旧 generation）");

  // ---- 停止：立刻补 control=0 ----
  io::encode_control(0xABCD0001, false, io::ControlColor::red, 3, frame);
  core.on_frame(config.control_canid, frame, 8, now);
  expect(core.snapshot(now).state == VisionLinkState::stopped, "v1: CONTROL enable=0 -> stopped");
  expect(
    core.next_frame(now, can_id, frame, kind) && kind == VisionLinkCore::FrameKind::relative &&
      frame[0] == 0x00 && decode_i16(frame + 2) == 0 && decode_i16(frame + 4) == 0,
    "v1: 停止后立刻补发 control=0（不等下一帧图像）");
  core.note_send(true, kind);

  // ---- 启停复用 ----
  io::encode_control(0xABCD0001, true, io::ControlColor::red, 4, frame);
  core.on_frame(config.control_canid, frame, 8, now);
  expect(core.snapshot(now).state == VisionLinkState::active, "v1: stopped 后 enable=1 重新激活");

  // ---- 畸形 CONTROL 不刷新心跳 ----
  core.publish(valid, core.snapshot(now).generation, now);
  core.next_frame(now, can_id, frame, kind);
  core.note_send(true, kind);

  now += std::chrono::milliseconds(400);
  io::encode_control(0xABCD0001, true, io::ControlColor::red, 5, frame);
  core.on_frame(config.control_canid, frame, 6, now);  // DLC 6 = 畸形
  expect(core.stats().rejected_malformed >= 1, "v1: 畸形 CONTROL 计入 rejected_malformed");
  expect(!core.snapshot(now).has_session, "v1: 畸形 CONTROL 立即 fail-closed");
  now += std::chrono::milliseconds(200);  // 距最后一条合法 CONTROL 已 600ms > 500ms
  core.poll(now);
  snapshot = core.snapshot(now);
  expect(
    snapshot.state == VisionLinkState::control_timeout && !snapshot.has_session &&
      core.stats().timeouts == 0,
    "v1: 畸形帧已作废会话，不靠等待超时停车");

  // ---- 超时后旧 session 不再被接受，必须重新握手 ----
  io::encode_control(0xABCD0001, true, io::ControlColor::red, 6, frame);
  core.on_frame(config.control_canid, frame, 8, now);
  expect(
    core.snapshot(now).state == VisionLinkState::control_timeout,
    "v1: 超时后旧 session 的 CONTROL 被忽略");

  io::encode_handshake_request(0xABCD0009, hello);
  core.on_frame(config.handshake_request_canid, hello, 8, now);
  snapshot = core.snapshot(now);
  expect(
    snapshot.state == VisionLinkState::wait_control && snapshot.session_id == 0xABCD0009,
    "v1: 超时后重新握手可建立新会话");

  // ---- 版本不兼容 ----
  io::encode_handshake_request(0xABCD0009, hello);
  hello[1] = 2;
  core.on_frame(config.handshake_request_canid, hello, 8, now);
  // 安全停止帧优先于 ACK。
  if (core.next_frame(now, can_id, frame, kind) && kind == VisionLinkCore::FrameKind::relative)
    core.note_send(true, kind);
  expect(
    core.next_frame(now, can_id, frame, kind) && frame[6] == 2 && frame[5] == 0x09,
    "v1: 版本不兼容回 VERSION_MISMATCH（并回显 session）");
  core.note_send(true, kind);
  expect(
    core.snapshot(now).state == VisionLinkState::wait_control && core.stats().rejected_version == 1,
    "v1: 版本不兼容不改会话状态");

  // ---- 无关 ID 不影响状态 ----
  const auto before_irrelevant = core.snapshot(now);
  io::encode_control(0xABCD0009, true, io::ControlColor::blue, 9, frame);
  core.on_frame(0x777, frame, 8, now);
  const auto after_irrelevant = core.snapshot(now);
  expect(
    after_irrelevant.state == before_irrelevant.state &&
      after_irrelevant.generation == before_irrelevant.generation,
    "v1: 与本协议无关的 ID 不改状态");

  // ---- 观测时效：有效观测过期后补 control=0（不依赖下一帧）----
  io::encode_control(0xABCD0009, true, io::ControlColor::blue, 10, frame);
  core.on_frame(config.control_canid, frame, 8, now);
  expect(core.publish(valid, core.snapshot(now).generation, now), "v1: 新会话下 publish 生效");
  expect(core.next_frame(now, can_id, frame, kind), "v1: 新会话发出角差帧");
  core.note_send(true, kind);
  now += std::chrono::milliseconds(150);  // > observation_timeout_ms(100)
  core.poll(now);
  expect(
    core.next_frame(now, can_id, frame, kind) && frame[0] == 0x00,
    "v1: 有效观测超过 relative_frame_timeout_ms -> 补发 control=0");
  core.note_send(true, kind);

  // ---- 无效观测（control=0）优先于任何旧样本下发 ----
  core.publish(idle, core.snapshot(now).generation, now);
  expect(
    core.next_frame(now, can_id, frame, kind) && frame[0] == 0x00, "v1: 无效观测 -> control=0 帧");
  core.note_send(true, kind);
}

void test_vision_legacy()
{
  using Clock = std::chrono::steady_clock;
  using io::VisionLinkCore;

  io::VisionProtocolConfig config;  // 默认 legacy
  config.relative_canid = 0xFE;
  VisionLinkCore core(config);

  auto now = Clock::now();
  uint8_t frame[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t hello[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint32_t can_id = 0;
  VisionLinkCore::FrameKind kind = VisionLinkCore::FrameKind::none;
  const io::Command valid{true, false, 0.05, 0.0, 0.0};

  expect(
    core.snapshot(now).state == io::VisionLinkState::active, "legacy: 没有握手，状态固定 active");
  expect(core.publish(valid, 0, now), "legacy: publish 直接生效");
  expect(
    core.next_frame(now, can_id, frame, kind) && can_id == 0xFE &&
      kind == VisionLinkCore::FrameKind::relative && frame[0] == 0x01 &&
      decode_i16(frame + 2) == 500,
    "legacy: 直接往 relative_canid 发角差帧（与旧版逐字节一致）");
  core.note_send(true, kind);
  expect(!core.next_frame(now, can_id, frame, kind), "legacy: 同一份观测不重复发");

  // 上行帧一律忽略（legacy 没有会话/启停语义）
  io::encode_handshake_request(1, hello);
  core.on_frame(config.handshake_request_canid, hello, 8, now);
  expect(
    core.stats().rx_handshake == 0 && core.snapshot(now).state == io::VisionLinkState::active,
    "legacy: 不解析任何上行帧（行为与旧版一致）");
}

void test_vision_safety()
{
  using Clock = std::chrono::steady_clock;
  using namespace std::chrono_literals;
  io::VisionProtocolConfig config;
  config.v1 = true;
  config.relative_canid = 0x300;
  config.handshake_request_canid = 0x301;
  config.handshake_response_canid = 0x302;
  config.control_canid = 0x303;
  config.tx_min_interval_ms = 1000;
  io::VisionLinkCore core(config);
  core.set_ready(true, "test");
  auto now = Clock::now();
  uint8_t bytes[8]{};
  uint32_t id = 0;
  io::VisionLinkCore::FrameKind kind;
  const io::Command valid{true, true, 0.1, -0.2};
  auto hello = [&](uint32_t session) {
    io::encode_handshake_request(session, bytes);
    core.on_frame(config.handshake_request_canid, bytes, 8, now);
  };
  auto control = [&](uint32_t session, bool enable, io::ControlColor color, uint8_t seq) {
    io::encode_control(session, enable, color, seq, bytes);
    core.on_frame(config.control_canid, bytes, 8, now);
  };
  auto flush = [&] { core.transmit(now, [](uint32_t, const uint8_t *) { return true; }); };
  hello(1);
  flush();
  now += 501ms;
  core.poll(now);
  expect(
    !core.snapshot(now).has_session && core.stats().timeouts == 1,
    "safety: wait_control 也受控制超时保护");
  hello(2);
  flush();
  control(2, true, io::ControlColor::blue, 255);
  const auto blue_generation = core.snapshot(now).generation;
  expect(
    !core.publish(valid, blue_generation, now - 1ms),
    "safety: 即便 generation 相同也拒绝控制生效前采集的缓存图像");
  core.publish(valid, blue_generation, now);
  flush();
  control(2, true, io::ControlColor::blue, 0);
  expect(core.snapshot(now).sequence == 0, "safety: CONTROL sequence 255→0 正常回绕");
  control(2, false, io::ControlColor::blue, 255);
  expect(core.snapshot(now).enable, "safety: 倒序 CONTROL 不覆盖最新控制");
  control(2, true, io::ControlColor::red, 1);
  expect(!core.publish(valid, blue_generation, now), "safety: 检测期间换色拒绝旧 generation");
  expect(
    core.next_frame(now, id, bytes, kind) && bytes[0] == 0,
    "safety: 换色立即停止，不受 1Hz 有效帧限频影响");
  core.note_send(false, kind);
  expect(core.next_frame(now, id, bytes, kind) && bytes[0] == 0, "safety: 停止发送失败后继续重试");
  core.note_send(true, kind);
  core.publish(io::Command{false, false, 0.0, 0.0}, core.snapshot(now).generation, now);
  expect(
    core.next_frame(now, id, bytes, kind) && bytes[0] == 0, "safety: 无效观测不受有效帧限频约束");
  core.note_send(true, kind);
  now += 1001ms;
  hello(3);
  flush();
  control(3, true, io::ControlColor::blue, 1);
  core.publish(valid, core.snapshot(now).generation, now);
  flush();
  hello(4);
  expect(
    core.next_frame(now, id, bytes, kind) && id == config.relative_canid && bytes[0] == 0,
    "safety: 活动会话切换先发停止再 ACK");
  core.note_send(true, kind);
  flush();
  io::encode_control(4, false, io::ControlColor::blue, 1, bytes);
  bytes[6] = 0;  // 即使停车帧也必须有合法颜色
  core.on_frame(config.control_canid, bytes, 8, now);
  expect(!core.snapshot(now).has_session, "safety: 非法停车颜色同样 fail-closed");
  hello(5);
  flush();
  io::encode_control(5, true, io::ControlColor::blue, 1, bytes);
  bytes[6] |= 0x80;
  core.on_frame(config.control_canid, bytes, 8, now);
  expect(!core.snapshot(now).has_session, "safety: CONTROL 保留位非零立即作废会话");
  io::encode_handshake_request(6, bytes);
  bytes[7] = 1;
  core.on_frame(config.handshake_request_canid, bytes, 8, now);
  expect(!core.snapshot(now).has_session, "safety: HELLO 保留字节非零不建立会话");
  bool known = false;
  expect(
    !io::can_interface_is_virtual("lo", known) && known, "safety: lo 虽无 device 也不能冒充 vcan");
  expect(
    !io::can_interface_is_virtual("no_such_can", known) && !known,
    "safety: 未知接口不能豁免 ID 确认");
}

void test_vision_transaction()
{
  using namespace std::chrono_literals;
  auto config = make_v1_config();
  io::VisionLinkCore core(config);
  core.set_ready(true, "test");
  const auto now = std::chrono::steady_clock::now();
  uint8_t bytes[8]{};
  io::encode_handshake_request(1, bytes);
  core.on_frame(config.handshake_request_canid, bytes, 8, now);
  core.transmit(now, [](uint32_t, const uint8_t *) { return true; });
  io::encode_control(1, true, io::ControlColor::blue, 1, bytes);
  core.on_frame(config.control_canid, bytes, 8, now);
  core.publish(io::Command{true, false, 0.1, 0.0}, core.snapshot(now).generation, now);
  std::promise<void> writing, release;
  auto release_future = release.get_future();
  auto sender = std::async(std::launch::async, [&] {
    core.transmit(now, [&](uint32_t, const uint8_t *) {
      writing.set_value();
      release_future.wait();
      return true;
    });
  });
  writing.get_future().wait();
  io::encode_control(1, false, io::ControlColor::blue, 2, bytes);
  std::promise<void> stopping;
  auto stop = std::async(std::launch::async, [&] {
    stopping.set_value();
    core.on_frame(config.control_canid, bytes, 8, now);
  });
  stopping.get_future().wait();
  expect(
    stop.wait_for(20ms) == std::future_status::timeout,
    "safety: 控制变更不能插入资格检查与 SocketCAN 写入之间");
  release.set_value();
  sender.get();
  stop.get();
  bool zero = false;
  core.transmit(now, [&](uint32_t, const uint8_t * data) {
    zero = data[0] == 0;
    return true;
  });
  expect(zero && !core.snapshot(now).enable, "safety: 串行写入后停止帧先于后续有效观测");
}

void test_vision_configuration()
{
  char path[] = "/tmp/vision_config_XXXXXX";
  const int fd = mkstemp(path);
  expect(fd >= 0, "config: 创建隔离临时 YAML");
  if (fd < 0) return;
  close(fd);
  const std::string base =
    "can_command_mode: relative\ncan_interface: lo\nrelative_canid: 0\n"
    "vision_handshake_request_canid: 0x301\nvision_handshake_response_canid: 0x302\n"
    "vision_control_canid: 0x303\nrelative_output_frame: camera_optical\n";
  auto load = [&](const std::string & text, io::VisionProtocolConfig & result) {
    {
      std::ofstream file(path);
      file << text;
    }
    try {
      result = io::load_vision_protocol_config(path);
      return true;
    } catch (const std::exception &) {
      return false;
    }
  };
  io::VisionProtocolConfig result;
  expect(!load(base, result), "config: lo 不豁免实机 ID 确认");
  expect(
    load(base + "vision_can_ids_confirmed: true\n", result) && result.v1 &&
      result.relative_canid == 0,
    "config: 默认 v1，坐标系键独立且合法 CAN ID 0x000 保留");
  expect(
    !load(base + "vision_protocol: legacy\n", result), "config: 显式 legacy 也不能绕过 ID 确认");
  expect(
    !load(base + "vision_can_ids_confirmed: true\nrelative_frame_timeout_ms: .nan\n", result),
    "config: NaN 超时拒绝启动");
  expect(
    !load(base + "vision_can_ids_confirmed: true\nvision_protocol: unknown\n", result),
    "config: 未知协议拒绝启动");
  unlink(path);
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
  test_vision_codec();
  test_vision_session();
  test_vision_legacy();
  test_vision_safety();
  test_vision_transaction();
  test_vision_configuration();

  fmt::print("\n{} checks, {} failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
