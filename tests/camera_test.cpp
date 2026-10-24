#include "io/camera.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

// ---------------------------------------------------------------------------
// 取流 / 帧间隔统计（USB 线材、Hub 端口、相机预设的 A/B 对比用）
//
// 为什么改：原来**每帧**都 `logger()->info("{:.2f} fps")`，而 tools::logger() 是
// flush_on(info) —— 120fps 时每帧一次落盘，本身就会拖慢采集循环，测出来的帧率既偏低
// 又看不出抖动。现在默认每 1s 打一行进度，退出时再打印帧间隔的 p50/p95/max：
// 「平均帧率接近标称」不等于「链路稳定」，长间隔（>50ms）次数才是丢帧/断连的证据。
// 逐帧打印仍可用 `-v` 打开（短时排查单帧异常时有用）。
//
// 预热：相机刚开流时 SDK/USB 侧还有缓冲填充、ISP 收敛，前 `-w` 秒不计入统计。
// 时长：`-t=<秒>` 到点自己退出（正常析构相机会打印 SDK 的 共/有效/丢帧 统计），
//       不要用 `timeout` 默认的 SIGTERM 强杀（会跳过 CameraUnInit，见 AGENTS §8.7）。
// ---------------------------------------------------------------------------
const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{config-path c  | configs/camera.yaml | yaml配置文件路径 }"
  "{d display      |                     | 显示视频流       }"
  "{t seconds      | 0                   | 采集时长（秒），0 = 一直跑到 Ctrl+C }"
  "{w warmup       | 5                   | 预热时长（秒），这段时间不计入统计 }"
  "{v verbose      |                     | 逐帧打印 fps（默认关闭，见文件头注释）}";

namespace
{
// 分位数（0~1）；空返回 0
double percentile(std::vector<double> values, double p)
{
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const std::size_t index = static_cast<std::size_t>(p * (values.size() - 1));
  return values[index];
}

// 标准差（总体，除以 n）
double std_dev(const std::vector<double> & values, double mean)
{
  if (values.size() < 2) return 0.0;
  double sum = 0;
  for (double v : values) sum += (v - mean) * (v - mean);
  return std::sqrt(sum / values.size());
}
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  auto config_path = cli.get<std::string>("config-path");
  auto display = cli.has("display");
  auto seconds = cli.get<double>("seconds");
  auto warmup = cli.get<double>("warmup");
  auto verbose = cli.has("verbose");

  io::Camera camera(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  std::vector<double> intervals_ms;  // 预热之后的帧间隔
  auto t_start = std::chrono::steady_clock::now();
  auto last_stamp = t_start;
  auto last_progress = t_start;

  int frames = 0;         // 收到的全部帧
  int accepted = 0;       // 计入统计的帧（预热之后）
  int long_gaps = 0;      // 帧间隔 > 50ms 的次数（丢帧 / 断连 / 重连的迹象）
  double max_gap_ms = 0;  // 最大帧间隔
  bool first = true;

  tools::logger()->info(
    "[camera_test] 开始取流：warmup={:.1f}s seconds={:.1f}s display={} verbose={}", warmup, seconds,
    display, verbose);
  if (display) tools::logger()->info("[camera_test] 显示窗口按 q 退出");

  while (!exiter.exit()) {
    camera.read(img, timestamp);
    ++frames;
    auto now = std::chrono::steady_clock::now();

    if (first) {
      // 第一帧没有前一帧可比，只记录起点
      first = false;
      last_stamp = timestamp;
    } else {
      const double dt_ms = tools::delta_time(timestamp, last_stamp) * 1000.0;
      last_stamp = timestamp;

      if (verbose) tools::logger()->info("{:.2f} fps ({:.2f} ms)", 1000.0 / dt_ms, dt_ms);

      if (tools::delta_time(now, t_start) >= warmup) {
        intervals_ms.push_back(dt_ms);
        ++accepted;
        if (dt_ms > 50.0) {
          ++long_gaps;
          if (dt_ms > max_gap_ms) max_gap_ms = dt_ms;
          tools::logger()->warn("[camera_test] 长间隔 {:.1f} ms（第 {} 帧）", dt_ms, frames);
        }
      }
    }

    if (tools::delta_time(now, last_progress) >= 1.0) {
      last_progress = now;
      const double elapsed = tools::delta_time(now, t_start);
      tools::logger()->info(
        "[camera_test] t={:.1f}s frames={} 平均={:.1f} fps 长间隔={}", elapsed, frames,
        frames / elapsed, long_gaps);
    }

    if (seconds > 0 && tools::delta_time(now, t_start) >= seconds) break;

    if (!display) continue;
    cv::imshow("img", img);
    if (cv::waitKey(1) == 'q') break;
  }

  const double elapsed = tools::delta_time(std::chrono::steady_clock::now(), t_start);
  tools::logger()->info(
    "[camera_test] 采集结束：共 {} 帧，用时 {:.2f} s，平均 {:.2f} fps（含预热）", frames, elapsed,
    frames / elapsed);

  if (intervals_ms.empty()) {
    tools::logger()->warn(
      "[camera_test] 预热 {}s 内没有采到足够帧（总时长 {:.2f}s），未生成统计；"
      "缩短 -w 或加长 -t 再测",
      warmup, elapsed);
    return 0;
  }

  const double p50 = percentile(intervals_ms, 0.50);
  const double p95 = percentile(intervals_ms, 0.95);
  const double pmax = percentile(intervals_ms, 1.0);
  double sum = 0;
  for (double dt : intervals_ms) sum += dt;
  const double mean = sum / intervals_ms.size();

  tools::logger()->info(
    "[camera_test] 统计（预热 {:.1f}s 之后）：帧数 {}，时长 {:.2f} s，平均 {:.2f} fps", warmup,
    accepted, sum / 1000.0, 1000.0 / mean);
  tools::logger()->info(
    "[camera_test] 帧间隔：p50={:.2f} ms（={:.2f} fps） p95={:.2f} ms p100={:.2f} ms "
    "抖动(std)={:.2f} ms",
    p50, 1000.0 / p50, p95, pmax, std_dev(intervals_ms, mean));
  tools::logger()->info(
    "[camera_test] 长间隔(>50 ms)：{} 次，最长 {:.1f} ms；丢帧以相机关闭时的 SDK "
    "统计为准（见上一行日志）",
    long_gaps, max_gap_ms);

  return 0;
}