// 检测管线频率可视化测试（新增独立可执行目标，不参与部署链路）
//
// 目的：在前端（imshow）里同时对「相机取流频率」与「识别链路频率」画曲线，并给出各阶段
//       耗时统计，用来判断瓶颈在相机还是模型；也是接新相机时的第一手验证工具。
//
// 数据源由 yaml 的 camera_name 决定，两种都能用（不用改本文件）：
//   camera_name: "video"      -> 回放 assets/demo/demo.avi（无需相机 / 模型也能测取流与显示）
//   camera_name: "mindvision" -> 真实工业相机（MV-SUA133GC-T1V-C，见 configs/mv_sua133gc.yaml）
//
// 用法（本仓库用 cv::CommandLineParser，短选项必须写成 -c=<path>，"键 值" 会被解析成空）：
//   ./build/detect_freq_visual_test -c=configs/detect_freq.yaml -m=replay -n=300 -d
//   ./build/detect_freq_visual_test -c=configs/detect_freq.yaml -m=bench  -n=300
//   ./build/detect_freq_visual_test -c=configs/mv_sua133gc.yaml -m=live -d
//   ./build/detect_freq_visual_test -c=configs/detect_freq.yaml -m=bench -n=60 -save=/tmp/shot.png
//     （无显示/SSH 下用 -save 把画面+频率曲线存图检查；单纯测频率时不要开）
//
// 阶段划分：
//   cap    camera.read()，含等待下一帧（因此 cap_ms 里有相机的节奏）
//   pre    共享 letterbox 预处理（单独测一次 yolov5_post::letterbox，与后端内部同一次调用）
//   detect yolo.detect() 全过程（letterbox + 推理 + NMS/解析）
//   draw   画框/指标/曲线 + imshow
// 三个频率：
//   取流频率 cam_fps  = 1 / (相邻两帧 timestamp 之差)，掉帧时等于处理频率
//   模型频率 model_fps= 1000 / detect_ms（模型本身能跑多快）
//   链路频率 pipe_fps = 1000 / (pre + detect + draw)（不含等待取帧，处理链路的最快速率）
//
// 注意：画面上的文字用 tools::draw_text（cv::putText / Hershey 字体），只能画 ASCII，
//       如果要给 HUD 加中文会显示成 "????"；中文请只放在 logger 输出里。
//
// 没有 TensorRT / OpenCV < 4.9（本机 OpenCV 4.5.4 跑不了真模型）时怎么验证本工具：
//       可以用一个"输出恒为 [1,N,22] 常量"的合成 ONNX 当桩模型（只用 Slice/Sub/Reshape/Add，
//       OpenCV 4.5.x 也能 forward），此时画框 / CSV / 曲线 / 汇总全部会走到，只是坐标是编造的。
//       真实精度仍按 AGENTS.md §4.1.4 与 Python 参考实现（cv2 5.x）对比，不能只看"能跑"。

#include <fmt/format.h>

#include <algorithm>
#include <chrono>
#include <deque>
#include <fstream>
#include <list>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "io/camera.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/yolos/yolov5_postprocess.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/yaml.hpp"

const std::string keys =
  "{help h usage ? |                             | 输出命令行参数说明}"
  "{config-path c  | configs/detect_freq.yaml    | yaml配置文件路径（决定相机/回放与推理后端）}"
  "{mode m         | live                        | live(实时)/replay(回放n帧)/bench(跑n帧并汇总)}"
  "{n frames       | 0                           | 处理多少帧后退出，0 = 不限；replay/bench 默认 "
  "300}"
  "{d display      |                             | 强制显示窗口（bench 默认不显示）}"
  "{w window       | 100                         | 滑动窗口长度（帧），统计 avg/p50/p95/max}"
  "{interval       | 30                          | 每多少帧打印一次实时日志}"
  "{dump           |                             | 把每帧检测结果写成 CSV（一致性对比用）}"
  "{save           |                             | 把可视化结果（画面+频率曲线）存成图片，每 "
  "interval 帧覆盖写一次（无显示/SSH 下检查用）}"
  "{no-yolo        |                             | 不加载检测模型，只测取流/显示频率}";

// 滑动窗口统计（只保留最近 window 帧，长时间运行不会一直涨内存）
class WindowStats
{
public:
  explicit WindowStats(std::size_t window) : window_(window == 0 ? 1 : window) {}

  void push(double v)
  {
    values_.push_back(v);
    sum_ += v;
    if (values_.size() > window_) {
      sum_ -= values_.front();
      values_.pop_front();
    }
  }

  bool empty() const { return values_.empty(); }

  double last() const { return values_.empty() ? 0.0 : values_.back(); }

  double avg() const { return values_.empty() ? 0.0 : sum_ / static_cast<double>(values_.size()); }

  double max() const
  {
    return values_.empty() ? 0.0 : *std::max_element(values_.begin(), values_.end());
  }

  // 分位数只在打印/汇总时调用（内部拷贝 + 排序）
  double percentile(double p) const
  {
    if (values_.empty()) return 0.0;
    auto sorted = std::vector<double>(values_.begin(), values_.end());
    std::sort(sorted.begin(), sorted.end());
    auto idx = static_cast<std::size_t>(p * static_cast<double>(sorted.size() - 1) + 0.5);
    return sorted[std::min(idx, sorted.size() - 1)];
  }

private:
  std::size_t window_;
  std::deque<double> values_;
  double sum_ = 0.0;
};

namespace
{
// 在曲线面板里画一条折线（超过 y_max 的值截断到顶）
void draw_curve(
  cv::Mat & panel, const std::deque<double> & values, double y_max, const cv::Scalar & color)
{
  if (values.size() < 2) return;

  const int x0 = 46, x1 = panel.cols - 12, top = 12, bottom = panel.rows - 24;

  std::vector<cv::Point> pts;
  pts.reserve(values.size());
  auto n = values.size();
  for (std::size_t i = 0; i < n; i++) {
    auto x = x0 + static_cast<int>((x1 - x0) * static_cast<double>(i) / static_cast<double>(n - 1));
    auto v = std::min(values[i], y_max);
    auto y = bottom - static_cast<int>(static_cast<double>(bottom - top) * v / y_max);
    pts.emplace_back(x, y);
  }

  cv::polylines(panel, pts, false, color, 1, cv::LINE_AA);
}

// 自动定纵轴上界（10 fps 取整，最小 30）
double pick_y_max(
  const std::deque<double> & cam_fps, const std::deque<double> & model_fps,
  const std::deque<double> & pipe_fps)
{
  double m = 0.0;
  for (const auto * v : {&cam_fps, &model_fps, &pipe_fps}) {
    for (auto x : *v) m = std::max(m, x);
  }
  return std::max(30.0, std::ceil(m / 10.0) * 10.0);
}

// 频率曲线面板：绿=取流、黄=模型、青=链路
cv::Mat make_freq_panel(
  int width, const std::deque<double> & cam_fps, const std::deque<double> & model_fps,
  const std::deque<double> & pipe_fps, double y_max, const std::string & note)
{
  cv::Mat panel(170, std::max(width, 480), CV_8UC3, cv::Scalar(24, 24, 24));

  const int x0 = 46, x1 = panel.cols - 12, top = 12, bottom = panel.rows - 24;

  for (int i = 0; i <= 2; i++) {
    auto y = top + (bottom - top) * i / 2;
    cv::line(panel, {x0, y}, {x1, y}, {60, 60, 60}, 1);
    tools::draw_text(
      panel, fmt::format("{:.0f}", y_max * (2 - i) / 2), {6, y + 5}, {170, 170, 170}, 0.45, 1);
  }
  cv::line(panel, {x0, top}, {x0, bottom}, {110, 110, 110}, 1);
  cv::line(panel, {x0, bottom}, {x1, bottom}, {110, 110, 110}, 1);

  draw_curve(panel, cam_fps, y_max, {0, 200, 0});
  draw_curve(panel, model_fps, y_max, {0, 220, 255});
  draw_curve(panel, pipe_fps, y_max, {255, 180, 0});

  tools::draw_text(panel, "fps", {6, top}, {170, 170, 170}, 0.45, 1);
  // 注意：tools::draw_text 用 cv::putText（Hershey 字体），只能画 ASCII，
  // 这里（以及画面 HUD）不要写中文，否则会显示成 "????"
  tools::draw_text(
    panel,
    "green = cap(1/dt)   yellow = model(1/detect)   cyan = pipe(1/(pre+detect+draw))   " + note,
    {x0 + 6, panel.rows - 9}, {200, 200, 200}, 0.45, 1);

  return panel;
}
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto config_path = cli.get<std::string>("config-path");
  auto mode = cli.get<std::string>("mode");
  auto n_frames = cli.get<int>("n");
  auto window = static_cast<std::size_t>(cli.get<int>("window"));
  auto interval = std::max(1, cli.get<int>("interval"));
  auto dump_path = cli.get<std::string>("dump");
  auto save_path = cli.get<std::string>("save");
  auto no_yolo = cli.has("no-yolo");

  if (mode != "live" && mode != "replay" && mode != "bench") {
    tools::logger()->error("unknown mode: {} （可选 live / replay / bench）", mode);
    return 1;
  }
  if (mode != "live" && n_frames == 0) n_frames = 300;
  auto display = mode == "bench" ? cli.has("display") : true;

  // 录制开关（yaml 新增可选键，默认 false，缺键时行为与之前完全一致，见 configs/detect_freq.yaml）：
  //   record_video: true -> 相机帧录成 records/<时间>.avi（tools::Recorder，MJPEG）
  // 作用是「有相机就能录一段」，之后派生 camera_name: "video" + video_path: records/xxx.avi
  // 的配置离线回放，用 scripts/replay_ab.sh 做同一段画面上的 A/B。本程序没有姿态来源，
  // txt 里的人机四元数是单位四元数（无意义），产物只取 avi。
  auto cfg = tools::load(config_path);
  auto record_video = tools::optional_bool(cfg, "record_video", false);
  auto record_fps = tools::optional_double(cfg, "record_fps", 30.0);
  std::unique_ptr<tools::Recorder> recorder;
  if (record_video) {
    recorder = std::make_unique<tools::Recorder>(record_fps);
    tools::logger()->warn(
      "record_video=true：录制到 records/（fps 上限 {:.1f}；txt 里的人机姿态是单位四元数，"
      "仅 avi 有意义）",
      record_fps);
  }

  tools::Exiter exiter;
  tools::Plotter plotter;

  io::Camera camera(config_path);

  // 模型加载失败（例如 yolov5_backend=trt 但本机没装 TensorRT）时不退出，
  // 降级为「只测取流/显示频率」，并打印原因（不静默退化）
  std::unique_ptr<auto_aim::YOLO> yolo;
  if (no_yolo) {
    tools::logger()->warn("--no-yolo：不加载检测模型，只测取流/显示频率");
  } else {
    try {
      yolo = std::make_unique<auto_aim::YOLO>(config_path, false);
    } catch (const std::exception & e) {
      tools::logger()->warn("检测模型加载失败，降级为只测取流/显示频率: {}", e.what());
    }
  }

  tools::logger()->info(
    "mode={} frames={} display={} window={} interval={} yolo={}", mode, n_frames, display, window,
    interval, yolo ? "on" : "off");

  WindowStats cap_stat(window), pre_stat(window), detect_stat(window), draw_stat(window),
    proc_stat(window), cam_fps_stat(window);
  std::deque<double> cam_fps_hist, model_fps_hist, pipe_fps_hist;
  constexpr std::size_t kHistLen = 300;

  std::ofstream dump;
  if (!dump_path.empty()) {
    dump.open(dump_path);
    if (!dump) {
      tools::logger()->warn("无法写入 dump 文件: {}", dump_path);
    } else {
      dump << "frame,armor_idx,confidence,color,name,type,cx,cy,x0,y0,x1,y1,x2,y2,x3,y3\n";
      tools::logger()->info("每帧检测结果写到 {}", dump_path);
    }
  }

  int frame = 0, armor_total = 0, frames_with_armor = 0;
  bool have_last_timestamp = false;
  std::chrono::steady_clock::time_point timestamp;
  auto last_timestamp = std::chrono::steady_clock::now();
  auto last_loop_t = last_timestamp;
  auto bench_start = last_timestamp;

  // 显示用的「上一帧」指标：画面/曲线滞后 1 帧，这样 imshow 也能算进 draw 阶段
  double last_cam_fps = 0, last_model_fps = 0, last_pipe_fps = 0, last_loop_fps = 0;
  double last_draw_ms = 0, last_proc_ms = 0;

  cv::Mat img;
  while (!exiter.exit()) {
    if (n_frames > 0 && frame >= n_frames) break;

    // ---------------- cap：取流 ----------------
    auto t0 = std::chrono::steady_clock::now();
    camera.read(img, timestamp);
    auto t1 = std::chrono::steady_clock::now();
    auto cap_ms = tools::delta_time(t1, t0) * 1000;

    if (recorder) recorder->record(img, Eigen::Quaterniond::Identity(), timestamp);

    // 取流频率用相机给出的 timestamp（掉帧时会退化成处理频率）
    auto cam_dt = tools::delta_time(timestamp, last_timestamp);
    auto cam_fps = (have_last_timestamp && cam_dt > 0) ? 1.0 / cam_dt : 0.0;
    last_timestamp = timestamp;

    // ---------------- pre：共享 letterbox 预处理 ----------------
    double scale = 1.0;
    auto pre_ms = 0.0;
    if (yolo) {
      auto tp0 = std::chrono::steady_clock::now();
      auto letterboxed = auto_aim::yolov5_post::letterbox(img, scale);
      pre_ms = tools::delta_time(std::chrono::steady_clock::now(), tp0) * 1000;
      (void)letterboxed;
    }

    // ---------------- detect：模型全链路 ----------------
    auto t2 = t1;
    std::list<auto_aim::Armor> armors;
    if (yolo) {
      try {
        armors = yolo->detect(img, frame);
      } catch (const std::exception & e) {
        // 例如 OpenCV DNN 版本太旧（< 4.9）在 forward() 断言失败：明确报错并降级，
        // 后续帧不再推理，只测取流/显示（不静默、不退出，方便先量相机侧频率）
        tools::logger()->error("detect 失败，后续帧不再推理（只测取流/显示）: {}", e.what());
        yolo.reset();
      }
      t2 = std::chrono::steady_clock::now();
    }
    auto detect_ms = tools::delta_time(t2, t1) * 1000;

    // ---------------- draw：画框 + 指标 + 曲线 + imshow ----------------
    auto t3 = std::chrono::steady_clock::now();
    bool user_quit = false;
    if (display || !save_path.empty()) {
      auto canvas = img.clone();
      for (const auto & armor : armors) {
        tools::draw_points(canvas, armor.points, {0, 255, 0});
        tools::draw_text(
          canvas,
          fmt::format(
            "{:.2f} {} {}", armor.confidence, auto_aim::COLORS[armor.color],
            auto_aim::ARMOR_NAMES[armor.name]),
          armor.center, {0, 255, 0});
      }

      tools::draw_text(
        canvas, fmt::format("[{}] frame {}  x{}", mode, frame, canvas.cols), {10, 30},
        {255, 255, 255});
      tools::draw_text(
        canvas,
        fmt::format(
          "cap {:5.1f} fps | model {:5.1f} fps | pipe {:5.1f} fps | loop {:5.1f} fps", last_cam_fps,
          last_model_fps, last_pipe_fps, last_loop_fps),
        {10, 60}, {0, 255, 255});
      tools::draw_text(
        canvas,
        fmt::format(
          "ms  cap {:.1f} | pre {:.1f} | detect {:.1f} | draw {:.1f} | proc {:.1f}", cap_ms, pre_ms,
          detect_ms, last_draw_ms, last_proc_ms),
        {10, 90}, {0, 255, 255});
      tools::draw_text(
        canvas,
        fmt::format(
          "armors {}  frames_with_armor {:.1f}%", armors.size(),
          frame > 0 ? 100.0 * frames_with_armor / frame : 0.0),
        {10, 120}, {0, 255, 255});
      if (!yolo) {
        tools::draw_text(canvas, "NO YOLO (capture/display only)", {10, 150}, {0, 0, 255});
      }

      auto y_max = pick_y_max(cam_fps_hist, model_fps_hist, pipe_fps_hist);
      cv::Mat shown;
      cv::vconcat(
        canvas,
        make_freq_panel(
          canvas.cols, cam_fps_hist, model_fps_hist, pipe_fps_hist, y_max,
          fmt::format("window {}", window)),
        shown);

      // 无显示环境（SSH / CI）下也能检查画面：每 interval 帧覆盖写一次，
      // 注意写入耗时也计入 draw 阶段，单纯测频率时不要开 -save
      if (
        !save_path.empty() && (frame % interval == 0 || (n_frames > 0 && frame + 1 >= n_frames))) {
        cv::imwrite(save_path, shown);
      }

      if (display) {
        cv::imshow("detect_freq", shown);
        auto key = cv::waitKey(1);
        if (key == 'q' || key == 27) user_quit = true;
      }
    }
    auto t4 = std::chrono::steady_clock::now();
    auto draw_ms = tools::delta_time(t4, t3) * 1000;

    // ---------------- 统计 ----------------
    auto proc_ms = pre_ms + detect_ms + draw_ms;
    auto model_fps = (yolo && detect_ms > 0.0) ? 1000.0 / detect_ms : 0.0;
    // 没有模型时 proc 只剩显示耗时，换算成频率没有意义，直接给 0
    auto pipe_fps = (yolo && proc_ms > 0.0) ? 1000.0 / proc_ms : 0.0;
    auto loop_dt = tools::delta_time(t1, last_loop_t);
    last_loop_t = t1;
    auto loop_fps = (frame > 0 && loop_dt > 0) ? 1.0 / loop_dt : 0.0;

    cap_stat.push(cap_ms);
    pre_stat.push(pre_ms);
    detect_stat.push(detect_ms);
    draw_stat.push(draw_ms);
    proc_stat.push(proc_ms);
    if (cam_fps > 0) cam_fps_stat.push(cam_fps);

    cam_fps_hist.push_back(cam_fps);
    model_fps_hist.push_back(yolo ? model_fps : 0.0);
    pipe_fps_hist.push_back(pipe_fps);
    for (auto * h : {&cam_fps_hist, &model_fps_hist, &pipe_fps_hist}) {
      if (h->size() > kHistLen) h->pop_front();
    }

    if (dump.is_open()) {
      int idx = 0;
      for (const auto & armor : armors) {
        dump << frame << ',' << idx++ << ',' << fmt::format("{:.6f}", armor.confidence) << ','
             << auto_aim::COLORS[armor.color] << ',' << auto_aim::ARMOR_NAMES[armor.name] << ','
             << auto_aim::ARMOR_TYPES[armor.type] << ','
             << fmt::format("{:.2f},{:.2f}", armor.center.x, armor.center.y);
        for (const auto & p : armor.points) dump << ',' << fmt::format("{:.2f},{:.2f}", p.x, p.y);
        dump << '\n';
      }
    }

    if (frame > 0 && frame % interval == 0) {
      tools::logger()->info(
        "frame {} | cap {:.1f} fps | model {:.1f} fps | pipe {:.1f} fps | loop {:.1f} fps | "
        "cap {:.1f} ms | detect {:.1f} ms | armors {} | 有目标帧 {:.1f}%",
        frame, cam_fps, model_fps, pipe_fps, loop_fps, cap_ms, detect_ms, armors.size(),
        100.0 * frames_with_armor / frame);

      plotter.plot({
        {"frame", frame},
        {"cap_fps", cam_fps},
        {"model_fps", model_fps},
        {"pipe_fps", pipe_fps},
        {"loop_fps", loop_fps},
        {"cap_ms", cap_ms},
        {"pre_ms", pre_ms},
        {"detect_ms", detect_ms},
        {"draw_ms", draw_ms},
        {"armors", static_cast<int>(armors.size())},
      });
    }

    armor_total += static_cast<int>(armors.size());
    if (!armors.empty()) frames_with_armor++;
    frame++;

    last_cam_fps = cam_fps;
    last_model_fps = model_fps;
    last_pipe_fps = pipe_fps;
    last_loop_fps = loop_fps;
    last_draw_ms = draw_ms;
    last_proc_ms = proc_ms;
    have_last_timestamp = true;

    if (user_quit) break;
  }

  dump.close();

  auto bench_elapsed = tools::delta_time(std::chrono::steady_clock::now(), bench_start);
  auto print_stat = [](const std::string & name, const WindowStats & s) {
    if (s.empty()) return;
    tools::logger()->info(
      "{:>28}: avg {:7.2f}  p50 {:7.2f}  p95 {:7.2f}  max {:7.2f}", name, s.avg(),
      s.percentile(0.5), s.percentile(0.95), s.max());
  };

  tools::logger()->info("========== detect_freq_visual_test 汇总 ==========");
  tools::logger()->info(
    "mode={} 处理 {} 帧，耗时 {:.2f} s（端到端 {:.2f} fps，含等帧/显示）", mode, frame,
    bench_elapsed, bench_elapsed > 0 ? frame / bench_elapsed : 0.0);

  tools::logger()->info("---------- 各阶段耗时（ms，窗口 {} 帧）----------", window);
  print_stat("cap（取流，含等帧）", cap_stat);
  print_stat("pre（letterbox 预处理）", pre_stat);
  print_stat("detect（模型全链路）", detect_stat);
  print_stat("draw（画框+曲线+imshow）", draw_stat);
  print_stat("proc（pre+detect+draw）", proc_stat);

  tools::logger()->info("---------- 频率 ----------");
  print_stat("取流频率（相机 timestamp）", cam_fps_stat);
  tools::logger()->info(
    "模型频率（1000/detect_avg）   : {:.1f} fps",
    detect_stat.avg() > 0.01 ? 1000.0 / detect_stat.avg() : 0.0);
  tools::logger()->info(
    "链路频率（1000/proc_avg）     : {:.1f} fps",
    proc_stat.avg() > 0.01 ? 1000.0 / proc_stat.avg() : 0.0);
  if (!yolo) {
    tools::logger()->info(
      "（本次没有可用推理后端，只统计了取流/显示：pre/detect/链路频率 均为 0，"
      "启动日志里有失败原因）");
  }

  tools::logger()->info("---------- 检测结果 ----------");
  tools::logger()->info(
    "有目标帧 {:.1f}%（{} / {}），平均每帧 {:.2f} 个装甲板",
    frame > 0 ? 100.0 * frames_with_armor / frame : 0.0, frames_with_armor, frame,
    frame > 0 ? static_cast<double>(armor_total) / frame : 0.0);
  if (!dump_path.empty()) {
    tools::logger()->info("detect 结果（每帧每个装甲板一行）已写到 {}", dump_path);
  }

  return 0;
}
