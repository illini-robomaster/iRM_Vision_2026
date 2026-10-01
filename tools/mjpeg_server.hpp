#ifndef TOOLS__MJPEG_SERVER_HPP
#define TOOLS__MJPEG_SERVER_HPP

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>

namespace tools
{
// 极简 MJPEG over HTTP 直播服务（零第三方依赖：POSIX socket + cv::imencode）。
//
// 为什么需要它：Jetson 装机后通常没有显示器（DISPLAY 为空，cv::imshow 直接抛异常），
// 但现场调试又必须能看到画面。这里把帧推成标准 MJPEG，浏览器/VLC/ffplay 都能直接播，
// 不需要 GStreamer / RTSP / X11。
//
// 路由：
//   /              → multipart/x-mixed-replace MJPEG 实时流（浏览器打开即看）
//   /snapshot.jpg  → 当前一帧 JPEG（抓图、脚本自检用）
//   其它路径       → 404（含浏览器自动请求的 /favicon.ico；不会"默默"返回视频流）
//   连接数上限     → 503
//
// 设计要点（核心目标：拉流端绝不能拖慢视觉主链路）：
//   - publish() 只在有客户端时才做 JPEG 编码；没人看时开销接近 0（只判一个原子量）
//   - 每个客户端一个线程，共享"最新一帧"：慢客户端只会丢帧，不会阻塞 publish() 的调用方
//   - 发送用 SO_SNDTIMEO + MSG_NOSIGNAL：客户端异常断开/网络卡住只会摘掉该客户端，
//     不会 SIGPIPE 杀掉整个进程，也不会把主线程卡死
//   - 客户端数量上限 kMaxClients，超了直接回 503
class MjpegServer
{
public:
  // port <= 0 表示不启动（不会建线程/套接字，调用方可以无条件构造）
  explicit MjpegServer(int port, int jpeg_quality = 80);
  ~MjpegServer();

  MjpegServer(const MjpegServer &) = delete;
  MjpegServer & operator=(const MjpegServer &) = delete;

  // 发布一帧（BGR / 灰度均可）。无客户端时不做编码
  void publish(const cv::Mat & bgr);

  int port() const { return port_; }
  int clients() const { return clients_.load(); }

private:
  struct Client
  {
    std::thread thread;
    std::shared_ptr<std::atomic<bool>> done;
  };

  void accept_loop();
  void serve(int fd, const std::shared_ptr<std::atomic<bool>> & done);
  void reap_finished_clients();
  void log_urls() const;

  int port_;
  int quality_;
  int listen_fd_ = -1;
  std::atomic<bool> quit_{false};
  std::atomic<int> clients_{0};
  std::thread accept_thread_;
  std::mutex clients_mutex_;
  std::vector<Client> client_threads_;

  // 最新一帧 JPEG 与序号：publish() 写，客户端线程读
  std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  std::vector<uchar> latest_jpeg_;
  unsigned long long sequence_ = 0;
};

}  // namespace tools

#endif  // TOOLS__MJPEG_SERVER_HPP
