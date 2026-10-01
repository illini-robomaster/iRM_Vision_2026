#include "tools/mjpeg_server.hpp"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "tools/logger.hpp"

namespace
{
constexpr int kMaxClients = 8;
constexpr int kAcceptTimeoutMs = 200;  // accept 轮询超时，用于快速响应析构
constexpr int kSendTimeoutMs = 2000;   // 写超时：超时即认为客户端太慢/已断开
constexpr int kRecvTimeoutMs = 500;    // 读 HTTP 请求头的超时
constexpr const char * kStreamHeader =
  "HTTP/1.1 200 OK\r\n"
  "Connection: close\r\n"
  "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
  "Cache-Control: no-cache, no-store, must-revalidate\r\n"
  "Pragma: no-cache\r\n"
  "\r\n";
constexpr const char * kSnapshotHeader =
  "HTTP/1.1 200 OK\r\n"
  "Connection: close\r\n"
  "Content-Type: image/jpeg\r\n"
  "Cache-Control: no-cache, no-store, must-revalidate\r\n";

// MSG_NOSIGNAL：客户端断开时 send 返回 EPIPE，而不是给进程发 SIGPIPE（否则会杀掉整个视觉程序）
bool send_all(int fd, const char * data, std::size_t len)
{
  std::size_t sent = 0;
  while (sent < len) {
    auto n = ::send(fd, data + sent, len - sent, MSG_NOSIGNAL);
    if (n < 0 && errno == EINTR) continue;
    if (n <= 0) return false;  // EAGAIN/EWOULDBLOCK（写超时）或 EPIPE/ECONNRESET
    sent += static_cast<std::size_t>(n);
  }
  return true;
}

bool send_all(int fd, const std::string & s) { return send_all(fd, s.data(), s.size()); }

bool send_all(int fd, const std::vector<uchar> & v)
{
  return send_all(fd, reinterpret_cast<const char *>(v.data()), v.size());
}

void set_timeout(int fd, int option, int ms)
{
  struct timeval tv;
  tv.tv_sec = ms / 1000;
  tv.tv_usec = (ms % 1000) * 1000;
  ::setsockopt(fd, SOL_SOCKET, option, &tv, sizeof(tv));
}

// 客户端线程退出时把 done 置位，accept 线程据此回收（join）线程对象
struct DoneGuard
{
  std::shared_ptr<std::atomic<bool>> done;
  ~DoneGuard() { *done = true; }
};
}  // namespace

namespace tools
{

MjpegServer::MjpegServer(int port, int jpeg_quality) : port_(port), quality_(jpeg_quality)
{
  if (port_ <= 0) return;  // 关闭状态：不占端口、不建线程

  listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd_ < 0) {
    throw std::runtime_error(std::string("MJPEG socket 创建失败: ") + std::strerror(errno));
  }

  int yes = 1;
  ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  struct sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(static_cast<uint16_t>(port_));

  if (::bind(listen_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    auto err = std::string(std::strerror(errno));
    ::close(listen_fd_);
    listen_fd_ = -1;
    throw std::runtime_error("MJPEG 端口 " + std::to_string(port_) + " 绑定失败: " + err);
  }
  if (::listen(listen_fd_, 4) < 0) {
    auto err = std::string(std::strerror(errno));
    ::close(listen_fd_);
    listen_fd_ = -1;
    throw std::runtime_error("MJPEG 端口 " + std::to_string(port_) + " listen 失败: " + err);
  }
  set_timeout(listen_fd_, SO_RCVTIMEO, kAcceptTimeoutMs);

  log_urls();
  accept_thread_ = std::thread(&MjpegServer::accept_loop, this);
}

MjpegServer::~MjpegServer()
{
  if (port_ <= 0) return;

  quit_ = true;
  frame_cv_.notify_all();
  if (listen_fd_ >= 0) ::shutdown(listen_fd_, SHUT_RDWR);

  if (accept_thread_.joinable()) accept_thread_.join();
  if (listen_fd_ >= 0) {
    ::close(listen_fd_);
    listen_fd_ = -1;
  }

  std::vector<Client> clients;
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    clients.swap(client_threads_);
  }
  for (auto & c : clients) {
    if (c.thread.joinable()) c.thread.join();
  }
  if (!clients.empty()) logger()->info("[MjpegServer] 直播服务已关闭");
}

void MjpegServer::publish(const cv::Mat & bgr)
{
  if (port_ <= 0 || bgr.empty()) return;
  if (clients_.load() == 0) return;  // 没人看：不做编码，publish 只花一次原子读

  std::vector<uchar> jpeg;
  std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, quality_};
  if (!cv::imencode(".jpg", bgr, jpeg, params)) return;

  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_jpeg_.swap(jpeg);
    sequence_++;
  }
  frame_cv_.notify_all();
}

void MjpegServer::accept_loop()
{
  while (!quit_) {
    struct sockaddr_in peer;
    socklen_t peer_len = sizeof(peer);
    int fd = ::accept(listen_fd_, reinterpret_cast<struct sockaddr *>(&peer), &peer_len);
    if (fd < 0) continue;  // EAGAIN（超时）/ EINTR：回去判 quit_

    if (clients_.load() >= kMaxClients) {
      const char * busy = "HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\n\r\n";
      send_all(fd, busy, std::strlen(busy));
      ::close(fd);
      logger()->warn("[MjpegServer] 客户端数已达上限 {}，拒绝新连接", kMaxClients);
      continue;
    }

    set_timeout(fd, SO_SNDTIMEO, kSendTimeoutMs);
    set_timeout(fd, SO_RCVTIMEO, kRecvTimeoutMs);

    reap_finished_clients();

    auto done = std::make_shared<std::atomic<bool>>(false);
    clients_++;
    logger()->info(
      "[MjpegServer] 客户端接入 {}:{}（当前 {} 个）", ::inet_ntoa(peer.sin_addr),
      ntohs(peer.sin_port), clients_.load());
    std::lock_guard<std::mutex> lock(clients_mutex_);
    client_threads_.push_back(Client{std::thread(&MjpegServer::serve, this, fd, done), done});
  }
}

// 回收已经结束的客户端线程（没有它，反复刷新页面会积累未 join 的线程对象）
void MjpegServer::reap_finished_clients()
{
  std::lock_guard<std::mutex> lock(clients_mutex_);
  for (auto it = client_threads_.begin(); it != client_threads_.end();) {
    if (it->done->load()) {
      if (it->thread.joinable()) it->thread.join();
      it = client_threads_.erase(it);
    } else {
      ++it;
    }
  }
}

void MjpegServer::serve(int fd, const std::shared_ptr<std::atomic<bool>> & done)
{
  DoneGuard guard{done};

  // 只读一包请求头就够：用它区分"实时流"和"抓图"
  char req[1024];
  auto n = ::recv(fd, req, sizeof(req) - 1, 0);
  std::string request = (n > 0) ? std::string(req, static_cast<std::size_t>(n)) : std::string();

  auto leave = [this, fd]() {
    ::close(fd);
    clients_--;
    logger()->info("[MjpegServer] 客户端离开（当前 {} 个）", clients_.load());
  };

  if (n <= 0) {  // 请求头都没读到（客户端直接断开 / 读超时）：不必为它起拉流循环
    leave();
    return;
  }

  // 请求行 "GET /snapshot.jpg HTTP/1.1" 里的路径（做前后缀匹配，容忍 query string）
  std::string path = "/";
  {
    auto sp1 = request.find(' ');
    if (sp1 != std::string::npos) {
      auto sp2 = request.find(' ', sp1 + 1);
      path = request.substr(
        sp1 + 1, sp2 == std::string::npos ? std::string::npos : sp2 - sp1 - 1);
    }
  }
  const bool snapshot = path.rfind("/snapshot", 0) == 0;
  const bool root = (path == "/" || path.rfind("/index", 0) == 0);

  if (!snapshot && !root) {
    // 其它路径（含浏览器自动请求的 /favicon.ico）统一 404，不默默给一个视频流
    const char * not_found =
      "HTTP/1.1 404 Not Found\r\nConnection: close\r\nContent-Length: 0\r\n\r\n";
    send_all(fd, not_found, std::strlen(not_found));
    leave();
    return;
  }

  if (snapshot) {
    std::vector<uchar> jpeg;
    {
      std::unique_lock<std::mutex> lock(frame_mutex_);
      frame_cv_.wait_for(
        lock, std::chrono::milliseconds(1000), [this] { return !latest_jpeg_.empty(); });
      jpeg = latest_jpeg_;
    }
    if (jpeg.empty()) {
      const char * no_frame = "HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\n\r\n";
      send_all(fd, no_frame, std::strlen(no_frame));
    } else {
      send_all(fd, kSnapshotHeader);
      send_all(fd, "Content-Length: " + std::to_string(jpeg.size()) + "\r\n\r\n");
      send_all(fd, jpeg);
    }
    ::shutdown(fd, SHUT_RDWR);
    leave();
    return;
  }

  if (!send_all(fd, kStreamHeader)) {
    leave();
    return;
  }

  unsigned long long last = 0;
  while (!quit_) {
    std::vector<uchar> jpeg;
    unsigned long long seq = 0;
    {
      std::unique_lock<std::mutex> lock(frame_mutex_);
      frame_cv_.wait_for(
        lock, std::chrono::milliseconds(1000), [this, &last] { return quit_ || sequence_ != last; });
      if (quit_) break;
      if (sequence_ == last) {
        jpeg.clear();  // 没有新帧：只发一个空行保活（顺便探测客户端是否已断开）
      } else {
        jpeg = latest_jpeg_;
        seq = sequence_;
      }
    }
    last = seq;

    if (jpeg.empty()) {
      if (!send_all(fd, "\r\n")) break;
      continue;
    }

    std::string part = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                       std::to_string(jpeg.size()) + "\r\n\r\n";
    if (!send_all(fd, part) || !send_all(fd, jpeg) || !send_all(fd, "\r\n")) break;
  }

  ::shutdown(fd, SHUT_RDWR);
  leave();
}

void MjpegServer::log_urls() const
{
  logger()->info(
    "[MjpegServer] MJPEG 直播已启动：实时流 http://<本机IP>:{}/ ，抓图 http://<本机IP>:{}/snapshot.jpg",
    port_, port_);

  struct ifaddrs * ifs = nullptr;
  if (::getifaddrs(&ifs) != 0) {
    logger()->info("[MjpegServer]   http://127.0.0.1:{}/   (lo)", port_);
    return;
  }
  for (auto * it = ifs; it != nullptr; it = it->ifa_next) {
    if (it->ifa_addr == nullptr || it->ifa_addr->sa_family != AF_INET) continue;
    auto * addr = reinterpret_cast<struct sockaddr_in *>(it->ifa_addr);
    logger()->info("[MjpegServer]   http://{}:{}/   ({})", ::inet_ntoa(addr->sin_addr), port_, it->ifa_name);
  }
  ::freeifaddrs(ifs);
}

}  // namespace tools
