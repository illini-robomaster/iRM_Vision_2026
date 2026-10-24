#ifndef IO__SOCKETCAN_HPP
#define IO__SOCKETCAN_HPP

#include <linux/can.h>
#include <net/if.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

constexpr int MAX_EVENTS = 10;

namespace io
{
class SocketCAN
{
public:
  SocketCAN(const std::string & interface, std::function<void(const can_frame & frame)> rx_handler)
  : interface_(interface),
    socket_fd_(-1),
    epoll_fd_(-1),
    rx_handler_(rx_handler),
    quit_(false),
    ok_(false)
  {
    try_open();

    // 守护线程
    daemon_thread_ = std::thread{[this] {
      while (!quit_) {
        std::this_thread::sleep_for(100ms);

        if (ok_) continue;

        if (read_thread_.joinable()) read_thread_.join();

        close();
        try_open();
      }
    }};
  }

  ~SocketCAN()
  {
    quit_ = true;
    if (daemon_thread_.joinable()) daemon_thread_.join();
    if (read_thread_.joinable()) read_thread_.join();
    close();
    tools::logger()->info("SocketCAN destructed.");
  }

  // 写入一帧（阻塞调用方，直到内核接受）。
  // 必须检查「写出去的字节数 == 整帧大小」：SocketCAN 默认不会截断，但只要返回值
  // 不是 sizeof(can_frame) 就说明这一帧没有完整送达，调用方（CBoard::send）据此
  // 判断发送失败，而不是「抛出没抛异常」。
  void write(can_frame * frame) const
  {
    if (socket_fd_ == -1) throw std::runtime_error("SocketCAN 未打开，无法写入");

    const ssize_t n = ::write(socket_fd_, frame, sizeof(can_frame));
    if (n == -1)
      throw std::runtime_error(
        std::string("SocketCAN write 失败：") + std::strerror(errno) + "（接口 " + interface_ +
        "）");
    if (n != static_cast<ssize_t>(sizeof(can_frame)))
      throw std::runtime_error(
        "SocketCAN write 只写入了 " + std::to_string(n) + "/" + std::to_string(sizeof(can_frame)) +
        " 字节，帧不完整");
  }

private:
  std::string interface_;
  int socket_fd_;
  int epoll_fd_;
  bool quit_;
  bool ok_;
  std::thread read_thread_;
  std::thread daemon_thread_;
  can_frame frame_;
  epoll_event events_[MAX_EVENTS];
  std::function<void(const can_frame & frame)> rx_handler_;

  void open()
  {
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) throw std::runtime_error("Error opening socket!");

    ifreq ifr;
    std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
      ::close(socket_fd_);
      socket_fd_ = -1;
      throw std::runtime_error("Error getting interface index!");
    }

    sockaddr_can addr;
    std::memset(&addr, 0, sizeof(sockaddr_can));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_fd_, (sockaddr *)&addr, sizeof(sockaddr_can)) < 0) {
      ::close(socket_fd_);
      socket_fd_ = -1;
      throw std::runtime_error("Error binding socket to interface!");
    }

    epoll_event ev;
    epoll_fd_ = epoll_create1(0);
    if (epoll_fd_ == -1) {
      ::close(socket_fd_);
      socket_fd_ = -1;
      throw std::runtime_error("Error creating epoll file descriptor!");
    }

    ev.events = EPOLLIN;
    ev.data.fd = socket_fd_;
    if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, ev.data.fd, &ev)) {
      ::close(epoll_fd_);
      epoll_fd_ = -1;
      ::close(socket_fd_);
      socket_fd_ = -1;
      throw std::runtime_error("Error adding socket to epoll file descriptor!");
    }

    // 接收线程
    read_thread_ = std::thread([this]() {
      ok_ = true;
      while (!quit_) {
        std::this_thread::sleep_for(10us);

        try {
          read();
        } catch (const std::exception & e) {
          tools::logger()->warn("SocketCAN::read() failed: {}", e.what());
          ok_ = false;
          break;
        }
      }
    });

    tools::logger()->info("SocketCAN opened.");
  }

  void try_open()
  {
    try {
      open();
    } catch (const std::exception & e) {
      tools::logger()->warn("SocketCAN::open() failed: {}", e.what());
    }
  }

  void read()
  {
    if (epoll_fd_ == -1 || socket_fd_ == -1) throw std::runtime_error("SocketCAN 未打开，无法读取");

    int num_events = epoll_wait(epoll_fd_, events_, MAX_EVENTS, 2);
    if (num_events == -1) {
      if (errno == EINTR) return;  // 被信号打断：不是错误，下一轮继续
      throw std::runtime_error(std::string("Error waiting for events: ") + std::strerror(errno));
    }

    for (int i = 0; i < num_events; i++) {
      ssize_t num_bytes = recv(socket_fd_, &frame_, sizeof(can_frame), MSG_DONTWAIT);
      if (num_bytes == -1) {
        // 事件已经被别的读操作消费掉：不是链路故障，继续等下一次 epoll
        if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
        throw std::runtime_error(
          std::string("Error reading from SocketCAN: ") + std::strerror(errno));
      }
      // 不完整的帧不交给上层解码（data[] 会是上一帧的残留值）
      if (num_bytes != static_cast<ssize_t>(sizeof(can_frame))) {
        tools::logger()->warn(
          "SocketCAN 收到不完整帧（{}/{} 字节），已丢弃", num_bytes, sizeof(can_frame));
        continue;
      }

      rx_handler_(frame_);
    }
  }

  void close()
  {
    // 关闭后必须把 fd 复位成 -1：否则守护线程下一轮 close() 会对同一个编号（可能已被
    // 别的 open 复用）再关一次 —— 那是"关掉别人 fd"的经典 bug；write() 也会往已关闭的
    // 编号里写。
    if (socket_fd_ != -1) {
      if (epoll_fd_ != -1) epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, socket_fd_, NULL);
      ::close(socket_fd_);
      socket_fd_ = -1;
    }
    if (epoll_fd_ != -1) {
      ::close(epoll_fd_);
      epoll_fd_ = -1;
    }
  }
};

}  // namespace io

#endif  // IO__SOCKETCAN_HPP